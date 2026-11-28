"""One interface between the voice loop and the agent, in-process or over TCP.

`AgentHost` wraps an Agent: one lock (the agent is not thread-safe), a pump thread that turns executor events / alerts into
spoken messages, and the fast stop path (`estop()` does NOT wait for the lock: a stop must not queue behind a 1 s LLM call).
`serve()` exposes an AgentHost as JSON lines over TCP, `RemoteHost` is the client with the same methods. That split exists
because the ROS agent lives in the sim image (rclpy) while the microphone / Whisper / Piper live in the `voice` image.

Protocol (one JSON object per line):  client -> {"op":"say","id":n,"text":..,"conf":..} | {"op":"estop"} | {"op":"state"}
                                      server -> {"op":"reply","id":n,"text":..} | {"op":"msg","text":..} | {"op":"state","moving":b,"places":[..]}
No authentication: bind it to the host only (docker -p 127.0.0.1:PORT:PORT)."""
from __future__ import annotations

import json
import socket
import threading
import time
from typing import Callable


def _error_text(e: BaseException) -> str:
    """What the robot says when a turn crashed: a plain sentence for "the language model is unreachable", else the generic message."""
    if hasattr(e, "user_text"):
        return e.user_text
    return f"Something went wrong: {type(e).__name__}: {e}"


class AgentHost:
    def __init__(self, agent, pump_s: float = 0.25, sim_speed: float | None = None):
        self.agent, self.pump_s = agent, pump_s
        self.sim_speed = sim_speed          # mock world only: advance its virtual clock this many times faster than wall time (None: someone else does)
        self.lock = threading.Lock()
        self.listeners: list[Callable[[str], None]] = []
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # ---- interface shared with RemoteHost
    def say(self, text: str, conf: float | None = None, started_ago: float | None = None) -> str:
        """`started_ago`: seconds (wall) since the user started speaking this utterance (None: typed, now)."""
        with self.lock:
            try:
                clk = self.agent.clock
                return self.agent.say(text, conf, None if started_ago is None else clk.wall() - started_ago).reply
            except Exception as e:                                # noqa: BLE001  (Agent.say already catches; this covers the clock / lock path)
                import sys
                import traceback
                traceback.print_exc(file=sys.stderr)
                return _error_text(e)

    def spoken_done(self) -> None:
        """The robot finished speaking (a question's answer window starts now)."""
        self.agent.clock.spoken_done()

    def answer_hint(self) -> "str | None":
        return self.agent.answer_hint()

    def estop(self) -> None:
        """Fast path: cancel the mission and stop the robot without taking the agent lock (idempotent)."""
        try:
            self.agent.executor.cancel()
        except Exception:
            pass
        self.agent.layer.call("stop")
        be = self.agent.layer.backend
        fs = getattr(be, "fetch_session", None)
        if callable(getattr(fs, "cancel_plan", None)):    # a fetch plan waiting for "which one?": dropped, announced by the pump
            try:
                fs.cancel_plan("emergency stop", announce=True)
            except Exception:                              # noqa: BLE001
                pass
        if callable(getattr(be, "estop", None)):          # real robot: also latch /estop (policy stands, gate blocks velocity) until released
            try:
                be.estop(True)
            except Exception:                              # noqa: BLE001  (the stop above already happened)
                pass

    def release_estop(self) -> None:
        """Release a latched e-stop (real robot only; the mock world has no latch)."""
        be = self.agent.layer.backend
        if callable(getattr(be, "estop", None)):
            be.estop(False)

    def is_moving(self) -> bool:
        return self.agent.executor.state == "running"

    def places(self) -> list[str]:
        return list(self.agent.layer.backend.known_places())

    def on_message(self, fn: Callable[[str], None]) -> None:
        self.listeners.append(fn)

    def step(self, dt_wall: float) -> list[str]:
        """One pump step: advance the mock world by `dt_wall * sim_speed` (mock only), let the agent turn executor events / alerts into messages,
        deliver them to the listeners and return them. The pump thread calls this every `pump_s`; the browser demo (no threads) calls it from a timer."""
        try:
            with self.lock:
                be = self.agent.layer.backend
                if self.sim_speed and hasattr(be, "advance"):      # without this a mock trip never ends
                    be.advance(dt_wall * self.sim_speed)
                msgs = self.agent.pump()
        except Exception as e:                                # noqa: BLE001  one bad pump must not kill the loop (alerts / mission results)
            import sys
            import traceback
            traceback.print_exc(file=sys.stderr)
            msgs = [_error_text(e)]
        for m in msgs:
            for fn in list(self.listeners):
                fn(m)
        return msgs

    def start(self) -> None:
        def loop():
            last = time.monotonic()
            while not self._stop.is_set():
                time.sleep(self.pump_s)
                now = time.monotonic()
                self.step(now - last)
                last = now
        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()


def serve(host: AgentHost, port: int, bind: str = "0.0.0.0") -> threading.Thread:
    """Serve `host` to one client at a time (a reconnect replaces the old one). Returns the accept thread (daemon)."""
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((bind, port))
    srv.listen(1)
    clients: list = []

    def push(m: str):
        for f in list(clients):
            try:
                f.write(json.dumps({"op": "msg", "text": m}) + "\n"); f.flush()
            except OSError:
                pass
    host.on_message(push)

    def handle(conn):
        f = conn.makefile("rw", encoding="utf-8", newline="\n")
        clients.append(f)
        wlock = threading.Lock()

        def send(o):
            with wlock:
                try:
                    f.write(json.dumps(o) + "\n"); f.flush()
                except OSError:
                    pass
        def _answer(r):
            """Always answers: a crash must not leave the client waiting for its 60 s timeout."""
            try:
                text = host.say(r.get("text", ""), r.get("conf"), r.get("started_ago"))
            except BaseException as e:                                # noqa: BLE001
                text = _error_text(e)
            send({"op": "reply", "id": r.get("id"), "text": text})
        try:
            for line in f:
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                op = r.get("op")
                if op == "estop":
                    host.estop(); send({"op": "ok"})
                elif op == "spoken":
                    host.spoken_done()
                elif op == "state":
                    send({"op": "state", "moving": host.is_moving(), "places": host.places(), "hint": host.answer_hint()})
                elif op == "say":
                    # off the reading thread: a slow LLM turn must not delay an {"op":"estop"} that arrives meanwhile
                    threading.Thread(target=_answer, args=(r,), daemon=True).start()
        finally:
            if f in clients:
                clients.remove(f)
            conn.close()

    def accept():
        while True:
            conn, _ = srv.accept()
            threading.Thread(target=handle, args=(conn,), daemon=True).start()
    t = threading.Thread(target=accept, daemon=True)
    t.start()
    return t


class RemoteHost:
    """Client for `serve()`. `is_moving()` / `places()` use a state cache refreshed every `state_s` seconds in the background."""

    def __init__(self, addr: str, state_s: float = 0.5, timeout_s: float = 60.0):
        h, p = addr.rsplit(":", 1)
        self.sock = socket.create_connection((h, int(p)), timeout=10)
        self.sock.settimeout(None)
        self.f = self.sock.makefile("rw", encoding="utf-8", newline="\n")
        self.wl = threading.Lock()
        self.timeout_s, self.state_s = timeout_s, state_s
        self.listeners: list[Callable[[str], None]] = []
        self._replies: dict[int, str] = {}
        self._cv = threading.Condition()
        self._id = 0
        self._state = {"moving": False, "places": []}
        self._closed = False
        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._poll, daemon=True).start()

    def _send(self, o):
        with self.wl:
            self.f.write(json.dumps(o) + "\n"); self.f.flush()

    def _read(self):
        try:
            for line in self.f:
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                if r["op"] == "reply":
                    with self._cv:
                        self._replies[r["id"]] = r["text"]; self._cv.notify_all()
                elif r["op"] == "msg":
                    for fn in list(self.listeners):
                        fn(r["text"])
                elif r["op"] == "state":
                    self._state = r
        except (OSError, ValueError):
            pass
        self._closed = True
        with self._cv:
            self._cv.notify_all()

    def _poll(self):
        while not self._closed:
            try:
                self._send({"op": "state"})
            except OSError:
                return
            time.sleep(self.state_s)

    def spoken_done(self) -> None:
        try:
            self._send({"op": "spoken"})
        except OSError:
            pass

    def say(self, text: str, conf: float | None = None, started_ago: float | None = None) -> str:
        with self._cv:
            self._id += 1
            i = self._id
        self._send({"op": "say", "id": i, "text": text, "conf": conf, "started_ago": started_ago})
        end = time.monotonic() + self.timeout_s
        with self._cv:
            while i not in self._replies and not self._closed and time.monotonic() < end:
                self._cv.wait(0.5)
            return self._replies.pop(i, "I lost the connection to the robot." if self._closed else "The robot did not answer in time.")

    def estop(self) -> None:
        self._send({"op": "estop"})

    def is_moving(self) -> bool:
        return bool(self._state.get("moving"))

    def places(self) -> list[str]:
        return list(self._state.get("places", []))

    def answer_hint(self) -> "str | None":
        return self._state.get("hint")

    def on_message(self, fn) -> None:
        self.listeners.append(fn)

    def start(self) -> None:
        pass

    def close(self) -> None:
        self._closed = True
        try:
            self.sock.close()
        except OSError:
            pass
