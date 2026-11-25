"""Mission-control API server (Phase 7 step 1). Standard library only: threading HTTP server + WebSocket (web/api/ws.py).

It sits in front of an `AgentHost` / `RemoteHost` (brain/voice/host.py): every command goes through `host.say` / `host.estop`, so Phase 5 safety
(checks, confirmations, e-stop fast path) is untouched. Telemetry comes from a `Telemetry` source. Design: docs/phase7_design.md.

Demo mode (`demo=True`, for a public simulated-robot page): no token, MockBackend ONLY (refused at construction for anything else, so a public
page can never reach a real robot), per-IP rate limits, shorter messages, and per-visitor sessions (see `Session`): every browser session id gets its own
mock world, agent and chat, created on first use, freed after 10 idle minutes, capped (default 30) with a "busy" answer when full.

Auth: one shared token (env GURDY_WEB_TOKEN, else random and printed once by the CLI, never written anywhere). Everything under /api and the
WebSocket needs it except /api/health; static files are public (they contain no secrets)."""
from __future__ import annotations

import base64
import hmac
import json
import mimetypes
import os
import queue
import re
import secrets
import socket
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlsplit

from . import ws
from .limits import RateLimiter
from .telemetry import MockTelemetry, Telemetry

VERSION = "0.1"
MAX_BODY = 64 * 1024
MAX_TEXT = 500
MAX_WS_CLIENTS = 8
CHAT_KEEP = 200
DEMO_MAX_TEXT = 200
DEMO_MAX_BODY = 4 * 1024
DEMO_MAX_WS_CLIENTS = 20
DEMO_WS_PER_IP = 3
DEMO_SAY = (5, 4.0)               # per IP: burst 5, then one message per 4 s
DEMO_ESTOP = (10, 1.0)
DEMO_ANY = (60, 0.05)             # per IP: all /api requests, burst 60, 20 per second sustained
DEMO_MAX_SESSIONS = 30            # per-visitor sessions alive at once; the 31st visitor gets "the demo is busy"
DEMO_SESSION_IDLE_S = 600.0       # a session with no request / command for 10 minutes is freed
DEMO_SESSIONS_PER_IP = 6          # one address cannot hold more than this many sessions (a script must not fill all slots)
DEMO_NEW_SESSION = (4, 30.0)      # per IP: new sessions burst 4, then one per 30 s
DEMO_MAX_WS_PER_SESSION = 3       # total WebSocket cap = this x max_sessions (also capped per IP)
SESSION_ID_RE = re.compile(r"^[0-9a-f]{32}$")
IDLE_TIMEOUT_S = 30
DEFAULT_STATIC = Path(__file__).resolve().parents[1] / "app"
# Pages allowed to call this server from a browser (CORS + WebSocket Origin check): the public static demo's "Connect to my robot".
# Same-origin use (the page served by this server, whatever address it is reached by) is always allowed. Never a wildcard. Env GURDY_ALLOWED_ORIGINS
# (comma-separated) or --allowed-origin override; "none" = no other origin. The access token is required either way: CORS only lets that page TRY.
DEFAULT_ALLOWED_ORIGINS = ("https://mrsssok2.github.io",)
CORS_HEADERS = "Authorization, Content-Type, X-Session"


def parse_origins(value: str | None) -> tuple[str, ...]:
    """"https://a.github.io, https://b" -> ("https://a.github.io", "https://b"); None -> the default; "none" / "" -> ()."""
    if value is None:
        return DEFAULT_ALLOWED_ORIGINS
    items = [x.strip().rstrip("/") for x in value.split(",") if x.strip() and x.strip().lower() != "none"]
    for o in items:
        if not re.match(r"^https?://[A-Za-z0-9.\-\[\]:]+$", o):
            raise ValueError(f"bad origin {o!r}: use exactly scheme://host[:port] (a wildcard is never allowed)")
    return tuple(items)


class ChatLog:
    """Shared conversation + announcements, newest last, monotonically numbered, so every device shows the same history and a client that
    reconnects asks only for what it missed (`since`)."""

    def __init__(self, on_add=None):
        self._lock = threading.Lock()
        self._items: deque[dict] = deque(maxlen=CHAT_KEEP)
        self._seq = 0
        self.on_add = on_add

    def clear(self) -> None:
        """Forget the history; numbering continues, so a polling client's `since` stays valid."""
        with self._lock:
            self._items.clear()

    def add(self, role: str, text: str, **meta) -> dict:
        """`meta`: optional extra fields on the item (voice messages: voice=True, confidence, stt_s)."""
        with self._lock:
            self._seq += 1
            item = {"seq": self._seq, "t": round(time.time(), 1), "role": role, "text": text, **{k: v for k, v in meta.items() if v is not None}}
            self._items.append(item)
        if self.on_add:
            self.on_add(item)
        return item

    def since(self, seq: int = 0) -> list[dict]:
        with self._lock:
            return [dict(i) for i in self._items if i["seq"] > seq]

    @property
    def last_seq(self) -> int:
        with self._lock:
            return self._seq


_CLOSE_AFTER = object()


class _WsClient:
    """One writer thread + bounded queue per client: a stalled phone can never block the broadcast to the others (it is dropped when full)."""

    def __init__(self, sock: socket.socket, on_dead):
        self.sock, self.on_dead = sock, on_dead
        self.ip = ""
        self.q: queue.Queue = queue.Queue(maxsize=50)
        self.dead = False
        threading.Thread(target=self._run, daemon=True).start()

    def send(self, frame: bytes) -> None:
        if self.dead:
            return
        try:
            self.q.put_nowait(frame)
        except queue.Full:
            self.kill()

    def _run(self) -> None:
        while True:
            frame = self.q.get()
            if frame is None:
                return
            if frame is _CLOSE_AFTER:                    # everything queued before it has been sent: now close
                self.kill()
                return
            try:
                self.sock.sendall(frame)
            except OSError:
                self.kill()
                return

    def close_after_flush(self) -> None:
        """Close the connection once the frames already queued (e.g. a last notice) have been written."""
        try:
            self.q.put_nowait(_CLOSE_AFTER)
        except queue.Full:
            self.kill()

    def kill(self) -> None:
        if self.dead:
            return
        self.dead = True
        try:
            self.q.put_nowait(None)
        except queue.Full:
            pass
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.on_dead(self)


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def handle_error(self, request, client_address) -> None:
        import sys
        if isinstance(sys.exc_info()[1], (ConnectionError, TimeoutError)):      # a phone going out of Wi-Fi range is normal, not a traceback
            return
        super().handle_error(request, client_address)


class _TlsServer(_Server):
    """Same server on a second port, TLS. The handshake happens lazily in the handler thread (do_handshake_on_connect=False), so a client that
    connects and says nothing cannot stall the accept loop."""

    def __init__(self, addr, handler, ctx):
        self._ctx = ctx
        super().__init__(addr, handler)

    def get_request(self):
        sock, addr = super().get_request()
        return self._ctx.wrap_socket(sock, server_side=True, do_handshake_on_connect=False), addr


class Session:
    """One visitor's isolated world: its own host (agent + mock robot), telemetry, chat log and WebSocket clients. In the private / single-robot
    modes there is exactly one, `default`; in the public demo every browser session gets its own (nothing is shared between sessions)."""

    def __init__(self, sid: str, host, telemetry, ip: str, now: float):
        self.sid, self.host, self.telemetry, self.ip = sid, host, telemetry, ip
        self.chat: ChatLog | None = None
        self.voice = None
        self.clients: list[_WsClient] = []
        self.created = self.last_active = now


class GurdyWebServer:
    def __init__(self, host=None, telemetry: Telemetry | None = None, token: str | None = None, static_dir: str | Path | None = DEFAULT_STATIC,
                 bind: str = "127.0.0.1", port: int = 8080, state_s: float = 0.5, demo: bool = False, host_factory=None,
                 trust_proxy: bool = False, clock=time.monotonic, speech=None,
                 tls_port: int | None = None, tls_cert: str | None = None, tls_key: str | None = None, ca_file: str | None = None,
                 max_sessions: int = DEMO_MAX_SESSIONS, session_idle_s: float = DEMO_SESSION_IDLE_S, reap_every_s: float = 15.0,
                 allowed_origins: tuple[str, ...] | list[str] | None = None):
        """`host_factory` (demo only) turns on per-visitor sessions: it returns a fresh `AgentHost` on the MockBackend for every new session id.
        Without it, `host` is the one robot everybody talks to (private mode, or the single-session demo used by some tests)."""
        self.allowed_origins = tuple(allowed_origins) if allowed_origins is not None else parse_origins(os.environ.get("GURDY_ALLOWED_ORIGINS") or None)
        self.state_s, self.clock = state_s, clock
        self.demo, self.trust_proxy = demo, trust_proxy
        self.multi = bool(demo and host_factory is not None)
        if host_factory is not None and not demo:
            raise ValueError("host_factory (per-visitor sessions) is for the public demo only")
        if not self.multi and host is None:
            raise ValueError("a host is required (or demo=True with a host_factory)")
        self.host_factory, self.max_sessions, self.session_idle_s, self.reap_every_s = host_factory, max_sessions, session_idle_s, reap_every_s
        self.max_text = DEMO_MAX_TEXT if demo else MAX_TEXT
        self.max_body = DEMO_MAX_BODY if demo else MAX_BODY
        self.max_ws = (DEMO_MAX_WS_PER_SESSION * max_sessions if self.multi else DEMO_MAX_WS_CLIENTS) if demo else MAX_WS_CLIENTS
        self.ws_per_ip = DEMO_WS_PER_IP if demo else None
        self.say_limiter = RateLimiter(*DEMO_SAY, clock=clock) if demo else None
        self.estop_limiter = RateLimiter(*DEMO_ESTOP, clock=clock) if demo else None
        self.any_limiter = RateLimiter(*DEMO_ANY, clock=clock) if demo else None
        self.new_session_limiter = RateLimiter(*DEMO_NEW_SESSION, clock=clock) if self.multi else None
        self._ip_clients: dict[str, int] = {}
        self._n_clients = 0
        self.token = token or os.environ.get("GURDY_WEB_TOKEN") or secrets.token_urlsafe(16)
        self.static_dir = Path(static_dir).resolve() if static_dir else None
        self.ca_file = Path(ca_file) if ca_file else None
        self._speech = speech
        self._clients_lock = threading.Lock()
        self._sessions: dict[str, Session] = {}
        self._sess_lock = threading.RLock()
        self._stop = threading.Event()
        self.default: Session | None = None
        if not self.multi:
            tel = telemetry or MockTelemetry(host)
            if demo:
                self._require_mock_only(host, tel)
            self.default = self._make_session("default", host, tel, "")
        server = self
        handler = type("Handler", (_Handler,), {"app": server})
        self.httpd = _Server((bind, port), handler)
        self.port = self.httpd.server_address[1]
        self.httpd_tls, self.tls_port = None, None
        if tls_cert and tls_key:                         # microphones need a secure context: https on a second port (localhost works without)
            import ssl
            ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ctx.minimum_version = ssl.TLSVersion.TLSv1_2
            ctx.load_cert_chain(tls_cert, tls_key)
            self.httpd_tls = _TlsServer((bind, tls_port if tls_port is not None else 0), handler, ctx)
            self.tls_port = self.httpd_tls.server_address[1]
        self._threads: list[threading.Thread] = []

    # ------------------------------------------------------------------ the single-robot attributes (private mode / single-session demo)
    @property
    def host(self):
        return self.default.host

    @property
    def telemetry(self):
        return self.default.telemetry

    @telemetry.setter
    def telemetry(self, value) -> None:
        self.default.telemetry = value

    @property
    def chat(self) -> ChatLog:
        return self.default.chat

    @property
    def voice(self):
        return self.default.voice if self.default else None

    # ------------------------------------------------------------------ sessions
    def _make_session(self, sid: str, host, telemetry, ip: str) -> Session:
        sess = Session(sid, host, telemetry, ip, self.clock())
        sess.chat = ChatLog(on_add=lambda item: self.broadcast(sess, {"type": "chat", **item}))
        if self._speech is not None and not self.demo:                       # no microphone in the public demo
            from .voice import VoiceGateway
            sess.voice = VoiceGateway(host, self._speech, sess.chat)
        if hasattr(host, "on_message"):
            host.on_message(lambda text: sess.chat.add("robot", text))        # pump announcements / alerts
        return sess

    def session_for(self, sid: str | None, ip: str):
        """-> (Session, None) or (None, (http_status, message, retry_after_s)). In multi-session mode a new valid id creates a session on its
        first request (within the caps); known ids are touched (idle clock restarts). Ids are random 128-bit values chosen by the browser."""
        if not self.multi:
            return self.default, None
        if not sid or not SESSION_ID_RE.match(sid):
            return None, (400, "missing or malformed session id", None)
        with self._sess_lock:
            sess = self._sessions.get(sid)
            if sess is not None:
                sess.last_active = self.clock()
                return sess, None
            if len(self._sessions) >= self.max_sessions:
                return None, (503, "The demo is busy, try again in a minute.", 60)
            if sum(1 for x in self._sessions.values() if x.ip == ip) >= DEMO_SESSIONS_PER_IP:
                return None, (429, "Too many demo sessions from your address. Close other tabs or wait a few minutes.", 60)
            ok, wait = self.new_session_limiter.allow(ip)
            if not ok:
                return None, (429, "You are opening demo sessions too quickly. Wait a moment.", int(wait) + 1)
            host = self.host_factory()
            tel = MockTelemetry(host)
            self._require_mock_only(host, tel)
            sess = self._make_session(sid, host, tel, ip)
            if hasattr(host, "start"):
                host.start()
            self._sessions[sid] = sess
            return sess, None

    def sessions(self) -> list[Session]:
        if not self.multi:
            return [self.default]
        with self._sess_lock:
            return list(self._sessions.values())

    @property
    def n_sessions(self) -> int:
        return len(self._sessions) if self.multi else 1

    def reap(self) -> int:
        """Free sessions idle for longer than `session_idle_s`; returns how many. Their pump threads, worlds and chats are dropped."""
        now = self.clock()
        with self._sess_lock:
            dead = [self._sessions.pop(sid) for sid, x in list(self._sessions.items()) if now - x.last_active > self.session_idle_s]
        for sess in dead:
            self._free(sess)
        return len(dead)

    def _free(self, sess: Session) -> None:
        self.broadcast(sess, {"type": "session_expired", "message": "This demo session timed out after a period of inactivity. A fresh one starts when you reload."})
        for c in list(sess.clients):
            c.close_after_flush()
        close = getattr(sess.host, "close", None)
        if close:
            try:
                close()
            except Exception:                            # noqa: BLE001
                pass

    def _reap_loop(self) -> None:
        while not self._stop.wait(self.reap_every_s):
            try:
                self.reap()
            except Exception:                            # noqa: BLE001  one failed sweep must not stop the next
                import sys
                import traceback
                traceback.print_exc(file=sys.stderr)

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> "GurdyWebServer":
        loops = [lambda: self.httpd.serve_forever(poll_interval=0.05), self._broadcast_loop]
        if self.httpd_tls is not None:
            loops.append(lambda: self.httpd_tls.serve_forever(poll_interval=0.05))
        if self.multi:
            loops.append(self._reap_loop)
        for fn in loops:
            t = threading.Thread(target=fn, daemon=True)
            t.start()
            self._threads.append(t)
        return self

    def close(self) -> None:
        self._stop.set()
        self.httpd.shutdown()
        self.httpd.server_close()
        if self.httpd_tls is not None:
            self.httpd_tls.shutdown()
            self.httpd_tls.server_close()
        for sess in self.sessions():
            for c in list(sess.clients):
                c.kill()
            if self.multi:
                close = getattr(sess.host, "close", None)
                if close:
                    close()

    # ------------------------------------------------------------------ helpers used by the handler
    @staticmethod
    def _require_mock_only(host, telemetry) -> None:
        """Demo = simulated robot only. Checked on the real objects, so a RemoteHost / RosBackend / any other telemetry is refused."""
        from brain.mock import MockBackend
        backend = getattr(getattr(getattr(host, "agent", None), "layer", None), "backend", None)
        from brain.incidents.backend import IncidentBackend
        while isinstance(backend, IncidentBackend):          # the timeline wrapper is transparent: what it wraps must still be the mock
            backend = backend._inner
        if not isinstance(backend, MockBackend) or not isinstance(telemetry, MockTelemetry):
            raise ValueError("demo mode serves the MockBackend only (it is meant to be public and must never reach a real robot)")

    def hello(self, sess: Session) -> dict:
        return {"type": "hello", "version": VERSION, "demo": self.demo, "voice": self.voice_available(sess), "map": sess.telemetry.map(), "chat": sess.chat.since(0)}

    def voice_available(self, sess: Session | None = None) -> bool:
        v = (sess or self.default).voice if (sess or self.default) else None
        return v is not None and v.available()

    def check_token(self, given: str | None) -> bool:
        if self.demo:
            return True                                  # public simulated robot: nothing to protect
        return bool(given) and hmac.compare_digest(given.encode("utf-8", "replace"), self.token.encode())

    def state(self, sess: Session) -> dict[str, Any]:
        try:
            snap = sess.telemetry.snapshot()
            return {**snap, "demo": True} if self.demo else snap
        except Exception as e:                       # noqa: BLE001  a broken source must not kill the stream
            return {"connected": False, "error": f"{type(e).__name__}: {e}"}

    def incidents(self, sess: Session, which: str, q: dict) -> dict:
        from .incidents_api import answer
        return answer(sess.host.agent.layer.backend, which, {k: v[0] for k, v in q.items() if v}, getattr(sess.host, "lock", None))

    def say(self, sess: Session, text: str) -> dict:
        sess.chat.add("user", text)
        reply = sess.host.say(text)
        return sess.chat.add("robot", reply)

    @staticmethod
    def _announces_estop(sess: Session) -> bool:
        """The real robot's backend announces "Emergency stop engaged / released." itself (an event message); then the page must not add its own line."""
        return callable(getattr(getattr(getattr(sess.host, "agent", None), "layer", None) and sess.host.agent.layer.backend, "estop", None))

    def estop(self, sess: Session) -> None:
        sess.host.estop()
        if not self._announces_estop(sess):
            sess.chat.add("system", "Emergency stop sent.")

    def release_estop(self, sess: Session) -> bool:
        fn = getattr(sess.host, "release_estop", None)
        if fn is None or self.demo:
            return False
        fn()
        if not self._announces_estop(sess):
            sess.chat.add("system", "Emergency stop released.")
        return True

    def broadcast(self, sess: Session, obj: dict) -> None:
        """To the WebSocket clients of THIS session only."""
        frame = ws.encode_frame(ws.OP_TEXT, json.dumps(obj, separators=(",", ":")).encode())
        with self._clients_lock:
            clients = list(sess.clients)
        for c in clients:
            c.send(frame)

    def add_client(self, sess: Session, sock: socket.socket, ip: str = "") -> _WsClient | None:
        with self._clients_lock:
            if self._n_clients >= self.max_ws or (self.ws_per_ip and self._ip_clients.get(ip, 0) >= self.ws_per_ip):
                return None
            c = _WsClient(sock, self._drop_client)
            c.ip, c.session = ip, sess
            sess.clients.append(c)
            self._n_clients += 1
            self._ip_clients[ip] = self._ip_clients.get(ip, 0) + 1
            return c

    def _drop_client(self, c: _WsClient) -> None:
        with self._clients_lock:
            sess = getattr(c, "session", None)
            if sess is not None and c in sess.clients:
                sess.clients.remove(c)
                self._n_clients -= 1
                n = self._ip_clients.get(c.ip, 1) - 1
                if n > 0:
                    self._ip_clients[c.ip] = n
                else:
                    self._ip_clients.pop(c.ip, None)

    @property
    def n_clients(self) -> int:
        with self._clients_lock:
            return self._n_clients

    def _broadcast_loop(self) -> None:
        while not self._stop.wait(self.state_s):
            for sess in self.sessions():
                if sess.clients:
                    self.broadcast(sess, {"type": "state", **self.state(sess)})


class _Handler(BaseHTTPRequestHandler):
    app: GurdyWebServer
    protocol_version = "HTTP/1.1"
    server_version = "GurdyWeb/" + VERSION
    timeout = IDLE_TIMEOUT_S                         # a silent HTTP connection is dropped (slowloris); WebSockets switch this off after the upgrade

    def log_message(self, *a) -> None:              # never log: WebSocket URLs carry the token in the query string
        pass

    # ---- cross-origin access (the static demo page on GitHub Pages connecting to this robot)
    _cors_origin: str | None = None

    def end_headers(self) -> None:
        if self._cors_origin:
            self.send_header("Access-Control-Allow-Origin", self._cors_origin)
            self.send_header("Vary", "Origin")
            self.send_header("Access-Control-Expose-Headers", "Retry-After")
        super().end_headers()

    def _origin_allowed(self, origin: str) -> bool:
        origin = origin.rstrip("/")
        host = self.headers.get("Host", "")
        return origin in self.app.allowed_origins or (bool(host) and origin in (f"http://{host}", f"https://{host}"))

    def _begin(self) -> bool:
        """Call first in every do_*: sets the CORS header for an allowed Origin and answers 403 (no CORS headers, so the browser blocks it) for any other
        browser origin. Requests without an Origin header (curl, the app itself on a GET, servers) are unaffected: the token decides."""
        self._cors_origin = None
        origin = self.headers.get("Origin")
        if not origin:
            return True
        if self._origin_allowed(origin):
            self._cors_origin = origin.rstrip("/")
            return True
        self._json({"error": "origin not allowed"}, 403)
        return False

    def do_OPTIONS(self) -> None:                    # CORS preflight: no token (browsers send none), only the Origin decides
        if not self._begin():
            return
        if not self._cors_origin:
            return self._err(403, "cross-origin requests only")
        self.send_response(204)
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", CORS_HEADERS)
        self.send_header("Access-Control-Max-Age", "600")
        if self.headers.get("Access-Control-Request-Private-Network", "").lower() == "true":      # public https page -> tailnet / loopback address
            self.send_header("Access-Control-Allow-Private-Network", "true")
        self.send_header("Content-Length", "0")
        self.end_headers()

    # ---- plumbing
    def _json(self, obj: Any, status: int = 200) -> None:
        body = json.dumps(obj).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def _err(self, status: int, msg: str) -> None:
        self._json({"error": msg}, status)

    def _ip(self) -> str:
        """Client address. Behind a reverse proxy (trust_proxy=True) the proxy's own entry is the LAST X-Forwarded-For element; an
        untrusted setup ignores the header entirely (it is client-controlled)."""
        if self.app.trust_proxy:
            xff = self.headers.get("X-Forwarded-For", "")
            if xff.strip():
                return xff.split(",")[-1].strip()
        return self.client_address[0]

    def _limited(self, limiter: RateLimiter | None) -> bool:
        """True (and a 429 is sent) when this client is over the limit."""
        if limiter is None:
            return False
        ok, wait = limiter.allow(self._ip())
        if ok:
            return False
        body = json.dumps({"error": "too many requests, slow down"}).encode()
        self.send_response(429)
        self.send_header("Retry-After", str(max(1, int(wait + 0.999))))
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)
        return True

    def _session(self, query: dict):
        """The visitor's Session (header `X-Session`, or `?sid=` for WebSockets), created on first use in the public demo. Sends the error and
        returns None when there is none (400 bad id, 503 demo busy, 429 too many sessions / too fast)."""
        sid = self.headers.get("X-Session") or (query.get("sid") or [None])[0]
        sess, err = self.app.session_for(sid, self._ip())
        if err is None:
            return sess
        status, msg, retry = err
        body = json.dumps({"error": msg, "busy": status == 503}).encode()
        self.send_response(status)
        if retry:
            self.send_header("Retry-After", str(retry))
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)
        return None

    def _authed(self, query: dict) -> bool:
        h = self.headers.get("Authorization", "")
        given = h[7:] if h.startswith("Bearer ") else (query.get("token") or [None])[0]
        return self.app.check_token(given)

    def _body(self) -> dict | None:
        try:
            n = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            n = -1
        if n < 0 or n > self.app.max_body:
            self.close_connection = True                 # the body was not read: it must not be parsed as the next request
            self._err(413 if n > self.app.max_body else 400, "bad or too large body")
            return None
        try:
            obj = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            self._err(400, "body must be JSON")
            return None
        if not isinstance(obj, dict):
            self._err(400, "body must be a JSON object")
            return None
        return obj

    def _send_bytes(self, status: int, body: bytes, ctype: str, headers: dict | None = None) -> None:
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (headers or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _raw_body(self, limit: int) -> bytes | None:
        try:
            n = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            n = -1
        if n <= 0 or n > limit:
            self.close_connection = True
            self._err(413 if n > limit else 400, "bad or too large body")
            return None
        return self.rfile.read(n)

    # ---- routes
    def do_GET(self) -> None:
        if not self._begin():
            return
        u = urlsplit(self.path)
        q = parse_qs(u.query)
        if u.path == "/api/health":
            return self._json({"ok": True, "version": VERSION, "demo": self.app.demo, "voice": self.app.voice_available()})
        if u.path == "/ca.crt" and self.app.ca_file is not None and self.app.ca_file.exists():      # the PUBLIC certificate of the local CA, for phones to trust
            return self._send_bytes(200, self.app.ca_file.read_bytes(), "application/x-x509-ca-cert", {"Content-Disposition": 'attachment; filename="gurdy-ca.crt"'})
        if u.path == "/ws":
            return self._websocket(q)
        if u.path.startswith("/api/"):
            if not self._authed(q):
                return self._err(401, "missing or wrong token")
            if self._limited(self.app.any_limiter):
                return
            sess = self._session(q)
            if sess is None:
                return
            if u.path == "/api/state":
                return self._json(self.app.state(sess))
            if u.path == "/api/map":
                return self._json(sess.telemetry.map())
            if u.path == "/api/camera":      # real robot only: one JPEG (base64) + the detector's 2D boxes; the page polls it at ~2 Hz
                cam = getattr(sess.telemetry, "camera", None)
                c = cam() if cam else None
                if c is None:
                    return self._json({"available": False})
                return self._json({"available": True, **{k: v for k, v in c.items() if k != "jpeg"}, "jpeg_b64": base64.b64encode(c["jpeg"]).decode()})
            if u.path in ("/api/timeline", "/api/changes"):
                return self._json(self.app.incidents(sess, u.path.rsplit("/", 1)[1], q))
            if u.path == "/api/messages":
                try:
                    since = int((q.get("since") or ["0"])[0])
                except ValueError:
                    return self._err(400, "since must be an integer")
                return self._json({"messages": sess.chat.since(since), "seq": sess.chat.last_seq})
            return self._err(404, "unknown route")
        self._static(u.path)

    def do_POST(self) -> None:
        if not self._begin():
            return
        u = urlsplit(self.path)
        if not u.path.startswith("/api/"):
            return self._err(404, "unknown route")
        if not self._authed(parse_qs(u.query)):
            return self._err(401, "missing or wrong token")
        if self._limited(self.app.any_limiter):
            return
        sess = self._session(parse_qs(u.query))
        if sess is None:
            return
        if u.path == "/api/estop":
            if self._limited(self.app.estop_limiter):
                return
            self.app.estop(sess)
            return self._json({"ok": True})
        if u.path in ("/api/voice", "/api/tts"):
            if sess.voice is None or not sess.voice.available():
                return self._err(503, "voice is not available on this server")
            from .voice import MAX_AUDIO_BYTES, MAX_TTS_CHARS, SpeechError
            try:
                if u.path == "/api/voice":
                    audio = self._raw_body(MAX_AUDIO_BYTES)
                    if audio is None:
                        return
                    return self._json(sess.voice.handle_audio(audio))
                body = self._body()
                if body is None:
                    return
                text = body.get("text")
                if not isinstance(text, str) or not text.strip() or len(text) > MAX_TTS_CHARS:
                    return self._err(400, "bad 'text'")
                return self._send_bytes(200, sess.voice.speak(text.strip()), "audio/wav")
            except SpeechError as e:
                return self._err(502, str(e))
        if u.path == "/api/estop/release":
            return self._json({"ok": self.app.release_estop(sess)})
        if u.path == "/api/say":
            body = self._body()
            if body is None:
                return
            text = body.get("text")
            if not isinstance(text, str) or not text.strip() or len(text) > self.app.max_text:
                return self._err(400, f"'text' must be a non-empty string of at most {self.app.max_text} characters")
            if self._limited(self.app.say_limiter):
                return
            item = self.app.say(sess, text.strip())
            return self._json({"reply": item["text"], "seq": item["seq"]})
        self._err(404, "unknown route")

    def _static(self, path: str) -> None:
        root = self.app.static_dir
        if root is None:
            return self._err(404, "no static files")
        rel = "index.html" if path in ("", "/") else path.lstrip("/")
        f = (root / rel).resolve()
        if not f.is_relative_to(root) or not f.is_file():
            return self._err(404, "not found")
        body = f.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", mimetypes.guess_type(f.name)[0] or "application/octet-stream")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    # ---- websocket
    def _websocket(self, q: dict) -> None:
        key = self.headers.get("Sec-WebSocket-Key")
        if "websocket" not in self.headers.get("Upgrade", "").lower() or not key:
            return self._err(400, "WebSocket upgrade required")
        if not self._authed(q):
            return self._err(401, "missing or wrong token")
        sess = self._session(q)
        if sess is None:
            return
        client = self.app.add_client(sess, self.connection, self._ip())
        if client is None:
            return self._err(503, "too many clients")
        self.close_connection = True
        self.connection.settimeout(None)                 # idle WebSockets are normal; dead peers are caught by failing sends
        self.wfile.write(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                          f"Sec-WebSocket-Accept: {ws.accept_key(key)}\r\n\r\n").encode())
        self.wfile.flush()
        send_json = lambda o: client.send(ws.encode_frame(ws.OP_TEXT, json.dumps(o).encode()))
        send_json(self.app.hello(sess))
        send_json({"type": "state", **self.app.state(sess)})
        try:
            while True:
                msg = ws.read_message(self.rfile, client.send)
                if msg is None:
                    break
                op, payload = msg
                if op == ws.OP_TEXT:
                    self._ws_command(payload, send_json, sess)
        except ws.WsError as e:
            client.send(ws.encode_frame(ws.OP_CLOSE, e.code.to_bytes(2, "big")))
        except (EOFError, OSError):
            pass
        finally:
            client.send(ws.encode_frame(ws.OP_CLOSE, (1000).to_bytes(2, "big")))
            client.kill()

    def _ws_command(self, payload: bytes, send_json, sess) -> None:
        sess.last_active = self.app.clock()                           # a command is activity: the idle clock restarts
        try:
            cmd = json.loads(payload)
            op = cmd["op"]
        except (ValueError, KeyError, TypeError):
            return send_json({"type": "error", "error": "expected a JSON object with 'op'"})
        if op == "estop":                                             # inline: no thread, no lock
            if self.app.estop_limiter is not None and not self.app.estop_limiter.allow(self._ip())[0]:
                return send_json({"type": "error", "error": "too many stop requests, slow down"})
            self.app.estop(sess)
            send_json({"type": "ack", "op": "estop"})
        elif op == "release":
            send_json({"type": "ack", "op": "release", "ok": self.app.release_estop(sess)})
        elif op == "say":
            text = cmd.get("text")
            if not isinstance(text, str) or not text.strip() or len(text) > self.app.max_text:
                return send_json({"type": "error", "error": "bad 'text'"})
            if self.app.say_limiter is not None and not self.app.say_limiter.allow(self._ip())[0]:
                return send_json({"type": "error", "error": "too many messages, slow down"})
            # off this reading thread: a slow LLM turn must not delay an estop arriving on the same socket.
            threading.Thread(target=self.app.say, args=(sess, text.strip()), daemon=True).start()
            send_json({"type": "ack", "op": "say"})
        else:
            send_json({"type": "error", "error": f"unknown op {op!r}"})
