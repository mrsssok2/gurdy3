"""Final Gazebo eval (5.11): run ONE mission of brain/eval/final_suite.py against the live robot through the real agent (typed input).
Runs inside the sim container after the stack is up (scripts/p5_final_eval.sh does that, one fresh stack per mission).
  python3 -m brain.ros.final_eval_runner --mission nav_room_b --ollama gurdy-sft-400 --places /tmp/chat/places.yaml --out RESULT.json
Exit codes: 0 = a result file was written (the mission may still have FAILED its checks), 3 = Gazebo stopped advancing (hang), 4 = other infrastructure error,
6 = Nav2 / localization not ready (readiness gate timed out, or the agent's very first reply said so within 2 s): status "setup_failed", an INFRASTRUCTURE failure
that p5_final_eval.sh retries (brain/ros/readiness.py).
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
import traceback

from ..eval.final_checks import evaluate
from ..eval.suites import suite_module
from .readiness import SetupFailed, first_reply_setup_failure, wait_ready


class SimHang(Exception):
    pass


class MissionTimeout(Exception):
    pass


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mission", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--places"); ap.add_argument("--map-id", default="flat_lab"); ap.add_argument("--odom-topic", default="/odom")
    ap.add_argument("--ollama", default="gurdy-sft-400"); ap.add_argument("--memory-db", default=None)
    ap.add_argument("--seed", type=int, default=None, help="held-out runs: seeds the start offset (random 0-8 sim s wait after readiness, before the first step); the LLM runs at temperature 0")
    ap.add_argument("--ready-s", type=float, default=90.0, help="wall seconds to wait for Nav2 / costmaps / localization / a real plan before the first command")
    ap.add_argument("--hang-s", type=float, default=90.0, help="wall seconds without /clock progress = Gazebo hang")
    a = ap.parse_args()
    m = suite_module().BY_ID[a.mission]
    rec: dict = {"mission": a.mission, "name": m["name"], "category": m["category"], "model": a.ollama, "status": "error", "messages": [], "turns": [],
                 "stops": [], "estops": [], "faults": [], "poses": [], "path_m": 0.0, "max_tilt": 0.0, "falls": 0, "semantic": [], "limit_s": m["limit_s"], "seed": a.seed}
    code = 4
    try:
        code = run(a, m, rec)
    except SimHang as e:
        rec["status"], rec["error"], code = "sim_hang", str(e), 3
    except SetupFailed as e:
        rec["status"], rec["error"], code = "setup_failed", str(e), 6
    except Exception as e:                                         # noqa: BLE001  a crash is data, not silence
        rec["status"], rec["error"], rec["traceback"], code = "error", repr(e), traceback.format_exc(), 4
        traceback.print_exc()
    finally:
        try:
            rec["verdict"] = evaluate(rec) if rec["poses"] or rec["status"] != "error" else None
        except Exception as e:                                     # noqa: BLE001
            rec["verdict_error"] = repr(e)
        json.dump(rec, open(a.out, "w"), indent=1, default=str)
    print(f"[final_eval] {a.mission}: status={rec['status']} success={(rec.get('verdict') or {}).get('success')}", flush=True)
    sys.stdout.flush()
    import os
    os._exit(code)           # rclpy's spin thread otherwise dies noisily at interpreter exit


def ros2_param(node: str, name: str, value: str) -> bool:
    try:
        r = subprocess.run(["ros2", "param", "set", node, name, value], capture_output=True, text=True, timeout=30)
        return r.returncode == 0 and "Set parameter successful" in r.stdout
    except Exception:                                              # noqa: BLE001
        return False


def run(a, m: dict, rec: dict) -> int:
    from ..agent.graph import Agent
    from ..agent.ollama_llm import OllamaLLM
    from ..tools import ToolLayer
    from .backend import RosBackend
    from .rclpy_port import RclpyPort
    port = RclpyPort(map_id=a.map_id, odom_topic=a.odom_topic)
    t_wait = time.time()
    while port.pose() is None:
        if time.time() - t_wait > 120:
            rec["error"] = "no TF map->base_link within 120 s"
            return 4
        time.sleep(1.0)
    be = RosBackend(port, a.places)
    ready = wait_ready(port, be, timeout_s=a.ready_s, log=lambda line: print(line, flush=True))      # BEFORE the first command (see readiness.py)
    rec["ready"] = ready
    print(f"[ready] ready={ready['ready']} after {ready['wall_s']} s wall, {ready['polls']} polls", flush=True)
    if not ready["ready"]:
        raise SetupFailed(f"Nav2 not ready after {ready['wall_s']} s wall; not ready: " + ", ".join(k for k, v in ready["checks"].items() if not v))
    if a.memory_db:
        from ..memory import MemoryBackend, MemoryStore
        be = MemoryBackend(be, MemoryStore(a.memory_db, a.map_id))
    agent = Agent(ToolLayer(be), OllamaLLM(model=a.ollama, constrain=not a.ollama.startswith("gurdy-sft")))

    t0, w0 = port.now(), time.time()
    state = {"last_pose": None, "tilt_since": None, "clock": (t0, time.time()), "battery": None}

    def sample() -> None:
        """Called every ~0.25 s wall by every wait loop: ground-truth pose / tilt bookkeeping + hang detection."""
        now = port.now()
        if now > state["clock"][0] + 1e-6:
            state["clock"] = (now, time.time())
        elif time.time() - state["clock"][1] > a.hang_s:
            raise SimHang(f"/clock stuck at {now:.1f} s for {a.hang_s:.0f} s wall")
        p = port.pose()
        if p is not None:
            lp = state["last_pose"]
            if lp is not None:
                d = math.hypot(p[0] - lp[0], p[1] - lp[1])
                if d > 0.01:                                       # ignore pose jitter
                    rec["path_m"] += d; state["last_pose"] = p
            else:
                state["last_pose"] = p
            if not rec["poses"] or now - rec["poses"][-1][0] >= 0.5:
                rec["poses"].append([round(now - t0, 2), round(p[0], 3), round(p[1], 3), round(p[2], 4)])
        tilt = port.tilt_deg()
        if tilt is not None:
            rec["max_tilt"] = max(rec["max_tilt"], round(tilt, 1))
            if tilt >= 60.0:
                state["tilt_since"] = state["tilt_since"] or time.time()
                if time.time() - state["tilt_since"] > 0.5 and not state.get("in_fall"):
                    rec["falls"] += 1; state["in_fall"] = True
            elif tilt < 30.0:
                state["tilt_since"], state["in_fall"] = None, False
        state["battery"] = port.battery_pct()
        if rec["estops"] and rec["estops"][-1].get("pose_off") is None and elapsed() - rec["estops"][-1]["t_on"] > 2.0:   # peak speed while latched (after the 2 s braking window)
            rec["estops"][-1]["max_speed"] = max(rec["estops"][-1]["max_speed"], port.speed() or 0.0)

    def say_msg(text: str, kind: str, who: str = "robot") -> None:
        p = port.pose()
        rec["messages"].append({"t": round(port.now() - t0, 1), "wall": round(time.time() - w0, 1), "who": who, "kind": kind, "text": text,
                                "pose": [round(p[0], 2), round(p[1], 2)] if p else None, "battery": state["battery"]})
        print(f"   [{rec['messages'][-1]['t']:6.1f}s] {who}: {text}", flush=True)

    def elapsed() -> float:
        return port.now() - t0

    def advance(sim_s: float, until_idle: bool = False) -> bool:
        """Pump the agent for sim_s sim seconds (or until idle). Returns False when the step ran out of time with work still active."""
        s0 = port.now()
        while port.now() - s0 < sim_s:
            time.sleep(0.25)
            sample()
            for msg in agent.pump():
                say_msg(msg, "pump")
            if elapsed() > m["limit_s"]:
                raise MissionTimeout()
            if until_idle and be.task is None and not agent.executor.active and port.now() - s0 > 1.5:
                return True
        return not until_idle

    def turn(text: str) -> None:
        sample()
        say_msg(text, "user", who="user")
        w = time.time()
        r = agent.say(text)
        rec["turns"].append({"say": text, "reply": r.reply, "calls": [(c.get("tool"), c.get("args")) for c in r.calls], "verdicts": r.verdicts,
                             "llm_calls": r.llm_calls, "grounding_final": len((r.grounding or {}).get("violations", [])),
                             "grounding_rejected": len((r.grounding or {}).get("rejected", [])), "latency_s": round(time.time() - w, 2),
                             "sim_t": round(elapsed(), 1)})
        sample()
        say_msg(r.reply, "turn")
        if len(rec["turns"]) == 1:                                 # second line of defence: the very first reply says Nav2 / the map was not ready
            bad = first_reply_setup_failure(rec["messages"])
            if bad:
                raise SetupFailed(f"the first reply, within 2 s, said: {bad!r}")

    def stop_turn(text: str) -> None:
        sp0 = port.speed() or 0.0
        sample()
        say_msg(text, "user", who="user")
        w = time.time()
        r = agent.say(text)
        say_s = time.time() - w
        rec["turns"].append({"say": text, "reply": r.reply, "calls": [(c.get("tool"), c.get("args")) for c in r.calls], "verdicts": r.verdicts,
                             "llm_calls": r.llm_calls, "grounding_final": len((r.grounding or {}).get("violations", [])), "latency_s": round(say_s, 2),
                             "sim_t": round(elapsed(), 1)})
        say_msg(r.reply, "turn")
        low, first = 0, None
        while time.time() - w < 12.0:                              # measured speed from odometry, 3 consecutive samples below 0.02 m/s
            sp = port.speed()
            if sp is not None and sp < 0.02:
                low += 1; first = first if first is not None else time.time()
                if low >= 3:
                    break
            else:
                low, first = 0, None
            time.sleep(0.03)
        lat = (first - w) if low >= 3 else None
        rec["stops"].append({"text": text, "moving_before": sp0 > 0.04, "speed_before": round(sp0, 3), "say_s": round(say_s, 2),
                             "latency_s": None if lat is None else round(lat, 2), "sim_t": round(elapsed(), 1)})
        print(f"   stop '{text}': speed before {sp0:.3f} m/s, speed < 0.02 after {lat if lat is None else round(lat, 2)} s", flush=True)

    status = "ok"
    try:
        if a.seed is not None:                                     # run-to-run variation for repeated held-out runs: when, relative to the sim, the mission starts
            import random
            rec["start_offset_s"] = round(random.Random(a.seed).uniform(0.0, 8.0), 2)
            advance(rec["start_offset_s"])
        for step in m["steps"]:
            kind = step[0]
            if kind == "say":
                turn(step[1])
            elif kind == "stop":
                stop_turn(step[1])
            elif kind == "wait":
                advance(step[1])
            elif kind == "idle":
                if not advance(step[1], until_idle=True):
                    status = "timeout"; rec["timeout_step"] = list(step)
                    break
            elif kind == "estop":
                rec["estops"].append({"pose_on": list(port.pose()[:2]), "t_on": round(elapsed(), 1), "max_speed": 0.0})
                be.estop(True); say_msg("[operator] !estop", "user", who="user")
                s0 = port.now()
                while port.now() - s0 < 0.1:
                    time.sleep(0.05)
            elif kind == "release":
                es = rec["estops"][-1]
                es["pose_off"] = list(port.pose()[:2]); be.estop(False); say_msg("[operator] !release", "user", who="user")
            elif kind == "battery":
                ok = ros2_param("/battery_sim_node", "level", str(float(step[1])))
                rec["faults"].append({"t": round(elapsed(), 1), "battery": step[1], "ok": ok}); print(f"   fault: battery -> {step[1]} ok={ok}", flush=True)
            elif kind == "leg":
                ok = ros2_param("/leg_health_node", "fake_failed_leg", step[1] if step[1] else '""')
                rec["faults"].append({"t": round(elapsed(), 1), "fake_failed_leg": step[1], "ok": ok}); print(f"   fault: leg {step[1]!r} ok={ok}", flush=True)
    except MissionTimeout:
        status = "timeout"
    rec["status"] = status
    try:
        if be.task is not None or agent.executor.active:
            be.stop()
        s0 = port.now()
        while port.now() - s0 < 2.0:
            time.sleep(0.25); sample()
    except MissionTimeout:
        pass
    rec["sim_s"], rec["wall_s"] = round(elapsed(), 1), round(time.time() - w0, 1)
    rec["semantic"] = port.semantic_objects() or []
    rec["events"] = [{"type": e.get("type"), "text": e.get("text")} for e in list(getattr(be, "events", []))[-40:]]
    rec["falls"] += sum(1 for e in getattr(be, "events", []) if e.get("type") == "fall" and not rec["falls"])
    rec["pose_end"] = list(port.pose() or [])
    rec["errors"] = list(agent.errors)
    return 0


if __name__ == "__main__":
    main()
