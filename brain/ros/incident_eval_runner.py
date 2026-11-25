"""Incident-graph evaluation (Phase 6): ONE session in the live lab4 stack (brain/eval/incident_suite.py), through the real agent with the temporal graph.
Runs inside the sim container after the stack is up (scripts/incident_eval.sh does that).
  python3 -m brain.ros.incident_eval_runner --out RESULT.json --ollama gurdy-sft-400 --places maps/lab4.places.yaml --map-id lab4

Steps (SCENARIO): `round` = the robot looks around every room (backend `explore` with an area, operator-level, one room after the other),
`say` / `stop` = typed to the agent, `world` = Gazebo set_pose / remove of the ground-truth objects (read back to verify), `wait`, `mark`.
Then every question is typed. The result holds the ground-truth log (`gt`), the typed turns, the graph's events and the pose track.
Exit codes: 0 result written, 3 Gazebo stopped advancing, 4 other error, 6 Nav2 not ready."""
from __future__ import annotations

import argparse
import json
import math
import re
import subprocess
import sys
import time
import traceback
from pathlib import Path

import importlib
import os

from ..eval import incident_suite as S
S = importlib.import_module("brain.eval." + os.environ.get("INCIDENT_SUITE", "incident_suite"))
from .readiness import SetupFailed, wait_ready

ROOT = Path(__file__).resolve().parents[2]
SDF = ROOT / "ros2_ws/src/gurdy_gazebo/worlds/lab4.sdf"


class SimHang(Exception):
    pass


# ---------------------------------------------------------------- Gazebo world operations (ground truth)
def _gz(args: list[str], timeout: float = 15.0) -> tuple[bool, str]:
    try:
        r = subprocess.run(["gz", *args], capture_output=True, text=True, timeout=timeout)
        return r.returncode == 0, (r.stdout + r.stderr)
    except Exception as e:                                          # noqa: BLE001
        return False, repr(e)


def model_sdf_pose(model: str) -> tuple[float, float, float, float]:
    """x y z yaw of a model in lab4.sdf (`<model name="l4_x"><static>true</static><pose>x y z r p yaw</pose>`)."""
    m = re.search(rf'<model name="{re.escape(model)}">.*?<pose>([^<]+)</pose>', SDF.read_text(), re.S)
    if not m:
        raise KeyError(f"{model} not in {SDF}")
    v = [float(t) for t in m.group(1).split()]
    return v[0], v[1], v[2], v[5]


def gz_pose_of(world: str, model: str) -> tuple[float, float, float] | None:
    """Current pose of a model from the world's pose stream (None when the model does not exist)."""
    ok, out = _gz(["topic", "-e", "-n", "1", "-t", f"/world/{world}/pose/info"], timeout=20)
    if not ok:
        return None
    m = re.search(rf'name: "{re.escape(model)}"\s*id: \d+\s*position \{{\s*x: ([-\d.e]+)\s*y: ([-\d.e]+)\s*(?:z: ([-\d.e]+))?', out)
    return (float(m.group(1)), float(m.group(2)), float(m.group(3) or 0.0)) if m else None


def apply_op(world: str, op: dict) -> dict:
    """One ground-truth change; returns the log entry (wall time, whether Gazebo accepted it, whether reading it back confirms it)."""
    rec = {k: op[k] for k in ("op", "model", "cls", "color", "from", "to")}
    rec["wall"] = time.time()
    if op["op"] == "move":
        _, _, z, yaw = model_sdf_pose(op["model"])
        x, y = op["to"][1], op["to"][2]
        req = f'name: "{op["model"]}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}'
        ok, out = _gz(["service", "-s", f"/world/{world}/set_pose", "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", req])
        rec["accepted"], rec["gz"] = ok and "true" in out.lower(), out.strip()[-200:]
        time.sleep(2.0)
        p = gz_pose_of(world, op["model"])
        rec["verified"] = bool(p and math.hypot(p[0] - x, p[1] - y) < 0.05)
        rec["readback"] = p
    else:
        ok, out = _gz(["service", "-s", f"/world/{world}/remove", "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean", "--timeout", "5000",
                       "--req", f'name: "{op["model"]}", type: MODEL'])
        rec["accepted"], rec["gz"] = ok and "true" in out.lower(), out.strip()[-200:]
        time.sleep(2.0)
        rec["verified"] = gz_pose_of(world, op["model"]) is None
    return rec


# ---------------------------------------------------------------- main
def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--places"); ap.add_argument("--map-id", default=S.MAP_ID); ap.add_argument("--odom-topic", default="/odom")
    ap.add_argument("--ollama", default="gurdy-sft-400"); ap.add_argument("--db", default=None, help="SQLite file of the incident graph (default: in RAM)")
    ap.add_argument("--ready-s", type=float, default=90.0); ap.add_argument("--hang-s", type=float, default=90.0)
    a = ap.parse_args()
    rec: dict = {"scenario": S.NAME, "model": a.ollama, "status": "error", "turns": [], "messages": [], "poses": [], "falls": 0, "max_tilt": 0.0,
                 "gt": {"ops": [], "rounds": {}, "stop": None, "left": None}, "graph_events": [], "started": time.strftime("%FT%TZ", time.gmtime())}
    try:
        rec["commit"] = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, cwd=ROOT).stdout.strip()
    except Exception:                                               # noqa: BLE001
        pass
    code = 4
    try:
        code = run(a, rec)
    except SimHang as e:
        rec["status"], rec["error"], code = "sim_hang", str(e), 3
    except SetupFailed as e:
        rec["status"], rec["error"], code = "setup_failed", str(e), 6
    except Exception as e:                                          # noqa: BLE001  a crash is data, not silence
        rec["status"], rec["error"], rec["traceback"] = "error", repr(e), traceback.format_exc()
        traceback.print_exc()
    finally:
        json.dump(rec, open(a.out, "w"), indent=1, default=str)
    print(f"[incident_eval] status={rec['status']} turns={len(rec['turns'])}", flush=True)
    import os
    os._exit(code)


def run(a, rec: dict) -> int:
    from ..agent.graph import Agent
    from ..agent.ollama_llm import OllamaLLM
    from ..incidents.backend import with_incidents
    from ..tools import ToolLayer
    from .backend import RosBackend
    from .rclpy_port import RclpyPort
    port = RclpyPort(map_id=a.map_id, odom_topic=a.odom_topic)
    t0w = time.time()
    while port.pose() is None:
        if time.time() - t0w > 120:
            rec["error"] = "no TF map->base_link within 120 s"
            return 4
        time.sleep(1.0)
    ros = RosBackend(port, a.places)
    ready = wait_ready(port, ros, timeout_s=a.ready_s, log=lambda line: print(line, flush=True))
    rec["ready"] = ready
    if not ready["ready"]:
        raise SetupFailed(f"Nav2 not ready after {ready['wall_s']} s wall; not ready: " + ", ".join(k for k, v in ready["checks"].items() if not v))
    be = with_incidents(ros, a.db, a.map_id)                           # real wall clock: Gazebo runs at about real time
    agent = Agent(ToolLayer(be), OllamaLLM(model=a.ollama, constrain=not a.ollama.startswith("gurdy-sft")))
    t0 = port.now()
    clock = {"sim": t0, "wall": time.time()}

    def sample() -> None:
        now = port.now()
        if now > clock["sim"] + 1e-6:
            clock["sim"], clock["wall"] = now, time.time()
        elif time.time() - clock["wall"] > a.hang_s:
            raise SimHang(f"/clock stuck at {now:.1f} s for {a.hang_s:.0f} s wall")
        p = port.pose()
        if p is not None and (not rec["poses"] or now - rec["poses"][-1][0] >= 1.0):
            rec["poses"].append([round(now - t0, 1), round(p[0], 2), round(p[1], 2), round(p[2], 3)])
        tilt = port.tilt_deg()
        if tilt is not None:
            rec["max_tilt"] = max(rec["max_tilt"], round(tilt, 1))
            if tilt >= 60.0 and not clock.get("in_fall"):
                rec["falls"] += 1; clock["in_fall"] = True
            elif tilt < 30.0:
                clock["in_fall"] = False

    def pump_for(sim_s: float, until_idle: bool = False) -> bool:
        s0 = port.now()
        while port.now() - s0 < sim_s:
            time.sleep(0.25)
            sample()
            for msg in agent.pump():
                rec["messages"].append({"t": round(port.now() - t0, 1), "text": msg})
            if until_idle and be.task is None and not agent.executor.active and port.now() - s0 > 1.5:
                return True
        return not until_idle

    def type_text(text: str, qid: str | None = None) -> dict:
        sample()
        w = time.time()
        r = agent.say(text)
        turn = {"say": text, "qid": qid, "reply": r.reply, "calls": [(c.get("tool"), c.get("args")) for c in r.calls], "verdicts": r.verdicts, "llm_calls": r.llm_calls,
                "grounding_final": len((r.grounding or {}).get("violations", [])), "latency_s": round(time.time() - w, 2), "wall": w, "sim_t": round(port.now() - t0, 1)}
        rec["turns"].append(turn)
        print(f"   [{turn['sim_t']:7.1f}s] user: {text}\n             robot: {r.reply}", flush=True)
        return turn

    for step in S.SCENARIO:
        kind = step[0]
        if kind == "round":
            rnd, rooms = step[1], step[2]
            rec["gt"]["rounds"][str(rnd)] = []
            for room in rooms:
                start = time.time()
                res = be.explore(max_time_s=S.SWEEP_LIMIT_S, area=room)
                if not res.ok:
                    print(f"   round {rnd} {room}: explore refused: {res.error}", flush=True)
                else:
                    pump_for(S.SWEEP_LIMIT_S + 30, until_idle=True)
                    if be.task is not None:
                        be.stop()
                rec["gt"]["rounds"][str(rnd)].append({"room": room, "start": start, "end": time.time(), "ok": bool(res.ok), "pose_end": list(port.pose() or [])})
        elif kind == "say":
            type_text(step[1])
        elif kind == "stop":
            t = type_text(step[1])
            rec["gt"]["stop"] = {"wall": t["wall"], "text": step[1]}
            pump_for(3.0)
        elif kind == "wait":
            pump_for(step[1])
        elif kind == "mark":
            rec["gt"][step[1]] = time.time()
        elif kind == "world":
            for op in S.OPS:
                rec["gt"]["ops"].append(apply_op(S.WORLD, op))
                print(f"   world: {op['op']} {op['model']} verified={rec['gt']['ops'][-1]['verified']}", flush=True)
            pump_for(2.0)
    for q in S.ALL_QUESTIONS:                                           # the questions, one after the other, nothing else typed in between
        t = type_text(q["text"], q["id"])
        t["started_motion"] = bool(be.task is not None or agent.executor.active)
        if t["started_motion"]:                                         # a planner-handled question must not wander off and spoil the next one
            be.stop(); agent.executor.cancel(); t["motion"] = "stopped by the evaluation"
        agent.gate.clear()
        pump_for(1.0)
    rec["gt"]["t_end"] = time.time()
    rec["graph_events"] = [{"etype": e["etype"], "label": e["label"], "cls": e["cls"], "color": e["color"] or "", "room": e["room"], "wall_t": e["wall_t"],
                            "sim_t": e["sim_t"], "data": e["data"]} for e in be.store.events()]
    rec["graph_counts"] = be.store.count()
    rec["errors"] = list(agent.errors)
    rec["status"] = "ok"
    return 0


if __name__ == "__main__":
    main()
