"""Gazebo VLN eval: run ONE episode of a VLN episode file against the live robot through the real agent (typed input, VLN on).
Runs inside the sim container after the stack is up; scripts/vln_eval.sh starts one fresh stack per episode.
  python3 -m brain.ros.vln_eval_runner --suite brain/eval/vln_episodes_dev.yaml --episode dev_unknown_sofa --out RESULT.json \\
      --places /tmp/chat/places.yaml --map-id lab4_vln [--ollama gurdy-sft-400] [--landmarks gt|map]
Record (scored on the host by brain/vln/gazebo_eval.py): status ok | timeout | sim_hang | setup_failed | error, the reply, the
plan (source, error, rooms, waypoints), the walk result (code, claims, rooms visited), the GROUND-TRUTH trajectory
(/odom/ground_truth = Gazebo's OdometryPublisher, sampled on every backend tick) next to the TF pose (map -> base_link).
Landmarks: 'gt' (default) = positions from the world's GT objects (sigma 0.10 m), confirmation by the live detector
(gazebo_eval.seed_landmarks): isolates VLN from detector recall; 'map' = the live semantic map only (needs exploring first).
Exit codes like final_eval_runner: 0 record written, 3 Gazebo hang, 4 other error, 6 Nav2 not ready (infrastructure, retried).
"""
from __future__ import annotations

import argparse
import json
import math
import threading
import time
import traceback

from ..vln import gazebo_eval as G


class EpisodeRunner:
    def __init__(self, port, backend, gt_pose, gt_objects=(), landmarks: str = "gt", planner=None, watchdog_s: float = 1.0,
                 hang_s: float = 90.0, sample_dt: float = 0.25):
        self.p, self.b, self.gt_pose, self.gt_objects = port, backend, gt_pose, list(gt_objects)
        self.landmarks, self.planner, self.watchdog_s, self.hang_s, self.sample_dt = landmarks, planner, watchdog_s, hang_s, sample_dt

    # ---- sampling (called on every backend tick, so the trajectory follows the executor's own clock)
    def _sample(self):
        now = self.p.now()
        if self.rec["gt_traj"] and now - self.rec["gt_traj"][-1][0] - self.t0 < self.sample_dt:
            pass
        else:
            g, tf = self.gt_pose(), self.p.pose()
            t = round(now - self.t0, 2)
            if g is not None:
                self.rec["gt_traj"].append([t, round(g[0], 3), round(g[1], 3)])
            if tf is not None:
                self.rec["tf_traj"].append([t, round(tf[0], 3), round(tf[1], 3)])
            if g is not None and tf is not None:
                self.rec["gt_tf_max_m"] = max(self.rec["gt_tf_max_m"], round(math.hypot(g[0] - tf[0], g[1] - tf[1]), 3))
            tilt = self.p.tilt_deg() if callable(getattr(self.p, "tilt_deg", None)) else None
            if tilt is not None:
                self.rec["max_tilt"] = max(self.rec["max_tilt"], round(tilt, 1))
        if now - self.t0 > self.limit_s and not self.timed_out:
            self.timed_out = True
            self._stop("sim time limit")

    def _stop(self, why):
        self.rec["stopped_by_runner"] = why
        try:
            self.b.stop()
        except Exception:                                          # noqa: BLE001
            pass

    def _watchdog(self):
        last, since = self.p.now(), time.time()
        while not self.done.wait(self.watchdog_s):
            now = self.p.now()
            if now > last + 1e-6:
                last, since = now, time.time()
            elif time.time() - since > self.hang_s and not self.hung:
                self.hung = True
                self._stop(f"/clock stuck at {now:.1f} s for {self.hang_s:.0f} s wall")
            if now - self.t0 > self.limit_s and not self.timed_out:
                self.timed_out = True
                self._stop("sim time limit")

    def _wait_sim(self, s: float, wall_cap: float = 30.0):
        t, w = self.p.now(), time.time()
        while self.p.now() - t < s and time.time() - w < wall_cap:
            adv = getattr(self.p, "advance", None)
            adv(0.25) if callable(adv) else time.sleep(0.25)
            self.b.tick()

    # ---- one episode
    def run(self, ep: dict, limit_s: float | None = None) -> dict:
        from ..agent.graph import Agent
        from ..agent.llm import KeywordLLM
        from ..tools import ToolLayer
        from ..vln.planner import VLNPlanner
        self.limit_s = float(limit_s if limit_s is not None else ep.get("limit_s", 900))
        self.timed_out = self.hung = False
        self.t0 = self.p.now()
        self.rec = rec = {"episode": ep["id"], "text": ep["text"], "status": "error", "landmarks": self.landmarks, "limit_s": self.limit_s,
                          "gt_traj": [], "tf_traj": [], "gt_tf_max_m": 0.0, "max_tilt": 0.0, "reply": None, "plan": None, "result": None}
        if self.landmarks == "gt":
            live = self.p.semantic_objects
            self.p.semantic_objects = lambda: G.seed_landmarks(self.gt_objects, live() or [])
        tick = self.b.tick

        def tick_and_sample():
            r = tick()
            self._sample()
            return r
        self.b.tick = tick_and_sample
        planner = self.planner or VLNPlanner(False)
        agent = Agent(ToolLayer(self.b), KeywordLLM(self.b.known_places), vln=planner)
        self.done = threading.Event()
        wd = threading.Thread(target=self._watchdog, daemon=True) if self.watchdog_s > 0 else None
        w0 = time.time()
        try:
            if wd:
                wd.start()
            self._sample()
            r = agent.say(ep["text"])
            rec["reply"] = r.reply
            self._wait_sim(2.0)                                     # the robot settles; the trajectory gets its last poses
            self._sample()
            p = planner.last_plan
            rec["plan"] = None if p is None else {k: getattr(p, k) for k in ("ok", "error", "reply", "source", "rooms", "avoid", "summary",
                                                                               "waypoints", "goal", "length_m", "notes")}
            v = agent.vln_last
            rec["result"] = None if v is None else {"ok": v.ok, "code": v.code, "reply": v.reply, "claims": v.claims,
                                                    "rooms_visited": v.rooms_visited, "legs": v.legs, "replans": v.replans,
                                                    "alerts": v.alerts, "backend_traj": v.trajectory[::4]}
            rec["routed"] = "vln" if (p is not None) else "planner"
            rec["calls"] = [(c.get("tool"), c.get("args")) for c in (r.calls or [])]
            rec["errors"] = list(agent.errors)
            rec["status"] = "sim_hang" if self.hung else ("timeout" if self.timed_out else "ok")
        except Exception as e:                                      # noqa: BLE001  a crash is data
            rec["status"], rec["error"], rec["traceback"] = "error", repr(e), traceback.format_exc()
        finally:
            self.done.set()
            self.b.tick = tick
            rec["sim_s"], rec["wall_s"] = round(self.p.now() - self.t0, 1), round(time.time() - w0, 1)
            rec["zones_left"] = [z.get("name") for z in getattr(self.b, "zones", []) if z.get("temporary")]
        return rec


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--suite", required=True); ap.add_argument("--episode", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--places", required=True); ap.add_argument("--map-id", default="lab4_vln")
    ap.add_argument("--odom-topic", default="/odom"); ap.add_argument("--gt-topic", default="/odom/ground_truth")
    ap.add_argument("--ollama", default=None, help="VLN plan model (+ keyword fallback); default: the keyword parser alone")
    ap.add_argument("--landmarks", choices=("gt", "map"), default="gt")
    ap.add_argument("--ready-s", type=float, default=90.0); ap.add_argument("--hang-s", type=float, default=90.0)
    a = ap.parse_args()
    suite = G.load_suite(a.suite)
    ep = next(e for e in suite["episodes"] if e["id"] == a.episode)
    rec = {"episode": a.episode, "status": "error"}
    code = 4
    try:
        from .backend import RosBackend
        from .rclpy_port import RclpyPort
        from .readiness import wait_ready
        port = RclpyPort(map_id=a.map_id, odom_topic=a.odom_topic, gt_topic=a.gt_topic)
        w = time.time()
        while port.pose() is None or port.gt_pose() is None:
            if time.time() - w > 120:
                raise RuntimeError(f"no TF map->base_link / no {a.gt_topic} within 120 s")
            time.sleep(1.0)
        be = RosBackend(port, a.places)
        ready = wait_ready(port, be, timeout_s=a.ready_s, log=lambda line: print(line, flush=True))
        if not ready["ready"]:
            rec.update(status="setup_failed", error="Nav2 not ready: " + ", ".join(k for k, v in ready["checks"].items() if not v))
            code = 6
        else:
            from ..vln.planner import VLNPlanner
            planner = VLNPlanner(model=a.ollama) if a.ollama else VLNPlanner(False)
            rec = EpisodeRunner(port, be, port.gt_pose, G.gt_objects(suite), a.landmarks, planner, hang_s=a.hang_s).run(ep)
            rec["ready"] = ready
            code = {"sim_hang": 3, "error": 4}.get(rec["status"], 0)
    except Exception as e:                                          # noqa: BLE001
        rec.update(status="error", error=repr(e), traceback=traceback.format_exc())
        traceback.print_exc()
    finally:
        json.dump(rec, open(a.out, "w"), indent=1, default=str)
    print(f"[vln_eval] {a.episode}: status={rec['status']} reply={rec.get('reply')!r}", flush=True)
    import os
    os._exit(code)


if __name__ == "__main__":
    main()
