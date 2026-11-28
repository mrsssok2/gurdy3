"""RosFetchRobot: the fetch backend for the REAL robot / Gazebo, the same primitive interface as brain.fetch.mock.FetchMock,
so brain.fetch.session.FetchSession + FetchMachine run unchanged on top of it (Phase 9h; behind run_text --fetch,
default off). NOT run in Gazebo yet.

    robot = RosFetchRobot(ros_backend, arm_port)            # RosBackend (body: Nav2, pose, semantic map, search) + ArmPort
    session = FetchSession(robot); ros_backend.fetch_session = session

Primitives (each returns what FetchMock returns; every wait loop checks the e-stop and raises FetchAborted):
  find        the object is in the semantic map (object3d mode gives x / y / z / yaw); `search(ref)` runs RosBackend.find
              (room sweep + explore) for objects nobody has seen (the session calls it when it has no candidate)
  approach    grasp-zone pose: the object must end up at the class zone BEHIND the robot (rear arm). Nav2 to a stand-off
              pose (zone distance + 0.25 m) FACING the object, turn round there (turning right next to the object could
              sweep a leg into it), then back up the 0.25 m on odometry (closed loop /cmd_vel). The candidate side is the one
              with free map cells around the footprint and the shortest planned path.
  fine_align  arm to the view pose, wrist-camera visual servo (gurdy_core.visual_servo, body steps = closed loop /cmd_vel)
  grasp       wrist-view yaw (gurdy_core.wrist_yaw; below 0.5 confidence roll 0), grasp plan (gurdy_core.grasp_test), gripper
              open, pre-grasp (MoveIt if available, else a joint goal), straight approach (joint goal), squeeze (contact)
  verify      lift 5 cm; fingers neither fully closed nor fully open; then arm to the view pose and the wrist view must show
              nothing STANDING at the old spot (gurdy_core.wrist_yaw.standing_points: 0.5-2.5 cm band above the fitted floor;
              a held object is lifted / occludes it, a missed one is still there). Failure -> open, back off (the machine
              retries the grasp)
  fold        arm `folded` (payload pose)
  carry_to    Nav2 with the controller speed limit at the carry speed (<= 0.08 m/s); fingers fully closed on the way =
              dropped (returns the drop point); the measured speed is logged (`speeds`, like the mock)
  release     lower to the class place pose behind the robot, open, back off, fold
  hand_over   turn so the arm points at the user, present the object (IK, ~15 cm up), keep a light grip and wait until the
              fingers close (the object was taken) or the timeout
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ..fetch.catalog import CARRY_SPEED_MPS, PICKABLE as CAT
from ..fetch.machine import FetchAborted
from ..fetch.mock import HOME, ROOMS, Obj

BACKUP_M = 0.25
GRIP_EFFORT = 3.0                  # N per finger (joint limit 10 N; 80 g at mu 1 needs ~0.4 N)
HOLD_EFFORT = 1.0                  # N while presenting (the user can pull it out)
def align_log_path(attempt_log: str) -> str:
    """attempts.jsonl -> attempts_align.jsonl (one record per wrist-camera alignment)."""
    p = Path(attempt_log)
    return str(p.with_name(p.stem + "_align" + (p.suffix or ".jsonl")))


FINGER_CLOSED = 0.004              # m per finger: below this nothing is between the fingers


@dataclass
class FetchConfig:
    nav_timeout_s: float = 240.0
    carry_timeout_s: float = 600.0
    # search time limit: None = sized from the map (RosBackend.search_budget_s: the room sweep from the current pose, incl. the
    # extra viewpoints of search_view_radius_m), clamped to [search_min_s, search_max_s]; a number = that fixed limit (was 300 s)
    search_timeout_s: float | None = None
    search_min_s: float = 300.0
    search_max_s: float = 2400.0
    search_view_radius_m: float | None = None       # multi-viewpoint room sweep for unseen objects (None = one viewpoint per room)
    handover_timeout_s: float = 20.0
    poll_s: float = 0.2
    yaw_min_conf: float = 0.5
    use_moveit: bool = True        # if available
    # retest diagnostics (2026-11-27 fetch dry run), all OFF by default:
    attempt_log: str | None = None  # JSONL file: one record per grasp attempt (also kept in RosFetchRobot.attempts)
    gt: str = "off"                 # off | log (record Gazebo ground truth next to the estimates) | use (grasp at the GT pose)
    approach: str = "nav"           # nav (Nav2 stand-off + turn + back-up) | teleport (gz set_pose to the grasp-zone pose, as grasp_test)
    # "bring me X" demo delivery (phase 9k, default OFF = hand-over waits for the user to take it, as before):
    # carry back to the REQUESTER (saved place 'me' / 'user' / 'requester', else the robot pose when the command came, else
    # home), present at the hand-over pose, open the gripper only when the body is stationary, and claim delivery only when
    # the state (fingers held it until the release, open after) and, with ground truth, the object's GT pose confirm it
    view_tries: int = 3               # wrist frames: at least this many requests (each <= 3 s wall in RclpyArmPort) ...
    view_window_s: float = 8.0        # ... and keep asking until this much SIM time passed (slow RTF: wall != sim)
    view_wall_cap_s: float = 60.0     # never wait longer than this wall time (a paused sim must not hang the fetch)
    bring_me: bool = False
    still_mps: float = 0.01         # stationary: |v| below this ...
    still_s: float = 1.0            # ... for this long (sim s) before the gripper opens
    still_timeout_s: float = 15.0
    gt_held_m: float = 0.10         # GT: a held object is within this of the grasp centre (xy)
    gt_release_m: float = 0.15      # GT: after the release it lies within this of the release point (xy) and on the floor


def task_engine(backend):
    """The innermost backend (follows the wrappers' `_inner`): the one whose tick() runs the task state machine."""
    seen = set()
    while getattr(backend, "_inner", None) is not None and id(backend) not in seen:
        seen.add(id(backend))
        backend = backend._inner
    return backend


class RosFetchRobot:
    def __init__(self, backend, arm, cfg: FetchConfig | None = None):
        from gurdy_core import arm as A
        self.A = A
        self.b, self.p, self.arm, self.cfg = backend, backend.p, arm, cfg or FetchConfig()
        self.ARM, self.RB = A.load_arm(), A.load_robot()
        self.carrying: str | None = None
        self.user = tuple(self._place_xy("home") or HOME)
        self.speeds: list = []
        self.log: list = []
        self.known: set = set()
        self._objects: dict = {}
        self._with_user: dict = {}
        self._moved: dict = {}        # name -> (x, y, t): placed by us; overrides the map until the detector sees it again
        self._servo = None
        self._plan = None
        self._rest = None             # (x, y) world where the object stood before the grasp
        self._floor = None            # MoveIt floor (fitted plane), added once before the first grasp
        self.attempts: list = []      # per grasp attempt diagnostics (cfg.attempt_log)
        self._att = None
        self.truth = None
        self.handover_code: str | None = None    # why the last hand_over() returned False (cfg.bring_me; None = not taken)
        if self.cfg.gt != "off" or self.cfg.approach == "teleport":
            from .gz_truth import GzTruth
            self.truth = GzTruth()
        elif self.cfg.bring_me:          # Gazebo: GT confirms the delivery too; no gz CLI (real robot) -> state only
            try:
                from .gz_truth import GzTruth
                self.truth = GzTruth()
            except (OSError, ImportError):
                self.truth = None

    # ------------------------------------------------------------------ state
    @property
    def engine(self):
        """The RosBackend itself (run_text wraps it: incidents, memory): the one whose tick() runs the task machine."""
        return task_engine(self.b)

    def now(self) -> float:
        return self.p.now()

    @property
    def robot(self):
        pz = self.p.pose()
        return (pz[0], pz[1]) if pz else (0.0, 0.0)

    @property
    def objects(self) -> dict:
        """Semantic-map tracks as fetch objects (name obj<id>); held / handed-over objects keep their own state."""
        for t in self.p.semantic_objects() or []:
            name = f"obj{t['id']}"
            if name in self._with_user or name == self.carrying:
                continue
            cls = t["class"]
            mass = CAT[cls][0] if cls in CAT else None
            x, y = float(t["x"]), float(t["y"])
            mv = self._moved.get(name)
            if mv and float(t.get("last_seen", -1e9)) <= mv[2]:
                x, y = mv[0], mv[1]
            o = Obj(name, cls, t.get("color") or "", x, y, mass, cls in CAT)
            o.track = t
            self._objects[name] = o
            self.known.add(name)
        for name, o in self._with_user.items():
            self._objects[name] = o
        return self._objects

    def _place_xy(self, name):
        pl = getattr(self.b, "places", {}) or {}
        v = next((v for k, v in pl.items() if k.lower() == name.lower()), None)
        return (v["x"], v["y"]) if isinstance(v, dict) and "x" in v else None

    def place_point(self, room: str):
        xy = self._place_xy(room)
        if xy:
            return xy
        a, b, c, d = ROOMS[room]
        return ((a + c) / 2 + 0.6, (b + d) / 2 + 0.6)

    # ------------------------------------------------------------------ waiting / body helpers
    def _check(self):
        if self.p.estop_active():
            self.p.send_cmd(0.0, 0.0, 0.0)
            raise FetchAborted("emergency stop")

    def _wait(self, s):
        end = self.now() + s
        while True:
            self._check()
            dt = min(self.cfg.poll_s, end - self.now())
            if dt <= 1e-6:
                return
            self.arm.wait(dt)
            # Tick the TASK ENGINE, not the wrapper: IncidentBackend.tick() only samples for its recorder and never ticks the
            # wrapped RosBackend, and the agent's pump (poll_events -> RosBackend.tick) is blocked by the agent lock while a
            # fetch runs. Without this a search stalled after its first Nav2 step (Gazebo dry run 2026-11-27, 2 of 2).
            # Events are not consumed here: the pump hands them to the wrappers / agent when the fetch returns.
            if hasattr(self.engine, "tick"):
                self.engine.tick()
            if self.engine is not self.b and hasattr(self.b, "tick"):
                self.b.tick()                    # the incident recorder still samples pose / objects during the fetch

    def _nav(self, x, y, yaw, timeout):
        h = self.p.nav_send(x, y, yaw)
        if h is None:
            return False, "Nav2 not available"
        t0 = self.now()
        while True:
            self._check()
            st = self.p.nav_state(h)
            if st["state"] == "succeeded":
                return True, None
            if st["state"] in ("aborted", "canceled"):
                return False, f"Nav2 {st['state']}"
            if self.now() - t0 > timeout:
                self.p.nav_cancel(h)
                return False, "Nav2 timeout"
            v = self.p.speed()
            if v is not None:
                self.speeds.append((round(v, 3), self.carrying is not None))
            self._wait(self.cfg.poll_s)

    def _turn_to(self, yaw, tol=0.05, timeout=30.0):
        t0 = self.now()
        while self.now() - t0 < timeout:
            x, y, cur = self.p.pose()
            e = math.atan2(math.sin(yaw - cur), math.cos(yaw - cur))
            if abs(e) < tol:
                break
            self.p.send_cmd(0.0, 0.0, math.copysign(min(0.3, max(0.1, abs(e))), e))
            self._wait(self.cfg.poll_s)
        self.p.send_cmd(0.0, 0.0, 0.0)
        x, y, cur = self.p.pose()
        return abs(math.atan2(math.sin(yaw - cur), math.cos(yaw - cur))) < 2 * tol

    def _move_body(self, dx, dy, speed=0.05, tol=0.01, timeout=15.0):
        """Closed-loop straight move by (dx, dy) in the CURRENT base frame (odometry / TF pose), yaw held."""
        x0, y0, yaw = self.p.pose()
        c, s = math.cos(yaw), math.sin(yaw)
        tx, ty = x0 + c * dx - s * dy, y0 + s * dx + c * dy
        t0 = self.now()
        while self.now() - t0 < timeout:
            x, y, cur = self.p.pose()
            ex, ey = tx - x, ty - y
            if math.hypot(ex, ey) < tol:
                break
            c, s = math.cos(cur), math.sin(cur)
            bx, by = c * ex + s * ey, -s * ex + c * ey
            n = math.hypot(bx, by)
            v = min(speed, 2.0 * n)
            self.p.send_cmd(bx / n * v, by / n * v, 0.0)
            self._wait(self.cfg.poll_s)
        self.p.send_cmd(0.0, 0.0, 0.0)
        x, y, _ = self.p.pose()
        return math.hypot(tx - x, ty - y) < 3 * tol

    def _base(self, xy):
        x, y, yaw = self.p.pose()
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = xy[0] - x, xy[1] - y
        return np.array([c * dx + s * dy, -s * dx + c * dy])

    def _world(self, bxy):
        x, y, yaw = self.p.pose()
        c, s = math.cos(yaw), math.sin(yaw)
        return (x + c * bxy[0] - s * bxy[1], y + s * bxy[0] + c * bxy[1])

    def _move_arm(self, qs, seconds, big=False):
        use = big and self.cfg.use_moveit and self.arm.moveit_available()
        ok = self.arm.move_arm(qs, seconds, use_moveit=use)
        if not ok and use:
            ok = self.arm.move_arm(qs, seconds, use_moveit=False)
        self.log.append({"t": round(self.now(), 1), "arm": "moveit" if use else "direct", "ok": ok})
        self._check()
        return ok

    # ------------------------------------------------------------------ primitives
    def search(self, ref: dict) -> bool:
        """Look for an object nobody has seen (RosBackend.find: room sweep + explore). True when the map has one now.
        cfg.search_view_radius_m (default None) switches the backend's multi-viewpoint room sweep on for this search only."""
        eng = self.engine                       # set on the RosBackend: on a wrapper it would only shadow the attribute
        prev = getattr(eng, "search_view_radius_m", None)
        if self.cfg.search_view_radius_m:
            eng.search_view_radius_m = self.cfg.search_view_radius_m
        try:
            limit = self.search_limit_s()
            if hasattr(self, "log"):                     # (tests build a bare robot with __new__)
                self.log.append({"t": round(self.now(), 1), "search_limit_s": limit})
            r = self.b.find(object=dict(ref), approach=False, max_time_s=limit)
            if not getattr(r, "ok", False):
                return False
            t0 = self.now()
            while self.engine.task is not None and self.now() - t0 < limit + 30:
                self._wait(self.cfg.poll_s)
        finally:
            eng.search_view_radius_m = prev            # also on an e-stop (FetchAborted)
        m = self.engine.missions[-1] if self.engine.missions else {}
        return bool((m.get("result") or {}).get("found"))

    def search_limit_s(self) -> float:
        """cfg.search_timeout_s if set, else the map-sized sweep budget (call with search_view_radius_m already applied)."""
        if self.cfg.search_timeout_s is not None:
            return float(self.cfg.search_timeout_s)
        f = getattr(self.engine, "search_budget_s", None)
        budget = f() if callable(f) else self.cfg.search_min_s
        return float(min(max(budget, self.cfg.search_min_s), self.cfg.search_max_s))

    def find(self, name):
        o = self.objects.get(name)
        if o is None:
            return False, "not in the semantic map"
        return True, o.room

    def _approach_candidates(self, o, zone):
        out = []
        for k in range(16):
            yaw = -math.pi + 2 * math.pi * k / 16
            c, s = math.cos(yaw), math.sin(yaw)
            bx, by = o.x - (c * zone[0] - s * zone[1]), o.y - (s * zone[0] + c * zone[1])
            sx, sy = bx + c * BACKUP_M, by + s * BACKUP_M           # stand-off: further from the object (it is behind, at -x)
            ring = [(px + 0.34 * math.cos(a), py + 0.34 * math.sin(a)) for px, py in ((bx, by), (sx, sy))
                    for a in np.linspace(0, 2 * math.pi, 12, endpoint=False)]
            if any(self.p.map_cell(px, py) not in (None, "free") for px, py in ring + [(bx, by), (sx, sy)]):
                continue
            path = self.p.compute_path(sx, sy)
            if path is None:
                continue
            out.append((path["length_m"], yaw, (bx, by), (sx, sy)))
        return sorted(out)

    def approach(self, name, speed=0.10):
        from gurdy_core.visual_servo import CLASS_INFO
        o = self.objects[name]
        zone = CLASS_INFO[o.cls][3]
        cands = self._approach_candidates(o, zone)
        if not cands:
            return False, f"no free spot next to the {o.cls}"
        if self.cfg.approach == "teleport" and self.truth is not None:
            # retest variant A: put the robot where grasp_test starts (object in the class zone behind it), no Nav2
            _, yaw, grasp_xy, _ = cands[0]
            bx, by = o.x - (math.cos(yaw) * zone[0] - math.sin(yaw) * zone[1]), o.y - (math.sin(yaw) * zone[0] + math.cos(yaw) * zone[1])
            ok = self.truth.teleport(bx, by, yaw)
            self._wait(3.0)
            self.log.append({"t": round(self.now(), 1), "teleport": [round(bx, 3), round(by, 3), round(yaw, 3)], "ok": ok})
            return (True, o.room) if ok else (False, "teleport failed")
        for _, yaw, grasp_xy, stand in cands[:3]:
            ok, why = self._nav(stand[0], stand[1], yaw + math.pi, self.cfg.nav_timeout_s)   # facing the object
            if not ok:
                continue
            if not self._turn_to(yaw):
                return False, "could not turn round"
            self._move_body(-BACKUP_M, 0.0, speed=0.05, tol=0.015)  # back up towards it (wrist camera looks behind)
            return True, o.room
        return False, why

    def _servo_robot(self):
        rob = self

        class _R:
            def wrist_view(self):
                v = rob.arm.wrist_view()
                if v is None:
                    raise RuntimeError("no wrist frame")
                return v

            def step(self, dx, dy):
                rob._move_body(dx, dy, speed=0.05, tol=0.005, timeout=10.0)
                rob._wait(0.5)

            def now(self):
                return rob.now()
        return _R()

    def fine_align(self, name):
        from gurdy_core import grasp_test as G
        from gurdy_core.visual_servo import VIEW_POSE, VisualServo
        o = self.objects[name]
        self._move_arm([self.ARM.poses[VIEW_POSE[o.cls]]], 3.0, big=True)
        self._wait(0.5)
        reach = lambda c, xy: G.grasp_plan(self.ARM, self.RB, c, xy).ok       # noqa: E731
        res = VisualServo(self._servo_robot(), arm_reachable=reach).run(o.cls, prior_xy=self._base((o.x, o.y)))
        self._servo = res
        self.log.append({"t": round(self.now(), 1), "servo": res.code, "steps": res.steps, "residual": res.residual})
        self.align_code = res.code
        if self.cfg.attempt_log:     # the servo outcome is saved even when no grasp attempt follows (own file: the attempt
            with open(align_log_path(self.cfg.attempt_log), "a") as fh:      # log's line counts are used by the run scripts)
                fh.write(json.dumps({"t": round(self.now(), 1), "align": res.code, "object": name, "cls": o.cls,
                                     "steps": res.steps, "residual": res.residual,
                                     "obs": res.obs.reason if res.obs is not None else None}, default=float) + "\n")
        # residual None = never measured (the wrist camera never saw the object): pass None on, never a made-up number
        return res.ok, res.residual

    def _gt_xy(self, cls, xy_world):
        if self.truth is None:
            return None
        r = self.truth.nearest(cls, xy_world)
        return None if r is None else (round(r[1], 4), round(r[2], 4))

    def _end_attempt(self, outcome: str, reason: str = ""):
        a, self._att = self._att, None
        if a is None:
            return
        a.update(outcome=outcome, reason=reason, t_end=round(self.now(), 1))
        vs = getattr(self.arm, "view_stats", None)
        if vs:          # wrist views since the last attempt: (got a frame, wall s from the request, triggers sent)
            a["wrist_views"] = [list(v) for v in vs]
            self.arm.view_stats = []
        if self._plan is not None and a.get("rest_world"):
            a["gt_after"] = self._gt_xy(a["cls"], a["rest_world"])
        self.attempts.append(a)
        if self.cfg.attempt_log:
            with open(self.cfg.attempt_log, "a") as fh:
                fh.write(json.dumps(a, default=float) + "\n")

    def grasp(self, name):
        """Same pipeline as scripts/grasp_test.py (25/25 there): fitted floor in MoveIt (once), the target as a MoveIt obstacle
        for the big moves, settle at the pre-grasp + centre correction (measured-FK), straight approach, contact-only squeeze.
        One diagnostics record per attempt (self.attempts, cfg.attempt_log)."""
        from gurdy_core import grasp_test as G
        from gurdy_core import wrist_yaw as WY
        o = self.objects[name]
        moveit = self.cfg.use_moveit and self.arm.moveit_available()
        if moveit and self._floor is None and hasattr(self.arm, "add_floor"):
            self._floor = self.arm.add_floor(0.005)
            self.log.append({"t": round(self.now(), 1), "floor": {k: self._floor.get(k) for k in ("added", "fitted", "z0")}})
        sv = self._servo
        xy_servo = np.array(sv.obs.xy) if sv and sv.obs and sv.obs.xy else None
        xy = xy_servo if xy_servo is not None else self._base((o.x, o.y))
        rp = self.p.pose()
        a = self._att = {"t": round(self.now(), 1), "attempt": len(self.attempts) + 1, "object": name, "cls": o.cls,
                         "robot": [round(v, 4) for v in rp] if rp else None, "map_world": [round(o.x, 4), round(o.y, 4)],
                         "servo": {"code": sv.code, "steps": sv.steps, "residual": sv.residual} if sv else None,
                         "est_base": [round(float(v), 4) for v in xy], "est_source": "servo" if xy_servo is not None else "map",
                         "dist_robot_obj_m": round(float(np.hypot(*xy)), 4)}
        if self.truth is not None:
            gt = self._gt_xy(o.cls, self._world(xy))
            a["gt_world"] = gt
            if gt is not None:
                gb = self._base(gt)
                a["gt_base"] = [round(float(v), 4) for v in gb]
                a["est_err_m"] = round(float(np.hypot(*(np.asarray(xy) - gb))), 4)
                if self.cfg.gt == "use":
                    xy = np.asarray(gb)                          # retest variant B: grasp at the ground-truth pose
                    a["est_source"] = "gt"
        v = self.arm.wrist_view()
        ye = WY.estimate_yaw(*v, o.cls, xy_hint=xy) if v else WY.YawEstimate(None, 0.0, "none")
        yaw = ye.yaw if (ye.yaw is not None and ye.confidence >= self.cfg.yaw_min_conf) else None
        plan = G.grasp_plan(self.ARM, self.RB, o.cls, xy, yaw)
        a.update(yaw=ye.to_dict(), plan_ok=plan.ok, plan_reason=plan.reason)
        self.log.append({"t": round(self.now(), 1), "yaw": ye.to_dict(), "plan": plan.ok, "reason": plan.reason})
        if not plan.ok:
            self._plan = None
            self._end_attempt("missed", f"no grasp plan: {plan.reason}")
            return False
        self._plan, self._rest = plan, self._world(xy)
        a["rest_world"] = [round(float(v), 4) for v in self._rest]
        self.arm.gripper(+GRIP_EFFORT)
        self._wait(0.5)
        tgt = moveit and hasattr(self.arm, "target_in_scene") and self.arm.target_in_scene(o.cls, xy, True)
        a["target_in_scene"] = bool(tgt)
        try:
            if not self._move_arm([plan.pre], 3.0, big=True):
                self._end_attempt("missed", "pre-grasp move failed")
                return False
            if hasattr(self.arm, "settle"):
                a["settle_pre_rad"] = round(self.arm.settle(plan.pre), 4)
                q_meas = self.arm.arm_q()
                if q_meas is not None:
                    delta = G.centre_correction(self.ARM, plan.pre, q_meas)
                    a["centre_shift_m"] = [round(float(d), 4) for d in delta]
                    if float(np.linalg.norm(delta[:2])) > 0.001:
                        p2 = G.grasp_plan(self.ARM, self.RB, o.cls, np.asarray(xy, float) + delta[:2], yaw)
                        a["centre_replan_ok"] = p2.ok
                        if p2.ok:
                            plan = self._plan = p2
                            self._move_arm([plan.pre], 1.0)
                            self.arm.settle(plan.pre, timeout=1.5)
        finally:
            if tgt:
                self.arm.target_in_scene(o.cls, xy, False)       # the final straight approach goes INTO it
        if not self._move_arm([plan.grasp], 2.0):
            self._end_attempt("missed", "approach move failed")
            return False
        if hasattr(self.arm, "settle"):
            a["settle_grasp_rad"] = round(self.arm.settle(plan.grasp, timeout=1.5), 4)
        self.arm.gripper(-GRIP_EFFORT)
        self._wait(1.5)
        a["fingers_closed"] = self.arm.fingers()
        return True

    def _wrist_view_retry(self):
        """A fresh wrist frame, asked for again until it comes: >= cfg.view_tries requests AND cfg.view_window_s of SIM time
        (2026-11-28 smoke: one 3 s wall window missed the frame and the held can was released), wall cap cfg.view_wall_cap_s.
        -> view or None (= no data: the caller must treat it as UNKNOWN, never as a negative result)."""
        import time
        t_sim, t_wall, n = self.now(), time.monotonic(), 0
        while True:
            v = self.arm.wrist_view()
            n += 1
            if v is not None:
                if n > 1:
                    self.log.append({"t": round(self.now(), 1), "wrist_view": f"frame after {n} requests"})
                return v
            if (n >= self.cfg.view_tries and self.now() - t_sim >= self.cfg.view_window_s) or \
                    time.monotonic() - t_wall >= self.cfg.view_wall_cap_s:
                why = (f"no frame after {n} requests, {self.now() - t_sim:.1f} s sim, {time.monotonic() - t_wall:.1f} s wall, "
                       f"sim now {self.now():.2f}"
                       + (f" ({self.arm.last_view_why})" if getattr(self.arm, "last_view_why", None) else ""))
                self.log.append({"t": round(self.now(), 1), "wrist_view": why})
                if self._att is not None:
                    self._att["wrist_view"] = why                 # saved with the attempt (attempts.jsonl)
                return None
            self._wait(0.5)

    def _fingers_retry(self, n=3):
        """Finger positions, re-read up to n times (None = unknown, never 'empty')."""
        for _ in range(n):
            f = self.arm.fingers()
            if f is not None and all(math.isfinite(v) for v in f):
                return f
            self._wait(0.3)
        return None

    def _standing_at_rest(self, cls) -> int | None:
        """Points of something still STANDING where the object was, seen from the class view pose."""
        from gurdy_core import wrist_yaw as WY
        from gurdy_core.visual_servo import VIEW_POSE
        self._move_arm([self.ARM.poses[VIEW_POSE[cls]]], 2.0)
        self._wait(0.5)
        v = self._wrist_view_retry()
        if v is None or self._rest is None:
            self._standing_why = "no wrist frame" if v is None else "rest spot unknown"
            return None
        r = 0.5 * max(self.A.PICKABLE[cls].get("size", (0.06, 0.06))[:2]) if "size" in self.A.PICKABLE[cls] else self.A.PICKABLE[cls]["radius"]
        sv = getattr(self, "_servo", None)            # floor fitted by the servo before the grasp (base frame; robot not moved)
        floor = sv.obs.floor if sv is not None and sv.obs is not None and getattr(sv.obs, "floor", None) else None
        n, why = WY.standing_check(*v, self._base(self._rest), r + 0.01, floor=floor)
        if n is None and floor is not None:           # e.g. the known floor plane is off: try the in-view fit too
            n, why2 = WY.standing_check(*v, self._base(self._rest), r + 0.01)
            why = why2 if n is not None else f"{why}; in-view fit: {why2}"
        self._standing_why = why
        return n

    def verify_grasp(self, name):
        """Primary hold check = GRIPPER + PAYLOAD (gurdy_core.hold_check): fingers stalled at the object's width through the
        lift AND the arm's joint efforts showing the object's weight. Both -> holding; a measured 'nothing there' -> missed
        (open, re-grasp); disagreement or missing data -> keep the gripper closed, fold, and say it cannot be confirmed.
        Fingers fully closed is never 'holding'. Secondary: the wrist view of the old spot, only when that spot is visible
        (the held object usually hides it); it can only VETO a hold (object still standing there), never create one."""
        from gurdy_core import hold_check as HC
        p = self._plan
        if p is None:
            return False
        o = self._objects[name]
        self.verify_code = None
        self._move_arm(p.lift, 2.5)
        if hasattr(self.arm, "settle"):
            self.arm.settle(p.lift[-1], timeout=2.0)
        self._wait(1.0)                                # the arm's integral term takes up the weight
        f = self._fingers_retry()
        eff = self.arm.efforts() if hasattr(self.arm, "efforts") else None
        q = self.arm.arm_q()
        mass, width = (CAT[o.cls][0], CAT[o.cls][1]) if o.cls in CAT else (0.05, 0.06)
        f_closed = self._att.get("fingers_closed") if self._att is not None else None
        hv = HC.hold_verdict(self.ARM, q, eff, f_closed, f, width, mass)
        standing = None
        if hv.fingers != "closed":                     # secondary check, only where it can see
            standing = self._standing_at_rest(o.cls)
            if hv.holding and standing is not None and standing >= 15:
                hv = HC.HoldVerdict(None, "DISAGREE", hv.fingers, hv.payload, hv.ratio,
                                    "the arm feels its weight but my wrist camera still sees it standing where it was")
        rec = {"fingers": f, "efforts": eff, "payload_ratio": hv.ratio, "hold": hv.code, "standing_points": standing,
               "standing_why": getattr(self, "_standing_why", "") if standing is None and hv.fingers != "closed" else ""}
        self.log.append({"t": round(self.now(), 1), "verify": rec})
        if self._att is not None:
            self._att.update(fingers_lifted=f, fingers_ok=hv.fingers == "width", efforts_lift=eff, payload_ratio=hv.ratio,
                             hold=hv.to_dict(), standing_points=standing, standing_why=rec["standing_why"])
        if hv.holding:
            self.carrying = name
            self._end_attempt("success")
            return True
        if hv.holding is None:                         # disagreement / missing data: never a release, never a claim
            self.verify_code = "UNCONFIRMED"
            self.verify_why = hv.why
            self.carrying = name
            self._end_attempt("unconfirmed", hv.why)
            self.fold_arm()
            return False
        self._end_attempt("missed", hv.why)             # measured: nothing in the gripper -> open, back off, re-grasp
        self.arm.gripper(+GRIP_EFFORT)
        self._wait(0.5)
        self._move_arm([p.pre], 1.5)
        return False

    def fold_arm(self):
        return self._move_arm([self.ARM.poses["folded"]], 3.0, big=True)

    def _dropped(self) -> bool:
        f = self.arm.fingers()
        return f is not None and max(f) <= FINGER_CLOSED

    def carry_to(self, xy, speed):
        speed = min(speed, CARRY_SPEED_MPS)
        self.arm.set_speed_limit(speed)
        try:
            h = self.p.nav_send(xy[0], xy[1], None)
            if h is None:
                return False, None
            t0 = self.now()
            while True:
                self._check()
                if self.carrying and (self._dropped() or self._gt_lost()):
                    self.p.nav_cancel(h)
                    self.p.send_cmd(0.0, 0.0, 0.0)
                    name, self.carrying = self.carrying, None
                    self._objects.pop(name, None)
                    return False, self.robot
                st = self.p.nav_state(h)
                if st["state"] == "succeeded":
                    return True, xy
                if st["state"] in ("aborted", "canceled") or self.now() - t0 > self.cfg.carry_timeout_s:
                    if st["state"] == "running":
                        self.p.nav_cancel(h)
                    return False, None
                v = self.p.speed()
                if v is not None:
                    self.speeds.append((round(v, 3), True))
                self._wait(self.cfg.poll_s)
        finally:
            self.arm.set_speed_limit(None)

    # ------------------------------------------------------------------ "bring me X" (cfg.bring_me)
    handover_text = "Here you are: {what}."

    def note_requester(self):
        """Called by FetchSession when a 'bring me' request starts: where the robot must come back to. Off -> unchanged (home)."""
        if not self.cfg.bring_me:
            return
        self.handover_text = "Here's your {what}."
        xy = next((xy for n in ("me", "user", "requester") if (xy := self._place_xy(n))), None)
        src = "saved place"
        if xy is None and self.p.pose() is not None:
            xy, src = self.robot, "robot pose at the command"
        if xy is None:
            xy, src = tuple(self._place_xy("home") or HOME), "home"
        self.user = (float(xy[0]), float(xy[1]))
        self.log.append({"t": round(self.now(), 1), "requester": [round(v, 3) for v in self.user], "source": src})

    def _gt_obj(self):
        """GT (x, y) of the carried object near the grasp centre, or None (no truth / not found within 0.6 m)."""
        if self.truth is None or not self.carrying or self.carrying not in self._objects:
            return None
        gc = self._world(self._gc_base()[:2])
        r = self.truth.nearest(self._objects[self.carrying].cls, gc)
        return None if r is None else (r[1], r[2], gc)

    def _gt_has_data(self) -> bool:
        """Ground truth currently has fresh poses for the carried object's class (else a None from nearest() = no data)."""
        o = self._objects.get(self.carrying) if self.carrying else None
        fresh = getattr(self.truth, "has_data", None)
        return bool(o is not None and self.truth is not None and (fresh(o.cls) if fresh else True))

    def _gc_base(self):
        q = self.arm.arm_q()
        return self.A.fk(self.ARM, np.asarray(q if q is not None else self.ARM.poses["folded"], float))["Gc"]

    def _gt_lost(self) -> bool:
        """With ground truth (cfg.bring_me only): the carried object is no longer at the grasp centre (fell out although the
        fingers did not close fully, e.g. jammed). Two polls in a row, so one stale pose sample does not count."""
        if not (self.cfg.bring_me and self.truth is not None and self.carrying):
            return False
        g = self._gt_obj()
        if g is None and not self._gt_has_data():
            return False                      # no ground-truth data (stale / missing poses): unknown, never "dropped"
        far = g is None or math.hypot(g[0] - g[2][0], g[1] - g[2][1]) > self.cfg.gt_held_m
        self._gt_far = (getattr(self, "_gt_far", 0) + 1) if far else 0
        return self._gt_far >= 2

    def _stationary(self) -> bool:
        """|v| < still_mps for still_s (zero command held meanwhile); False after still_timeout_s."""
        t0 = self.now()
        since = None
        while self.now() - t0 < self.cfg.still_timeout_s:
            self.p.send_cmd(0.0, 0.0, 0.0)
            v = self.p.speed()
            if v is not None and abs(v) < self.cfg.still_mps:
                since = self.now() if since is None else since
                if self.now() - since >= self.cfg.still_s:
                    return True
            else:
                since = None
            self._wait(self.cfg.poll_s)
        return False

    def _lost_on_the_way(self, why):
        name, self.carrying = self.carrying, None
        self.handover_code = "DROPPED"
        o = self._objects.pop(name, None)
        self.log.append({"t": round(self.now(), 1), "deliver": "lost", "why": why})
        return o

    def deliver_release(self) -> bool:
        """cfg.bring_me hand-over: back to the requester, present, stationary check, open, confirm. True ONLY when confirmed.
        On False, `handover_code` says why: DROPPED (lost before the release), NOT_STATIONARY / NOT_RELEASED (still holding),
        DELIVERY_UNCONFIRMED (opened, but the state / GT does not show the object at the release point)."""
        self.handover_code = None
        self.verify_code = None               # verify_grasp: "UNCONFIRMED" = holding (fingers) but not confirmed: kept
        x, y, _ = self.p.pose()
        ux, uy = self.user
        self._turn_to(math.atan2(uy - y, ux - x) + math.pi)        # the arm is at the back: back towards the user
        if self._dropped() or self._gt_lost():
            self._lost_on_the_way("before presenting")
            return False
        self._move_arm([self.handover_pose()], 3.0, big=True)
        self._wait(0.5)
        f_before = self._fingers_retry()
        g_before = self._gt_obj()
        if f_before is None:                  # unknown: not "lost", and opening blind could drop it -> keep holding
            self.fold_arm()
            self.handover_code = "HOLD_UNCONFIRMED"
            self.log.append({"t": round(self.now(), 1), "deliver": "fingers unknown: kept holding"})
            return False
        held_state = min(f_before) > FINGER_CLOSED
        if self.truth is None or (g_before is None and not self._gt_has_data()):
            held_gt = True                    # no ground truth (real robot) or no GT data now: decide on the state alone
        else:
            held_gt = g_before is not None and \
                math.hypot(g_before[0] - g_before[2][0], g_before[1] - g_before[2][1]) <= self.cfg.gt_held_m
        if not held_state or not held_gt:
            self.fold_arm()
            self._lost_on_the_way(f"at the hand-over pose (fingers {f_before}, gt {g_before})")
            return False
        if not self._stationary():
            self.fold_arm()
            self.handover_code = "NOT_STATIONARY"
            self.log.append({"t": round(self.now(), 1), "deliver": "not stationary"})
            return False
        release_xy = self._world(self._gc_base()[:2])
        name, o = self.carrying, self._objects.get(self.carrying)
        self.arm.gripper(+GRIP_EFFORT)                               # open: only now, standing still
        self._wait(1.0)
        f_after = self._fingers_retry()
        # open = clearly wider than while holding (a jammed gripper still reads the object's width, which is > FINGER_CLOSED);
        # unknown fingers after the open command = not shown open -> treated like a jam (close again, keep it, say so)
        opened = f_after is not None and min(f_after) > min(f_before) + 0.003
        rec = {"t": round(self.now(), 1), "deliver": "released", "fingers_before": f_before, "fingers_after": f_after,
               "release_xy": [round(v, 3) for v in release_xy]}
        if not opened:
            self.handover_code = "NOT_RELEASED" if f_after is not None else "RELEASE_UNCONFIRMED"
            self.log.append({**rec, "deliver": "gripper did not open"})
            self.arm.gripper(-GRIP_EFFORT)
            self.fold_arm()
            return False
        self.carrying = None
        self.fold_arm()                    # a held object would follow the arm: GT is read AFTER folding
        ok = True
        if self.truth is not None and o is not None:
            self._wait(0.5)
            r = self.truth.nearest(o.cls, release_xy)
            rec["gt_after"] = None if r is None else [round(r[1], 3), round(r[2], 3)]
            ok = r is not None and math.hypot(r[1] - release_xy[0], r[2] - release_xy[1]) <= self.cfg.gt_release_m
        self.log.append({**rec, "confirmed": ok})
        self._objects.pop(name, None)
        if o is not None:
            o.x, o.y = release_xy
            o.with_user = True
            self._with_user[name] = o
        if not ok:
            self.handover_code = "DELIVERY_UNCONFIRMED"
        return ok

    def _place_plan(self):
        from gurdy_core import grasp_test as G
        from gurdy_core.visual_servo import CLASS_INFO
        o = self._objects.get(self.carrying)
        cls = o.cls if o else "small_box"
        xy = np.array(CLASS_INFO[cls][3])
        return G.grasp_plan(self.ARM, self.RB, cls, xy), xy

    def release(self):
        if not self.carrying:
            return False
        p, place_xy = self._place_plan()
        if p.ok:
            self._move_arm([p.pre], 3.0, big=True)
            self._move_arm([p.grasp], 2.0)
        self.arm.gripper(+GRIP_EFFORT)
        self._wait(1.0)
        f = self.arm.fingers()
        opened = f is not None and min(f) > FINGER_CLOSED
        if p.ok:
            self._move_arm([p.pre], 1.5)
        self.fold_arm()
        if opened:
            name, self.carrying = self.carrying, None
            wx, wy = self._world(place_xy)
            self._moved[name] = (wx, wy, self.now())
            o = self._objects.get(name)
            if o:
                o.x, o.y = wx, wy
        return opened

    def handover_pose(self):
        """Present the object behind the robot, ~15 cm above the floor, horizontal approach (fallback: `ready`)."""
        A = self.A
        tgt = np.array([-0.35, 0.0, A.ground_z(self.RB) + 0.15])
        for ap in (0.0, -0.3, 0.3):
            for eu in (True, False):
                q = A.ik(self.ARM, tgt, ap, elbow_up=eu)
                if q is not None and A.within_limits(self.ARM, q) and A.collision_free(self.ARM, q, self.RB, A.ground_z(self.RB)):
                    return q
        return np.array(self.ARM.poses["ready"])

    def hand_over(self, timeout_s=None):
        timeout_s = timeout_s or self.cfg.handover_timeout_s
        if not self.carrying:
            return False
        if self.cfg.bring_me:
            return self.deliver_release()
        x, y, _ = self.p.pose()
        ux, uy = self.user
        self._turn_to(math.atan2(uy - y, ux - x) + math.pi)        # the arm is at the back: back towards the user
        self._move_arm([self.handover_pose()], 3.0, big=True)
        self.arm.gripper(-HOLD_EFFORT)
        t0 = self.now()
        while self.now() - t0 < timeout_s:
            if self._dropped():                                     # fingers closed: the user took it
                self.arm.gripper(+GRIP_EFFORT)
                self._wait(0.5)
                self.fold_arm()
                name, self.carrying = self.carrying, None
                o = self._objects.pop(name, None)
                if o:
                    o.x, o.y = self.user
                    o.with_user = True
                    self._with_user[name] = o
                return True
            self._wait(self.cfg.poll_s)
        self.arm.gripper(-GRIP_EFFORT)
        self.fold_arm()
        return False
