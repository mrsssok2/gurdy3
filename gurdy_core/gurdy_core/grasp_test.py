"""Phase 9 grasp test, pure part (numpy, no ROS): robot placement next to a small object, grasp / lift waypoints from the
analytic IK, and the outcome classification of one try. The ROS runner is scripts/grasp_test.py (Gazebo lab4_fetch).

Placement: the arm picks BEHIND the robot (rear mount turned 180 deg). The robot is put so that the object lies at the
class grasp-zone centre (visual_servo.CLASS_INFO, base_link) PLUS a known offset per try, so the visual servo has to
correct it. The robot yaw is the candidate (of 24) with the most footprint clearance from the lab4 walls and the other
objects (footprint circle 0.34 m, >= 5 cm margin).

Grasp: plan_grasp (gurdy_core.arm: limits + collision model, top grasp for box-like classes, side grasp for the rest)
gives q_grasp and the approach angle; the pre-grasp is pulled back 5 cm along the approach; the lift is a straight
vertical line of `n` IK waypoints (+5 cm). For top grasps the wrist roll is chosen numerically so the finger axis (palm y)
is across the object's SHORT side (yaw from ground truth in the first version: an oracle input, see the runner).

Outcome of one try (object pose samples from Gazebo, finger joint positions):
  missed    the object never rose >= 60 % of the lift (fingers closed on nothing or it slid out at once)
  dropped   lifted, then back within 1 cm of its rest height during / at the end of the hold
  slipped   lifted and still held, but it sank > 1 cm relative to the lift height during the hold
  success   lifted and held: sank <= 1 cm, still >= 60 % of the lift above rest at the end
  knocked   (flag) the object moved > 3 cm in xy before the gripper closed (approach pushed it)
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field

import numpy as np

from . import arm as A
from .visual_servo import CLASS_INFO

FOOT_R = 0.34
MARGIN = 0.05
LIFT = 0.05
HOLD_S = 5.0
SMALL = ("small_box", "mini_toolbox", "can", "mini_cone", "mini_extinguisher")
TRY_OFFSETS = [(0.03, 0.02), (-0.02, -0.03), (0.04, 0.0)]      # object offset from the zone centre (base_link, m), per try


# ------------------------------------------------------------------ placement
def _dist_rect(x, y, cx, cy, sx, sy):
    return math.hypot(max(abs(x - cx) - sx / 2, 0.0), max(abs(y - cy) - sy / 2, 0.0))


def clearance(x, y, walls, others) -> float:
    """Distance from (x, y) to the nearest wall rectangle (name, (cx, cy), (sx, sy)) or object circle (x, y, r)."""
    d = [_dist_rect(x, y, c[0], c[1], s[0], s[1]) for _, c, s in walls]
    d += [math.hypot(x - ox, y - oy) - r for ox, oy, r in others]
    return min(d) if d else 9.0


def base_pose_for(obj_xy, zone_xy, yaw):
    """Robot base pose (x, y, yaw) that puts the object at base-frame `zone_xy`."""
    c, s = math.cos(yaw), math.sin(yaw)
    zx, zy = zone_xy
    return obj_xy[0] - (c * zx - s * zy), obj_xy[1] - (s * zx + c * zy), yaw


def world_to_base(pose, p):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = p[0] - x, p[1] - y
    return c * dx + s * dy, -s * dx + c * dy


def place_robot(obj: dict, walls, others, offset=(0.0, 0.0), n_yaw: int = 24) -> dict | None:
    """-> {x, y, yaw, clearance, zone, offset} or None (no yaw with enough clearance)."""
    zone = np.array(CLASS_INFO[obj["class"]][3]) + np.asarray(offset, float)
    best = None
    for k in range(n_yaw):
        yaw = -math.pi + 2 * math.pi * k / n_yaw
        x, y, _ = base_pose_for((obj["x"], obj["y"]), zone, yaw)
        cl = clearance(x, y, walls, others) - FOOT_R
        if cl >= MARGIN and (best is None or cl > best["clearance"]):
            best = {"x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 4), "clearance": round(cl, 3),
                    "zone": [float(v) for v in zone], "offset": list(offset)}
    return best


def other_obstacles(gt_objects, target_name) -> list:
    return [(o["x"], o["y"], 0.5 * max(o["size"][:2]) if "size" in o else 0.15) for o in gt_objects if o["name"] != target_name]


# ------------------------------------------------------------------ grasp waypoints
def _finger_axis_xy(a, q):
    R = A.T_base_palm(a, q)[:3, :3]
    v = R[:2, 1]
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-6 else None


def best_roll(a, q, obj_yaw_base: float | None, cls: str) -> float:
    """Top grasp: wrist roll so the finger closing axis is across the object's short side. None yaw / side grasp: 0."""
    p = A.PICKABLE[cls]
    if obj_yaw_base is None or p["grasp"] != "top":
        return 0.0
    sx, sy = p["size"][:2]
    short = obj_yaw_base + (0.0 if sx <= sy else math.pi / 2)     # object axis along its SHORT side = finger closing axis
    u = np.array([math.cos(short), math.sin(short)])
    lo, hi = a.limits["arm_wrist_roll"]
    best, bv = 0.0, -1.0
    for r in np.linspace(lo, hi, 64):
        qq = np.array(q, float)
        qq[4] = r
        f = _finger_axis_xy(a, qq)
        if f is None:
            continue
        v = abs(float(f @ u))
        if v > bv + 1e-9:
            best, bv = float(r), v
    return best


@dataclass
class GraspPlan:
    ok: bool
    reason: str = ""
    approach: float | None = None
    pre: np.ndarray | None = None
    grasp: np.ndarray | None = None
    lift: list = field(default_factory=list)
    opening: float = 0.08

    def to_dict(self):
        f = lambda q: None if q is None else [round(float(v), 4) for v in q]   # noqa: E731
        return {"ok": self.ok, "reason": self.reason, "approach": self.approach, "pre": f(self.pre), "grasp": f(self.grasp),
                "lift": [f(q) for q in self.lift], "opening": self.opening}


def grasp_plan(a, robot, cls: str, xy_base, obj_yaw_base: float | None = None, lift: float = LIFT, n: int = 5,
               pregrasp: float = 0.05) -> GraspPlan:
    obj = A.PICKABLE[cls]
    gz = A.ground_z(robot)
    pg = A.plan_grasp(a, robot, obj, xy_base, gz, pregrasp)
    if pg is None:
        return GraspPlan(False, "no feasible grasp (limits / collisions) at this xy")
    qg, ap = pg
    eu = qg[2] <= 0                                              # elbow-up solution has q3 < 0 (gurdy_core.arm.ik)
    roll = best_roll(a, qg, obj_yaw_base, cls)
    tgt = np.array([xy_base[0], xy_base[1], gz + A.grasp_point(obj)])
    u = np.array([math.cos(a.yaw0 + qg[0]) * math.cos(ap), math.sin(a.yaw0 + qg[0]) * math.cos(ap), math.sin(ap)])
    qp = A.ik(a, tgt - pregrasp * u, ap, roll, elbow_up=eu)
    qg = A.ik(a, tgt, ap, roll, elbow_up=eu)
    lifts = []
    for k in range(1, n + 1):
        q = A.ik(a, tgt + np.array([0.0, 0.0, lift * k / n]), ap, roll, elbow_up=eu)
        if q is None or not A.within_limits(a, q):
            return GraspPlan(False, f"lift waypoint {k}/{n} out of reach / limits", ap, qp, qg)
        lifts.append(q)
    if qp is None or qg is None or not (A.within_limits(a, qp) and A.within_limits(a, qg)):
        return GraspPlan(False, "roll / pre-grasp outside the limits", ap, qp, qg)
    return GraspPlan(True, "", float(ap), qp, qg, lifts, min(0.08, obj["grip_w"] + 0.015))


# ------------------------------------------------------------------ outcome
@dataclass
class TryResult:
    outcome: str                 # success | slipped | dropped | missed | error
    lifted: float = 0.0          # max rise above rest (m)
    sank: float = 0.0            # rise at hold start - min rise during the hold (m)
    end_rise: float = 0.0
    knocked: bool = False
    detail: str = ""

    def to_dict(self):
        r = lambda v: round(float(v), 4)    # noqa: E731
        return {"outcome": self.outcome, "lifted": r(self.lifted), "sank": r(self.sank), "end_rise": r(self.end_rise),
                "knocked": self.knocked, "detail": self.detail}


def classify(z_rest: float, lift_z: list, hold_z: list, xy_rest=None, xy_at_close=None, lift: float = LIFT,
             slip_tol: float = 0.01, drop_tol: float = 0.01, lifted_frac: float = 0.6, knock_tol: float = 0.03) -> TryResult:
    """z samples of the object (world z of the model origin): during the lift and during the hold."""
    knocked = bool(xy_rest is not None and xy_at_close is not None
                   and math.hypot(xy_at_close[0] - xy_rest[0], xy_at_close[1] - xy_rest[1]) > knock_tol)
    if not lift_z and not hold_z:
        return TryResult("error", detail="no object pose samples", knocked=knocked)
    rise = [z - z_rest for z in list(lift_z) + list(hold_z)]
    top = max(rise)
    if top < lifted_frac * lift:
        return TryResult("missed", top, 0.0, rise[-1], knocked, f"rose only {top * 100:.1f} cm")
    h = [z - z_rest for z in hold_z] or [rise[-1]]
    start, low, end = h[0], min(h), h[-1]
    sank = max(start - low, 0.0)
    if low <= drop_tol or end <= drop_tol:
        return TryResult("dropped", top, sank, end, knocked, "back on the floor during the hold")
    if sank > slip_tol or end < lifted_frac * lift:
        return TryResult("slipped", top, sank, end, knocked, f"sank {sank * 100:.1f} cm in the fingers")
    return TryResult("success", top, sank, end, knocked)


def summarize(rows: list, min_success: int = 2) -> dict:
    """Per class: counts, mean time of successful tries, PASS when >= min_success of the tries succeeded and none dropped."""
    out = {}
    for r in rows:
        c = out.setdefault(r["class"], {"tries": 0, "success": 0, "slipped": 0, "dropped": 0, "missed": 0, "error": 0,
                                        "knocked": 0, "times": []})
        c["tries"] += 1
        c[r["outcome"]] = c.get(r["outcome"], 0) + 1
        c["knocked"] += int(bool(r.get("knocked")))
        if r["outcome"] == "success" and r.get("time_s") is not None:
            c["times"].append(r["time_s"])
    for c in out.values():
        c["mean_time_s"] = round(sum(c["times"]) / len(c["times"]), 1) if c["times"] else None
        c["pass"] = c["success"] >= min_success and c["dropped"] == 0
        del c["times"]
    return out


# ------------------------------------------------------------------ gz pose stream text
class PoseTextParser:
    """Line parser for `gz topic -e -t /world/W/(dynamic_)pose/info` (gz.msgs.Pose_V as protobuf text). Protobuf text
    OMITS zero-valued fields, so a missing x / y / z inside `position { }` is 0. feed(line) -> (name, x, y, z) when a
    position block closes, else None. Format NOT verified against Harmonic output yet (first run)."""

    _NAME = re.compile(r'^name: "(.+)"$')
    _VAL = re.compile(r"^([xyz]): ([-+\d.eE]+)$")

    def __init__(self):
        self.name, self.block, self.pos = None, None, {}

    def feed(self, line: str):
        s = line.strip()
        m = self._NAME.match(s)
        if m:
            self.name, self.pos, self.block = m.group(1), {}, None
            return None
        if s.endswith("{"):
            self.block = s[:-1].strip()
            return None
        if s == "}":
            done = self.block == "position" and self.name is not None
            self.block = None
            if done:
                return self.name, self.pos.get("x", 0.0), self.pos.get("y", 0.0), self.pos.get("z", 0.0)
            return None
        m = self._VAL.match(s)
        if m and self.block == "position":
            self.pos[m.group(1)] = float(m.group(2))
        return None
