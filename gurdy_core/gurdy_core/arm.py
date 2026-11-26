"""Phase 9a: offline model of the planned 5-DOF arm + gripper (config arm.yaml). numpy, no ROS.

Kinematics (all in the BODY frame = base_link: x forward, y left, z up, origin at the body centre):
  mount M on the body top, rotated 180 deg (arm points backward). yaw joint q1 about z at M + base height; shoulder
  axis `shoulder_h` above it. The arm moves in a vertical plane at heading psi = pi + q1 (body frame).
  Plane angles (from horizontal, + = up): a1 = q2, a2 = q2 + q3, a3 = q2 + q3 + q4 (approach direction of the gripper).
  shoulder S -> elbow (L1 along a1) -> wrist pitch axis W (L2 along a2) -> grasp centre G (L3 along a3).
  q5 = wrist roll about the approach axis (only orients the jaws; no effect on positions).
Collision model: capsules along the arm vs the body box, the lidar puck, the six legs at the nominal stance (straight
capsules hip -> knee -> foot) and the floor. Approximate, for the design study only; MoveIt uses the URDF meshes.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from .kinematics import leg_fk, nominal_joint_angles, segment_coms
from .robot import LEG_NAMES, load_robot

G = 9.81
ARM_JOINTS = ["arm_yaw", "arm_shoulder", "arm_elbow", "arm_wrist_pitch", "arm_wrist_roll"]


def default_arm_yaml() -> Path:
    return Path(__file__).resolve().parents[2] / "ros2_ws/src/gurdy_description/config/arm.yaml"


@dataclass
class ArmGeom:
    M: np.ndarray            # mount point (body frame)
    yaw0: float
    base_h: float
    shoulder: np.ndarray     # shoulder axis point at q1 = any (on the yaw axis)
    L1: float
    L2: float
    Lw: float                # wrist pitch axis -> roll/palm start
    Lp: float
    Lf: float
    L3: float                # wrist pitch axis -> grasp centre
    L_tip: float             # wrist pitch axis -> finger tips
    limits: dict
    effort: dict
    masses: dict
    poses: dict
    payload_max: float
    width_max: float
    cam: dict
    raw: dict


def load_arm(path=None) -> ArmGeom:
    d = yaml.safe_load(Path(path or default_arm_yaml()).read_text())
    lk, mt = d["links"], d["mount"]
    M = np.array(mt["xyz"], float)
    base_h = float(mt["base"]["size"][2])
    S = M + [0, 0, base_h + lk["turret"]["shoulder_h"]]
    Lw, Lp, Lf = lk["wrist"]["length"], lk["palm"]["length"], lk["finger"]["length"]
    tip = Lw + Lp + Lf
    j = d["joints"]
    lim = {n: (float(j[n]["lower"]), float(j[n]["upper"])) for n in ARM_JOINTS + ["gripper_finger"]}
    eff = {n: float(j[n]["effort"]) for n in ARM_JOINTS + ["gripper_finger"]}
    masses = {"base": mt["base"]["mass"], "turret": lk["turret"]["mass"], "upper_arm": lk["upper_arm"]["mass"],
              "elbow_servo": lk["upper_arm"]["end_mass"], "forearm": lk["forearm"]["mass"],
              "wrist_servo": lk["forearm"]["end_mass"], "wrist": lk["wrist"]["mass"], "palm": lk["palm"]["mass"],
              "fingers": 2 * lk["finger"]["mass"]}
    return ArmGeom(M, float(mt["yaw"]), base_h, S, lk["upper_arm"]["length"], lk["forearm"]["length"], Lw, Lp, Lf,
                   tip - lk["grasp_offset"], tip, lim, eff, masses, {k: np.array(v, float) for k, v in d["poses"].items()},
                   float(d["payload"]["max_kg"]), float(d["payload"]["max_width"]), d["wrist_camera"], d)


def total_mass(a: ArmGeom) -> float:
    return float(sum(a.masses.values()))


# ------------------------------------------------------------------ kinematics
def plane_points(a: ArmGeom, q):
    """-> dict of (r, z) in the arm plane (r from the yaw axis) for shoulder, elbow, wrist, palm start, grasp, tip."""
    q2, q3, q4 = q[1], q[2], q[3]
    a1, a2, a3 = q2, q2 + q3, q2 + q3 + q4
    S = np.array([0.0, a.shoulder[2]])
    E = S + a.L1 * np.array([math.cos(a1), math.sin(a1)])
    W = E + a.L2 * np.array([math.cos(a2), math.sin(a2)])
    u3 = np.array([math.cos(a3), math.sin(a3)])
    return {"S": S, "E": E, "W": W, "P": W + a.Lw * u3, "Gc": W + a.L3 * u3, "T": W + a.L_tip * u3, "a3": a3}


def to_body(a: ArmGeom, q, rz) -> np.ndarray:
    psi = a.yaw0 + q[0]
    r, z = rz
    return np.array([a.M[0] + r * math.cos(psi), a.M[1] + r * math.sin(psi), z])


def fk(a: ArmGeom, q) -> dict:
    p = plane_points(a, q)
    out = {k: to_body(a, q, v) for k, v in p.items() if k != "a3"}
    out["a3"] = p["a3"]
    return out


def within_limits(a: ArmGeom, q, tol=1e-9) -> bool:
    return all(a.limits[n][0] - tol <= q[i] <= a.limits[n][1] + tol for i, n in enumerate(ARM_JOINTS))


def ik(a: ArmGeom, target, approach: float, roll: float = 0.0, elbow_up: bool = True, over_top: bool = False):
    """Grasp centre at `target` (body frame) with approach angle `approach` (rad from horizontal in the arm plane,
    -pi/2 = top-down). over_top: the arm plane heading is turned by pi and the target lies at negative plane radius
    (the arm reaches over the yaw axis). -> q (5,) or None (out of reach). Joint limits are NOT checked here."""
    d = np.asarray(target[:2], float) - a.M[:2]
    r = float(np.hypot(*d))
    psi = math.atan2(d[1], d[0])
    if over_top:
        psi, r = psi + math.pi, -r
    q1 = (psi - a.yaw0 + math.pi) % (2 * math.pi) - math.pi
    wr = r - a.L3 * math.cos(approach)
    wz = target[2] - a.L3 * math.sin(approach) - a.shoulder[2]
    c3 = (wr * wr + wz * wz - a.L1 ** 2 - a.L2 ** 2) / (2 * a.L1 * a.L2)
    if abs(c3) > 1.0 + 1e-9:
        return None
    c3 = max(-1.0, min(1.0, c3))
    q3 = -math.acos(c3) if elbow_up else math.acos(c3)
    q2 = math.atan2(wz, wr) - math.atan2(a.L2 * math.sin(q3), a.L1 + a.L2 * math.cos(q3))
    q4 = approach - q2 - q3
    q4 = (q4 + math.pi) % (2 * math.pi) - math.pi
    return np.array([q1, q2, q3, q4, roll])


# ------------------------------------------------------------------ collision model
def leg_capsules(robot) -> list:
    """(p0, p1, radius) per leg piece at the nominal stance, body frame."""
    qn = nominal_joint_angles(robot)
    caps = []
    l1, l2, _ = robot.lengths
    for leg in LEG_NAMES:
        m = robot.mounts[leg]
        q = (qn[f"{leg}_hip_yaw"], qn[f"{leg}_hip_pitch"], qn[f"{leg}_knee"])
        c, s = math.cos(m.yaw), math.sin(m.yaw)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        hip = np.array([m.xy[0], m.xy[1], 0.0])
        yaw, pitch, _ = q
        knee_l = np.array([(l1 + l2 * math.cos(pitch)) * math.cos(yaw), (l1 + l2 * math.cos(pitch)) * math.sin(yaw),
                           l2 * math.sin(pitch)])
        foot_l = leg_fk(robot, q)
        caps.append((hip, hip + R @ knee_l, 0.022))
        caps.append((hip + R @ knee_l, hip + R @ foot_l, 0.015))
    return caps


def _seg_seg(p1, q1, p2, q2):
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    c, b = d1 @ r, d1 @ d2
    den = a * e - b * b
    s = np.clip((b * f - c * e) / den, 0, 1) if den > 1e-12 else 0.0
    t = (b * s + f) / e if e > 1e-12 else 0.0
    if t < 0:
        t, s = 0.0, np.clip(-c / a, 0, 1) if a > 1e-12 else 0.0
    elif t > 1:
        t, s = 1.0, np.clip((b - c) / a, 0, 1) if a > 1e-12 else 0.0
    return float(np.linalg.norm(p1 + d1 * s - (p2 + d2 * t)))


def _box_dist(p, half):
    q = np.abs(p) - half
    return float(np.linalg.norm(np.maximum(q, 0)) + min(max(q[0], max(q[1], q[2])), 0))


def arm_capsules(a: ArmGeom, q, opening: float = 0.08):
    f = fk(a, q)
    return [("upper_arm", f["S"], f["E"], 0.017), ("forearm", f["E"], f["W"], 0.015),
            ("wrist_palm", f["W"], f["P"] + (f["T"] - f["W"]) * (a.Lp / a.L_tip), 0.036),
            ("fingers", f["P"] + (f["T"] - f["W"]) * (a.Lp / a.L_tip), f["T"], max(0.012, opening / 2 + 0.006))]


def clearances(a: ArmGeom, q, robot, ground_z: float, opening: float = 0.08, n: int = 12) -> dict:
    """Smallest clearance (m, negative = penetration) of the arm capsules to the body, lidar puck, legs and floor."""
    body_half = np.array(robot.body_size) / 2
    legs = leg_capsules(robot)
    out = {"body": math.inf, "lidar": math.inf, "legs": math.inf, "floor": math.inf}
    for name, p0, p1, rad in arm_capsules(a, q, opening):
        for t in np.linspace(0, 1, n):
            p = p0 + (p1 - p0) * t
            if name != "upper_arm" or t > 0.25:             # the shoulder end sits on the turret above the body
                out["body"] = min(out["body"], _box_dist(p, body_half) - rad)
            lp = np.array([0.0, 0.0, 0.05])
            dz = max(abs(p[2] - lp[2]) - 0.015, 0.0)
            out["lidar"] = min(out["lidar"], math.hypot(max(math.hypot(p[0], p[1]) - 0.025, 0.0), dz) - rad)
            fr = 0.006 if name == "fingers" else rad                   # finger pads are thin vertically
            out["floor"] = min(out["floor"], p[2] - fr - ground_z)
        for c0, c1, cr in legs:
            out["legs"] = min(out["legs"], _seg_seg(p0, p1, c0, c1) - rad - cr)
    return out


def collision_free(a, q, robot, ground_z, margin=0.005, opening=0.08) -> bool:
    return all(v >= margin for v in clearances(a, q, robot, ground_z, opening).values())


# ------------------------------------------------------------------ masses, CoM, torques
def arm_point_masses(a: ArmGeom, q, payload: float = 0.0) -> list:
    f = fk(a, q)
    m = a.masses
    base = a.M + [0, 0, a.base_h / 2]
    pts = [(m["base"], base), (m["turret"], f["S"]), (m["upper_arm"], (f["S"] + f["E"]) / 2), (m["elbow_servo"], f["E"]),
           (m["forearm"], (f["E"] + f["W"]) / 2), (m["wrist_servo"], f["W"]),
           (m["wrist"], f["W"] + (f["P"] - f["W"]) / 2), (m["palm"], f["P"] + (f["T"] - f["W"]) * (a.Lp / a.L_tip / 2)),
           (m["fingers"], f["T"] - (f["T"] - f["W"]) * (a.Lf / a.L_tip / 2))]
    if payload:
        pts.append((payload, f["Gc"]))
    return pts


def robot_point_masses(robot) -> list:
    qn = nominal_joint_angles(robot)
    pts = [(robot.body_mass, np.zeros(3))]
    for leg in LEG_NAMES:
        mnt = robot.mounts[leg]
        c, s = math.cos(mnt.yaw), math.sin(mnt.yaw)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        q = (qn[f"{leg}_hip_yaw"], qn[f"{leg}_hip_pitch"], qn[f"{leg}_knee"])
        for mass, p in segment_coms(robot, q):
            pts.append((mass, np.array([mnt.xy[0], mnt.xy[1], 0.0]) + R @ p))
    return pts


def com(pts) -> tuple:
    m = sum(p[0] for p in pts)
    return m, sum(p[0] * np.asarray(p[1]) for p in pts) / m


def joint_torques(a: ArmGeom, q, payload: float = 0.0) -> dict:
    """Static gravity torque (N·m) at shoulder, elbow, wrist pitch: sum of m g x (horizontal lever in the arm plane)."""
    f = fk(a, q)
    pts = arm_point_masses(a, q, payload)
    psi = a.yaw0 + q[0]
    u = np.array([math.cos(psi), math.sin(psi)])

    def lever(p, j):
        return float((np.asarray(p)[:2] - j[:2]) @ u)

    out = {}
    for name, j, start in (("arm_shoulder", f["S"], 2), ("arm_elbow", f["E"], 4), ("arm_wrist_pitch", f["W"], 6)):
        out[name] = abs(sum(m * G * lever(p, j) for m, p in pts[start:]))
    return out


def support_margin(robot, com_xy) -> float:
    """Distance from the CoM ground projection to the edge of the 6-foot support polygon (static, all feet down)."""
    from .kinematics import nominal_foot
    feet = []
    nf = nominal_foot(robot)
    for leg in LEG_NAMES:
        mnt = robot.mounts[leg]
        c, s = math.cos(mnt.yaw), math.sin(mnt.yaw)
        feet.append((mnt.xy[0] + c * nf[0] - s * nf[1], mnt.xy[1] + s * nf[0] + c * nf[1]))
    pts = sorted(feet)
    hull = _hull(pts)
    p = np.asarray(com_xy, float)
    d = math.inf
    for (x0, y0), (x1, y1) in zip(hull, hull[1:] + hull[:1]):
        e = np.array([x1 - x0, y1 - y0])
        n = np.array([e[1], -e[0]]) / np.linalg.norm(e)
        d = min(d, float((np.array([x0, y0]) - p) @ n))
    return d


def tripod_margin(robot, com_xy) -> float:
    """Worst case over the two tripods (walking: only 3 feet down): distance to the support triangle edge."""
    from .kinematics import nominal_foot
    nf = nominal_foot(robot)
    feet = {}
    for leg in LEG_NAMES:
        mnt = robot.mounts[leg]
        c, s = math.cos(mnt.yaw), math.sin(mnt.yaw)
        feet[leg] = (mnt.xy[0] + c * nf[0] - s * nf[1], mnt.xy[1] + s * nf[0] + c * nf[1])
    worst = math.inf
    for tri in (("lf", "rm", "lr"), ("rf", "lm", "rr")):
        hull = _hull(sorted(feet[k] for k in tri))
        p = np.asarray(com_xy, float)
        d = math.inf
        for (x0, y0), (x1, y1) in zip(hull, hull[1:] + hull[:1]):
            e = np.array([x1 - x0, y1 - y0])
            n = np.array([e[1], -e[0]]) / np.linalg.norm(e)
            d = min(d, float((np.array([x0, y0]) - p) @ n))
        worst = min(worst, d)
    return worst


def _hull(pts):
    def cross(o, a_, b):
        return (a_[0] - o[0]) * (b[1] - o[1]) - (a_[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]          # counter-clockwise


# ------------------------------------------------------------------ objects + grasp strategies
# Pickable test objects (NEW, small and light; the existing lab classes are all too big / heavy, see the plan).
PICKABLE = {
    "small_box":      {"shape": "box", "size": (0.05, 0.05, 0.05), "mass": 0.05, "grasp": "top", "grip_w": 0.05},
    "mini_toolbox":   {"shape": "box", "size": (0.12, 0.05, 0.075), "mass": 0.08, "grasp": "top", "grip_w": 0.02,
                       "grasp_h": 0.068},                 # handle on top (bar 0.02 wide, top at 0.075)
    "can":            {"shape": "cyl", "radius": 0.033, "height": 0.12, "mass": 0.06, "grasp": "side", "grip_w": 0.066},
    "mini_cone":      {"shape": "cone", "radius": 0.04, "height": 0.10, "mass": 0.03, "grasp": "side", "grip_w": 0.045,
                       "grasp_h": 0.05},
    "mini_extinguisher": {"shape": "cyl", "radius": 0.025, "height": 0.15, "mass": 0.08, "grasp": "side", "grip_w": 0.05},
}
# Existing classes (semantic_lab / lab4 sizes, docs/results/data/phase8c/semantic_lab_gt3d.json; masses = real-world
# class estimates, NOT from the sim, where these models are static).
EXISTING = {
    "box": {"size": (0.30, 0.30, 0.30), "mass_est": 1.0},
    "toolbox": {"size": (0.45, 0.205, 0.24), "mass_est": 3.0},
    "fire_extinguisher": {"size": (0.188, 0.156, 0.485), "mass_est": 3.0},
    "traffic_cone": {"size": (0.36, 0.36, 0.65), "mass_est": 1.0},
    "barrel": {"size": (0.556, 0.556, 0.88), "mass_est": 10.0},
    "chair": {"size": (0.42, 0.42, 0.925), "mass_est": 4.0},
    "mannequin": {"size": (0.40, 0.56, 1.66), "mass_est": 10.0},
}


def grasp_point(obj: dict) -> float:
    """Height of the grasp centre above the floor."""
    if "grasp_h" in obj:
        return obj["grasp_h"]
    if obj["grasp"] == "top":
        h = obj["size"][2]
        return h - 0.02 if h > 0.04 else h / 2           # fingers 2 cm down the sides
    return obj["height"] / 2


APPROACH = {"top": [-math.pi / 2], "side": [0.0, -0.3, -0.6]}


def plan_grasp(a, robot, obj: dict, xy, ground_z: float, pregrasp: float = 0.05):
    """Feasible (q_grasp, approach) for the object at body-frame xy (on the floor), checking limits + collisions of the
    grasp AND the pre-grasp (pulled back `pregrasp` along the approach). None if no strategy works."""
    zg = ground_z + grasp_point(obj)
    opening = min(0.08, obj["grip_w"] + 0.015)
    for ap, eu in [(ap, eu) for ap in APPROACH[obj["grasp"]] for eu in (True, False)]:
        tgt = np.array([xy[0], xy[1], zg])
        q = ik(a, tgt, ap, elbow_up=eu)
        if q is None or not within_limits(a, q):
            continue
        u = np.array([math.cos(a.yaw0 + q[0]) * math.cos(ap), math.sin(a.yaw0 + q[0]) * math.cos(ap), math.sin(ap)])
        qp = ik(a, tgt - pregrasp * u, ap, elbow_up=eu)
        if qp is None or not within_limits(a, qp):
            continue
        if collision_free(a, qp, robot, ground_z, opening=opening) and _grasp_clear(a, q, robot, ground_z, opening):
            return q, ap
    return None


def _grasp_clear(a, q, robot, ground_z, opening):
    c = clearances(a, q, robot, ground_z, opening)
    return c["body"] >= 0.005 and c["lidar"] >= 0.005 and c["legs"] >= 0.005 and c["floor"] >= 0.0


def object_clear_of_legs(robot, obj: dict, xy, margin: float = 0.02) -> bool:
    """The object's footprint (circle bound) does not touch any leg capsule (projected to the floor) or the body."""
    r = (max(obj["size"][:2]) / 2 * math.sqrt(2)) if obj["shape"] == "box" else obj["radius"]
    p = np.array([xy[0], xy[1]])
    hx, hy = robot.body_size[0] / 2, robot.body_size[1] / 2
    if abs(p[0]) < hx + r + margin and abs(p[1]) < hy + r + margin:
        return False
    for c0, c1, cr in leg_capsules(robot):
        d = _seg_seg(np.array([*p, 0.0]), np.array([*p, 0.0]), np.array([*c0[:2], 0.0]), np.array([*c1[:2], 0.0]))
        if d < r + cr + margin:
            return False
    return True


def ground_z(robot) -> float:
    return -float(robot.stance["body_height"])


def load_all(robot_yaml=None, arm_yaml=None):
    return load_arm(arm_yaml), load_robot(robot_yaml)


def folded_ok(a, robot, q, payload: float = 0.0, lidar_plane: float = 0.05, margin: float = 0.01):
    """Folded / carry pose rules: within limits; clear of body, lidar puck, legs by `margin`; every arm capsule
    (except the shoulder end on the turret) above the lidar plane; grasp centre over the body (carried object inside the
    footprint, not hanging out behind). -> (ok, info)."""
    if not within_limits(a, q):
        return False, {}
    f = fk(a, q)                                             # cheap checks first
    if not (abs(f["Gc"][0]) <= robot.body_size[0] / 2 and abs(f["Gc"][1]) <= robot.body_size[1] / 2):
        return False, {}
    if min(f["E"][2], f["W"][2], f["T"][2]) < lidar_plane + 0.02:
        return False, {}
    c = clearances(a, q, robot, ground_z(robot), opening=0.08)
    low = math.inf
    for name, p0, p1, rad in arm_capsules(a, q, 0.08):
        for t in np.linspace(0, 1, 9):
            if name == "upper_arm" and t < 0.3:
                continue
            p = p0 + (p1 - p0) * t
            low = min(low, p[2] - (0.012 if name == "fingers" else rad))
    over_body = abs(f["Gc"][0]) <= robot.body_size[0] / 2 and abs(f["Gc"][1]) <= robot.body_size[1] / 2
    m, cm = com(robot_point_masses(robot) + arm_point_masses(a, q, payload))
    ok = min(c["body"], c["lidar"], c["legs"]) >= margin and low >= lidar_plane + 0.005 and over_body
    return ok, {"clear": c, "lowest": low, "com": cm, "grasp": f["Gc"]}


def search_folded(a, robot, payload: float = 0.10, step: float = 0.1, max_xy: float = 0.01):
    """Grid search over shoulder / elbow / wrist pitch (yaw 0): valid folded poses whose CoM (robot + arm + payload) is
    within `max_xy` of the body centre in xy, LOWEST CoM first (the policy never saw a raised CoM; it did see xy payload
    offsets up to 2 cm). -> list of (CoM z, q, info)."""
    out = []
    for q2 in np.arange(a.limits["arm_shoulder"][0], a.limits["arm_shoulder"][1] + 1e-9, step):
        for q3 in np.arange(a.limits["arm_elbow"][0], a.limits["arm_elbow"][1] + 1e-9, step):
            for q4 in np.arange(a.limits["arm_wrist_pitch"][0], a.limits["arm_wrist_pitch"][1] + 1e-9, step):
                q = np.array([0.0, q2, q3, q4, 0.0])
                ok, info = folded_ok(a, robot, q, payload)
                if ok and np.hypot(info["com"][0], info["com"][1]) <= max_xy:
                    out.append((float(info["com"][2]), q, info))
    out.sort(key=lambda t: t[0])
    return out
