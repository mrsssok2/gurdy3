#!/usr/bin/env python3
"""Phase 9e: generate the MoveIt SRDF for Gurdy + arm, with the disabled-collision list computed OFFLINE from the URDF.

    python3 scripts/gen_arm_srdf.py            # writes ros2_ws/src/gurdy_arm_moveit/config/gurdy.srdf (+ a report)
    python3 scripts/gen_arm_srdf.py --check    # exit 1 if the committed SRDF is out of date
Needs the `xacro` Python module (host venv) and numpy; no ROS.

Method (like moveit_setup_assistant, but conservative): expand gurdy_with_arm.urdf.xacro arm:=true moveit_collision:=true
(leg collision proxies: the leg links have no collision boxes of their own); every collision
geometry -> oriented bounding box (cylinders / spheres by their bounding box: over-estimates contacts, so fewer pairs are
called 'never'); sample N random configurations (all arm joints AND all 18 leg joints uniform in their limits, fingers
open/closed); separating-axis test per link pair. Disabled: Adjacent (joined by one joint, fixed chains merged), Always
(colliding in >= 95 % of samples) and Never (0 of N) -- EXCEPT every pair of an arm link with a leg link or with the
body, which stays ENABLED whatever the statistics say (the arm must never touch the robot; legs move with the gait).
"""
import itertools
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DESC = ROOT / "ros2_ws/src/gurdy_description"
OUT = ROOT / "ros2_ws/src/gurdy_arm_moveit/config/gurdy.srdf"
REPORT = ROOT / "docs/results/data/phase9e/srdf_collision_report.md"
sys.path.insert(0, str(ROOT / "gurdy_core"))
N_SAMPLES, SEED = 3000, 0
ARM_JOINTS = ["arm_yaw", "arm_shoulder", "arm_elbow", "arm_wrist_pitch", "arm_wrist_roll"]


def expand():
    import xacro
    return xacro.process_file(str(DESC / "urdf/gurdy_with_arm.urdf.xacro"), mappings={
        "robot_yaml": str(DESC / "config/robot.yaml"), "arm_yaml": str(DESC / "config/arm.yaml"), "arm": "true",
        "moveit_collision": "true"}).toxml()


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def axis_R(k, a):
    k = np.asarray(k, float) / np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def origin(el):
    o = el.find("origin")
    xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()] if o is not None else [0, 0, 0]
    r = [float(v) for v in (o.get("rpy") or "0 0 0").split()] if o is not None else [0, 0, 0]
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = rpy(*r), xyz
    return T


class Model:
    def __init__(self, xml):
        root = ET.fromstring(xml)
        self.links = {l.get("name"): l for l in root.findall("link")}
        self.joints = [j for j in root.findall("joint")]
        self.child = {j.find("child").get("link"): j for j in self.joints}
        self.root = next(n for n in self.links if n not in self.child)
        self.boxes = {}                               # link -> list of (T_link_geom, half extents)
        for n, l in self.links.items():
            for c in l.findall("collision"):
                g = c.find("geometry")[0]
                if g.tag == "box":
                    he = np.array([float(v) for v in g.get("size").split()]) / 2
                elif g.tag == "cylinder":
                    r, L = float(g.get("radius")), float(g.get("length"))
                    he = np.array([r, r, L / 2])
                elif g.tag == "sphere":
                    he = np.full(3, float(g.get("radius")))
                else:
                    continue
                self.boxes.setdefault(n, []).append((origin(c), he))
        self.limits = {}
        for j in self.joints:
            lim = j.find("limit")
            if j.get("type") in ("revolute", "prismatic") and lim is not None:
                self.limits[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))

    def fk(self, q):
        T = {self.root: np.eye(4)}

        def get(link):
            if link in T:
                return T[link]
            j = self.child[link]
            Tj = origin(j)
            if j.get("type") in ("revolute", "continuous"):
                R = np.eye(4)
                R[:3, :3] = axis_R([float(v) for v in j.find("axis").get("xyz").split()], q.get(j.get("name"), 0.0))
                Tj = Tj @ R
            elif j.get("type") == "prismatic":
                P = np.eye(4)
                P[:3, 3] = np.array([float(v) for v in j.find("axis").get("xyz").split()]) * q.get(j.get("name"), 0.0)
                Tj = Tj @ P
            T[link] = get(j.find("parent").get("link")) @ Tj
            return T[link]
        for l in self.links:
            get(l)
        return T


def obb_overlap(A, a, B, b, eps=1e-9):
    """Separating axis test of two oriented boxes (4x4 poses A, B; half extents a, b)."""
    Ra, Rb = A[:3, :3], B[:3, :3]
    t = Ra.T @ (B[:3, 3] - A[:3, 3])
    R = Ra.T @ Rb
    AR = np.abs(R) + eps
    for i in range(3):
        if abs(t[i]) > a[i] + b @ AR[i]:
            return False
    for j in range(3):
        if abs(t @ R[:, j]) > a @ AR[:, j] + b[j]:
            return False
    for i in range(3):
        for j in range(3):
            ra = a[(i + 1) % 3] * AR[(i + 2) % 3, j] + a[(i + 2) % 3] * AR[(i + 1) % 3, j]
            rb = b[(j + 1) % 3] * AR[i, (j + 2) % 3] + b[(j + 2) % 3] * AR[i, (j + 1) % 3]
            if abs(t[(i + 2) % 3] * R[(i + 1) % 3, j] - t[(i + 1) % 3] * R[(i + 2) % 3, j]) > ra + rb:
                return False
    return True


def collide(Tl, m, l1, l2):
    return any(obb_overlap(Tl[l1] @ G1, h1, Tl[l2] @ G2, h2) for G1, h1 in m.boxes[l1] for G2, h2 in m.boxes[l2])


def is_arm(n):
    return n.startswith("arm_") or n.startswith("wrist_camera")


def is_robot(n):
    return n == "body_link" or any(n.startswith(p) for p in ("lf_", "lm_", "lr_", "rf_", "rm_", "rr_"))   # incl. *_proxy


def build():
    m = Model(expand())
    links = sorted(m.boxes)
    # adjacency through one joint, with fixed-joint chains merged (a link and its fixed parent are one body)
    def body_of(l):
        while l in m.child and m.child[l].get("type") == "fixed":
            l = m.child[l].find("parent").get("link")
        return l
    adjacent = set()
    for j in m.joints:
        p, c = body_of(j.find("parent").get("link")), body_of(j.find("child").get("link"))
        adjacent.add(frozenset((p, c)))
    pairs = [frozenset(p) for p in itertools.combinations(links, 2)]
    rng = np.random.default_rng(SEED)
    hits = {p: 0 for p in pairs}
    for _ in range(N_SAMPLES):
        q = {n: rng.uniform(*lim) for n, lim in m.limits.items()}
        Tl = m.fk(q)
        for p in pairs:
            a, b = tuple(p)
            if body_of(a) == body_of(b) or frozenset((body_of(a), body_of(b))) in adjacent:
                continue
            if collide(Tl, m, a, b):
                hits[p] += 1
    disabled, kept_arm_robot, report = [], [], []
    for p in pairs:
        a, b = sorted(p)
        protected = (is_arm(a) and is_robot(b)) or (is_arm(b) and is_robot(a))
        adj = body_of(a) == body_of(b) or frozenset((body_of(a), body_of(b))) in adjacent
        frac = hits[p] / N_SAMPLES
        reason = "Adjacent" if adj else "Always" if frac >= 0.95 else "Never" if hits[p] == 0 else None
        if protected and reason != "Adjacent":
            kept_arm_robot.append((a, b, frac))
            continue
        if reason:
            disabled.append((a, b, reason))
    return m, disabled, kept_arm_robot, hits


def srdf_text(disabled):
    import yaml
    poses = yaml.safe_load((DESC / "config/arm.yaml").read_text())["poses"]
    lines = ['<?xml version="1.0"?>',
             "<!-- GENERATED by scripts/gen_arm_srdf.py from gurdy_with_arm.urdf.xacro arm:=true (edit the script / arm.yaml).",
             f"     Disabled collisions: Adjacent / Always (>= 95 %) / Never (0 of {N_SAMPLES} random arm + leg configurations,",
             "     OBB approximation). Arm-vs-leg and arm-vs-body pairs are NEVER disabled (except adjacent). -->",
             '<robot name="gurdy">',
             '  <group name="arm"><chain base_link="arm_mount_link" tip_link="arm_grasp_link"/></group>',
             '  <group name="gripper"><joint name="arm_finger_left_joint"/><joint name="arm_finger_right_joint"/></group>']
    for name, q in poses.items():
        lines.append(f'  <group_state name="{name}" group="arm">' + "".join(
            f'<joint name="{j}" value="{v}"/>' for j, v in zip(ARM_JOINTS, q)) + "</group_state>")
    lines.append('  <group_state name="pre_grasp" group="arm">' + "".join(
        f'<joint name="{j}" value="{v}"/>' for j, v in zip(ARM_JOINTS, poses["ready"])) + "</group_state>"
        "  <!-- placeholder: the real pre-grasp is computed per object (gurdy_core.arm.plan_grasp) -->")
    lines += ['  <group_state name="open" group="gripper"><joint name="arm_finger_left_joint" value="0.04"/><joint name="arm_finger_right_joint" value="0.04"/></group_state>',
              '  <group_state name="closed" group="gripper"><joint name="arm_finger_left_joint" value="0.0"/><joint name="arm_finger_right_joint" value="0.0"/></group_state>',
              '  <end_effector name="gripper" parent_link="arm_palm_link" group="gripper" parent_group="arm"/>',
              '  <virtual_joint name="world_to_base" type="fixed" parent_frame="odom" child_link="base_link"/>',
              '  <!-- the 18 leg joints are passive for MoveIt (not in any group): their states come from /joint_states -->']
    for a, b, r in sorted(disabled):
        lines.append(f'  <disable_collisions link1="{a}" link2="{b}" reason="{r}"/>')
    lines.append("</robot>")
    return "\n".join(lines) + "\n"


def joint_limits_text():
    """MoveIt joint_limits.yaml from arm.yaml (velocity as in the URDF; acceleration limits are a conservative choice,
    servos are not acceleration-limited hardware: 2 rad/s^2 arm, 0.1 m/s^2 fingers; MoveIt scales them down by default)."""
    import yaml
    j = yaml.safe_load((DESC / "config/arm.yaml").read_text())["joints"]
    out = ["# GENERATED by scripts/gen_arm_srdf.py from config/arm.yaml (edit arm.yaml).",
           "default_velocity_scaling_factor: 0.3", "default_acceleration_scaling_factor: 0.3", "joint_limits:"]
    for n in ARM_JOINTS + ["arm_finger_left_joint", "arm_finger_right_joint"]:
        src = j["gripper_finger"] if n.startswith("arm_finger") else j[n]
        acc = 0.1 if n.startswith("arm_finger") else 2.0
        out.append(f"  {n}: {{has_velocity_limits: true, max_velocity: {src['velocity']}, has_acceleration_limits: true, "
                   f"max_acceleration: {acc}, has_position_limits: true, min_position: {src['lower']}, max_position: {src['upper']}}}")
    return "\n".join(out) + "\n"


JL = ROOT / "ros2_ws/src/gurdy_arm_moveit/config/joint_limits.yaml"


def main():
    m, disabled, kept, hits = build()
    text = srdf_text(disabled)
    jl = joint_limits_text()
    rep = ["# SRDF collision report (generated by scripts/gen_arm_srdf.py)", "",
           f"{len(m.boxes)} links with collision geometry, {len(hits)} pairs, {N_SAMPLES} random configurations.", "",
           f"Disabled: {sum(r == 'Adjacent' for *_, r in disabled)} Adjacent, {sum(r == 'Always' for *_, r in disabled)} Always, "
           f"{sum(r == 'Never' for *_, r in disabled)} Never.", "",
           f"Arm-vs-robot pairs kept enabled: {len(kept)} (of which in contact in some sample: "
           f"{sum(f > 0 for *_, f in kept)}).", "", "| arm link | robot link | contact share in random samples |", "|---|---|---|"]
    rep += [f"| {a} | {b} | {f * 100:.1f} % |" for a, b, f in sorted(kept, key=lambda t: -t[2]) if f > 0]
    if "--check" in sys.argv:
        sys.exit(0 if OUT.exists() and OUT.read_text() == text and JL.exists() and JL.read_text() == jl else 1)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(text)
    JL.write_text(jl)
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text("\n".join(rep) + "\n")
    print(f"wrote {OUT} ({len(disabled)} disabled pairs) and {REPORT}")


if __name__ == "__main__":
    main()
