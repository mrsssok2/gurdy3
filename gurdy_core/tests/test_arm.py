"""Phase 9a arm design: offline sanity checks (joint limits, IK/FK, folded pose, mass / CoM vs the RL domain
randomisation, servo torques, grasp feasibility) + URDF parity of gurdy_arm.xacro when the `xacro` module is available."""
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from gurdy_core import arm as A

ROOT = Path(__file__).resolve().parents[2]
DESC = ROOT / "ros2_ws/src/gurdy_description"
DR_PAYLOAD_MAX = 0.30          # rl/envs/randomization.py DEFAULTS["payload_kg"][1]
DR_PAYLOAD_OFFSET = 0.02       # DEFAULTS["payload_offset_m"]


@pytest.fixture(scope="module")
def ar():
    return A.load_all()


def test_joint_limits_sane(ar):
    a, _ = ar
    for n, (lo, hi) in a.limits.items():
        assert lo < hi, n
        if n != "gripper_finger":
            assert hi - lo <= math.pi + 1e-3, f"{n}: more than a 180-degree hobby servo"
    assert a.limits["gripper_finger"][1] * 2 >= a.width_max + 0.01      # opening covers the widest object + margin


def test_named_poses_within_limits_with_margin(ar):
    a, rb = ar
    for name, q in a.poses.items():
        assert A.within_limits(a, q), name
    q = a.poses["folded"]
    for i, n in enumerate(A.ARM_JOINTS[1:4], start=1):
        lo, hi = a.limits[n]
        assert min(q[i] - lo, hi - q[i]) >= 0.1, n


def test_ik_fk_roundtrip(ar):
    a, _ = ar
    rng = np.random.default_rng(0)
    n = 0
    for _ in range(400):
        q = np.array([rng.uniform(*a.limits[j]) for j in A.ARM_JOINTS])
        f = A.fk(a, q)
        ot = A.plane_points(a, q)["Gc"][0] < 0                   # grasp centre beyond the yaw axis (same plane)
        sols = [A.ik(a, f["Gc"], f["a3"], elbow_up=eu, over_top=ot) for eu in (True, False)]
        sols = [s_ for s_ in sols if s_ is not None]
        assert sols
        for q2 in sols:
            f2 = A.fk(a, q2)
            assert np.allclose(f2["Gc"], f["Gc"], atol=1e-9)
        n += 1
    assert n == 400


def test_out_of_reach_returns_none(ar):
    a, _ = ar
    assert A.ik(a, np.array([-1.0, 0.0, 0.0]), -math.pi / 2) is None


def test_folded_pose_is_safe_for_walking(ar):
    a, rb = ar
    for payload in (0.0, a.payload_max):
        ok, info = A.folded_ok(a, rb, a.poses["folded"], payload)
        assert ok, payload
        assert info["lowest"] >= 0.055                         # above the lidar plane (0.05) everywhere but the base
        assert math.hypot(*info["com"][:2]) <= 0.01


def test_mass_inside_what_the_policy_has_seen(ar):
    a, rb = ar
    assert A.total_mass(a) + a.payload_max <= DR_PAYLOAD_MAX + 1e-9
    m0, c0 = A.com(A.robot_point_masses(rb))
    assert m0 == pytest.approx(rb.total_mass)
    m1, c1 = A.com(A.robot_point_masses(rb) + A.arm_point_masses(a, a.poses["folded"], a.payload_max))
    assert math.hypot(*(c1[:2] - c0[:2])) <= DR_PAYLOAD_OFFSET   # xy shift inside the trained payload offset
    assert c1[2] - c0[2] < 0.02                                # CoM rises < 2 cm (height was NOT randomised: Gazebo test)


def test_static_torques_within_servo_ratings(ar):
    """Gravity torque <= 80 % of the rated torque in the named poses and in every feasible grasp pose (with payload)."""
    a, rb = ar
    gz = A.ground_z(rb)
    poses = [a.poses["folded"], a.poses["ready"]]
    for obj in A.PICKABLE.values():
        for x in np.arange(-0.40, -0.18, 0.02):
            for y in (-0.06, 0.0, 0.06):
                r = A.plan_grasp(a, rb, obj, (x, y), gz)
                if r:
                    poses.append(r[0])
    assert len(poses) > 20
    for q in poses:
        for j, t in A.joint_torques(a, q, a.payload_max).items():
            assert t <= 0.8 * a.effort[j], (j, t, q)


def test_every_pickable_class_has_a_grasp_behind_the_robot(ar):
    a, rb = ar
    gz = A.ground_z(rb)
    for name, obj in A.PICKABLE.items():
        assert obj["mass"] <= a.payload_max and obj["grip_w"] <= a.width_max, name
        ok = [x for x in np.arange(-0.40, -0.18, 0.01)
              if A.object_clear_of_legs(rb, obj, (x, 0.0)) and A.plan_grasp(a, rb, obj, (x, 0.0), gz)]
        assert ok, name


def test_existing_lab_classes_are_too_big(ar):
    a, _ = ar
    for name, o in A.EXISTING.items():
        assert min(o["size"][:2]) > a.width_max or o["mass_est"] > a.payload_max, name


# ------------------------------------------------------------------ URDF parity (needs the xacro module)
def _expand(**args):
    xacro = pytest.importorskip("xacro")
    argv = {"robot_yaml": str(DESC / "config/robot.yaml"), "arm_yaml": str(DESC / "config/arm.yaml"), **args}
    doc = xacro.process_file(str(DESC / "urdf/gurdy_with_arm.urdf.xacro"), mappings={k: str(v) for k, v in argv.items()})
    return doc.toxml()


def _strip(xml):
    return re.sub(r"<!--.*?-->", "", xml, flags=re.S).split()


def test_default_robot_unchanged_without_arm():
    xacro = pytest.importorskip("xacro")
    base = xacro.process_file(str(DESC / "urdf/gurdy.urdf.xacro"),
                              mappings={"robot_yaml": str(DESC / "config/robot.yaml")}).toxml()
    assert _strip(_expand()) == _strip(base)


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def _axis_R(axis, ang):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def _urdf_fk(root, q: dict, tip: str):
    joints = {j.find("child").get("link"): j for j in root.findall("joint")}
    chain = []
    link = tip
    while link in joints:
        chain.append(joints[link])
        link = joints[link].find("parent").get("link")
    T = np.eye(4)
    for j in reversed(chain):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()] if o is not None else [0, 0, 0]
        rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()] if o is not None else [0, 0, 0]
        Tj = np.eye(4)
        Tj[:3, :3], Tj[:3, 3] = _rpy(*rpy), xyz
        if j.get("type") == "revolute":
            R = np.eye(4)
            R[:3, :3] = _axis_R([float(v) for v in j.find("axis").get("xyz").split()], q.get(j.get("name"), 0.0))
            Tj = Tj @ R
        T = T @ Tj
    return T


def test_urdf_matches_python_model(ar):
    a, _ = ar
    root = ET.fromstring(_expand(arm="true"))
    rng = np.random.default_rng(1)
    for _ in range(50):
        q = np.array([rng.uniform(*a.limits[j]) for j in A.ARM_JOINTS])
        T = _urdf_fk(root, dict(zip(A.ARM_JOINTS, q)), "arm_grasp_link")
        f = A.fk(a, q)
        assert np.allclose(T[:3, 3], f["Gc"], atol=1e-9)
        ap = np.array([math.cos(a.yaw0 + q[0]) * math.cos(f["a3"]), math.sin(a.yaw0 + q[0]) * math.cos(f["a3"]),
                       math.sin(f["a3"])])
        assert np.allclose(T[:3, 0], ap, atol=1e-9)              # gripper +x = approach direction


def test_urdf_masses_and_limits_match_yaml(ar):
    a, _ = ar
    root = ET.fromstring(_expand(arm="true"))
    arm_links = [l for l in root.findall("link") if l.get("name").startswith("arm_")]
    m = sum(float(l.find("inertial/mass").get("value")) for l in arm_links if l.find("inertial") is not None)
    assert m == pytest.approx(A.total_mass(a))
    for j in root.findall("joint"):
        n = j.get("name")
        if n in a.limits:
            lim = j.find("limit")
            assert (float(lim.get("lower")), float(lim.get("upper"))) == pytest.approx(a.limits[n])
    for l in arm_links:                                          # positive-definite inertia, triangle inequality
        i = l.find("inertial/inertia")
        if i is None:
            continue
        ixx, iyy, izz = (float(i.get(k)) for k in ("ixx", "iyy", "izz"))
        assert min(ixx, iyy, izz) > 0 and ixx + iyy >= izz and iyy + izz >= ixx and ixx + izz >= iyy


def test_gazebo_variant_has_arm_control_camera_and_optional_fallback():
    xml = _expand(arm="true", sim_gazebo="true")
    assert "GurdyArmSystem" in xml and "wrist_camera" in xml and "<triggered>true</triggered>" in xml
    assert "gz::sim::systems::DetachableJoint" not in xml        # real contact grasping is the default
    assert "gz::sim::systems::DetachableJoint" in _expand(arm="true", sim_gazebo="true", arm_fallback_model="small_box_1")


def test_srdf_draft_matches_urdf_and_named_poses(ar):
    a, _ = ar
    root = ET.fromstring(_expand(arm="true"))
    links = {l.get("name") for l in root.findall("link")}
    joints = {j.get("name") for j in root.findall("joint")}
    s = ET.parse(DESC / "config/gurdy_arm.srdf").getroot()
    for d in s.findall("disable_collisions"):
        assert d.get("link1") in links and d.get("link2") in links
    for gs in s.findall("group_state"):
        vals = {j.get("name"): float(j.get("value")) for j in gs.findall("joint")}
        assert set(vals) <= joints
        if gs.get("group") == "arm":
            assert [vals[n] for n in A.ARM_JOINTS] == pytest.approx(list(a.poses[gs.get("name")]))
