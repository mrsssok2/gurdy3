"""URDF <-> robot.yaml / gurdy_core parity.

Expands the xacro with the source robot.yaml, then checks: check_urdf passes, 18 revolute joints,
masses and inertias, limits/dynamics, mount poses, one-sided knees and
forward kinematics (chaining the URDF transforms must match gurdy_core.leg_fk => axis signs right).
Run:  colcon test --packages-select gurdy_description   or   python3 -m pytest test/
"""
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

PKG = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "gurdy_core"))

from gurdy_core.kinematics import leg_fk  # noqa: E402
from gurdy_core.robot import JOINT_TYPES, LEG_NAMES, load_robot  # noqa: E402

ROBOT_YAML = PKG / "config" / "robot.yaml"


@pytest.fixture(scope="module")
def robot():
    return load_robot(ROBOT_YAML)


@pytest.fixture(scope="module")
def urdf_text():
    import xacro
    doc = xacro.process_file(str(PKG / "urdf" / "gurdy.urdf.xacro"),
                             mappings={"robot_yaml": str(ROBOT_YAML)})
    return doc.toprettyxml(indent="  ")


@pytest.fixture(scope="module")
def urdf(urdf_text):
    return ET.fromstring(urdf_text)


def _floats(s):
    return np.array([float(v) for v in s.split()])


def _links(urdf):
    return {l.get("name"): l for l in urdf.findall("link")}


def _joints(urdf):
    return {j.get("name"): j for j in urdf.findall("joint")}


def _inertia_diag(link):
    i = link.find("inertial/inertia")
    off = [float(i.get(k)) for k in ("ixy", "ixz", "iyz")]
    assert off == [0.0, 0.0, 0.0]
    return np.array([float(i.get(k)) for k in ("ixx", "iyy", "izz")])


# ------------------------------------------------------------------ structure
@pytest.mark.skipif(shutil.which("check_urdf") is None, reason="check_urdf not installed (apt liburdfdom-tools; in the ROS image and in CI)")
def test_check_urdf_passes(urdf_text, tmp_path):
    f = tmp_path / "gurdy.urdf"
    f.write_text(urdf_text)
    res = subprocess.run(["check_urdf", str(f)], capture_output=True, text=True)
    assert res.returncode == 0, res.stdout + res.stderr
    assert "Successfully Parsed XML" in res.stdout


def test_eighteen_revolute_joints(urdf, robot):
    rev = [j.get("name") for j in urdf.findall("joint") if j.get("type") == "revolute"]
    assert sorted(rev) == sorted(robot.joint_names)


# ------------------------------------------------------------------ mass / inertia
def test_total_mass(urdf, robot):
    total = sum(float(m.get("value")) for m in urdf.iter("mass"))
    assert total == pytest.approx(robot.total_mass, rel=1e-9)


def test_inertias_from_gurdy_core(urdf, robot):
    links = _links(urdf)
    assert links["base_link"].find("inertial") is None, "root must be massless (KDL)"
    assert _inertia_diag(links["body_link"]) == pytest.approx(robot.body_inertia(), rel=1e-9)
    for leg in LEG_NAMES:
        for seg in ("coxa", "femur", "tibia"):
            link = links[f"{leg}_{seg}"]
            assert float(link.find("inertial/mass").get("value")) == pytest.approx(robot.segments[seg].mass)
            assert _inertia_diag(link) == pytest.approx(robot.link_inertia(seg), rel=1e-9), f"{leg}_{seg}"
            com = _floats(link.find("inertial/origin").get("xyz"))
            assert com == pytest.approx(robot.segments[seg].com)


# ------------------------------------------------------------------ limits / dynamics
def test_joint_limits_and_dynamics(urdf, robot):
    joints = _joints(urdf)
    jc = robot.joint_common
    for name in robot.joint_names:
        j = joints[name]
        jtype = name.split("_", 1)[1]
        lim, dyn = j.find("limit"), j.find("dynamics")
        assert (float(lim.get("lower")), float(lim.get("upper"))) == pytest.approx(robot.joint_limits[jtype])
        assert float(lim.get("effort")) == pytest.approx(jc["effort"])
        assert float(lim.get("velocity")) == pytest.approx(jc["velocity"])
        assert float(dyn.get("damping")) == pytest.approx(jc["damping"])
        assert float(dyn.get("friction")) == pytest.approx(jc["friction"])


def test_every_knee_is_one_sided(urdf):
    knees = [j for j in urdf.findall("joint") if j.get("name").endswith("_knee")]
    assert len(knees) == 6
    for j in knees:
        assert float(j.find("limit").get("lower")) == 0.0


# ------------------------------------------------------------------ kinematics parity
def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis, a):
    k = np.asarray(axis) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


def _tf(joint, q=0.0):
    o = joint.find("origin")
    T = np.eye(4)
    T[:3, :3] = _rpy(*_floats(o.get("rpy", "0 0 0")))
    T[:3, 3] = _floats(o.get("xyz", "0 0 0"))
    if joint.get("type") == "revolute":
        R = np.eye(4)
        R[:3, :3] = _axis_angle(_floats(joint.find("axis").get("xyz")), q)
        T = T @ R
    return T


def test_mounts_match_yaml(urdf, robot):
    joints = _joints(urdf)
    for leg, m in robot.mounts.items():
        o = joints[f"{leg}_hip_yaw"].find("origin")
        assert _floats(o.get("xyz")) == pytest.approx([m.xy[0], m.xy[1], 0.0])
        assert _floats(o.get("rpy")) == pytest.approx([0.0, 0.0, m.yaw])


def test_urdf_fk_matches_gurdy_core(urdf, robot):
    """Foot position from chained URDF transforms == gurdy_core.leg_fk (rotated into the body)."""
    joints = _joints(urdf)
    rng = np.random.default_rng(1)
    for leg in LEG_NAMES:
        m = robot.mounts[leg]
        Rm = _rpy(0, 0, m.yaw)
        for _ in range(50):
            q = np.array([rng.uniform(*robot.joint_limits[t]) for t in JOINT_TYPES])
            T = (_tf(joints[f"{leg}_hip_yaw"], q[0]) @ _tf(joints[f"{leg}_hip_pitch"], q[1])
                 @ _tf(joints[f"{leg}_knee"], q[2]) @ _tf(joints[f"{leg}_foot_joint"]))
            expected = np.array([m.xy[0], m.xy[1], 0.0]) + Rm @ leg_fk(robot, q)
            assert T[:3, 3] == pytest.approx(expected, abs=1e-9), (leg, q)


# ------------------------------------------------------------------ MuJoCo cross-check
def test_mujoco_accepts_urdf_and_joint_inertias_match(urdf_text, robot):
    """MuJoCo loads the URDF (all inertias physical) and its fixed-base mass-matrix diagonal
    equals gurdy_core.joint_effective_inertias for random poses."""
    mujoco = pytest.importorskip("mujoco")
    from gurdy_core.kinematics import joint_effective_inertias
    m = mujoco.MjModel.from_xml_string(urdf_text)
    d = mujoco.MjData(m)
    rng = np.random.default_rng(2)
    M = np.zeros((m.nv, m.nv))
    for _ in range(20):
        qs = {leg: np.array([rng.uniform(*robot.joint_limits[t]) for t in JOINT_TYPES]) for leg in LEG_NAMES}
        for leg, q in qs.items():
            for t, v in zip(JOINT_TYPES, q):
                d.qpos[m.joint(f"{leg}_{t}").qposadr[0]] = v
        mujoco.mj_forward(m, d)
        mujoco.mj_fullM(m, M, d.qM)
        for leg, q in qs.items():
            dofs = [m.joint(f"{leg}_{t}").dofadr[0] for t in JOINT_TYPES]
            assert np.diag(M)[dofs] == pytest.approx(joint_effective_inertias(robot, q), rel=1e-6), leg
