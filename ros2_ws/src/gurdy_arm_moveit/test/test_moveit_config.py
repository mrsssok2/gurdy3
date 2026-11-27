"""Phase 9e MoveIt config, offline (needs numpy, pyyaml and the `xacro` Python module; no ROS / MoveIt)."""
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[4]
PKG = ROOT / "ros2_ws/src/gurdy_arm_moveit"
DESC = ROOT / "ros2_ws/src/gurdy_description"
sys.path[:0] = [str(ROOT / "scripts"), str(ROOT / "gurdy_core")]
pytest.importorskip("xacro")
pytest.importorskip("numpy")
import gen_arm_srdf as G  # noqa: E402

ARM_YAML = yaml.safe_load((DESC / "config/arm.yaml").read_text())


@pytest.fixture(scope="module")
def model():
    return G.Model(G.expand())


@pytest.fixture(scope="module")
def srdf():
    return ET.parse(PKG / "config/gurdy.srdf").getroot()


def test_generated_files_up_to_date():
    r = subprocess.run([sys.executable, str(ROOT / "scripts/gen_arm_srdf.py"), "--check"], cwd=ROOT)
    assert r.returncode == 0, "run scripts/gen_arm_srdf.py"


def test_srdf_refers_to_existing_links_and_joints(model, srdf):
    joints = {j.get("name") for j in model.joints}
    for d in srdf.findall("disable_collisions"):
        assert d.get("link1") in model.links and d.get("link2") in model.links
    for gs in srdf.findall("group_state"):
        assert {j.get("name") for j in gs.findall("joint")} <= joints
    ch = srdf.find("group[@name='arm']/chain")
    assert ch.get("base_link") in model.links and ch.get("tip_link") in model.links
    names = {gs.get("name") for gs in srdf.findall("group_state")}
    assert {"folded", "ready", "pre_grasp", "servo_view", "servo_view_far", "open", "closed"} <= names


def test_named_states_equal_arm_yaml(srdf):
    for gs in srdf.findall("group_state[@group='arm']"):
        if gs.get("name") == "pre_grasp":
            continue
        vals = [float(gs.find(f"joint[@name='{n}']").get("value")) for n in G.ARM_JOINTS]
        assert vals == pytest.approx(ARM_YAML["poses"][gs.get("name")])


def test_arm_vs_robot_collisions_never_disabled(srdf):
    for d in srdf.findall("disable_collisions"):
        a, b = d.get("link1"), d.get("link2")
        if (G.is_arm(a) and G.is_robot(b)) or (G.is_arm(b) and G.is_robot(a)):
            assert d.get("reason") == "Adjacent", (a, b)       # only arm_mount_link on the body


def test_leg_proxies_exist_and_carry_no_mass(model):
    proxies = [n for n in model.links if n.endswith("_proxy")]
    sensors = {"camera_collision_proxy", "lidar_collision_proxy", "wrist_camera_collision_proxy"}   # body camera, LiDAR, wrist camera
    assert sensors <= set(proxies) and len(proxies) == 18 + len(sensors) and all(model.links[p].find("inertial") is None for p in proxies)
    assert all(p in model.boxes for p in proxies)


def test_named_poses_collision_free_at_nominal_stance(model, srdf):
    from gurdy_core.kinematics import nominal_joint_angles
    from gurdy_core.robot import load_robot
    disabled = {frozenset((d.get("link1"), d.get("link2"))) for d in srdf.findall("disable_collisions")}
    q0 = nominal_joint_angles(load_robot())
    for name in ("folded", "ready", "servo_view", "servo_view_far"):
        q = {**q0, **dict(zip(G.ARM_JOINTS, ARM_YAML["poses"][name]))}
        Tl = model.fk(q)
        hits = [(a, b) for a, b in __import__("itertools").combinations(sorted(model.boxes), 2)
                if frozenset((a, b)) not in disabled and G.collide(Tl, model, a, b)]
        assert hits == [], (name, hits)


def test_joint_limits_and_controllers_match(model):
    jl = yaml.safe_load((PKG / "config/joint_limits.yaml").read_text())["joint_limits"]
    for n in G.ARM_JOINTS:
        assert (jl[n]["min_position"], jl[n]["max_position"]) == pytest.approx(model.limits[n])
    ctl = yaml.safe_load((PKG / "config/ros2_controllers.yaml").read_text())
    mc = yaml.safe_load((PKG / "config/moveit_controllers.yaml").read_text())
    assert ctl["arm_controller"]["ros__parameters"]["joints"] == G.ARM_JOINTS
    assert mc["moveit_simple_controller_manager"]["arm_controller"]["joints"] == G.ARM_JOINTS
    k = yaml.safe_load((PKG / "config/kinematics.yaml").read_text())
    assert k["arm"]["kinematics_solver"] == "pick_ik/PickIkPlugin"


def test_launch_parses():
    import ast
    ast.parse((PKG / "launch/move_group.launch.py").read_text())
