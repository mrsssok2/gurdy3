"""Phase 9h: FetchSession on the ROS fetch robot (brain/ros/fetch_robot.py) over FakePort + FakeArmPort (no ROS, no Gazebo).
World = lab4_fetch ground truth (the 8 small objects), robot starts at home (0, 0)."""
import json
import math
from pathlib import Path

import pytest

from brain.fetch.session import FetchSession
from brain.ros.backend import RosBackend
from brain.ros.fake_arm_port import FakeArmPort
from brain.ros.fake_port import FakePort
from brain.ros.fetch_robot import RosFetchRobot

ROOT = Path(__file__).resolve().parents[2]
SMALL = [o for o in json.loads((ROOT / "docs/results/data/lab4_fetch/lab4_fetch_gt.json").read_text())["objects"] if o["pickable"]]


def rig(faults=None, moveit=True, **kw):
    fp = FakePort(pose=(0.0, 0.0, 0.0))
    arm = FakeArmPort(fp, SMALL, faults=faults, moveit=moveit, **kw)
    be = RosBackend(fp, ROOT / "maps/lab4_fetch.places.yaml")
    rob = RosFetchRobot(be, arm)
    be.fetch_session = FetchSession(rob)
    return be.fetch_session, rob, arm, fp


def obj_of(arm, name):
    return next((o for o in arm.objs if o["name"] == name), None)


@pytest.mark.parametrize("text,label", [
    ("bring me the red cube", "red cube"), ("bring me the blue can", "blue can"), ("bring me the orange mini cone", "orange mini cone"),
    ("get me the red mini extinguisher", "red mini extinguisher"), ("bring me the yellow mini toolbox", "yellow mini toolbox"),
])
def test_bring_each_class_end_to_end(text, label):
    s, rob, arm, fp = rig()
    assert s.handle(text) == f"Here you are: the {label}."
    assert rob.carrying is None and arm.held is None
    assert arm.speed_limits == [0.08, None]                       # carried at the carry speed, limit lifted afterwards
    steps = [x for x in rob.log if "servo" in x]
    assert steps and steps[-1]["servo"] in ("ALIGNED", "ALIGNED_ARM")


def test_approach_puts_the_object_in_the_grasp_zone_behind_the_robot():
    from gurdy_core.visual_servo import CLASS_INFO
    s, rob, arm, fp = rig()
    name = next(n for n, o in rob.objects.items() if o.cls == "can" and o.color == "red")
    assert rob.find(name)[0] and rob.approach(name)[0]
    o = rob.objects[name]
    b = rob._base((o.x, o.y))
    assert b[0] < 0 and math.hypot(*(b - CLASS_INFO["can"][3])) < 0.03   # behind, at the zone (before the servo)
    assert fp.nav_log[-1][2] is not None                          # Nav2 to the stand-off, facing the object, then turned round


def test_top_grasp_uses_the_wrist_yaw_and_moveit_only_for_big_moves():
    s, rob, arm, fp = rig()
    assert s.handle("bring me the red cube") == "Here you are: the red cube."
    y = next(x["yaw"] for x in rob.log if "yaw" in x)
    assert y["method"] == "rect" and y["confidence"] >= 0.5
    kinds = [x["arm"] for x in rob.log if "arm" in x]
    assert "moveit" in kinds and "direct" in kinds                # straight approach + lift are direct joint goals


def test_without_moveit_everything_is_direct():
    s, rob, arm, fp = rig(moveit=False)
    assert s.handle("bring me the red cube") == "Here you are: the red cube."
    assert not any(m["moveit"] for m in arm.moves)


def test_place_in_a_room_and_the_map_override():
    s, rob, arm, fp = rig()
    assert s.handle("put the yellow mini toolbox in room B") == "I put the yellow mini toolbox down in room B."
    o = obj_of(arm, "yellow_mini_toolbox")
    name = next(n for n, x in rob.objects.items() if x.cls == "mini_toolbox")
    assert rob.objects[name].room == "room B" and o["z0"] == 0.0     # on the floor in room B, and the session knows it


def test_pick_then_place_in_two_turns():
    s, rob, arm, fp = rig()
    assert "holding it" in s.handle("pick up the green cone")
    assert rob.carrying is not None and arm.held is not None
    assert s.handle("put it in room A") == "I put the green mini cone down in room A."


@pytest.mark.parametrize("fault", ["slip", "miss"])
def test_failed_grip_is_detected_by_verify_and_retried(fault):
    s, rob, arm, fp = rig({fault: {"blue_can"}})
    r = s.handle("bring me the blue can")
    assert "couldn't get a grip" in r and "3 attempts" in r and "Here you are" not in r
    v = [x["verify"] for x in rob.log if "verify" in x]
    assert len(v) == 3 and not any(x["fingers_ok"] and (x["standing_points"] or 0) < 15 for x in v)
    assert rob.carrying is None


def test_drop_while_carrying():
    s, rob, arm, fp = rig({"drop_carry": {"red_can"}})
    r = s.handle("bring me the red can")
    assert "dropped the red can" in r and "Here you are" not in r and rob.carrying is None


def test_user_does_not_take_it():
    s, rob, arm, fp = rig({"user_takes": False})
    r = s.handle("bring me the red can")
    assert "didn't take" in r and rob.carrying is not None


def test_handover_turns_the_back_to_the_user():
    s, rob, arm, fp = rig()
    s.handle("bring me the red cube")
    x, y, yaw = fp.pose()
    to_user = math.atan2(rob.user[1] - y, rob.user[0] - x)
    assert abs(math.atan2(math.sin(yaw - to_user - math.pi), math.cos(yaw - to_user - math.pi))) < 0.15


def test_two_objects_one_after_another():
    s, rob, arm, fp = rig()
    assert s.handle("take the red can and the blue cube to me") == "Here you are: the red can. Here you are: the blue cube."


def test_unseen_object_is_searched_for():
    s, rob, arm, fp = rig(unseen={"blue_cube"})
    assert not any(o.cls == "small_box" and o.color == "blue" for o in rob.objects.values())
    r = s.handle("bring me the blue cube")
    assert r == "Here you are: the blue cube."
    assert any(m["tool"] == "find" for m in rob.b.missions)


def test_estop_mid_task_stops_and_says_what_is_true():
    s, rob, arm, fp = rig(estop_at=30.0)
    r = s.handle("bring me the red can")
    assert r.startswith("I stopped (emergency stop).") and "Here you are" not in r
    assert fp.cmd == (0.0, 0.0, 0.0) and s.queue == [] and s.pending is None


def test_mock_session_is_unchanged_by_the_hooks():
    from brain.fetch.mock import FetchMock
    m = FetchSession(FetchMock.lab4_fetch(all_known=False))
    assert not hasattr(m.r, "search")
    assert "Here you are" in m.handle("bring me the red can")


def test_handover_pose_is_feasible():
    from gurdy_core import arm as A
    s, rob, arm, fp = rig()
    q = rob.handover_pose()
    assert A.within_limits(rob.ARM, q) and A.fk(rob.ARM, q)["Gc"][2] > A.ground_z(rob.RB) + 0.10
