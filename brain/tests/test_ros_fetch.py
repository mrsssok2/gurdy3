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


def rig(faults=None, moveit=True, wrapped=False, **kw):
    fp = FakePort(pose=(0.0, 0.0, 0.0))
    arm = FakeArmPort(fp, SMALL, faults=faults, moveit=moveit, **kw)
    be = RosBackend(fp, ROOT / "maps/lab4_fetch.places.yaml")
    if wrapped:                                   # exactly as brain/ros/run_text.py builds it: the fetch robot gets the WRAPPED backend
        from brain.incidents.backend import with_incidents
        be = with_incidents(be, None, "lab4_fetch")
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


def test_search_from_fetch_through_the_wrappers_advances_the_task():
    """2026-11-27 Gazebo dry run: the search stalled after its first Nav2 step. run_text wraps RosBackend (incidents, memory);
    IncidentBackend.tick() only samples for the recorder and never ticks the wrapped task engine, and the agent's pump (which
    normally ticks it through poll_events) is blocked by the agent lock while the fetch runs."""
    s, rob, arm, fp = rig(unseen={"blue_cube"}, wrapped=True)
    r = s.handle("bring me the blue cube")
    assert r == "Here you are: the blue cube."
    m = next(m for m in rob.engine.missions if m["tool"] == "find")
    assert m["state"] != "running" and (m.get("result") or {}).get("found")
    assert rob.engine.task is None or rob.engine.task.tool != "find"


def test_search_view_radius_reaches_the_task_engine_through_the_wrappers():
    from brain.ros.fetch_robot import FetchConfig
    s, rob, arm, fp = rig(unseen={"blue_cube"}, wrapped=True)
    rob.cfg = FetchConfig(search_view_radius_m=1.5, search_timeout_s=120.0)   # only checks that the setting arrives (a 1.5 m sweep of all 4 rooms needs > 900 s)
    seen = []
    real = rob.engine._sweep_steps
    rob.engine._sweep_steps = lambda *a, **k: (seen.append(rob.engine.search_view_radius_m), real(*a, **k))[1]
    s.handle("bring me the blue cube")
    assert seen and seen[0] == 1.5 and rob.engine.search_view_radius_m is None      # set for the search only, then restored
    assert len(rob.engine._sweep_steps()) < len((lambda: (setattr(rob.engine, "search_view_radius_m", 1.5), rob.engine._sweep_steps())[1])())


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


# ---- 2026-11-27 fetch dry run: the grasp_test pipeline in the fetch path + one diagnostics record per attempt
def test_grasp_uses_floor_target_obstacle_and_logs_each_attempt(tmp_path):
    from brain.ros.fetch_robot import FetchConfig
    s, rob, arm, fp = rig({"miss": {"blue_can"}})
    rob.cfg = FetchConfig(attempt_log=str(tmp_path / "att.jsonl"))
    assert "couldn't get a grip" in s.handle("bring me the blue can")
    assert [e[0] for e in arm.scene].count("floor") == 1                       # fitted floor added once
    tgt = [e for e in arm.scene if e[0] == "target"]
    assert len(tgt) == 6 and [e[3] for e in tgt] == [True, False] * 3          # on for the big moves, off before the approach
    lines = [json.loads(x) for x in (tmp_path / "att.jsonl").read_text().splitlines()]
    assert len(lines) == len(rob.attempts) == 3 and [a["attempt"] for a in lines] == [1, 2, 3]
    for a in lines:
        assert a["outcome"] == "missed" and a["reason"] == "fingers fully closed: nothing between them"
        assert a["plan_ok"] and a["est_source"] == "servo" and 0.25 < a["dist_robot_obj_m"] < 0.45
        assert list(a["fingers_closed"]) == [0.0, 0.0] and "settle_pre_rad" in a and "centre_shift_m" in a


def test_success_attempt_record():
    s, rob, arm, fp = rig()
    assert s.handle("pick up the red cube") == "I picked up the red cube in room A and I'm holding it with the arm folded."
    a = rob.attempts[-1]
    assert a["outcome"] == "success" and a["reason"] == "" and a["fingers_ok"] and a["target_in_scene"]


def test_centring_corrects_a_sagging_arm(monkeypatch):
    """The effort-controlled arm settles off its command (constant joint offset): the measured-FK centre correction
    re-plans so the finger midpoint lands on the object (grasp_test fix 1). Same sag without the correction: 3 misses."""
    sag = {"sag": [0.0, -0.10, 0.0, 0.0, 0.0]}
    s, rob, arm, fp = rig(sag)
    assert s.handle("bring me the red cube") == "Here you are: the red cube."
    a = rob.attempts[0]
    assert a["outcome"] == "success" and a.get("centre_replan_ok") and abs(a["centre_shift_m"][2]) > 0.01
    import numpy as np
    from gurdy_core import grasp_test as G
    monkeypatch.setattr(G, "centre_correction", lambda *a, **k: np.zeros(3))
    s, rob, arm, fp = rig(sag)
    assert "couldn't get a grip" in s.handle("bring me the red cube")
    assert [a["outcome"] for a in rob.attempts] == ["missed"] * 3


class FakeTruth:
    """Stands in for brain.ros.gz_truth.GzTruth: ground truth = the fake arm port's object list; teleport moves the FakePort."""
    def __init__(self, arm, fp, bias=(0.0, 0.0)):
        self.arm, self.fp, self.bias, self.teleports = arm, fp, bias, []

    def nearest(self, cls, xy, max_d=0.6):
        c = [(math.hypot(o["x"] - xy[0], o["y"] - xy[1]), o) for o in self.arm.objs if o["class"] == cls]
        d, o = min(c, key=lambda t: t[0]) if c else (9, None)
        return None if d > max_d else (o["model"], o["x"] + self.bias[0], o["y"] + self.bias[1])

    def teleport(self, x, y, yaw):
        self.teleports.append((x, y, yaw))
        self.fp.x, self.fp.y, self.fp.yaw = x, y, yaw
        return True


def test_variant_a_teleport_approach_puts_the_object_in_the_zone():
    from brain.ros.fetch_robot import FetchConfig
    s, rob, arm, fp = rig()
    rob.cfg, rob.truth = FetchConfig(approach="teleport", gt="log"), FakeTruth(arm, fp)
    assert s.handle("pick up the blue can").startswith("I picked up the blue can")
    t = rob.truth.teleports
    assert len(t) == 1 and any("teleport" in x for x in rob.log)
    a = rob.attempts[0]
    assert a["gt_world"] == (-1.3, 0.3) and a["est_err_m"] < 0.02 and a["est_source"] == "servo"


def test_variant_b_grasp_at_the_ground_truth_pose():
    """gt=use: the grasp plan takes the GT pose; a 4 cm GT offset the servo cannot see moves the plan target with it."""
    from brain.ros.fetch_robot import FetchConfig
    s, rob, arm, fp = rig()
    rob.cfg, rob.truth = FetchConfig(gt="use"), FakeTruth(arm, fp, bias=(0.0, 0.04))
    s.handle("pick up the blue can")
    a = rob.attempts[0]
    assert a["est_source"] == "gt" and 0.03 < a["est_err_m"] < 0.05
