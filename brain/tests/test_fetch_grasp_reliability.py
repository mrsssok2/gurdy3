"""Grasp reliability fixes of 2026-11-28 on the fake robot (FakePort / FakeArmPort; no ROS):
1. re-align with the wrist camera before EVERY re-grasp; if the object is no longer seen, say so;
2. never go in (or 'correct') when the arm is held sideways off the pre-grasp pose;
3. the pre-grasp is planned with the fingers OPEN, so MoveIt accepts it (the direct swing is the exception)."""
from gurdy_core import grasp_test as G
from gurdy_core import visual_servo as VS

from brain.tests.test_ros_fetch import obj_of, rig


def aligns(rob):
    return [x for x in rob.log if "servo" in x]


def test_re_align_before_the_re_grasp_finds_the_pushed_object():
    """Attempt 1 misses and pushes the can 4 cm: the old code grasped again at the OLD estimate (3 misses); now it looks
    again and picks it up where it is."""
    s, rob, arm, fp = rig({"miss_push_once": {"blue_can": (0.0, 0.04)}})
    r = s.handle("pick up the blue can")
    assert r.startswith("I picked up the blue can")
    assert [a["outcome"] for a in rob.attempts] == ["missed", "success"]
    assert len(aligns(rob)) == 2                                    # one before each grasp
    assert rob.attempts[1]["est_base"] != rob.attempts[0]["est_base"]


def test_every_re_grasp_is_preceded_by_a_fresh_alignment():
    s, rob, arm, fp = rig({"miss": {"blue_can"}})
    r = s.handle("pick up the blue can")
    assert "couldn't get a grip" in r and "still where it was" not in r
    assert len(rob.attempts) == 3 and len(aligns(rob)) == 3


def test_object_no_longer_seen_after_a_miss_is_said(monkeypatch):
    s, rob, arm, fp = rig({"miss": {"blue_can"}})
    real, n = VS.VisualServo.run, {"k": 0}

    def run(self, cls, prior_xy=None, box_fn=None):
        n["k"] += 1
        return real(self, cls, prior_xy, box_fn) if n["k"] == 1 else VS.ServoResult(False, "TARGET_LOST", steps=3)
    monkeypatch.setattr(VS.VisualServo, "run", run)
    r = s.handle("pick up the blue can")
    assert "missed" in r and "can't see it any more" in r and "pushed it away" in r
    assert len(rob.attempts) == 1                                   # no blind second grasp


def test_arm_held_sideways_at_the_pre_grasp_never_goes_in():
    """Yaw joint 0.03 rad off (the open gripper resting against the object): no approach, no squeeze, no 'correction'."""
    s, rob, arm, fp = rig({"sag": [0.03, 0.0, 0.0, 0.0, 0.0]})
    closes, orig = [], arm.gripper

    def gripper(effort):
        if effort < 0:
            closes.append(fp.now())
        orig(effort)
    arm.gripper = gripper
    before = dict(obj_of(arm, "blue_can"))
    r = s.handle("pick up the blue can")
    assert "couldn't get a grip" in r
    assert closes == [] and all("sideways" in a["reason"] for a in rob.attempts)
    assert all(abs(a["lateral_shift_m"]) > G.PRE_LATERAL_M for a in rob.attempts)
    assert (obj_of(arm, "blue_can")["x"], obj_of(arm, "blue_can")["y"]) == (before["x"], before["y"])   # never touched


def test_pre_grasp_is_planned_with_open_fingers_so_moveit_accepts_it():
    """Fingers travel at 0.05 m/s from closed; the planner rejects a goal with a finger inside the padded target."""
    s, rob, arm, fp = rig({"finger_dynamics": True})
    r = s.handle("pick up the blue can")
    assert r.startswith("I picked up the blue can")
    assert not any(m.get("rejected") for m in arm.moves)            # no rejected MoveIt goal -> no direct fallback
    a = rob.attempts[0]
    assert min(a["fingers_open"]) >= G.FINGERS_OPEN_MIN and a["pre_via"] == "moveit"
