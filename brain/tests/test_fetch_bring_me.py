"""Phase 9k demo: "bring me X" on the ROS fetch robot with FetchConfig.bring_me (default OFF), over FakePort + FakeArmPort
+ a fake ground truth (no ROS, no Gazebo). Delivery is claimed ONLY when the state and the ground truth confirm it."""
import math

import pytest

from brain.fetch.replies import ground
from brain.ros.fetch_robot import FetchConfig
from brain.tests.test_ros_fetch import FakeTruth, obj_of, rig


def bring_rig(faults=None, truth=True, pose=None, **cfg):
    s, rob, arm, fp = rig(faults)
    if pose is not None:
        fp.x, fp.y, fp.yaw = pose
    rob.cfg = FetchConfig(bring_me=True, **cfg)
    rob.truth = FakeTruth(arm, fp) if truth else None
    return s, rob, arm, fp


def released(rob):
    return next((x for x in rob.log if x.get("deliver") == "released"), None)


def test_default_off_is_the_old_hand_over():
    s, rob, arm, fp = rig()
    assert not rob.cfg.bring_me
    assert s.handle("bring me the red can") == "Here you are: the red can."
    assert not any("requester" in x or "deliver" in x for x in rob.log)


@pytest.mark.parametrize("truth", [True, False])
def test_bring_me_delivers_and_says_heres_your(truth):
    s, rob, arm, fp = bring_rig(truth=truth)
    r = s.handle("bring me the red can")
    assert r == "Here's your red can."
    assert not ground(r, {"handed_over"}, False)                  # the claim regex knows the new phrase
    rec = released(rob)
    assert rec and rec["confirmed"] and rob.carrying is None and arm.held is None
    o = obj_of(arm, "red_can")
    assert o["z0"] == 0.0 and math.hypot(o["x"] - rec["release_xy"][0], o["y"] - rec["release_xy"][1]) < 0.02
    if truth:
        assert rec["gt_after"] is not None


def test_comes_back_to_the_requester_pose_not_home():
    """The request comes from (1.0, -1.0) (robot pose at the command, no saved 'me' place): it returns there, not to (0, 0)."""
    s, rob, arm, fp = bring_rig(pose=(1.0, -1.0, 0.0))
    assert s.handle("bring me the red can") == "Here's your red can."
    req = next(x for x in rob.log if "requester" in x)
    assert req["requester"] == [1.0, -1.0] and req["source"] == "robot pose at the command"
    x, y, _ = fp.pose()
    assert math.hypot(x - 1.0, y - -1.0) < 0.9                     # stopped next to the requester (0.6 m stand-off)
    assert math.hypot(x, y) > 0.5


def test_saved_me_place_wins_over_the_command_pose():
    s, rob, arm, fp = bring_rig(pose=(1.0, -1.0, 0.0))
    rob.b.places["me"] = {"x": -1.0, "y": -1.0, "kind": "place"}
    s.handle("bring me the red can")
    req = next(x for x in rob.log if "requester" in x)
    assert req["requester"] == [-1.0, -1.0] and req["source"] == "saved place"


def test_gripper_opens_only_when_stationary():
    s, rob, arm, fp = bring_rig()
    opens = []
    orig = arm.gripper

    def gripper(effort):
        if effort > 0 and rob.carrying:
            opens.append(fp.speed())
        orig(effort)
    arm.gripper = gripper
    assert s.handle("bring me the red can") == "Here's your red can."
    assert opens and all(abs(v) < rob.cfg.still_mps for v in opens)


def test_never_opens_while_moving():
    s, rob, arm, fp = bring_rig(still_timeout_s=3.0)
    real = fp.speed
    s.r.arm.gripper  # noqa: B018
    moving = {"on": False}
    fp.speed = lambda: 0.05 if moving["on"] else real()
    orig_turn = rob._turn_to

    def turn(*a, **k):                                             # the body keeps drifting once it is at the requester
        out = orig_turn(*a, **k)
        moving["on"] = True
        return out
    rob._turn_to = turn
    r = s.handle("bring me the red can")
    assert "still holding" in r and "Here's your" not in r
    assert rob.carrying is not None and arm.held is not None and released(rob) is None


def test_dropped_on_the_way_fingers_close():
    s, rob, arm, fp = bring_rig({"drop_carry": {"red_can"}})
    r = s.handle("bring me the red can")
    assert "dropped the red can" in r and "Here's your" not in r and "Here you are" not in r
    assert rob.carrying is None and released(rob) is None


def test_dropped_on_the_way_seen_only_by_ground_truth():
    """Jammed fingers keep the object's width after it fell out: only the GT check sees the drop. Must say so."""
    s, rob, arm, fp = bring_rig({"drop_silent": {"red_can"}})
    r = s.handle("bring me the red can")
    assert "dropped the red can" in r and "Here's your" not in r
    assert rob.carrying is None and released(rob) is None
    o = obj_of(arm, "red_can")
    assert o["z0"] == 0.0 and math.hypot(o["x"], o["y"]) > 0.5      # it lies on the route, not with the user


def test_known_limit_state_only_cannot_see_a_jammed_silent_drop():
    """KNOWN LIMIT (documented, not fixed): without ground truth (real robot) the state is the fingers only. Fingers jammed at
    the object's width after it fell out look like a held object, the gripper then opens wider, and the robot claims the
    delivery. In Gazebo (the demo) the GT check catches it (test above). Fix needs a payload / wrist-view check."""
    s, rob, arm, fp = bring_rig({"drop_silent": {"red_can"}}, truth=False)
    assert s.handle("bring me the red can") == "Here's your red can."      # false claim: the limit this test pins


def test_jammed_gripper_is_never_a_delivery():
    s, rob, arm, fp = bring_rig({"jam": True})
    r = s.handle("bring me the red can")
    assert r == "My gripper did not open, so I'm still holding the red can."
    assert rob.carrying is not None


def test_object_not_at_the_release_point_is_unconfirmed():
    s, rob, arm, fp = bring_rig()

    class Off(FakeTruth):
        def nearest(self, cls, xy, max_d=0.6):              # GT lost it after the release (rolled away / off the map)
            r = super().nearest(cls, xy, max_d)
            return r if rob.carrying else None
    rob.truth = Off(arm, fp)
    r = s.handle("bring me the red can")
    assert "can't confirm" in r and "Here's your" not in r
    assert not ground(r, set(), False)


def test_estop_during_delivery_never_claims():
    s, rob, arm, fp = bring_rig()
    orig = rob.deliver_release

    def stop_then():
        fp.set_estop(True)
        return orig()
    rob.deliver_release = stop_then
    r = s.handle("bring me the red can")
    assert r.startswith("I stopped") and "Here's your" not in r
