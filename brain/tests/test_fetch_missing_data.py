"""Rule for every fetch path (2026-11-28): MISSING DATA IS NEVER A NEGATIVE RESULT that triggers a release or a drop.
No wrist frame / unreadable fingers / no ground-truth data -> keep holding and say it cannot be confirmed.
Trigger: Gazebo smoke can_1 -- the verify view got no frame in its 3 s window, 'not lifted', the held can was released.
FakePort / FakeArmPort / FakeTruth only."""
import pytest

from brain.fetch.replies import ground
from brain.ros.fetch_robot import FetchConfig
from brain.tests.test_ros_fetch import FakeTruth, rig


def track_opens(arm):
    """Every gripper OPEN command sent while the fake arm holds an object."""
    opens, orig = [], arm.gripper

    def gripper(effort):
        if effort > 0 and arm.held is not None:
            opens.append(arm.held["name"])
        orig(effort)
    arm.gripper = gripper
    return opens


def no_frame_while_holding(arm, after=None):
    """The wrist camera gives no frame while an object is in the gripper (from request `after` on, if given)."""
    orig, n = arm.wrist_view, {"k": 0}

    def view():
        if arm.held is not None:
            n["k"] += 1
            if after is None or n["k"] <= after:
                return None
        return orig()
    arm.wrist_view = view
    return n


def test_no_wrist_frame_at_verify_keeps_holding_and_says_so():
    s, rob, arm, fp = rig()
    opens = track_opens(arm)
    n = no_frame_while_holding(arm)
    t0 = fp.now()
    r = s.handle("pick up the blue can")
    assert "I can't confirm I'm holding it" in r and "no picture" in r
    assert arm.held is not None and arm.held["name"] == "blue_can" and rob.carrying is not None
    assert opens == []                                                  # never opened while holding
    assert not ground(r, set(), True)                                   # no success claim
    assert n["k"] >= rob.cfg.view_tries and fp.now() - t0 >= rob.cfg.view_window_s
    assert any("no frame after" in str(x.get("wrist_view", "")) for x in rob.log)
    att = rob.attempts[-1]
    assert att["outcome"] == "unconfirmed" and att["standing_points"] is None


def test_frame_after_a_retry_confirms_the_pick():
    s, rob, arm, fp = rig()
    no_frame_while_holding(arm, after=2)                                 # first 2 requests fail, the 3rd works
    r = s.handle("pick up the blue can")
    assert r.startswith("I picked up the blue can")
    assert any(x.get("wrist_view") == "frame after 3 requests" for x in rob.log)


def test_bring_me_with_no_verify_frame_does_not_carry_on_or_claim():
    s, rob, arm, fp = rig()
    opens = track_opens(arm)
    no_frame_while_holding(arm)
    r = s.handle("bring me the blue can")
    assert "I can't confirm I'm holding it" in r and "Here you are" not in r
    assert opens == [] and arm.held is not None


def test_unreadable_fingers_at_verify_keep_holding():
    s, rob, arm, fp = rig()
    opens = track_opens(arm)
    orig = arm.move_arm
    lifted = {"on": False}

    def move_arm(qs, seconds, use_moveit=False):
        out = orig(qs, seconds, use_moveit)
        if arm.held is not None:
            lifted["on"] = True                                         # from the lift on, finger readings are lost
        return out
    arm.move_arm = move_arm
    real_f = arm.fingers
    arm.fingers = lambda: None if lifted["on"] else real_f()
    r = s.handle("pick up the blue can")
    assert "I can't confirm I'm holding it" in r and "couldn't read my gripper" in r
    assert opens == [] and arm.held is not None


def bring_rig(**cfg):
    s, rob, arm, fp = rig()
    rob.cfg = FetchConfig(bring_me=True, **cfg)
    return s, rob, arm, fp


def test_unreadable_fingers_at_the_hand_over_never_open():
    s, rob, arm, fp = bring_rig()
    rob.truth = None
    opens = track_opens(arm)
    at_user = {"on": False}
    orig_turn = rob._turn_to

    def turn(*a, **k):
        if rob.carrying:
            at_user["on"] = True
        return orig_turn(*a, **k)
    rob._turn_to = turn
    real_f = arm.fingers
    arm.fingers = lambda: None if at_user["on"] else real_f()
    r = s.handle("bring me the blue can")
    assert r == "I can't confirm I'm holding the blue can (I couldn't read my gripper), so I didn't let go."
    assert opens == [] and arm.held is not None and rob.carrying is not None
    assert not any(x.get("deliver") == "lost" for x in rob.log)


class NoData(FakeTruth):
    """Ground truth with no fresh poses at all (gz topic stalled): every lookup None, has_data False."""
    def nearest(self, cls, xy, max_d=0.6):
        return None

    def has_data(self, cls):
        return False


def test_missing_ground_truth_is_not_a_drop_on_the_way():
    s, rob, arm, fp = bring_rig()
    rob.truth = NoData(arm, fp)
    r = s.handle("bring me the blue can")
    assert "dropped" not in r                                           # before: no GT pose = "far" = DROPPED
    assert not any(x.get("deliver") == "lost" for x in rob.log)
    rec = next(x for x in rob.log if x.get("deliver") == "released")    # released at the hand-over, standing still
    assert rec["confirmed"] is False and "can't confirm that the blue can reached you" in r   # GT can't confirm: say so


def test_ground_truth_far_is_still_a_drop():
    """Real GT data saying 'not in the gripper' remains a drop (the rule is about MISSING data only)."""
    s, rob, arm, fp = bring_rig()

    class Far(FakeTruth):
        def nearest(self, cls, xy, max_d=0.6):
            return None if rob.carrying else super().nearest(cls, xy, max_d)

        def has_data(self, cls):
            return True
    rob.truth = Far(arm, fp)
    r = s.handle("bring me the blue can")
    assert "dropped the blue can" in r


def test_unreadable_fingers_after_the_open_command_say_so_honestly():
    s, rob, arm, fp = bring_rig()
    rob.truth = None
    orig = arm.gripper
    opened = {"on": False}

    def gripper(effort):
        if effort > 0 and arm.held is not None:
            opened["on"] = True
        orig(effort)
    arm.gripper = gripper
    real_f = arm.fingers
    arm.fingers = lambda: None if opened["on"] else real_f()
    r = s.handle("bring me the blue can")
    assert "couldn't check that my gripper opened" in r and "did not open" not in r and "Here's your" not in r


@pytest.mark.parametrize("cmd", ["pick up the blue can", "bring me the blue can"])
def test_default_paths_unchanged_with_all_data(cmd):
    s, rob, arm, fp = rig()
    r = s.handle(cmd)
    assert r.startswith("I picked up") or r == "Here you are: the blue can."
