"""scripts/fetch_retest_report.py: success judged from simulation ground truth, never from the robot's reply."""
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("fetch_retest_report", ROOT / "scripts/fetch_retest_report.py")
R = importlib.util.module_from_spec(spec)
spec.loader.exec_module(R)

HELD = {"fingers_lifted": [0.038, 0.028], "robot": [-0.94, 0.27, 0.0], "outcome": "success", "reason": ""}


def test_folded_grasp_centre_constant_matches_the_arm_model():
    from gurdy_core import arm as A, grasp_test as G
    a = A.load_arm()
    assert G.grasp_centre(a, a.poses["folded"]) == pytest.approx(R.FOLDED_GC, abs=0.001)


def t(before, after, robot_after, reply="I picked up the blue can."):
    return {"can_before": before, "can_after": after, "robot_after": robot_after, "reply": reply}


def test_retest_c_try1_xy_only_is_a_success():          # the real 20261127_114257 record
    g = R.gt_verdict(t("-1.3000 0.3000", "-0.7533 0.2562", "-0.9272 0.2675"), [HELD])
    assert g["success"] and g["lifted"] and g["held_at_end"] and g["between_fingers"]


def test_claimed_success_but_can_still_on_the_floor_is_a_failure():
    g = R.gt_verdict(t("-1.3 0.3 0.0", "-1.31 0.29 0.0", "-0.94 0.27 0.107"), [HELD])
    assert not g["success"] and not g["lifted"] and "not lifted" in g["why"]


def test_fingers_closed_is_not_between_the_fingers():
    g = R.gt_verdict(t("-1.3 0.3 0.0", "-0.80 0.27 0.20", "-0.93 0.27 0.107"), [{**HELD, "fingers_lifted": [0.0, 0.001]}])
    assert g["lifted"] and not g["between_fingers"] and not g["success"]


def test_dropped_on_the_way_is_not_held_at_the_end():
    g = R.gt_verdict(t("-1.3 0.3 0.0", "-0.60 0.10 0.0", "0.5 0.3 0.107"), [HELD])
    assert not g["held_at_end"] and not g["success"]


def test_raw_json_reply_is_unwrapped():
    assert R.unwrap('{"reply": "I picked up the blue can.", "seq": 2}') == "I picked up the blue can."
