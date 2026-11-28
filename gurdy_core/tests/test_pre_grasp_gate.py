"""Pre-grasp gate + finger / target clearance (gurdy_core.grasp_test), replayed on the fetch attempts recorded 2026-11-28:
every first attempt of the day's can runs (smoke_1128b..e, the bring-me takes). The gate must let all grasps through that
HELD the can and stop the ones that closed on nothing with the arm off its pre-grasp pose."""
import glob
import json
from pathlib import Path

import numpy as np
import pytest

from gurdy_core import arm as A
from gurdy_core import grasp_test as G
from gurdy_core.robot import load_robot

ROOT = Path(__file__).resolve().parents[2]
ARM, RB = A.load_arm(), load_robot()


def first_attempts():
    pats = ["docs/results/data/smoke_1128[bcde]/can_*/fetch/can_1/chat/attempts.jsonl",
            "docs/results/data/demo_bring_1128*/raw/chat/attempts.jsonl"]
    out = []
    for pat in pats:
        for f in sorted(glob.glob(str(ROOT / pat))):
            a = json.loads(open(f).readline())
            if a.get("settle_pre_rad") is not None and a.get("fingers_closed"):
                out.append((f.split("data/")[1].split("/fetch")[0].split("/raw")[0], a))
    if len(out) < 10:
        pytest.skip("recorded fetch attempts not in this checkout")
    return out


def test_gate_separates_todays_held_grasps_from_the_misses():
    held = missed = 0
    for name, a in first_attempts():
        ok, why = G.pre_grasp_ok(a["settle_pre_rad"], a["centre_shift_m"], a["est_base"])
        gripped = min(a["fingers_closed"]) > 0.02                       # fingers stalled on the can (33 mm)
        if gripped:
            held += 1
            assert ok, (name, why, a["settle_pre_rad"], a["centre_shift_m"])
        else:
            missed += 1
            assert not ok, (name, a["settle_pre_rad"], a["centre_shift_m"])
    assert held >= 8 and missed >= 4


def test_gate_reasons():
    assert G.pre_grasp_ok(0.235, [0.0072, -0.0005, -0.0187], [-0.34, -0.016]) == (False, "arm did not reach the pre-grasp pose")
    ok, why = G.pre_grasp_ok(0.038, [-0.0003, 0.0083, 0.0016], [-0.365, 0.018])
    assert not ok and "sideways" in why
    assert G.pre_grasp_ok(0.10, [0.0, 0.0, 0.02], [-0.3, 0.0])[0]        # in-plane sag: allowed, centre correction handles it
    assert not G.pre_grasp_ok(None, None, [-0.3, 0.0])[0]


@pytest.mark.parametrize("cls,xy,yaw,half_open_collides", [
    ("can", (-0.3557, 0.0021), None, True), ("can", (-0.345, 0.033), None, True), ("small_box", (-0.30, 0.0), 0.0, True),
    ("mini_extinguisher", (-0.35, 0.0), None, False)])                 # 25 mm radius: clear even half open
def test_half_open_fingers_are_inside_the_padded_target_at_the_pre_grasp(cls, xy, yaw, half_open_collides):
    """Why MoveIt 'could never plan' the pre-grasp in the fetch: 0.5 s after the open command the fingers are at ~25 mm."""
    p = G.grasp_plan(ARM, RB, cls, np.array(xy), yaw)
    assert p.ok
    assert bool(G.finger_target_clearance(ARM, p.pre, cls, xy, 0.025) < 0) is half_open_collides   # rejected goal state
    assert G.finger_target_clearance(ARM, p.pre, cls, xy, G.FINGERS_OPEN_MIN) > 0.001
    assert G.finger_target_clearance(ARM, p.pre, cls, xy, 0.040) > 0.004
