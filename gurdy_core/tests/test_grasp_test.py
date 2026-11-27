"""Phase 9 grasp test, pure part: placement, grasp / lift waypoints, outcome classification (no ROS, no Gazebo)."""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from gurdy_core import arm as A
from gurdy_core import grasp_test as G
from gurdy_core.visual_servo import CLASS_INFO

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import gen_world_lab4 as L4  # noqa: E402

ARM, ROBOT = A.load_arm(), A.load_robot()
GT = json.loads((ROOT / "docs/results/data/lab4_fetch/lab4_fetch_gt.json").read_text())["objects"]
SMALL_GT = [o for o in GT if o["class"] in G.SMALL]


def test_every_small_class_has_an_object_in_lab4_fetch():
    assert {o["class"] for o in SMALL_GT} == set(G.SMALL)


@pytest.mark.parametrize("obj", SMALL_GT, ids=lambda o: o["name"])
@pytest.mark.parametrize("off", G.TRY_OFFSETS)
def test_placement_puts_the_object_in_the_zone_with_clearance(obj, off):
    pl = G.place_robot(obj, L4.walls(), G.other_obstacles(GT, obj["name"]), off)
    assert pl is not None and pl["clearance"] >= G.MARGIN
    bx, by = G.world_to_base((pl["x"], pl["y"], pl["yaw"]), (obj["x"], obj["y"]))
    zone = np.array(CLASS_INFO[obj["class"]][3]) + off
    assert bx == pytest.approx(zone[0], abs=1e-3) and by == pytest.approx(zone[1], abs=1e-3)
    assert bx < 0                                                     # behind the robot: the arm side


@pytest.mark.parametrize("cls", G.SMALL)
@pytest.mark.parametrize("off", G.TRY_OFFSETS)
def test_grasp_plan_feasible_and_lift_is_vertical(cls, off):
    xy = np.array(CLASS_INFO[cls][3]) + off
    p = G.grasp_plan(ARM, ROBOT, cls, xy, obj_yaw_base=0.4)
    assert p.ok, p.reason
    g = A.fk(ARM, p.grasp)["Gc"]
    assert g[:2] == pytest.approx(xy, abs=2e-3)
    assert g[2] == pytest.approx(A.ground_z(ROBOT) + A.grasp_point(A.PICKABLE[cls]), abs=2e-3)
    zs = [A.fk(ARM, q)["Gc"] for q in p.lift]
    assert all(b[2] > a_[2] for a_, b in zip([g] + zs, zs))          # monotonically up
    assert zs[-1][2] - g[2] == pytest.approx(G.LIFT, abs=2e-3)
    assert max(np.hypot(*(z[:2] - g[:2])) for z in zs) < 2e-3        # straight up
    pre = A.fk(ARM, p.pre)["Gc"]
    assert np.linalg.norm(pre - g) == pytest.approx(0.05, abs=2e-3)
    assert all(A.within_limits(ARM, q) for q in [p.pre, p.grasp, *p.lift])
    assert p.opening <= 0.08


def test_roll_puts_the_fingers_across_the_short_side():
    for yaw in (0.0, 0.5, 1.2, -0.7):
        xy = np.array(CLASS_INFO["mini_toolbox"][3])
        p = G.grasp_plan(ARM, ROBOT, "mini_toolbox", xy, obj_yaw_base=yaw)
        assert p.ok
        f = G._finger_axis_xy(ARM, p.grasp)
        short = np.array([math.cos(yaw + math.pi / 2), math.sin(yaw + math.pi / 2)])   # toolbox 0.12 x 0.05: short = y
        assert abs(float(f @ short)) > 0.97, yaw
    assert G.best_roll(ARM, np.zeros(5), None, "small_box") == 0.0
    assert G.best_roll(ARM, np.zeros(5), 0.3, "can") == 0.0           # side grasp: roll 0


def test_no_grasp_far_away():
    p = G.grasp_plan(ARM, ROBOT, "can", (-0.9, 0.0))
    assert not p.ok and "no feasible grasp" in p.reason


# ------------------------------------------------------------------ outcome
Z0 = 0.0005


def lift_series(rise):
    return [Z0 + rise * k / 5 for k in range(6)]


def test_classify_success():
    r = G.classify(Z0, lift_series(0.05), [Z0 + 0.049] * 10)
    assert r.outcome == "success" and r.lifted == pytest.approx(0.05, abs=1e-6) and r.sank < 0.01


def test_classify_missed():
    r = G.classify(Z0, [Z0 + 0.002] * 6, [Z0] * 10)
    assert r.outcome == "missed"


def test_classify_dropped_during_hold():
    r = G.classify(Z0, lift_series(0.05), [Z0 + 0.05] * 4 + [Z0 + 0.003] * 6)
    assert r.outcome == "dropped"


def test_classify_slipped():
    r = G.classify(Z0, lift_series(0.05), [Z0 + 0.05 - 0.004 * k for k in range(6)])
    assert r.outcome == "slipped" and r.sank == pytest.approx(0.02, abs=1e-6)


def test_classify_knocked_flag_and_no_samples():
    r = G.classify(Z0, lift_series(0.05), [Z0 + 0.05] * 5, xy_rest=(1.0, 1.0), xy_at_close=(1.05, 1.0))
    assert r.outcome == "success" and r.knocked
    assert G.classify(Z0, [], []).outcome == "error"


def test_summarize_pass_rule():
    rows = [{"class": "can", "outcome": o, "time_s": 40.0} for o in ("success", "success", "missed")]
    rows += [{"class": "small_box", "outcome": o, "time_s": 30.0} for o in ("success", "success", "dropped")]
    s = G.summarize(rows)
    assert s["can"]["pass"] and s["can"]["success"] == 2 and s["can"]["mean_time_s"] == 40.0
    assert not s["small_box"]["pass"]                                 # a drop fails the class


def test_pose_text_parser_handles_omitted_zero_fields():
    text = """header {
  stamp {
    sec: 12
  }
}
pose {
  name: "f_red_cube"
  id: 41
  position {
    x: 1.2
    y: -1.4
  }
  orientation {
    z: 0.149
    w: 0.988
  }
}
pose {
  name: "gurdy"
  id: 7
  position {
    x: 0.5
    z: 0.112
  }
  orientation {
    w: 1
  }
}
"""
    p = G.PoseTextParser()
    out = [r for r in (p.feed(line) for line in text.splitlines()) if r]
    assert out == [("f_red_cube", 1.2, -1.4, 0.0), ("gurdy", 0.5, 0.0, 0.112)]
