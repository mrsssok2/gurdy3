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


# ------------------------------------------------------------------ wrist yaw in the grasp test
from gurdy_core import wrist_yaw as WY  # noqa: E402


def _est(yaw, conf, method="rect"):
    return WY.YawEstimate(yaw, conf, method, WY.PERIOD.get("small_box"))


def test_yaw_record_modes():
    r = G.yaw_record("small_box", _est(0.10, 0.9), gt_yaw_base=0.10 + math.pi / 2 + math.radians(2))
    assert r["used_src"] == "wrist" and r["used"] == pytest.approx(0.10) and r["err_deg"] == pytest.approx(2.0, abs=0.01)
    r = G.yaw_record("small_box", _est(0.10, 0.3), 0.5)
    assert r["used_src"] == "low_confidence" and r["used"] is None and r["err_deg"] is not None   # error still reported
    r = G.yaw_record("small_box", _est(0.10, 0.9), 0.5, mode="gt")
    assert r["used_src"] == "gt" and r["used"] == pytest.approx(0.5)
    assert G.yaw_record("small_box", _est(0.10, 0.9), 0.5, mode="none")["used"] is None
    r = G.yaw_record("can", WY.YawEstimate(None, 1.0, "symmetric"), 0.5)
    assert r["used_src"] == "symmetric" and r["err_deg"] is None and r["used"] is None


def test_summary_reports_yaw_error_per_class():
    rows = [{"class": "small_box", "outcome": "success", "time_s": 30.0, "yaw": {"err_deg": e, "used_src": s}}
            for e, s in ((1.0, "wrist"), (3.0, "wrist"), (20.0, "low_confidence"))]
    s = G.summarize(rows)["small_box"]
    assert s["yaw_err_median_deg"] == 3.0 and s["yaw_err_max_deg"] == 20.0 and s["yaw_low_conf"] == 1
    assert G.summarize([{"class": "can", "outcome": "missed"}])["can"]["yaw_err_median_deg"] is None


def test_wrist_yaw_drives_the_toolbox_roll_end_to_end():
    sys.path.insert(0, str(ROOT / "gurdy_core/tests"))
    from servo_sim import SimRobot
    for deg in (0, 30, 75, 140):
        r = SimRobot("mini_toolbox", CLASS_INFO["mini_toolbox"][3], yaw_obj=0.3 + math.radians(deg), depth_noise=0.02, seed=deg)
        d, K, T = r.wrist_view()
        xy = r.obj_rel()
        rec = G.yaw_record("mini_toolbox", WY.estimate_yaw(d, K, T, "mini_toolbox", xy_hint=xy), math.radians(deg))
        assert rec["used_src"] == "wrist" and rec["err_deg"] <= 2.0
        p = G.grasp_plan(ARM, ROBOT, "mini_toolbox", xy, rec["used"])
        f = G._finger_axis_xy(ARM, p.grasp)
        across = np.array([math.cos(math.radians(deg) + math.pi / 2), math.sin(math.radians(deg) + math.pi / 2)])
        assert abs(float(f @ across)) > 0.97, deg                      # fingers close across the handle


# ---------------------------------------------------------------- 2026-11-27: centring + fingers-apart scorer rule
def test_object_at_the_finger_midpoint_has_zero_offset():
    a, rb = A.load_arm(), A.load_robot()
    q = G.grasp_plan(a, rb, "can", (-0.33, 0.0)).grasp
    o = G.object_in_gripper(a, q, G.grasp_centre(a, q))
    assert all(abs(v) < 1e-12 for v in o.values())


def test_finger_axis_offset_is_measured_along_the_palm_y_axis():
    a, rb = A.load_arm(), A.load_robot()
    q = G.grasp_plan(a, rb, "can", (-0.33, 0.0)).grasp
    R = A.T_base_palm(a, q)[:3, :3]
    o = G.object_in_gripper(a, q, G.grasp_centre(a, q) + 0.012 * R[:, 1])
    assert o["finger_axis"] == pytest.approx(0.012) and abs(o["approach"]) < 1e-12


def test_centre_correction_cancels_a_tracking_error():
    a, rb = A.load_arm(), A.load_robot()
    p = G.grasp_plan(a, rb, "mini_extinguisher", (-0.33, 0.02))
    q_meas = np.array(p.pre) + np.array([0.03, 0.0, 0.0, 0.0, 0.0])          # yaw lags 0.03 rad
    d = G.centre_correction(a, p.pre, q_meas)
    assert np.linalg.norm(d) > 0.005                                          # ~1 cm at this reach
    # commanding the shifted target with the same joint error puts the midpoint back near the original target
    p2 = G.grasp_plan(a, rb, "mini_extinguisher", np.array([-0.33, 0.02]) + d[:2])
    reached = G.grasp_centre(a, np.array(p2.grasp) + np.array([0.03, 0.0, 0.0, 0.0, 0.0]))
    assert np.linalg.norm(reached[:2] - G.grasp_centre(a, p.grasp)[:2]) < 0.002
    assert np.linalg.norm(G.centre_correction(a, p.pre, np.array(p.pre) + 0.5)) <= 0.02 + 1e-12   # clipped


def test_fingers_apart_rule_and_classify():
    assert G.fingers_apart([0.0254, 0.025], "small_box")
    assert not G.fingers_apart([0.0, 0.0], "can")
    assert G.fingers_apart(None, "can") and G.fingers_apart([float("nan"), 0.0], "can")
    lift = [0.01, 0.03, 0.05]
    hold = [0.05] * 10
    assert G.classify(0.0, lift, hold, fingers_hold=[0.033, 0.033], cls="can").outcome == "success"
    r = G.classify(0.0, lift, hold, fingers_hold=[0.0, 0.0], cls="can")
    assert r.outcome == "missed" and "wedged" in r.detail
    assert G.classify(0.0, lift, hold).outcome == "success"                  # no finger data: old behaviour


def test_walk_start_pose_is_in_front_and_clear_for_every_class():
    walls = L4.walls()
    gt = json.loads((Path(__file__).resolve().parents[2] / "docs/results/data/lab4_fetch/lab4_fetch_gt.json").read_text())["objects"] \
        if (Path(__file__).resolve().parents[2] / "docs/results/data/lab4_fetch/lab4_fetch_gt.json").exists() else None
    if gt is None:
        pytest.skip("lab4_fetch GT not found")
    for cls in G.SMALL:
        obj = next(o for o in gt if o["class"] == cls)
        others = G.other_obstacles(gt, obj["name"])
        pl = G.place_robot(obj, walls, others, G.TRY_OFFSETS[0])
        st = G.start_pose_for(pl, walls, others)
        assert st is not None, cls
        assert 1.0 <= st["dist"] <= 2.0 and abs(st["dir_deg"]) <= 30.0 and st["min_clearance"] >= G.MARGIN
        # in front of the placement: the object (behind the placement) is farther from the start than the placement is
        d_obj = math.hypot(st["x"] - obj["x"], st["y"] - obj["y"])
        assert d_obj > math.hypot(st["x"] - pl["x"], st["y"] - pl["y"])
