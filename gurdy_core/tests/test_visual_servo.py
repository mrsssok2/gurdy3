"""Phase 9e visual servoing: wrist depth -> object pose -> body steps, with synthetic depth and a kinematic robot."""
import math

import numpy as np
import pytest

from gurdy_core import arm as A
from gurdy_core import visual_servo as VS

from servo_sim import SimRobot

ARM, ROBOT = A.load_all()
GZ = A.ground_z(ROBOT)


def arm_ok(cls, xy):
    return A.plan_grasp(ARM, ROBOT, A.PICKABLE[cls], xy, GZ) is not None


def err(r, xy):
    return float(np.hypot(*(np.array(xy) - r.obj_rel())))


@pytest.mark.parametrize("cls", list(VS.CLASS_INFO))
def test_zone_targets_are_graspable(cls):
    t = VS.CLASS_INFO[cls][3]
    assert all(arm_ok(cls, (t[0] + dx, t[1] + dy)) for dx in (-0.02, 0, 0.02) for dy in (-0.02, 0, 0.02))


@pytest.mark.parametrize("pose,lo,hi", [("servo_view", -0.36, -0.30), ("servo_view_far", -0.46, -0.40)])
def test_servo_view_pose_safe_and_looks_at_the_zones(pose, lo, hi):
    q = ARM.poses[pose]
    assert A.within_limits(ARM, q)
    c = A.clearances(ARM, q, ROBOT, GZ, 0.08)
    assert min(c["body"], c["lidar"], c["legs"]) >= 0.015
    T = A.T_base_wrist_optical(ARM, q)
    z = T[:3, 2]
    look = T[:3, 3] + (GZ - T[:3, 3][2]) / z[2] * z
    assert lo <= look[0] <= hi and abs(look[1]) < 1e-6 and T[2, 3] - GZ > 0.25


@pytest.mark.parametrize("cls", list(VS.CLASS_INFO))
def test_observe_accuracy_full_view(cls):
    t = VS.CLASS_INFO[cls][3]
    for off in [(0, 0), (0.03, 0.02), (-0.05, -0.03)]:
        for yaw in (0.0, 0.7):
            r = SimRobot(cls, (t[0] + off[0], t[1] + off[1]), yaw_obj=yaw)
            o = VS.observe(*r.wrist_view(), cls)
            assert o.ok, (off, yaw, o.reason)
            if not o.partial:
                assert err(r, o.xy) < 0.01, (off, yaw)


def test_observe_uses_the_fitted_floor_not_a_fixed_height():
    """Body 3 deg tilted and standing 2 cm higher: a fixed floor at base z -0.11 would be wrong by cm; the fitted plane is right."""
    t = VS.CLASS_INFO["small_box"][3]
    for base_z, tilt in ((0.13, (0.0, 0.0)), (0.11, (0.0, 0.05)), (0.12, (0.04, -0.03))):
        r = SimRobot("small_box", t, base_z=base_z, tilt=tilt)
        o = VS.observe(*r.wrist_view(), "small_box")
        assert o.ok and err(r, o.xy) < 0.015 and o.height == pytest.approx(0.05, abs=0.01)
        n, d = o.floor
        assert -d / n[2] == pytest.approx(-base_z, abs=0.02)        # fitted floor height in base_link


def test_observe_with_depth_noise():
    t = VS.CLASS_INFO["can"][3]
    for s in range(3):
        r = SimRobot("can", (t[0] + 0.02, t[1]), seed=s, depth_noise=0.004)
        o = VS.observe(*r.wrist_view(), "can")
        assert o.ok and err(r, o.xy) < 0.015


def test_box_from_depth_finds_the_object_without_a_detector():
    t = VS.CLASS_INFO["mini_cone"][3]
    r = SimRobot("mini_cone", t)
    o = VS.observe(*r.wrist_view(), "mini_cone")
    assert o.ok and o.box is not None


@pytest.mark.parametrize("ang", np.linspace(0, 2 * math.pi, 8, endpoint=False))
@pytest.mark.parametrize("cls", ["small_box", "can"])
def test_servo_from_nav2_error_12cm(cls, ang):
    t = VS.CLASS_INFO[cls][3]
    r = SimRobot(cls, (t[0] + 0.12 * math.cos(ang), t[1] + 0.12 * math.sin(ang)), seed=int(ang * 10))
    res = VS.VisualServo(r, arm_reachable=arm_ok).run(cls, prior_xy=(t[0] + 0.12 * math.cos(ang), t[1] + 0.12 * math.sin(ang)))
    assert res.ok, (res.code, res.log)
    assert res.steps <= 3
    true_err = err(r, t)
    assert true_err <= 0.02 + 1e-9 if res.code == "ALIGNED" else arm_ok(cls, r.obj_rel())   # ground truth, not the estimate


def test_success_never_claimed_with_a_wrong_position():
    for seed in range(6):
        t = VS.CLASS_INFO["mini_extinguisher"][3]
        r = SimRobot("mini_extinguisher", (t[0] + 0.07, t[1] - 0.06), seed=seed, step_gain=(0.7, 1.3), step_noise=0.01)
        res = VS.VisualServo(r, arm_reachable=arm_ok).run("mini_extinguisher", prior_xy=(t[0] + 0.07, t[1] - 0.06))
        if res.ok:
            assert err(r, t) <= 0.025 or arm_ok("mini_extinguisher", r.obj_rel())


def test_target_lost_without_prior_and_recovered_with_prior():
    t = VS.CLASS_INFO["small_box"][3]
    far = (t[0] - 0.40, t[1])                                         # outside the wrist view (it ends ~0.53 m behind)
    r = SimRobot("small_box", far)
    assert VS.VisualServo(r).run("small_box").code == "TARGET_LOST"
    r = SimRobot("small_box", far, step_gain=(1.0, 1.0), step_noise=0.0)
    res = VS.VisualServo(r, VS.ServoParams(max_tries=9, timeout_s=200), arm_ok).run("small_box", prior_xy=far)
    assert res.ok and any(e.get("why") == "lost: toward prior" for e in res.log)


def test_too_close_steps_away():
    t = VS.CLASS_INFO["small_box"][3]
    r = SimRobot("small_box", (-0.20, 0.0))                          # right behind the body, between the rear legs
    res = VS.VisualServo(r, arm_reachable=arm_ok).run("small_box", prior_xy=(-0.20, 0.0))
    assert res.ok
    first = next(e for e in res.log if "step" in e)
    assert first["step"][0] > 0                                       # moved forward = away from the object


def test_occluder_is_not_taken_as_the_object():
    t = VS.CLASS_INFO["small_box"][3]

    def post(robot):                                                  # 25 cm tall thin post right next to the cube
        c, s = math.cos(robot.yaw), math.sin(robot.yaw)
        rx, ry = t[0] + 0.0, t[1] + 0.0
        return dict(kind="box", centre=(robot.x + c * rx - s * ry, robot.y + s * rx + c * ry, 0.125), size=(0.03, 0.03, 0.25), yaw=0.0)
    r = SimRobot("small_box", (t[0] + 0.30, t[1]), extra=[post])      # cube far away, post at the view centre
    res = VS.VisualServo(r, arm_reachable=arm_ok).run("small_box")
    assert not res.ok and res.code in ("OCCLUDED", "TARGET_LOST")


def test_not_aligned_when_steps_do_not_work():
    t = VS.CLASS_INFO["small_box"][3]
    r = SimRobot("small_box", (t[0] + 0.08, t[1]), step_gain=(0.1, 0.1), step_noise=0.0)   # the legs slip: 10 % of each step
    res = VS.VisualServo(r, arm_reachable=arm_ok).run("small_box", prior_xy=(t[0] + 0.08, t[1]))
    assert not res.ok and res.code == "NOT_ALIGNED" and res.steps == 3 and res.residual > 0.02


def test_timeout():
    t = VS.CLASS_INFO["small_box"][3]
    r = SimRobot("small_box", (t[0] + 0.1, t[1]))
    res = VS.VisualServo(r, VS.ServoParams(timeout_s=1.0), arm_ok).run("small_box", prior_xy=(t[0] + 0.1, t[1]))
    assert res.code == "TIMEOUT"


def test_step_limit():
    t = VS.CLASS_INFO["can"][3]
    r = SimRobot("can", (t[0] + 0.12, t[1] + 0.05))
    res = VS.VisualServo(r, arm_reachable=arm_ok).run("can", prior_xy=(t[0] + 0.12, t[1] + 0.05))
    assert all(math.hypot(*e["step"]) <= 0.06 + 1e-3 for e in res.log if "step" in e)      # log rounded to 0.1 mm
