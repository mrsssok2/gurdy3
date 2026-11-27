"""Wrist-view yaw (gurdy_core.wrist_yaw) on synthetic wrist frames (tests/servo_sim.py: arm view pose, synth_depth, floor
at base_link z -0.11). The sim robot's yaw is 0.3 rad, so an object at world yaw 0.3 + a has base_link yaw a."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from servo_sim import SOLIDS, SimRobot  # noqa: E402

from gurdy_core import wrist_yaw as W  # noqa: E402
from gurdy_core.visual_servo import CLASS_INFO  # noqa: E402

ROBOT_YAW = 0.3


def view(cls, deg, noise=0.0, seed=0, rel=None, **kw):
    r = SimRobot(cls, rel if rel is not None else CLASS_INFO[cls][3], yaw_obj=ROBOT_YAW + math.radians(deg), depth_noise=noise,
                 seed=seed, **kw)
    d, K, T = r.wrist_view()
    return r, d, K, T


def err_deg(est, deg, cls):
    return math.degrees(W.circ_err(est.yaw, math.radians(deg), W.PERIOD[cls]))


def test_circ_err():
    assert W.circ_err(0.01, math.pi / 2 - 0.01, math.pi / 2) == pytest.approx(0.02)
    assert W.circ_err(0.1, math.pi + 0.1, math.pi) == pytest.approx(0.0, abs=1e-12)
    assert W.circ_err(0.0, math.pi / 4, math.pi / 2) == pytest.approx(math.pi / 4)


@pytest.mark.parametrize("deg", range(0, 91, 5))
@pytest.mark.parametrize("noise", [0.0, 0.02])
def test_cube_rotations_0_to_90(deg, noise):
    r, d, K, T = view("small_box", deg, noise, seed=deg)
    e = W.estimate_yaw(d, K, T, "small_box", xy_hint=r.obj_rel())
    assert e.method == "rect" and not e.partial
    assert err_deg(e, deg, "small_box") <= 3.0
    assert e.confidence >= 0.8 and 0.0 <= e.yaw < math.pi / 2


@pytest.mark.parametrize("deg", range(0, 180, 10))
@pytest.mark.parametrize("noise", [0.0, 0.05])
def test_toolbox_handle_rotations_0_to_180(deg, noise):
    r, d, K, T = view("mini_toolbox", deg, noise, seed=deg)
    e = W.estimate_yaw(d, K, T, "mini_toolbox", xy_hint=r.obj_rel())
    assert e.method == "handle" and e.period == pytest.approx(math.pi)
    assert err_deg(e, deg, "mini_toolbox") <= 2.0 and e.confidence >= 0.8


@pytest.mark.parametrize("deg", [0, 35, 70, 125])
def test_toolbox_without_handle_points_uses_the_body_top(deg):
    r, d, K, T = view("mini_toolbox", deg)
    r.solids = r.solids[:1]                                          # body only (handle hidden / broken off)
    d, K, T = r.wrist_view()
    e = W.estimate_yaw(d, K, T, "mini_toolbox", xy_hint=r.obj_rel())
    assert e.method == "body" and err_deg(e, deg, "mini_toolbox") <= 3.0 and e.confidence > 0.5


def test_heavy_noise_and_body_tilt():
    for deg in (10, 40, 80):
        r, d, K, T = view("small_box", deg, noise=0.05, seed=deg, tilt=(0.03, -0.02))
        e = W.estimate_yaw(d, K, T, "small_box", xy_hint=r.obj_rel())
        assert err_deg(e, deg, "small_box") <= 4.0, deg
        r, d, K, T = view("mini_toolbox", deg, noise=0.05, seed=deg, tilt=(0.03, -0.02))
        e = W.estimate_yaw(d, K, T, "mini_toolbox", xy_hint=r.obj_rel())
        assert err_deg(e, deg, "mini_toolbox") <= 3.0, deg


@pytest.mark.parametrize("cls", ["small_box", "mini_toolbox"])
def test_partial_view_is_flagged_and_confidence_halved(cls):
    zone = np.array(CLASS_INFO[cls][3])
    _, d, K, T = view(cls, 20)
    full = W.estimate_yaw(d, K, T, cls, xy_hint=None)
    for dy in np.arange(0.0, 0.40, 0.01):                            # slide the object sideways until its top touches the border
        r, d, K, T = view(cls, 20, rel=zone + [0.0, dy])
        e = W.estimate_yaw(d, K, T, cls, xy_hint=r.obj_rel())
        if e.partial:
            assert e.confidence <= 0.5 * full.confidence + 1e-9
            assert "border" in e.reason
            return
    pytest.fail("never partial")


@pytest.mark.parametrize("cls", W.SYMMETRIC)
def test_symmetric_classes_need_no_yaw(cls):
    r, d, K, T = view(cls, 30)
    e = W.estimate_yaw(d, K, T, cls, xy_hint=r.obj_rel())
    assert e.yaw is None and e.confidence == 1.0 and e.method == "symmetric"


def test_empty_floor_gives_no_yaw():
    r, d, K, T = view("small_box", 0)
    r.solids = []
    d, K, T = r.wrist_view()
    e = W.estimate_yaw(d, K, T, "small_box", xy_hint=r.obj_rel())
    assert e.yaw is None and e.confidence == 0.0 and e.method == "none"


def test_without_hint_and_with_a_distractor_nearby():
    r, d, K, T = view("small_box", 25)
    assert err_deg(W.estimate_yaw(d, K, T, "small_box"), 25, "small_box") <= 3.0
    ox, oy = r.obj
    c, s = math.cos(ROBOT_YAW), math.sin(ROBOT_YAW)
    side = (ox - s * 0.12, oy + c * 0.12)                            # another cube 12 cm to the side, other yaw
    r.solids.append(SOLIDS["small_box"](*side, ROBOT_YAW + 0.0))
    d, K, T = r.wrist_view()
    e = W.estimate_yaw(d, K, T, "small_box", xy_hint=r.obj_rel())
    assert err_deg(e, 25, "small_box") <= 3.0


def test_unknown_class():
    r, d, K, T = view("small_box", 0)
    assert W.estimate_yaw(d, K, T, "barrel").method == "none"
