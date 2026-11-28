"""wrist_yaw.standing_check (grasp verification from the view pose, 2026-11-28): with a known floor (the servo's fit before the
grasp) it works when the in-view floor fit cannot; a spot the camera cannot SEE is UNKNOWN (None), never 0 = 'lifted'.
Synthetic wrist frames (tests/servo_sim.py)."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from servo_sim import SimRobot  # noqa: E402

from gurdy_core import wrist_yaw as W  # noqa: E402
from gurdy_core.visual_servo import CLASS_INFO  # noqa: E402

FLOOR = (np.array([0.0, 0.0, 1.0]), 0.11)          # base_link 0.11 m above the floor (SimRobot base_z)
R = 0.033 + 0.01


def rig(lifted=False, occluder=None):
    r = SimRobot("can", CLASS_INFO["can"][3])
    if lifted:
        r.solids = []
    if occluder:
        r.solids = r.solids + [occluder(r)]
    return r


def on_ray(r, xy_base, dist, size):
    """A box `dist` m in front of the lens on the ray to the base-frame floor point xy (the held object, close to the camera)."""
    Tw = r.T_world_base() @ r.T_bo
    cam = Tw[:3, 3]
    p = (r.T_world_base() @ np.array([xy_base[0], xy_base[1], -r.base_z, 1.0]))[:3]
    u = (p - cam) / np.linalg.norm(p - cam)
    c = cam + dist * u
    return dict(kind="box", centre=tuple(c), size=(size, size, size), yaw=0.0)


def check(r, **kw):
    d, K, T = r.wrist_view()
    return W.standing_check(d, K, T, r.obj_rel(), R, **kw)


def test_standing_object_counts_and_a_lifted_one_reads_zero():
    n, why = check(rig(), floor=FLOOR)
    assert n is not None and n > 50 and why == ""
    n, why = check(rig(lifted=True), floor=FLOOR)
    assert n == 0 and why == ""
    assert check(rig(lifted=True))[0] == 0                                  # in-view floor fit, nothing in the way: same


def test_view_blocked_completely_is_unknown():
    r = rig(lifted=True, occluder=lambda r: on_ray(r, r.obj_rel(), 0.10, 0.15))   # camera outside the box
    n, why = check(r, floor=FLOOR)
    assert n is None and ("valid depth points" in why or "not visible" in why)


def test_spot_hidden_behind_the_held_object_is_unknown_not_lifted():
    """The can is gone from the spot, but the held object hides exactly that spot: before the fix a known/fitted floor with
    nothing counted there read 0 -> 'lifted' although the camera never saw the spot."""
    occ = lambda r: on_ray(r, r.obj_rel(), 0.12, 0.10)                     # noqa: E731
    r = rig(lifted=True, occluder=occ)
    n, why = check(r, floor=FLOOR)
    assert n is None and "not visible" in why
    r2 = rig(lifted=False, occluder=occ)                                     # also when it still stands there: unknown
    assert check(r2, floor=FLOOR)[0] is None


def test_known_floor_works_when_most_of_the_floor_is_hidden():
    """A held object covering a large part of the view (in-view fit may fail) with the spot itself still visible."""
    def occ(r):
        rel = r.obj_rel()
        side = np.array([rel[0], rel[1] + 0.25])                             # aim beside the spot, not at it
        return on_ray(r, side, 0.10, 0.09)
    r = rig(lifted=True, occluder=occ)
    n, why = check(r, floor=FLOOR)
    assert n == 0 and why == ""


def test_wrapper_keeps_the_old_signature():
    r = rig()
    d, K, T = r.wrist_view()
    assert W.standing_points(d, K, T, r.obj_rel(), R) == W.standing_check(d, K, T, r.obj_rel(), R)[0]
    assert math.isfinite(W.standing_points(d, K, T, r.obj_rel(), R))
