"""Wrist alignment with the body-camera map prior (2026-11-28 fetch traces: take r2, smoke_1128c can_1, smoke_1128d can_2 --
the servo followed 'objects' 9-11 cm off the can and then judged the view implausible; the can was within 2.3 cm of the
grasp zone by ground truth). Synthetic wrist frames (tests/servo_sim.py). The real frames of those failures were not saved;
alignment frames are saved from now on (fetch: servo_frames/)."""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from servo_sim import SimRobot  # noqa: E402

from gurdy_core import arm as A  # noqa: E402
from gurdy_core import visual_servo as VS  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
ARM = A.load_arm()
TARGET = VS.CLASS_INFO["can"][3]


def arm_ok(cls, xy):
    from gurdy_core import grasp_test as G
    from gurdy_core.robot import load_robot
    return G.grasp_plan(ARM, load_robot(), cls, np.asarray(xy, float)).ok


def decoy(dx, dy, h=0.12):
    """A can-sized cylinder at (dx, dy) from the can in the ROBOT frame (world = robot yaw applied)."""
    def mk(r):
        c, s = math.cos(r.yaw), math.sin(r.yaw)
        return dict(kind="cylinder", centre=(r.obj[0] + c * dx - s * dy, r.obj[1] + s * dx + c * dy, h / 2), radius=0.033, height=h)
    return mk


def obs_of(r, prior=None):
    d, K, T = r.wrist_view()
    return VS.observe(d, K, T, "can", None, VS.ServoParams(), prior_xy=prior)


def test_blob_nearest_the_prior_not_the_image_centre():
    """Can at the grasp zone edge (-0.33, -0.06), a decoy 9 cm beside it, nearer the image centre (as in the traces)."""
    rel = (-0.33, -0.06)
    r = SimRobot("can", rel, yaw_obj=0.0, extra=(decoy(0.0, 0.09),))
    old = obs_of(r)
    assert old.ok and math.hypot(old.xy[0] - rel[0], old.xy[1] - rel[1]) > 0.07      # the old picker took the decoy
    new = obs_of(r, prior=(rel[0] + 0.02, rel[1] - 0.01))                            # map prior 2 cm off
    assert new.ok and math.hypot(new.xy[0] - rel[0], new.xy[1] - rel[1]) < 0.01


def test_servo_aligns_on_the_can_with_a_decoy_in_view():
    rel = (-0.33, -0.06)
    r = SimRobot("can", rel, yaw_obj=0.0, extra=(decoy(0.0, 0.09),), step_noise=0.0)
    res = VS.VisualServo(r, arm_reachable=arm_ok).run("can", prior_xy=(rel[0] + 0.02, rel[1] - 0.01))
    assert res.ok and float(np.hypot(*(r.obj_rel() - np.array(res.obs.xy)))) < 0.01
    assert all(abs(e.get("d_prior") or 0) <= VS.ServoParams().prior_gate for e in res.log if "obs" in e)


def test_one_bad_view_is_re_looked_after_backing_off():
    """The first view is contaminated (a tall thing next to the can, gone at the second look): back off + re-look."""
    rel = (-0.34, 0.0)
    r = SimRobot("can", rel, yaw_obj=0.0, step_noise=0.0)
    real = r.wrist_view
    n = {"k": 0}
    pole = decoy(0.0, 0.0, h=0.30)                                  # a 30 cm thing right ON the can's spot: 'too tall'

    def view():
        n["k"] += 1
        if n["k"] == 1:
            keep = r.solids
            r.solids = keep + [pole(r)]
            try:
                return real()
            finally:
                r.solids = keep
        return real()
    r.wrist_view = view
    res = VS.VisualServo(r, arm_reachable=arm_ok).run("can", prior_xy=rel)
    assert res.ok, res.log
    assert any("back off" in (e.get("why") or "") for e in res.log)


def test_two_agreeing_wrist_views_beat_a_wrong_map_prior():
    rel = (-0.35, 0.0)
    r = SimRobot("can", rel, yaw_obj=0.0, step_noise=0.0)
    res = VS.VisualServo(r, arm_reachable=arm_ok).run("can", prior_xy=(rel[0] + 0.09, rel[1]))   # map 9 cm off
    assert res.ok
    assert any("two wrist views agree" in str(e.get("obs")) for e in res.log)


def test_no_prior_keeps_the_old_behaviour():
    rel = (-0.35, 0.02)
    r = SimRobot("can", rel, yaw_obj=0.0, step_noise=0.0)
    res = VS.VisualServo(r, arm_reachable=arm_ok).run("can")
    assert res.ok


@pytest.mark.parametrize("trace,align_file", [
    ("r2", "docs/results/data/demo_bring_1128_r2/raw/chat/attempts_align.jsonl"),
    ("c/can_1", "docs/results/data/smoke_1128c/can_1/fetch/can_1/chat/attempts_align.jsonl"),
    ("d/can_2", "docs/results/data/smoke_1128d/can_2/fetch/can_1/chat/attempts_align.jsonl")])
def test_recorded_off_estimates_would_be_suspect(trace, align_file):
    """Replay of what the 3 failed traces recorded: residuals of 5-11 cm (wrist estimate vs the grasp zone) while ground truth
    had the can within 2.3 cm of the zone -> the estimates were >= 3 cm off the can; the ones >= 9 cm exceed the prior gate."""
    f = ROOT / align_file
    if not f.exists():
        pytest.skip("trace not in this checkout")
    recs = [json.loads(x) for x in f.read_text().splitlines()]
    assert all(r["align"] in ("NOT_ALIGNED", "OCCLUDED") for r in recs)
    worst = max((r["residual"] or 0.0) for r in recs)
    gt_zone_err = 0.023                                              # can vs zone at the end, all 3 traces <= 2.3 cm
    assert worst - gt_zone_err >= 0.027                              # the wrist estimate was off the can by >= 2.7 cm
    if worst >= 0.09:
        assert worst - gt_zone_err > VS.ServoParams().prior_gate - 0.01
