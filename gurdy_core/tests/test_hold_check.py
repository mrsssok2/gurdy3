"""gurdy_core.hold_check (gripper + payload) replayed on RECORDED data of 2026-11-27/28:
* grasp_test coupled_1128: 9 real holds in Gazebo (cube / extinguisher / can x3) with fingers, lift-pose joint angles and arm
  efforts -> HOLDING;
* fetch smoke_1128e: the 3 can tries of the day (fingers 33 / 33 recorded; the fetch did not log efforts then) -> with no
  efforts UNKNOWN (keep holding), with the efforts of a real can hold -> HOLDING;
* fetch smoke_1128d can_3: 'closed on nothing' (fingers 0 / 0 recorded) + the empty arm -> MISSED;
* 'weight but fingers fully closed': fingers 0 / 0 + the efforts of a real can hold -> never holding (DISAGREE);
* overnight_1127 small_box_2: fingers slid to 40 / 10 mm -> not holding.
The empty-arm efforts are the arm model at the same pose (the model matched the measured holds within 0-28 %, Phase 9i: 0.4 %)."""
import json
from pathlib import Path

import pytest

from gurdy_core import arm as A
from gurdy_core import hold_check as HC

ROOT = Path(__file__).resolve().parents[2]
ARM = A.load_arm()
CAT = {"small_box": (0.05, 0.05), "mini_toolbox": (0.08, 0.02), "can": (0.06, 0.066), "mini_cone": (0.03, 0.045),
       "mini_extinguisher": (0.08, 0.05)}                                    # brain/fetch/catalog.py (mass, grip width)
GT_ORDER = ["small_box"] * 3 + ["mini_extinguisher"] * 3 + ["can"] * 3          # grasp_test coupled_1128 run order


def rows(p):
    p = ROOT / p
    if not p.exists():
        pytest.skip(f"{p} not in this checkout")
    return [json.loads(x) for x in p.read_text().splitlines() if x.strip()]


def grasp_holds():
    out = []
    for cls, r in zip(GT_ORDER, rows("docs/results/data/grasp_test/coupled_1128/tries.jsonl")):
        out.append((cls, r["plan"]["lift"][-1], r["effort_hold"], r["fingers_closed"], r["fingers_hold"]))
    return out


def empty(q):
    return A.joint_torques(ARM, q, 0.0)


def verdict(cls, q, eff, fc, fl):
    return HC.hold_verdict(ARM, q, eff, fc, fl, CAT[cls][1], CAT[cls][0])


def test_all_9_real_holds_are_holding():
    for cls, q, eff, fc, fl in grasp_holds():
        v = verdict(cls, q, eff, fc, fl)
        assert v.code == "HOLDING" and v.holding is True and 0.9 <= v.ratio <= 1.3, (cls, v)


def test_todays_3_fetch_can_tries():
    can_q, can_eff = next((q, e) for cls, q, e, *_ in grasp_holds() if cls == "can")
    for k in (1, 2, 3):
        a = rows(f"docs/results/data/smoke_1128e/can_{k}/fetch/can_1/chat/attempts.jsonl")[-1]
        fc, fl = a["fingers_closed"], a["fingers_lifted"]
        assert HC.fingers_state(fl, 0.066) == "width"
        v = verdict("can", can_q, None, fc, fl)                          # as recorded: no efforts -> cannot say
        assert v.holding is None and v.code == "UNKNOWN"
        v = verdict("can", can_q, can_eff, fc, fl)                       # with a real can-hold effort: holding
        assert v.code == "HOLDING"


def test_closed_on_nothing_is_missed():
    can_q = next(q for cls, q, *_ in grasp_holds() if cls == "can")
    a = rows("docs/results/data/smoke_1128d/can_3/fetch/can_1/chat/attempts.jsonl")[0]
    assert max(a["fingers_closed"]) < 1e-3
    v = verdict("can", can_q, empty(can_q), a["fingers_closed"], a["fingers_lifted"])
    assert v.code == "MISSED" and v.holding is False and v.payload == "no"


def test_weight_but_fingers_fully_closed_is_never_holding():
    can_q, can_eff = next((q, e) for cls, q, e, *_ in grasp_holds() if cls == "can")
    v = verdict("can", can_q, can_eff, [0.0, 0.0], [0.0, 0.0])
    assert v.holding is None and v.code == "DISAGREE" and "closed fully" in v.why


def test_fingers_at_width_without_weight_disagree():
    can_q = next(q for cls, q, *_ in grasp_holds() if cls == "can")
    v = verdict("can", can_q, empty(can_q), [0.033, 0.033], [0.033, 0.033])
    assert v.holding is None and v.code == "DISAGREE" and "does not feel its weight" in v.why


def test_overnight_slide_to_40_10_is_not_holding():
    a = rows("docs/results/data/overnight_1127/fetch/small_box_2/chat/attempts.jsonl")[0]
    q = next(q for cls, q, *_ in grasp_holds() if cls == "small_box")
    assert HC.fingers_state(a["fingers_lifted"], 0.05) == "wide"           # 40 / 10 mm (GT: never lifted)
    v = verdict("small_box", q, empty(q), a["fingers_closed"], a["fingers_lifted"])
    assert v.holding is False and v.code == "MISSED"


@pytest.mark.parametrize("cls", list(CAT))
def test_every_class_weight_is_resolvable_from_the_model(cls):
    """Even the 30 g mini cone gives a ratio of 1 with its weight and 0 without, at a real lift pose."""
    q = next(q for c, q, *_ in grasp_holds() if c == "can")
    m = CAT[cls][0]
    full = {j: v for j, v in A.joint_torques(ARM, q, m).items()}
    assert HC.payload_ratio(ARM, q, full, m) == pytest.approx(1.0, abs=1e-3)
    assert HC.payload_ratio(ARM, q, empty(q), m) == pytest.approx(0.0, abs=1e-3)


def test_unknown_inputs_are_never_negative():
    q = next(q for c, q, *_ in grasp_holds() if c == "can")
    assert verdict("can", q, None, None, None).holding is None
    assert verdict("can", None, None, [0.033, 0.033], [0.033, 0.033]).holding is None
