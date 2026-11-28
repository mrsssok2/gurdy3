"""Is the gripper holding the object? (2026-11-28) Primary check = GRIPPER + PAYLOAD, both measured, no camera:

1. fingers: after the squeeze AND after the lift, both fingers stalled at about the object's half width (coupled rack
   gripper: each finger = half the opening) -- not fully closed (nothing between them) and not wide open;
2. payload: the arm's static joint efforts at the lift pose (shoulder + elbow, effort-controlled arm: the integral term carries
   gravity) compared with the arm model (gurdy_core.arm.joint_torques) WITHOUT and WITH the object's mass:
   ratio = (measured - empty) / (with object - empty) ~ 1 when it is in the gripper, ~ 0 when not.
   Calibration (grasp_test coupled_1128, 9 holds: cube / extinguisher / can): ratio 0.97-1.28 on both joints; Phase 9i:
   measured 0.208 N m vs model 0.2085 (empty 0.139).

holding = fingers at width AND payload present. Fingers fully closed is NEVER holding (whatever the efforts say).
Anything else is a measured negative ("missed") or a disagreement / missing data ("unconfirmed": keep the gripper closed and
say so). The wrist view is only a secondary check, used when the old spot is actually visible (wrist_yaw.standing_check).
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

from . import arm as A

FINGER_CLOSED = 0.004            # m per finger: below this nothing is between them
WIDTH_TOL = 0.012                # m per finger around half the object width
PAYLOAD_YES, PAYLOAD_NO, PAYLOAD_MAX = 0.5, 0.25, 2.5
JOINTS = ("arm_shoulder", "arm_elbow")


@dataclass
class HoldVerdict:
    holding: bool | None          # True holding, False not holding (measured), None = cannot say (keep closed)
    code: str                     # HOLDING | MISSED | DISAGREE | UNKNOWN
    fingers: str                  # width | closed | wide | unknown
    payload: str                  # yes | no | ambiguous | unknown
    ratio: float | None
    why: str

    def to_dict(self):
        return asdict(self)


def fingers_state(f, width: float) -> str:
    if f is None or len(f) < 2 or any(v is None or v != v for v in f):
        return "unknown"
    if min(f) <= FINGER_CLOSED:
        return "closed"
    half = width / 2
    if all(abs(v - half) <= WIDTH_TOL for v in f):
        return "width"
    return "wide" if max(f) > half + WIDTH_TOL else "closed" if max(f) < half - WIDTH_TOL else "wide"


def payload_ratio(arm, q, efforts: dict | None, mass: float) -> float | None:
    if q is None or not efforts or any(efforts.get(j) is None or efforts[j] != efforts[j] for j in JOINTS):
        return None
    t0, tm = A.joint_torques(arm, q, 0.0), A.joint_torques(arm, q, mass)
    den = sum(tm[j] - t0[j] for j in JOINTS)
    if den <= 1e-6:
        return None
    return round((sum(float(efforts[j]) for j in JOINTS) - sum(t0[j] for j in JOINTS)) / den, 3)


def payload_state(r: float | None) -> str:
    if r is None:
        return "unknown"
    if PAYLOAD_YES <= r <= PAYLOAD_MAX:
        return "yes"
    if r <= PAYLOAD_NO:
        return "no"
    return "ambiguous"


def hold_verdict(arm, q, efforts, f_closed, f_lifted, width: float, mass: float) -> HoldVerdict:
    fc, fl = fingers_state(f_closed, width), fingers_state(f_lifted, width)
    if "closed" in (fc, fl):
        fs = "closed"                    # fully closed at any point: nothing (stays) between them
    elif fl == "unknown":
        fs = "unknown"
    elif fl == "width" and fc in ("width", "unknown"):
        fs = "width"                     # stalled at the object's width through the lift
    else:
        fs = "wide"
    r = payload_ratio(arm, q, efforts, mass)
    ps = payload_state(r)
    if fs == "closed":
        if ps in ("yes", "ambiguous"):
            return HoldVerdict(None, "DISAGREE", fs, ps, r, "fingers closed fully but the arm feels a weight")
        return HoldVerdict(False, "MISSED", fs, ps, r, "fingers fully closed: nothing between them")
    if fs == "width" and ps == "yes":
        return HoldVerdict(True, "HOLDING", fs, ps, r, "")
    if fs == "width" and ps == "no":
        return HoldVerdict(None, "DISAGREE", fs, ps, r, "fingers at its width but the arm does not feel its weight")
    if fs == "wide" and ps == "no":
        return HoldVerdict(False, "MISSED", fs, ps, r, "fingers wider than the object and no weight")
    if fs == "wide" and ps == "yes":
        return HoldVerdict(None, "DISAGREE", fs, ps, r, "the arm feels a weight but the fingers are wider than the object")
    if fs == "unknown":
        return HoldVerdict(None, "UNKNOWN", fs, ps, r, "I couldn't read my gripper")
    return HoldVerdict(None, "UNKNOWN", fs, ps, r, "I couldn't measure the weight in my arm" if ps == "unknown"
                       else "the weight in my arm is unclear")
