"""Gripper coupling (gurdy_control/CoupledGripperController, arm.yaml gripper_coupling): the same law simulated on the host
with the sim's rates (controller 500 Hz zero-order hold, physics 1 kHz, finger mass from arm.yaml, URDF joint damping
0.5 N s/m implicit) -- stability margin of the chosen gains, the sideways stiffness of the pair, and the squeeze unchanged."""
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
ARM = yaml.safe_load((ROOT / "ros2_ws/src/gurdy_description/config/arm.yaml").read_text())
M = ARM["links"]["finger"]["mass"]
DAMP = 0.5                       # gurdy_arm.xacro finger joint <dynamics damping="0.5"/>
LIM = ARM["joints"]["gripper_finger"]["upper"]


def simulate(kp, kd, u=0.0, side=0.0, q0=(0.02, 0.02), t_end=1.0, eff=10.0):
    """Two fingers (position = opening of each, 0..LIM); `side` N pushes the pair sideways (left opens, right closes)."""
    ql, qr = q0
    vl = vr = 0.0
    dt, ctrl_every = 0.001, 2
    tl = tr = 0.0
    peak = 0.0
    for k in range(int(t_end / dt)):
        if k % ctrl_every == 0:               # 500 Hz controller, held between updates
            c = kp * (ql - qr) + kd * (vl - vr)
            tl, tr = max(-eff, min(eff, u - c)), max(-eff, min(eff, u + c))
        fl, fr = tl + side, tr - side
        vl = (vl + dt * fl / M) / (1 + dt * DAMP / M)       # implicit joint damping (DART)
        vr = (vr + dt * fr / M) / (1 + dt * DAMP / M)
        ql, qr = ql + dt * vl, qr + dt * vr
        if not (0.0 <= ql <= LIM):
            ql, vl = min(max(ql, 0.0), LIM), 0.0
        if not (0.0 <= qr <= LIM):
            qr, vr = min(max(qr, 0.0), LIM), 0.0
        if k * dt > 0.5 * t_end:
            peak = max(peak, abs(ql - qr))
    return ql, qr, peak


def test_config_is_present_and_inside_the_stability_bounds():
    g = ARM["gripper_coupling"]
    dt = 1.0 / 500
    assert 2 * g["kp"] * dt ** 2 / M <= 0.25 and 2 * g["kd"] * dt / M <= 0.5
    assert g["effort_limit"] == ARM["joints"]["gripper_finger"]["effort"]


def test_chosen_gains_settle_an_offset_pair_and_keep_it_centred():
    g = ARM["gripper_coupling"]
    ql, qr, peak = simulate(g["kp"], g["kd"], q0=(0.035, 0.010))           # the overnight 40 / 10 mm reading
    assert abs(ql - qr) < 1e-4 and peak < 1e-3


def test_a_sideways_push_moves_the_pair_only_millimetres():
    g = ARM["gripper_coupling"]
    ql, qr, _ = simulate(g["kp"], g["kd"], side=1.0)                      # 1 N on the pair
    slide = (ql - qr) / 2
    assert slide == pytest.approx(1.0 / (2 * g["kp"]), rel=0.02)          # s = F / (2 kp): 4 mm; free fingers: to the limit
    ql0, qr0, _ = simulate(0.0, 0.0, side=1.0)
    assert ql0 == pytest.approx(LIM) and qr0 == pytest.approx(0.0, abs=1e-9)


def test_squeeze_is_unchanged_and_both_fingers_close_together():
    g = ARM["gripper_coupling"]
    ql, qr, _ = simulate(g["kp"], g["kd"], u=-3.0, q0=(0.04, 0.03), t_end=2.0)
    assert ql == pytest.approx(0.0, abs=1e-6) and qr == pytest.approx(0.0, abs=1e-6)


def test_gains_far_beyond_the_bound_would_be_unstable():
    """The bound matters: 4x the stiffness limit oscillates at the controller rate instead of settling."""
    _, _, peak = simulate(4 * 156 * 4, 0.0, q0=(0.021, 0.019))
    assert peak > 0.002
