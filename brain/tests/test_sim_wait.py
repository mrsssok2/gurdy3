"""brain/ros/sim_wait.wait_sim: arm-motion waits in SIM time (2026-11-28 GUI demo take: RTF ~0.2, a 3 s trajectory took ~15 s
wall, the old `seconds + 10` wall timeout gave up -> "pre-grasp move failed" x3). Fake clocks, no ROS."""
from pathlib import Path

from brain.ros.sim_wait import wait_sim

ROOT = Path(__file__).resolve().parents[2]


class Clock:
    def __init__(self, rtf=1.0, paused_after=None):
        self.wall, self.rtf, self.paused_after = 0.0, rtf, paused_after

    def sleep(self, dt):
        self.wall += dt

    def wall_now(self):
        return self.wall

    def sim_now(self):
        s = self.wall * self.rtf
        return min(s, self.paused_after) if self.paused_after is not None else s


def test_rtf_02_trajectory_finishes_inside_a_sim_timeout():
    c = Clock(rtf=0.2)
    done = lambda: c.sim_now() >= 3.0                              # noqa: E731  3 s sim trajectory = 15 s wall
    assert wait_sim(done, c.sim_now, 13.0, wall_now=c.wall_now, sleep=c.sleep)
    assert 14.9 <= c.wall <= 15.2


def test_the_old_wall_timeout_would_have_failed_at_rtf_02():
    c = Clock(rtf=0.2)
    done = lambda: c.sim_now() >= 3.0                              # noqa: E731
    assert not wait_sim(done, lambda: None, 13.0, wall_now=c.wall_now, sleep=c.sleep)   # no sim clock = wall seconds
    assert 12.9 <= c.wall <= 13.1


def test_paused_sim_stops_at_the_wall_cap():
    c = Clock(rtf=1.0, paused_after=1.0)
    assert not wait_sim(lambda: False, c.sim_now, 5.0, wall_cap=30.0, wall_now=c.wall_now, sleep=c.sleep)
    assert 29.9 <= c.wall <= 30.1


def test_sim_timeout_still_ends_a_stuck_motion():
    c = Clock(rtf=0.2)
    assert not wait_sim(lambda: False, c.sim_now, 13.0, wall_now=c.wall_now, sleep=c.sleep)
    assert 64.9 <= c.wall <= 65.2                                  # 13 s sim at RTF 0.2


def test_effort_average_spans_half_a_second_of_sim_time_at_low_rtf():
    c = Clock(rtf=0.2)
    samples = []
    wait_sim(lambda: samples.append(c.sim_now()) and False, c.sim_now, 0.5, wall_cap=10.0, poll=0.05,
             wall_now=c.wall_now, sleep=c.sleep)
    assert samples[-1] - samples[0] >= 0.45 and len(samples) >= 40


def test_rclpy_port_uses_sim_waits_for_motions_settle_and_efforts():
    src = (ROOT / "brain/ros/rclpy_arm_port.py").read_text()
    assert src.count("self._done_sim(h.get_result_async()") == 2            # direct trajectory + MoveIt execution
    assert "def settle" in src and "wait_sim(close, self.sim_now, timeout" in src
    assert "wait_sim(sample, self.sim_now, 0.5" in src
