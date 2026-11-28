"""Honesty rule for fetch replies: a reply may state a number ONLY if it was measured (or is a count the robot really did,
or came from the user's own words). Overnight 2026-11-27 found "residual 99 cm" = a placeholder for "never measured"
(brain/ros/fetch_robot.py fine_align). FakePort / FakeArmPort + the mock robot; no ROS."""
import json
import re

import pytest

from brain.fetch.machine import MAX_ALIGN, MAX_GRASP
from brain.fetch.mock import FetchMock
from brain.fetch.session import FetchSession
from brain.ros.fetch_robot import FetchConfig, align_log_path
from brain.tests.test_ros_fetch import rig
from gurdy_core import visual_servo as VS

NUM = re.compile(r"\d+(?:\.\d+)?")


def numbers(text):
    return NUM.findall(text)


def measured_numbers(rob, command):
    """Numbers a reply may contain: from the command, the attempt / try counts, and each measured servo residual (cm)."""
    ok = set(numbers(command)) | {str(MAX_ALIGN), str(MAX_GRASP)}
    for x in getattr(rob, "log", []):
        r = x.get("residual") if isinstance(x, dict) else None
        if r is not None:
            ok |= {f"{r * 100:.0f}", f"{r * 100:.1f}"}
    return ok


def servo_fails(monkeypatch, code, residual):
    def run(self, cls, prior_xy=None, box_fn=None):
        return VS.ServoResult(False, code, steps=3, residual=residual)
    monkeypatch.setattr(VS.VisualServo, "run", run)


@pytest.mark.parametrize("code", ["TARGET_LOST", "OCCLUDED", "TIMEOUT"])
def test_alignment_without_a_measurement_states_no_number(monkeypatch, code, tmp_path):
    servo_fails(monkeypatch, code, None)
    s, rob, arm, fp = rig()
    rob.cfg = FetchConfig(attempt_log=str(tmp_path / "attempts.jsonl"))
    r = s.handle("bring me the blue can")
    assert "line my gripper up" in r and "measure" in r
    assert "99" not in r and "cm" not in r
    assert set(numbers(r)) <= measured_numbers(rob, "bring me the blue can")
    assert not (tmp_path / "attempts.jsonl").exists()                       # no grasp attempt: attempt log untouched
    al = [r for r in (json.loads(x) for x in open(align_log_path(str(tmp_path / "attempts.jsonl")))) if "align" in r]
    assert len(al) == MAX_ALIGN and {a["align"] for a in al} == {code} and all(a["residual"] is None for a in al)


def test_alignment_with_a_measured_residual_reports_it(monkeypatch):
    servo_fails(monkeypatch, "NOT_ALIGNED", 0.047)
    s, rob, arm, fp = rig()
    r = s.handle("bring me the blue can")
    assert "residual 5 cm" in r                                              # measured: 4.7 cm
    assert set(numbers(r)) <= measured_numbers(rob, "bring me the blue can")


ROS_CASES = [None, {"miss": {"blue_can"}}, {"slip": {"blue_can"}}, {"drop_carry": {"blue_can"}}, {"jam": True}]


@pytest.mark.parametrize("faults", ROS_CASES)
@pytest.mark.parametrize("command", ["bring me the blue can", "pick up the blue can", "bring the blue can to room B",
                                     "get me the blue can"])
def test_no_ros_fetch_reply_contains_an_unmeasured_number(faults, command):
    s, rob, arm, fp = rig(faults)
    r = s.handle(command)
    assert set(numbers(r)) <= measured_numbers(rob, command), r


MOCK_FAULTS = [{}, {"not_found": {"blue_can"}}, {"unreachable": {"blue_can"}}, {"grasp_fail": {"blue_can": 3}},
               {"drop": {"blue_can"}}]


@pytest.mark.parametrize("faults", MOCK_FAULTS)
def test_no_mock_fetch_reply_contains_an_unmeasured_number(faults):
    """Mock: its fine_align residual (6 cm when unreachable) is the mock's simulated measurement, logged by the machine."""
    m = FetchMock.lab4_fetch(faults=faults)
    s = FetchSession(m)
    r = s.handle("bring me the blue can")
    allowed = set(numbers("bring me the blue can")) | {str(MAX_ALIGN), str(MAX_GRASP), "6"}
    assert set(numbers(r)) <= allowed, r
