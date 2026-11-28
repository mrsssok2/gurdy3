"""scripts/start_web_robot.sh: with FETCH=true the detector sighting log is ON by default (/tmp/chat/sightings.jsonl,
kept on the host by CHAT_LOG_DIR), DETECTOR_LOG=false turns it off, FETCH=false never adds it (2026-11-28)."""
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]


def launch_args(fetch, detector_log=None):
    lines = (ROOT / "scripts/start_web_robot.sh").read_text().splitlines()
    i = next(k for k, ln in enumerate(lines) if ln.strip().startswith('FETCH_LAUNCH=""'))
    snippet = "\n".join(lines[i:i + 3]) + '\necho "$FETCH_LAUNCH"'
    env = {"PATH": "/usr/bin:/bin", "FETCH": fetch, "VLN": "false"}
    if detector_log is not None:
        env["DETECTOR_LOG"] = detector_log
    return subprocess.run(["bash", "-c", snippet], env=env, capture_output=True, text=True).stdout.strip()


@pytest.mark.parametrize("fetch,dl,expect", [("true", None, True), ("true", "true", True), ("true", "false", False),
                                             ("false", None, False)])
def test_sighting_log_default_on_for_fetch_runs(fetch, dl, expect):
    out = launch_args(fetch, dl)
    assert ("detector_log:=/tmp/chat/sightings.jsonl" in out) == expect
    assert ("arm:=true object3d:=true" in out) == (fetch == "true")


def test_env_is_passed_into_the_container():
    s = (ROOT / "scripts/start_web_robot.sh").read_text()
    assert "DETECTOR_LOG=${DETECTOR_LOG:-true}" in s and "-e DETECTOR_LOG" in s
