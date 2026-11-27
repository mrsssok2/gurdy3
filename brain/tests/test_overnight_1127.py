"""scripts/overnight_1127.sh control flow in DRY_RUN (stubs: no Docker / Gazebo), its redaction, and the target plan."""
import json
import math
import os
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
TOK = "dryTOKEN_" + "x7Qp2Lm9Zr4Kt8Vw1Ys6Nb3"
PLAN = json.loads((ROOT / "docs/results/data/overnight_1127/plan.json").read_text())["placements"]


def run(tmp_path, *args, **env):
    out = tmp_path / "out"
    e = {**os.environ, "DRY_RUN": "1", "DRY_TOKEN": TOK, **{k: str(v) for k, v in env.items()}}
    p = subprocess.run(["bash", str(ROOT / "scripts/overnight_1127.sh"), *args, "--out", str(out)], cwd=ROOT, env=e,
                       capture_output=True, text=True, timeout=300)
    assert p.returncode == 0, p.stdout[-2000:] + p.stderr[-2000:]
    ev = [json.loads(x) for x in (out / "events.jsonl").read_text().splitlines()]
    return out, ev, (out / "summary.md").read_text(), p.stdout


def no_token_anywhere(out):
    for f in out.rglob("*"):
        if f.is_file():
            assert TOK.encode() not in f.read_bytes(), f
    assert not (ROOT / ".overnight_raw").exists() or not any((ROOT / ".overnight_raw").rglob("*"))   # raw logs deleted


def test_full_flow_skips_a_block_after_3_failed_starts_and_grades_from_ground_truth(tmp_path):
    out, ev, md, log = run(tmp_path, DRY_START_FAIL="3,4,5", DRY_FETCH_FAIL="small_box_2", DRY_CLAIM_ONLY="small_box_1",
                           DRY_WALK_FAIL="2", RUNS=2)
    assert [e["type"] for e in ev if e["type"].startswith("block")] == [
        "block_start", "block_skip", "block_end", "block_start", "block_end", "block_start", "block_end"]
    skip = next(e for e in ev if e["type"] == "block_skip")
    assert skip["block"] == "fetch" and skip["reason"] == "3 failed stack starts in a row" and skip["at"] == "mini_toolbox_3"
    tries = [json.loads(x) for x in (out / "fetch/tries.jsonl").read_text().splitlines()]
    assert [t["status"] for t in tries] == ["ran", "ran", "start_failed", "start_failed", "start_failed"]
    assert "| small_box_1 | small_box | 1.06 | no |" in md                 # claimed "I picked up" but GT: still on the floor
    assert "Robot claimed success where ground truth says no: 1." in md
    assert "fingers fully closed" in md                                    # small_box_2: the attempt's reason is reported
    assert md.count("| yes |") == 6                                       # A and B, 3 tries each
    assert "Walk runs: 5 with a result of 6 started." in md and "walk_fail" in md
    for blk in ("estimate a) fetch", "estimate b)", "estimate c) walking", "estimate total"):
        assert blk in log
    no_token_anywhere(out)


def test_smoke_runs_one_fetch_try_only(tmp_path):
    out, ev, md, log = run(tmp_path, "--smoke")
    tries = [json.loads(x) for x in (out / "fetch/tries.jsonl").read_text().splitlines()]
    assert len(tries) == 1 and tries[0]["id"] == "can_1" and tries[0]["status"] == "ran"
    assert not (out / "ab").exists() and not (out / "walk").exists()
    assert "| can_1 | can | 1.2 | yes |" in md
    no_token_anywhere(out)


def test_deadline_skips_units_that_could_overrun(tmp_path):
    out, ev, md, log = run(tmp_path, DEADLINE_H="0.1")                    # 6 min: no fetch try (25 min worst case) fits
    skips = {e["block"]: e["reason"] for e in ev if e["type"] == "block_skip"}
    assert skips == {"fetch": "deadline", "ab": "deadline", "walk": "deadline"}
    assert not (out / "fetch/tries.jsonl").exists()


def test_plan_placements_respect_the_rules():
    doors = json.loads((ROOT / "docs/results/data/lab4_fetch/lab4_fetch_gt.json").read_text())["doors"].values()
    assert len(PLAN) == 15 and sorted({p["class"] for p in PLAN}) == sorted(
        ["small_box", "mini_toolbox", "can", "mini_cone", "mini_extinguisher"])
    for p in PLAN:
        lo, hi = p["band_m"]
        assert 1.0 <= lo <= p["dist_m"] <= hi <= 2.5 and p["approach_headings"] >= 3
        assert min(math.hypot(p["x"] - dx, p["y"] - dy) for dx, dy in doors) >= 0.8


@pytest.mark.parametrize("cls", ["small_box", "mini_toolbox", "can", "mini_cone", "mini_extinguisher"])
def test_plan_commands_pick_the_planned_object(cls):
    """The plan's command, with the target moved to its planned spot, picks THAT object (fake robot)."""
    from brain.tests.test_ros_fetch import rig
    p = next(p for p in PLAN if p["class"] == cls)
    s, rob, arm, fp = rig()
    o = next(o for o in arm.objs if o["model"] == p["model"])
    o["x"], o["y"] = p["x"], p["y"]
    arm._sync()
    r = s.handle(p["command"])
    assert r.startswith("I picked up"), r
    assert arm.held is not None and arm.held["model"] == p["model"]
