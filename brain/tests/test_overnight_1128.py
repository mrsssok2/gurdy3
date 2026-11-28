"""scripts/overnight_1128.sh in DRY_RUN (stubs: no Docker / Gazebo / training): block order, cone-stage failure, the switch
decision, RESUME after an interruption (finished fetch tries + a made decision are not redone), redaction; and the
decision rule of scripts/p9b_v32_decide.py."""
import importlib.util
import json
import os
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
TOK = "dryTOKEN_" + "q8Wn3Lp7Zk2Rt5Vx9Ys4Mb1"


def run(tmp_path, out=None, **env):
    out = out or tmp_path / "out"
    e = {**os.environ, "DRY_RUN": "1", "DRY_TOKEN": TOK, **{k: str(v) for k, v in env.items()}}
    p = subprocess.run(["bash", str(ROOT / "scripts/overnight_1128.sh"), "--out", str(out)], cwd=ROOT, env=e,
                       capture_output=True, text=True, timeout=300)
    assert p.returncode == 0, p.stdout[-2000:] + p.stderr[-2000:]
    ev = [json.loads(x) for x in (out / "events.jsonl").read_text().splitlines()]
    return out, ev, (out / "summary.md").read_text(), (out / "run.log").read_text()


def tries(out):
    return [json.loads(x) for x in (out / "fetch/tries.jsonl").read_text().splitlines()]


def no_token(out):
    for f in out.rglob("*"):
        if f.is_file():
            assert TOK.encode() not in f.read_bytes(), f


def test_full_flow_cone_then_fetch_x15(tmp_path):
    out, ev, md, log = run(tmp_path)
    assert [e["block"] for e in ev if e["type"] == "block_start"] == ["cone", "fetch"]
    st = [json.loads(x)["stage"] for x in (out / "cone/stages.jsonl").read_text().splitlines()]
    assert st == ["make", "render", "train", "eval", "report", "decide"]
    assert len(tries(out)) == 15 and all(t["status"] == "ran" for t in tries(out))
    assert "**all** | **15/15**" in md and "Decision: runtime detector = `det_v31_s`" in md and "kept det_v31_s" in md
    assert "det_v32_s on COLOURED cones (v3.2 test sets) | 83.0 %" in md
    no_token(out)
    assert 'DEFAULT_MODEL = "det_v31_s.onnx"' in (ROOT / "ros2_ws/src/gurdy_perception/gurdy_perception/detector_node.py").read_text()


def test_switch_decision_is_reported(tmp_path):
    out, ev, md, log = run(tmp_path, DRY_CONE_DECISION="switch", SKIP_FETCH=1)
    assert "Decision: runtime detector = `det_v32_s`** (SWITCHED" in md


def test_a_failed_cone_stage_stops_the_block_but_not_the_fetch(tmp_path):
    out, ev, md, log = run(tmp_path, DRY_CONE_FAIL="train")
    skip = next(e for e in ev if e["type"] == "block_skip")
    assert skip["block"] == "cone" and skip["at"] == "train" and "stage train failed" in skip["reason"]
    assert "cone train: exit code 1" in md and not (out / "cone/decision.json").exists()
    assert len(tries(out)) == 15


def test_resume_skips_finished_tries_and_the_decision(tmp_path):
    cone = tmp_path / "cone_outd"
    out, ev, md, log = run(tmp_path, CONE_OUTD=cone, DRY_START_FAIL="4,5,6")       # 3 tries, then 3 failed starts: block skipped
    assert [t["status"] for t in tries(out)] == ["ran"] * 3 + ["start_failed"] * 3
    assert (cone / "decision.json").exists()
    out, ev, md, log = run(tmp_path, out=out, CONE_OUTD=cone)                       # same --out: resume
    assert "a) already decided (resume)" in log and log.count("a) make rc=") == 1  # no cone stage ran again
    for i in ("small_box_1", "small_box_2", "small_box_3"):
        assert f"b) {i}: done before (resume), skipped" in log
    ran = {t["id"] for t in tries(out) if t["status"] == "ran"}
    assert len(ran) == 15
    assert "started 2 time(s) (RESUMED" in md and "**all** | **15/15**" in md
    no_token(out)


def test_deadline_skips_the_cone_block(tmp_path):
    out, ev, md, log = run(tmp_path, DEADLINE_H="1.0", SKIP_FETCH=1)
    assert any(e["type"] == "block_skip" and e["block"] == "cone" and e["reason"] == "deadline" for e in ev)


# ---- scripts/p9b_v32_decide.py
def decide_mod():
    spec = importlib.util.spec_from_file_location("p9b_v32_decide", ROOT / "scripts/p9b_v32_decide.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


@pytest.mark.parametrize("acc,new,ref,onnx,switch", [
    (True, 0.80, 0.85, True, True),          # 0.80 >= 0.9 x 0.85 = 0.765
    (True, 0.70, 0.85, True, False),         # cone recall not recovered
    (False, 0.85, 0.85, True, False),        # acceptance failed
    (True, 0.85, 0.85, False, False),        # no ONNX
    (True, None, 0.85, True, False),         # no measurement = no switch
])
def test_decision_rule(acc, new, ref, onnx, switch):
    D = decide_mod()
    d = D.decide({"det_v32_s": acc}, {"recall_all": new}, {"recall_all": ref}, onnx)
    assert d["switch"] is switch and d["model"] == ("det_v32_s" if switch else "det_v31_s")
    assert bool(d["why_not"]) is (not switch)


def test_apply_switches_and_keeps_the_rollback(tmp_path, monkeypatch):
    D = decide_mod()
    node = tmp_path / "detector_node.py"
    node.write_text('x = 1\nDEFAULT_MODEL = "det_v31_s.onnx"  # old\ny = 2\n')
    monkeypatch.setattr(D, "NODE", node)
    assert D.apply("det_v31_s") is False and "det_v31_s.onnx\"  # old" in node.read_text()
    assert D.apply("det_v32_s") is True
    txt = node.read_text()
    assert 'DEFAULT_MODEL = "det_v32_s.onnx"' in txt and "rollback: model:=det_v31_s.onnx" in txt and txt.endswith("y = 2\n")
