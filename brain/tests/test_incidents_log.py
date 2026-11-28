"""docs/incidents/incidents.jsonl stays honest: every record has the fields, quotes exist verbatim where they say, commits / tests exist,
'full' evidence has a quote from the run's own logs, and the backfilled part matches scripts/incidents_backfill.py."""
import json
import re
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
LOG = ROOT / "docs/incidents/incidents.jsonl"
FIELDS = {"id", "date", "robot", "symptom", "logs", "root_cause", "evidence", "evidence_status", "fix_commit", "test_added"}
ROWS = [json.loads(line) for line in LOG.read_text().splitlines() if line.strip()]
GIT = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--git-dir"], capture_output=True).returncode == 0


def test_fields_ids_dates():
    ids = [r["id"] for r in ROWS]
    assert len(ids) == len(set(ids)) and len(ROWS) >= 60
    for r in ROWS:
        assert FIELDS <= set(r), r["id"]
        assert re.fullmatch(r"INC-\d{8}-\d{2}", r["id"]) and r["id"][4:12] == r["date"].replace("-", ""), r["id"]
        assert r["evidence"] and r["root_cause"] and r["symptom"] and r["evidence_status"] in ("full", "partial")


@pytest.mark.parametrize("r", ROWS, ids=[r["id"] for r in ROWS])
def test_quotes_are_verbatim(r):
    for e in r["evidence"]:
        parts = [p for p in e["text"].split("...") if p.strip()]
        if "file" in e:
            line = (ROOT / e["file"]).read_text(errors="replace").splitlines()[e["line"] - 1]
        else:
            if not GIT:
                pytest.skip("no git")
            line = subprocess.run(["git", "-C", str(ROOT), "log", "-1", "--format=%B", e["commit"]], capture_output=True, text=True).stdout
        assert all(p.strip() in line for p in parts), (r["id"], e)


@pytest.mark.parametrize("r", ROWS, ids=[r["id"] for r in ROWS])
def test_status_commits_tests_logs(r):
    logq = [e for e in r["evidence"] if "file" in e and re.search(r"(^docs/results/data/|\.log$|\.jsonl$)", e["file"])]
    assert (r["evidence_status"] == "full") == (bool(logq) and bool(r["logs"])), r["id"]
    for p in r["logs"]:
        assert p.startswith("...") or (ROOT / p).exists(), (r["id"], p)
    for t in r["test_added"] or []:
        assert (ROOT / t).exists(), (r["id"], t)
    if GIT:
        for h in r["fix_commit"] or []:
            assert subprocess.run(["git", "-C", str(ROOT), "cat-file", "-e", f"{h}^{{commit}}"], capture_output=True).returncode == 0, (r["id"], h)


@pytest.mark.skipif(not GIT, reason="the generator reads commit messages")
def test_backfill_is_reproducible():
    assert subprocess.run([sys.executable, str(ROOT / "scripts/incidents_backfill.py"), "--check"], capture_output=True).returncode == 0
