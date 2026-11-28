"""docs/RESULTS.md: every number traces to a committed file. For each table row, every standalone number in the result / value / n /
note cells must appear as a standalone number in the cited source line(s) (or in the value / n re-computed from the cited JSON
files); 'not run yet' rows carry no number at all; there are no numbers outside the tables (except the title); the dates are the
sources' last-commit dates; and the file is not stale (the generator reproduces it)."""
import importlib.util
import re
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
MDF = ROOT / "docs/RESULTS.md"
spec = importlib.util.spec_from_file_location("results_summary", ROOT / "scripts/results_summary.py")
RS = importlib.util.module_from_spec(spec)
spec.loader.exec_module(RS)
GIT = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--git-dir"], capture_output=True).returncode == 0
NUM = re.compile(r"(?<![\w.])(?<![A-Za-z]-)\d+(?:\.\d+)?(?![\w.]*[A-Za-z_]|\.\d)")      # not inside identifiers (v1.1, sft-400, mAP50-95)
LINES = MDF.read_text().splitlines()
ROWS = [line for line in LINES if line.startswith("| ") and not line.startswith("| result |") and not line.startswith("|---")]


def cells(row):
    c = [x.strip() for x in row.strip().strip("|").split(" | ")]
    assert len(c) == 7, row
    return dict(zip(("result", "value", "n", "split", "source", "date", "note"), c))


def nums(text):
    return set(NUM.findall(text.replace("**", "")))


def source_texts(c):
    out = []
    for sha in re.findall(r"`commit ([0-9a-f]{7,40})`", c["source"]):          # a commit message line
        if not GIT:
            pytest.skip("no git")
        out.append(subprocess.run(["git", "-C", str(ROOT), "log", "-1", "--format=%B", sha], capture_output=True, text=True).stdout)
    for path, line in re.findall(r"`([^`:]+?)(?::(\d+))?`", c["source"]):
        if path.startswith("commit "):
            continue
        f = ROOT / path
        assert f.exists(), path
        if line:
            out.append(f.read_text(errors="replace").splitlines()[int(line) - 1])
    if "(sum over classes" in c["source"] or "(count of" in c["source"]:          # JSON: re-compute with the generator's own function
        entry = next(e for e in RS.ENTRIES if e[1] == c["result"])
        x = RS.extract(entry[3])
        out.append(f"{x['value']} {x['n']}")
    return out


def test_table_shape():
    assert len(ROWS) >= 25
    splits = {cells(r)["split"] for r in ROWS}
    assert splits <= {"held-out", "blind", "test split", "dev"} and {"held-out", "blind", "dev"} <= splits


@pytest.mark.parametrize("row", ROWS, ids=[cells(r)["result"][:60] for r in ROWS])
def test_every_number_traces_to_its_source(row):
    c = cells(row)
    if c["value"] == "**not run yet**":
        assert c["n"] == c["source"] == c["date"] == "-"
        assert not nums(c["result"] + " " + c["note"]), row
        return
    src = source_texts(c)
    assert src, row
    have = set().union(*(nums(t) for t in src))
    for k in ("result", "value", "n", "note"):
        missing = nums(c[k]) - have
        assert not missing, (k, missing, row, src)


def test_no_numbers_outside_the_tables():
    for line in LINES:
        if line.startswith("|") or line.startswith("# "):
            continue
        assert not nums(line), line


def test_held_out_and_dev_are_separate_sections():
    heads = [line for line in LINES if line.startswith("## ")]
    for h in heads:
        kinds = h.split(": ", 1)[1].split(" / ")
        assert not ("dev" in kinds and len(kinds) > 1), h


@pytest.mark.skipif(not GIT, reason="dates and staleness need git history")
def test_dates_and_not_stale():
    for r in ROWS:
        c = cells(r)
        if c["date"] != "-":
            entry = next(e for e in RS.ENTRIES if e[1] == c["result"])
            assert c["date"] == RS.extract(entry[3])["date"], r          # the newest last-commit date of all its sources
    assert subprocess.run(["python3", str(ROOT / "scripts/results_summary.py"), "--check"], capture_output=True).returncode == 0


def _slug(h):
    return re.sub(r"[^a-z0-9 -]", "", h.lower()).replace(" ", "-")


def test_readme_sections_link_to_results_and_hard_code_no_numbers():
    readme = (ROOT / "README.md").read_text()
    sec = readme.split("<!-- results-linked:start", 1)[1].split("<!-- results-linked:end -->", 1)[0]
    prose = re.sub(r"`[^`]*`|\]\([^)]*\)", " ", sec.split("-->", 1)[1])          # code spans and link targets are not prose
    assert not nums(prose), nums(prose)
    anchors = {_slug(line[3:]) for line in LINES if line.startswith("## ")}
    links = re.findall(r"\(docs/RESULTS\.md#([^)]+)\)", sec)
    assert links and all(a in anchors for a in links), (set(links) - anchors, anchors)
