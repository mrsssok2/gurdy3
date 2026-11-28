#!/usr/bin/env python3
"""Generate docs/RESULTS.md from COMMITTED result files only. Every number is copied (or, for JSON, computed) from a source file that
is cited next to it with its line, the date that file was last committed, and n. Nothing is estimated: a result that has no
committed file is listed as "not run yet" with no number.

  python3 scripts/results_summary.py            # write docs/RESULTS.md
  python3 scripts/results_summary.py --check    # exit 1 if docs/RESULTS.md is stale (CI step `results`)

Source kinds:
  MD(path, regex)    the first line of `path` matching `regex`; named groups `v` (the value) and `n` (sample size) are copied verbatim
  JS(name, glob)     a value computed from JSON result files by a named function below (re-computed by the test)
Splits: "held-out" (a suite committed before its first run / never tuned on), "blind" (written by the user, never seen by the
assistant before the run), "test split" (detector test images never trained on), "dev" (used while developing: numbers can be tuned).
Dates: `git log -1 --format=%cs -- <source>` (the day the source was last committed); a source with uncommitted changes gets today.
"""
from __future__ import annotations

import datetime as dt
import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/RESULTS.md"


def MD(path, rx, after=None):
    """The first line matching rx; with `after`, only lines after the first line matching that anchor (tables with several sections)."""
    return ("md", path, rx, after)


def GC(sha, rx):
    """A line of a commit MESSAGE (immutable); date = that commit's date."""
    return ("gc", sha, rx)


def JS(name, glob):
    return ("js", name, glob)


# (section, result, split, source, note). Labels and notes carry NO standalone numbers (the test checks that every number of a row
# appears in its cited source lines); n comes from the source (a group `n` of the value regex, or an extra MD as the source's 2nd item).
ENTRIES = [
 # ------------------------------------------------------------------ held-out / blind suites
 ("Held-out and blind suites", "lab4 v1 (Gazebo, gurdy-sft-400): mission success, mean (range) of the runs", "held-out",
  (MD("docs/results/phase5_heldout_lab4.md", r"^\| success rate \(all missions\) \|.*\| (?P<v>\d+ %) \| (?P<r>\d+ % - \d+ %) \|"),
   MD("docs/results/phase5_heldout_lab4.md", r"^\| missions passed \(of (?P<n>\d+)\) \|")), "n = missions per run"),
 ("Held-out and blind suites", "lab4 v2 (Gazebo): mission success, mean (range) of the runs", "held-out",
  MD("docs/results/phase5_heldout_lab4_v2.md", r"^\| success \|.*\| \*\*(?P<v>\d+ %)\*\* \| (?P<r>\d+-\d+ %) \|"), "n not stated in the source line"),
 ("Held-out and blind suites", "lab4 v3 (Gazebo): mission success, mean (range) of the runs", "held-out",
  (MD("docs/results/phase5_heldout_lab4_v3.md", r"^\| success \|.*\| \*\*(?P<v>\d+ %)\*\* \| (?P<r>\d+-\d+ %) \|"),
   MD("docs/results/phase5_heldout_lab4_v3.md", r"\((?P<n>\d+) missions in the lab4 world\)")),
  "held-out at run time; its failures were fixed afterwards (Phase 7c)"),
 ("Held-out and blind suites", "doorway_v1 (Gazebo, after the Nav2 fix): passed over all runs", "held-out",
  MD("docs/results/phase8a_nav2_doorways.md", r"mean 93 % \(range 93-93 %\), (?P<v>\d+ / (?P<n>\d+))\.\*\*"), "suite committed before the fix"),
 ("Held-out and blind suites", "redteam_blind_v1 (owner-worded), before the Phase 7b fixes", "blind",
  MD("docs/results/phase7b_redteam_fixes.md", r"blind (?P<v>\d+/(?P<n>\d+))\."), "mock robot"),
 ("Held-out and blind suites", "redteam_blind_v1 after the Phase 7b / 7c fixes: passed, unsafe executions", "blind",
  MD("docs/results/phase7c_overrefusal.md", r"^\| redteam_blind_v1 \|.*\| (?P<v>\d+ / (?P<n>\d+), 0 unsafe), 0 injections, 0 leaks \|$"), "mock robot"),
 ("Held-out and blind suites", "incident questions blind_v1 (user questions, offline replay): correct, picker off / on", "blind",
  (MD("docs/results/phase6e_blind.md", r"^\| \*\*blind_v1 \(before fix\)\*\* \| \*\*(?P<v>[\d.]+)\*\* .*\| \*\*(?P<w>[\d.]+)\*\* "),
   MD("docs/results/phase6e_blind.md", r"^\| suite \((?P<n>\d+) q\) \|")), "no robot in the replay"),
 ("Held-out and blind suites", "ood_v1 (casual commands, held out from training): % correct", "held-out",
  MD("docs/results/phase7c_overrefusal.md", r"^\| testset v1 / v1\.1 / ood_v1 \|.*\| \*\*\d+ / \d+ / (?P<v>\d+)\*\* \|$"), "offline; written by the assistant, so not blind"),
 ("Held-out and blind suites", "VLN blind suite (Gazebo)", "blind", None, "the user writes the suite; scripts/vln_eval.sh SUITE.yaml"),
 # ------------------------------------------------------------------ detector (YOLO) test splits
 ("Detector (YOLO) evaluations", "det_v1_s, synth_v1 test split: mAP50", "test split",
  MD("docs/results/phase4_detector.md", r"^\| det_v1_s \| (?P<v>0\.\d+) \|"), "synthetic Gazebo renders"),
 ("Detector (YOLO) evaluations", "det_v2_s, synth_v2 test split: mAP50 / mAP50-95", "test split",
  MD("docs/results/phase4c_v2.md", r"^\| mAP50 / mAP50-95, v2 test \| [\d.]+ / [\d.]+ \| (?P<v>0\.\d+ / 0\.\d+) \|"), ""),
 ("Detector (YOLO) evaluations", "det_v31_s (runtime default), small classes, synth_v31_body test: mAP50 vs acceptance", "test split",
  MD("docs/results/overnight_1127.md", r"^\| small classes mAP50, synth_v31_body test \| (?P<p>>= [\d.]+) \|.*\| (?P<v>0\.\d+ PASS) \|$"), ""),
 ("Detector (YOLO) evaluations", "det_v31_s, old classes, synth_v1 test: mAP50", "test split",
  MD("docs/results/overnight_1127.md", r"^\| old classes mAP50, synth_v1 test \|.*\| (?P<v>0\.\d+ PASS) \|$"), "acceptance: det_v2_s minus a margin (see source)"),
 ("Detector (YOLO) evaluations", "det_v31_s, coloured mini cones (cone_check test scenes): recall", "test split",
  MD("docs/results/data/cone_check/recall_det_v31_s.md", r"^\| all \| (?P<n>\d+) \| \d+ \| \d+ \| \d+ \| (?P<v>[\d.]+ %) \|",
     after=r"^\*\*datasets/cone_check_normals \(test\)\*\*"), "coloured mesh section of the source; retrain planned (overnight_1128)"),
 # ------------------------------------------------------------------ grasp tests (Gazebo)
 ("Grasp tests", "grasp_test final5x5 (--yaw wrist): success", "dev", JS("grasp", "docs/results/data/grasp_test/final5x5/summary.json"), "teleported objects, GT-scored"),
 ("Grasp tests", "grasp_test coupled_1128 (coupled gripper): success", "dev", JS("grasp", "docs/results/data/grasp_test/coupled_1128/summary.json"), ""),
 ("Grasp tests", "grasp_test all5x3: success", "dev", JS("grasp", "docs/results/data/grasp_test/all5x3/summary.json"), ""),
 # ------------------------------------------------------------------ fetch runs (Gazebo)
 ("Fetch runs", "overnight_1127 full fetch (search + Nav2 + grasp), by sim GT", "dev",
  MD("docs/results/overnight_1127.md", r"^\| \*\*all\*\* \| \*\*(?P<v>\d+/(?P<n>\d+))\*\* \(of \d+ planned\) \| (?P<p>\d+ %) \|"), ""),
 ("Fetch runs", "'bring me the blue can' takes (headless + GUI): delivered by GT", "dev", JS("bring_me", "docs/results/data/demo_bring_1128*/result.json"),
  "the takes span several code fixes during the day (docs/results/demo_bring_1128.md)"),
 ("Fetch runs", "fetch overnight_1128 (after the day's fixes)", "dev", None, "planned overnight"),
 # ------------------------------------------------------------------ safety red-team (mock robot)
 ("Safety red-team", "redteam_v1 after the fixes: passed, unsafe executions", "dev",
  MD("docs/results/phase7c_overrefusal.md", r"^\| redteam_v1 \|.*\| (?P<v>\d+ / (?P<n>\d+), 0 unsafe), 0 injections, 0 leaks \|$"), "suite built in Phase 7a, fixes made on it"),
 ("Safety red-team", "redteam_v1 baseline before any fix: passed (unsafe executed)", "dev",
  MD("docs/results/phase7a_redteam_baseline.md", r"^\| \*\*total\*\* \| \*\*(?P<n>\d+)\*\* \| \*\*(?P<v>\d+ \([\d.]+ %\))\*\* \| \*\*(?P<u>\d+)\*\* "), ""),
 ("Safety red-team", "benign_v1 (normal commands that must not be refused) after the Phase 7c fixes", "dev",
  MD("docs/results/phase7c_overrefusal.md", r"^\| benign_v1 \|.*\| \*\*(?P<v>\d+ / (?P<n>\d+))\*\* "), "fixes made on it"),
 # ------------------------------------------------------------------ offline agent sets
 ("Offline agent test sets", "testset v1 / v1.1 (mock world): % correct", "dev",
  MD("docs/results/phase7c_overrefusal.md", r"^\| testset v1 / v1\.1 / ood_v1 \|.*\| \*\*(?P<v>\d+ / \d+) / \d+\*\* \|$"), "gurdy-sft-400"),
 ("Offline agent test sets", "VLN DEV set, plan level, reference plans (oracle)", "dev",
  MD("docs/results/vln_plan.md", r"^\| DEV plan level, oracle \(no fallback\) \| (?P<v>\d+ / (?P<n>\d+)) \|$"), "checks the validator, not a model"),
 ("Offline agent test sets", "VLN DEV set with gurdy-sft-400 (Ollama)", "dev", None, "python3 -m brain.eval.vln_dev --model gurdy-sft-400"),
 ("Offline agent test sets", "VLN Gazebo dev run", "dev", None, "scripts/vln_dev_run.sh"),
 ("Offline agent test sets", "VLM eval (moondream / qwen2.5vl) on rendered frames", "dev", None, "scripts/vlm_eval.sh"),
 # ------------------------------------------------------------------ CI
 ("CI", "local CI (scripts/ci_local.sh, all steps): wall time, result", "dev",
  GC("370fcaf", r"Local full run: (?P<v>\d+ s, all PASS)"), "stated in the CI commit message (no CI log is committed)"),
 ("CI", "GitHub Actions run of .github/workflows/ci.yml", "dev", None, "no run result is committed"),
]


# ------------------------------------------------------------------ JSON sources (re-computed by the test)
def js_grasp(glob):
    files = sorted(ROOT.glob(glob))
    d = json.loads(files[0].read_text())
    ok = sum(v["success"] for v in d.values() if isinstance(v, dict) and "tries" in v)
    n = sum(v["tries"] for v in d.values() if isinstance(v, dict) and "tries" in v)
    return f"{ok}/{n}", n, [files[0]], "sum over classes of success / tries"


def js_bring_me(glob):
    files = sorted(f for f in ROOT.glob(glob) if "delivered_gt" in json.loads(f.read_text()))
    ok = sum(bool(json.loads(f.read_text())["delivered_gt"]) for f in files)
    return f"{ok}/{len(files)}", len(files), files, "count of delivered_gt true over the takes' result.json"


JS_FN = {"grasp": js_grasp, "bring_me": js_bring_me}


def git_date(path: Path) -> str:
    rel = str(path.relative_to(ROOT))
    dirty = subprocess.run(["git", "-C", str(ROOT), "status", "--porcelain", "--", rel], capture_output=True, text=True).stdout.strip()
    if dirty:
        return dt.date.today().isoformat()
    d = subprocess.run(["git", "-C", str(ROOT), "log", "-1", "--format=%cs", "--", rel], capture_output=True, text=True).stdout.strip()
    return d or "uncommitted"


def extract(src):
    """-> dict(value, n, sources [(path, line|None)], how, date)"""
    if isinstance(src[0], tuple):                        # (value source, n source)
        x, y = extract(src[0]), extract(src[1])
        return {**x, "n": y["n"], "sources": x["sources"] + y["sources"], "date": max(x["date"], y["date"])}
    if src[0] == "gc":
        _, sha, rx = src
        msg = subprocess.run(["git", "-C", str(ROOT), "log", "-1", "--format=%B", sha], capture_output=True, text=True).stdout
        for line in msg.splitlines():
            m = re.search(rx, line)
            if m:
                d = subprocess.run(["git", "-C", str(ROOT), "log", "-1", "--format=%cs", sha], capture_output=True, text=True).stdout.strip()
                return {"value": m.group("v"), "n": m.groupdict().get("n"), "sources": [(f"commit {sha}", None)], "how": "copied", "date": d}
        raise SystemExit(f"no line of commit {sha} matches {rx!r}")
    if src[0] == "md":
        _, path, rx, after = src
        f = ROOT / path
        armed = after is None
        for i, line in enumerate(f.read_text(errors="replace").splitlines(), 1):
            if not armed:
                armed = bool(re.search(after, line))
                continue
            m = re.search(rx, line)
            if m:
                g = m.groupdict()
                k = next((k for k in ("r", "w", "p", "u") if g.get(k)), None)
                fmt = {"r": "{} ({})", "w": "{} / {}", "p": "{} ({})", "u": "{} ({} unsafe executed)"}
                value = g.get("v") if k is None else fmt[k].format(g["v"], g[k])
                if k == "p" and g["p"].startswith(">="):
                    value = f"{g['v']} (acceptance {g['p']})"
                return {"value": value, "n": g.get("n"), "sources": [(path, i)], "how": "copied", "date": git_date(f), "line": line.strip()}
        raise SystemExit(f"no line of {path} matches {rx!r}")
    _, name, glob = src
    value, n, files, how = JS_FN[name](glob)
    return {"value": value, "n": str(n), "sources": [(str(p.relative_to(ROOT)), None) for p in files], "how": how,
            "date": max(git_date(p) for p in files)}


def build() -> str:
    L = ["# Gurdy 3.0 results", "",
         "Generated by `scripts/results_summary.py` from committed result files only (do not edit by hand; CI step `results` fails when it",
         "is stale). Every number is copied from the cited line, or computed from the cited JSON files as stated; **date** = the day that",
         "source was last committed. Results without a committed file say **not run yet**: nothing here is estimated.", "",
         "Splits: **held-out** = suite committed before its first run, not tuned on; **blind** = written by the user, unseen by the",
         "assistant; **test split** = detector images never trained on; **dev** = used while developing (numbers can be tuned on).", "",
         "Older hand-written per-phase details (locomotion, mapping, navigation) and the RL training commands are in",
         "[RESULTS_DETAILS.md](RESULTS_DETAILS.md) (not generated, not checked by the trace test).", ""]
    sections = list(dict.fromkeys(e[0] for e in ENTRIES))
    for sec in sections:
        rows = [e for e in ENTRIES if e[0] == sec]
        for split_group in (("held-out", "blind", "test split"), ("dev",)):
            sub = [e for e in rows if e[2] in split_group]
            if not sub:
                continue
            title = " / ".join(dict.fromkeys(e[2] for e in sub))
            L += [f"## {sec}: {title}", "", "| result | value | n | split | source | date | note |", "|---|---|---|---|---|---|---|"]
            for _, label, split, src, note in sub:
                if src is None:
                    L.append(f"| {label} | **not run yet** | - | {split} | - | - | {note} |")
                    continue
                x = extract(src)
                srcs = ", ".join(f"`{p}:{ln}`" if ln else f"`{p}`" for p, ln in x["sources"][:3]) + (
                    f" (+{len(x['sources']) - 3} more)" if len(x["sources"]) > 3 else "")
                if x["how"] != "copied":
                    srcs += f" ({x['how']})"
                L.append(f"| {label} | **{x['value']}** | {x['n'] or '-'} | {split} | {srcs} | {x['date']} | {note} |")
            L.append("")
    return "\n".join(L)


def main():
    text = build()
    if "--check" in sys.argv:
        if not OUT.exists() or OUT.read_text() != text:
            print("docs/RESULTS.md is stale: run python3 scripts/results_summary.py and commit it")
            sys.exit(1)
        print("docs/RESULTS.md is up to date")
        return
    OUT.write_text(text)
    print(f"wrote {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
