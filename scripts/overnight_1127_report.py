#!/usr/bin/env python3
"""Morning summary of scripts/overnight_1127.sh: fetch graded from Gazebo ground truth (scripts/fetch_retest_report.gt_verdict),
variants A / B, walking variance, and what failed and why. Stdlib only (runs on the host's system python).

    python3 scripts/overnight_1127_report.py docs/results/data/overnight_1127 [--out docs/results/overnight_1127.md]
"""
import argparse
import importlib.util
import json
from collections import Counter, defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location("fetch_retest_report", HERE / "fetch_retest_report.py")
FR = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(FR)
CLASSES = ["small_box", "mini_toolbox", "can", "mini_cone", "mini_extinguisher"]


def rows(p: Path) -> list:
    return [json.loads(x) for x in p.read_text().splitlines() if x.strip()] if p.exists() else []


def attempts_of(d: Path, t: dict) -> list:
    a = rows(d / "chat" / "attempts.jsonl")
    if isinstance(t.get("attempts"), list) and len(t["attempts"]) == 2:
        return a[t["attempts"][0]:t["attempts"][1]]
    return a


def last_line(p: Path, pattern: str = "") -> str:
    if not p.exists():
        return ""
    ls = [x for x in p.read_text(errors="replace").splitlines() if x.strip() and pattern in x]
    return ls[-1].strip()[:160] if ls else ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir")
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    D = Path(a.out_dir)
    ev = rows(D / "events.jsonl")
    log = (D / "run.log").read_text().splitlines() if (D / "run.log").exists() else []
    L = [f"# Overnight 2026-11-27 reliability run ({D})", ""]
    if log:
        L += [f"Run: {log[0][:8]} - {log[-1][:8]} UTC (run.log). Success is judged from Gazebo ground truth, NOT from the robot's reply.", ""]
    failed_why = []

    # ---------------- a) full fetch
    tries = rows(D / "fetch" / "tries.jsonl")
    L += ["## a) Full fetch (search + Nav2 + detector), fresh stack per try", ""]
    if not tries:
        L += ["Not run.", ""]
    else:
        L += ["| try | class | distance (m) | success (sim GT) | time (s) | failure reason | robot said |", "|---|---|---|---|---|---|---|"]
        per = defaultdict(lambda: [0, 0])
        claimed_wrong = 0
        for t in tries:
            if t.get("status") != "ran":
                L.append(f"| {t['id']} | {t['class']} | {t.get('dist_m')} | not run | - | stack did not start | - |")
                failed_why.append(f"fetch {t['id']}: stack did not start ({last_line(D / 'fetch' / t['id'] / 'stack.out') or 'no log'})")
                continue
            g = FR.gt_verdict(t, attempts_of(D / "fetch" / t["id"], t), t["class"])
            per[t["class"]][1] += 1
            per[t["class"]][0] += g["success"]
            said = FR.unwrap(t.get("reply", ""))
            if said.startswith("I picked up") and not g["success"]:
                claimed_wrong += 1
            if not g["success"]:
                failed_why.append(f"fetch {t['id']}: {g['why']}")
            L.append(f"| {t['id']} | {t['class']} | {t.get('dist_m')} | {'yes' if g['success'] else 'no'} | {t.get('wall_s', '-')} | "
                     f"{g['why'] or '-'} | {said[:80]} |")
        n_ok, n_ran = sum(v[0] for v in per.values()), sum(v[1] for v in per.values())
        L += ["", "| class | success (sim GT) | rate |", "|---|---|---|"]
        for c in CLASSES:
            if c in per:
                L.append(f"| {c} | {per[c][0]}/{per[c][1]} | {100 * per[c][0] / per[c][1]:.0f} % |")
        L += [f"| **all** | **{n_ok}/{n_ran}** (of {len(tries)} planned) | {100 * n_ok / n_ran:.0f} % |" if n_ran else "| **all** | 0/0 | - |",
              "", f"Robot claimed success where ground truth says no: {claimed_wrong}.", ""]

    # ---------------- b) variants A / B
    L += ["## b) Variants A (teleport + detector pose) and B (Nav2 + GT pose), blue can at its GT spot", ""]
    any_ab = False
    for v in ("A", "B"):
        tr = rows(D / "ab" / v / "tries.jsonl")
        if not tr:
            continue
        any_ab = True
        L += [f"**{v}** ({FR.WHAT[v]})", "", "| try | success (sim GT) | time (s) | failure reason |", "|---|---|---|---|"]
        for t in tr:
            g = FR.gt_verdict(t, attempts_of(D / "ab" / v, t), "can")
            L.append(f"| {t['try']} | {'yes' if g['success'] else 'no'} | {t.get('wall_s', '-')} | {g['why'] or '-'} |")
            if not g["success"]:
                failed_why.append(f"variant {v} try {t['try']}: {g['why']}")
        L.append("")
    if not any_ab:
        L += ["Not run.", ""]

    # ---------------- c) walking
    L += ["## c) Walking variance with the arm", ""]
    rep = D / "walk" / "report.md"
    runs = sorted(p.name for p in (D / "walk").glob("*_r*") if p.is_dir()) if (D / "walk").exists() else []
    if rep.exists():
        L += [rep.read_text().strip(), ""]
    ok_runs = [r for r in runs if (D / "walk" / r / "walk.json").exists()]
    L += [f"Walk runs: {len(ok_runs)} with a result of {len(runs)} started.", ""] if runs else ["Not run.", ""]
    for r in sorted(set(runs) - set(ok_runs)):
        failed_why.append(f"walk {r}: no result ({last_line(D / 'walk' / r / 'console.log') or 'no console.log'})")

    # ---------------- d) YOLO v3.1
    L += ["## d) YOLO v3.1 (render -> train -> eval), v3 vs v3.1; runtime detector NOT switched", ""]
    Y = D / "yolo"
    st = rows(Y / "stages.jsonl")
    if not st:
        L += ["Not run.", ""]
    else:
        L += ["| stage | exit code | wall (min) | training budget (h) |", "|---|---|---|---|"]
        for x in st:
            L.append(f"| {x['stage']} | {x['rc']} | {int(x['wall_s']) / 60:.0f} | {x.get('train_time_h', '0')} |")
            if str(x["rc"]) != "0":
                failed_why.append(f"yolo {x['stage']}: exit code {x['rc']} (log: yolo/logs/{x['stage']}.log)")
        L.append("")
        rc = Y / "det_v31_s_results.csv"
        if rc.exists():
            r = [ln.split(",") for ln in rc.read_text().splitlines()[1:] if ln.strip()]
            if r:
                best = max(r, key=lambda v: 0.1 * float(v[7]) + 0.9 * float(v[8]))
                L += [f"det_v31_s trained {len(r)} epochs in {float(r[-1][1]) / 3600:.2f} h (det_v3_s: 104 epochs in 2.09 h, "
                      f"stopped by the power cut); best epoch {best[0]}: val mAP50 {float(best[7]):.3f}, mAP50-95 {float(best[8]):.3f} "
                      "(val sets differ: v3.1 labels include the tiny objects).", ""]
        for f, title in (("phase9b_v3_vs_v31.md", "Acceptance, side by side (scripts/p9b_report.py)"),
                         ("phase9b_recall_by_distance_det_v3_s.md", "Recall by distance, det_v3_s"),
                         ("phase9b_recall_by_distance_det_v31_s.md", "Recall by distance, det_v31_s")):
            if (Y / f).exists():
                L += [f"**{title}**", "", (Y / f).read_text().strip(), ""]
        rd = rows(Y / "runtime_detector.jsonl")
        if rd:
            L += [f"Runtime detector default after the block: `{rd[-1]['default_model']}`"
                  + (" (unchanged)" if rd[-1]["default_model"] == "det_v3_s.onnx" else " (CHANGED: check!)"), ""]

    # ---------------- what failed and why
    L += ["## Blocks, skips and failures", "", "| event | details |", "|---|---|"]
    for e in ev:
        if e["type"] in ("block_skip", "stack_fail", "walk_fail"):
            L.append(f"| {e['type']} ({e['t']}) | " + ", ".join(f"{k}={v}" for k, v in e.items() if k not in ("type", "t")) + " |")
    if not any(e["type"] in ("block_skip", "stack_fail", "walk_fail") for e in ev):
        L.append("| - | no skipped block, no failed start |")
    L += ["", "What failed and why:", ""]
    L += [f"* {w}" for w in failed_why] or ["* nothing"]
    reasons = Counter(w.split(": ", 1)[1].split(" [")[0] for w in failed_why if ": " in w)
    if reasons:
        L += ["", "Failure reasons (count): " + "; ".join(f"{k} x{n}" for k, n in reasons.most_common())]
    txt = "\n".join(L) + "\n"
    print(txt)
    if a.out:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(txt)


if __name__ == "__main__":
    main()
