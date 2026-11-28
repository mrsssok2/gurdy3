#!/usr/bin/env python3
"""Morning summary of scripts/overnight_1128.sh: a) mini-cone retrain (v3.2) + runtime-detector decision, b) full fetch x15
graded from Gazebo ground truth (scripts/fetch_retest_report.gt_verdict), with the 'can't confirm' replies, the wrist-view
diagnostics and the sighting logs. Stdlib only.

    python3 scripts/overnight_1128_report.py docs/results/data/overnight_1128 [--out docs/results/overnight_1128.md]
"""
import argparse
import importlib.util
import json
from collections import Counter, defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
_s = importlib.util.spec_from_file_location("overnight_1127_report", HERE / "overnight_1127_report.py")
R27 = importlib.util.module_from_spec(_s)
_s.loader.exec_module(R27)
FR, rows, attempts_of, last_line, CLASSES = R27.FR, R27.rows, R27.attempts_of, R27.last_line, R27.CLASSES


def jload(p: Path) -> dict:
    return json.loads(p.read_text()) if p.exists() else {}


def pct(x):
    return "-" if x is None else f"{100 * x:.1f} %"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir")
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    D = Path(a.out_dir)
    ev = rows(D / "events.jsonl")
    log = (D / "run.log").read_text().splitlines() if (D / "run.log").exists() else []
    starts = [e for e in ev if e["type"] == "run_start"]
    L = [f"# Overnight 2026-11-28 ({D})", ""]
    if log:
        L += [f"Run: {log[0][:8]} - {log[-1][:8]} UTC (run.log); started {len(starts)} time(s)"
              + (" (RESUMED after an interruption: finished units were skipped)" if len(starts) > 1 else "")
              + ". Fetch success is judged from Gazebo ground truth, NOT from the robot's reply.", ""]
    failed = []

    # ---------------- a) cone retrain
    C = D / "cone"
    L += ["## a) Mini-cone retrain (v3.2: coloured cones) and the runtime detector", ""]
    st = rows(C / "stages.jsonl")
    if not st and not (C / "decision.json").exists():
        L += ["Not run.", ""]
    else:
        if st:
            L += ["| stage | exit code | wall (min) |", "|---|---|---|"]
            L += [f"| {x['stage']} | {x['rc']} | {int(x['wall_s']) / 60:.0f} |" for x in st]
            L.append("")
            failed += [f"cone {x['stage']}: exit code {x['rc']} (log cone/logs/{x['stage']}.log)" for x in st if str(x["rc"]) != "0"]
        res = C / "det_v32_s_results.csv"
        if res.exists():
            r = [ln.split(",") for ln in res.read_text().replace("\0", "").splitlines()[1:] if ln.strip()]
            if r and len(r[0]) > 8:
                best = max(r, key=lambda v: 0.1 * float(v[7]) + 0.9 * float(v[8]))
                L += [f"det_v32_s: {len(r)} epochs, best epoch {best[0]}: val mAP50 {float(best[7]):.3f}, mAP50-95 {float(best[8]):.3f}.", ""]
            elif r:
                L += [f"det_v32_s: {len(r)} epochs.", ""]
        new, ref, v31c = jload(C / "cone_new.json"), jload(C / "cone_ref.json"), jload(C / "cone_v31_coloured.json")
        if new or ref:
            L += ["| mini-cone recall (all instances, conf 0.35) | value |", "|---|---|",
                  f"| det_v31_s on WHITE cones (v3.1 test sets) = reference | {pct(ref.get('recall_all'))} |",
                  f"| det_v31_s on COLOURED cones (v3.2 test sets) | {pct(v31c.get('recall_all'))} |",
                  f"| det_v32_s on COLOURED cones (v3.2 test sets) | {pct(new.get('recall_all'))} |", ""]
        cmp_md = C / "phase9b_v31_vs_v32.md"
        if cmp_md.exists():
            L += ["**Acceptance, det_v31_s vs det_v32_s (scripts/p9b_report.py)**", "", cmp_md.read_text().strip(), ""]
        dec = jload(C / "decision.json")
        if dec:
            L += [f"**Decision: runtime detector = `{dec['model']}`** ("
                  + ("SWITCHED: acceptance PASS and coloured-cone recall "
                     f"{pct(dec.get('cone_recall_coloured'))} >= needed {pct(dec.get('cone_needed'))}" if dec.get("switch")
                     else "kept det_v31_s: " + "; ".join(dec.get("why_not") or ["-"])) + ")", ""]
        rd = rows(C / "runtime_detector.jsonl")
        if rd:
            L += [f"Runtime default before / after the block: `{rd[-1]['default_before']}` -> `{rd[-1]['default_model']}` "
                  "(rollback: `detector_model:=det_v31_s.onnx`).", ""]

    # ---------------- b) fetch x15
    tries = [t for t in rows(D / "fetch" / "tries.jsonl")]
    L += ["## b) Full fetch x15 (search + Nav2 + detector + wrist servo), fresh stack per try", ""]
    if not tries:
        L += ["Not run.", ""]
    else:
        last = {}
        for t in tries:                     # a resumed run may have recorded a start failure first, then the real try
            last[t["id"]] = t
        tries = list(last.values())
        L += ["| try | class | dist (m) | success (sim GT) | time (s) | detector | robot said | failure reason | sightings | wrist views ok / all |",
              "|---|---|---|---|---|---|---|---|---|---|"]
        per = defaultdict(lambda: [0, 0])
        wrong_claims = unconfirmed_held = unconfirmed = 0
        view_fail_whys = []
        for t in tries:
            if t.get("status") != "ran":
                L.append(f"| {t['id']} | {t['class']} | {t.get('dist_m')} | not run | - | - | - | stack did not start | - | - |")
                failed.append(f"fetch {t['id']}: stack did not start ({last_line(D / 'fetch' / t['id'] / 'stack.out') or 'no log'})")
                continue
            att = attempts_of(D / "fetch" / t["id"], t)
            g = FR.gt_verdict(t, att, t["class"])
            per[t["class"]][1] += 1
            per[t["class"]][0] += g["success"]
            said = FR.unwrap(t.get("reply", ""))
            if said.startswith("I picked up") and not g["success"]:
                wrong_claims += 1
            if "can't confirm" in said:
                unconfirmed += 1
                unconfirmed_held += g["success"]
            sl = D / "fetch" / t["id"] / "chat" / "sightings.jsonl"
            n_sl = sum(1 for _ in sl.open()) if sl.exists() else None
            views = [v for x in att for v in (x.get("wrist_views") or [])]
            view_fail_whys += [x["wrist_view"] for x in att if x.get("wrist_view")]
            if not g["success"]:
                failed.append(f"fetch {t['id']}: {g['why']}")
            L.append(f"| {t['id']} | {t['class']} | {t.get('dist_m')} | {'yes' if g['success'] else 'no'} | {t.get('wall_s', '-')} | "
                     f"{t.get('detector', '-')} | {said[:70]} | {g['why'][:70] or '-'} | {n_sl if n_sl is not None else 'none'} | "
                     f"{sum(1 for v in views if v[0])} / {len(views)} |")
        n_ok, n_ran = sum(v[0] for v in per.values()), sum(v[1] for v in per.values())
        L += ["", "| class | success (sim GT) |", "|---|---|"]
        L += [f"| {c} | {per[c][0]}/{per[c][1]} |" for c in CLASSES if c in per]
        L += [f"| **all** | **{n_ok}/{n_ran}** (of {len(tries)} planned) |", "",
              f"Robot claimed a pick that ground truth does not show: {wrong_claims}. "
              f"'I can't confirm I'm holding it' replies: {unconfirmed} (held by ground truth in {unconfirmed_held}).", ""]
        if view_fail_whys:
            L += ["Wrist views that gave no frame (RclpyArmPort diagnostics, per attempt):", ""]
            L += [f"* {w}" for w in view_fail_whys] + [""]

    # ---------------- events, failures
    L += ["## Blocks, skips and failures", "", "| event | details |", "|---|---|"]
    skips = [e for e in ev if e["type"] in ("block_skip", "stack_fail")]
    L += [f"| {e['type']} ({e['t']}) | " + ", ".join(f"{k}={v}" for k, v in e.items() if k not in ("type", "t")) + " |" for e in skips] \
        or ["| - | no skipped block, no failed start |"]
    L += ["", "What failed and why:", ""] + ([f"* {w}" for w in failed] or ["* nothing"])
    reasons = Counter(w.split(": ", 1)[1].split(" [")[0][:90] for w in failed if ": " in w)
    if reasons:
        L += ["", "Failure reasons (count): " + "; ".join(f"{k} x{n}" for k, n in reasons.most_common())]
    txt = "\n".join(L) + "\n"
    print(txt)
    if a.out:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(txt)


if __name__ == "__main__":
    main()
