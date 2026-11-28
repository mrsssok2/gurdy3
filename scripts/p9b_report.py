#!/usr/bin/env python3
"""Acceptance report for the small-object detector (Phase 9b), one or more models side by side, from vision/eval_detector.py
outputs (per-class AP tables).

    python3 scripts/p9b_report.py --model det_v3_s:docs/results/phase9b_eval --model det_v31_s:docs/results/phase9b_v31_eval \
        --baseline docs/results/phase9b_eval --out docs/results/phase9b_v3_vs_v31.md

`--model NAME:PREFIX` reads PREFIX_<set>.md for set in synth_v1, synth_v2 (old classes, regression guard), synth_v3_body,
synth_v3_wrist (new test sets; v3.1 runs also synth_v31_body / synth_v31_wrist if present). The baseline (deployed det_v2_s)
is PREFIX_<set>_baseline_v2s.md. Acceptance (agreed 2026-11-27): old classes drop <= 0.01 mAP50 vs det_v2_s on synth_v1 and
synth_v2; small classes mAP50 >= 0.80 on the wrist test set and >= 0.70 on the body test set.
"""
import argparse
import re
from pathlib import Path

OLD = ["box", "barrel", "fire_extinguisher", "traffic_cone", "toolbox", "mannequin", "chair"]
SMALL = ["small_box", "mini_toolbox", "can", "mini_cone", "mini_extinguisher"]
ROW = re.compile(r"^\|\s*([a-z_]+)\s*\|\s*(\d+)\s*\|\s*([0-9.]+)\s*\|")


def per_class(path: Path) -> dict:
    out = {}
    if not path.exists():
        return out
    for ln in path.read_text().splitlines():
        m = ROW.match(ln)
        if m and m.group(1) in OLD + SMALL and m.group(1) not in out:    # first table = per-class AP (later: confusion matrix)
            out[m.group(1)] = (int(m.group(2)), float(m.group(3)))
    return out


def mean_ap(pc: dict, names) -> float | None:
    v = [pc[n][1] for n in names if n in pc and pc[n][0] > 0]
    return sum(v) / len(v) if v else None


def fmt(x):
    return "-" if x is None else f"{x:.3f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", action="append", required=True, help="NAME:PREFIX")
    ap.add_argument("--baseline", required=True, help="PREFIX of the det_v2_s baseline files (PREFIX_<set>_baseline_v2s.md)")
    ap.add_argument("--out", default="")
    ap.add_argument("--json", default="", help="also write {model: acceptance true/false} here")
    a = ap.parse_args()
    models = [m.split(":", 1) for m in a.model]
    base = {s: mean_ap(per_class(Path(f"{a.baseline}_{s}_baseline_v2s.md")), OLD) for s in ("synth_v1", "synth_v2")}
    rows = ["| check | target | det_v2_s | " + " | ".join(n for n, _ in models) + " |", "|---|---|---|" + "---|" * len(models)]
    verdict = {n: True for n, _ in models}
    for s in ("synth_v1", "synth_v2"):
        vals = []
        for n, p in models:
            v = mean_ap(per_class(Path(f"{p}_{s}.md")), OLD)
            ok = v is not None and base[s] is not None and v >= base[s] - 0.01
            verdict[n] &= ok
            vals.append(f"{fmt(v)} {'PASS' if ok else 'FAIL'}")
        rows.append(f"| old classes mAP50, {s} test | >= det_v2_s - 0.01 | {fmt(base[s])} | " + " | ".join(vals) + " |")
    for s, tgt in (("synth_v3_wrist", 0.80), ("synth_v3_body", 0.70), ("synth_v31_wrist", 0.80), ("synth_v31_body", 0.70),
                   ("synth_v32_wrist", 0.80), ("synth_v32_body", 0.70)):
        vals, any_file = [], False
        for n, p in models:
            f = Path(f"{p}_{s}.md")
            any_file |= f.exists()
            v = mean_ap(per_class(f), SMALL)
            ok = v is not None and v >= tgt
            if s.startswith("synth_v3_"):                      # the agreed acceptance sets
                verdict[n] &= ok
            vals.append(f"{fmt(v)} {'PASS' if ok else ('FAIL' if v is not None else '')}")
        if any_file:
            rows.append(f"| small classes mAP50, {s} test | >= {tgt:.2f} | - | " + " | ".join(vals) + " |")
    rows.append("| **acceptance (old + v3 sets)** | | | " + " | ".join("**PASS**" if verdict[n] else "**FAIL**" for n, _ in models) + " |")
    rows.append("")
    for s in ("synth_v3_body", "synth_v3_wrist"):
        rows.append(f"Per small class, {s} test (AP50): " + "; ".join(
            f"{n}: " + ", ".join(f"{c} {fmt(per_class(Path(f'{p}_{s}.md')).get(c, (0, None))[1])}" for c in SMALL) for n, p in models))
    txt = "\n".join(rows)
    print(txt)
    if a.out:
        Path(a.out).write_text(txt + "\n")
    if a.json:
        import json
        Path(a.json).write_text(json.dumps(verdict))


if __name__ == "__main__":
    main()
