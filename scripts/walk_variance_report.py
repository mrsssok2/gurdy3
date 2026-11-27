#!/usr/bin/env python3
"""Walking variance with the arm: aggregate N runs per configuration (day0927_probe walk.json files).
    python3 scripts/walk_variance_report.py DIR [--out report.md]
DIR holds <config>_r<k>/walk.json for config in 05_walk_no_arm, 06_walk_arm_folded, 07_walk_arm_object.
Reports per config: forward / turn error mean, SD, SE, min-max; falls; arm max deviation (arm configs); and the difference of
each arm config to no-arm with an approximate 95 % interval (Welch, normal approximation)."""
import argparse
import glob
import json
import math
import statistics as st
from pathlib import Path

CONFIGS = [("05_walk_no_arm", "no arm"), ("06_walk_arm_folded", "arm folded"), ("07_walk_arm_object", "arm folded + 80 g")]


def load(d, cfg):
    out = []
    for f in sorted(glob.glob(f"{d}/{cfg}_r*/walk.json")):
        out.append(json.load(open(f)))
    return out


def stats(v):
    n = len(v)
    m = st.mean(v) if v else float("nan")
    sd = st.stdev(v) if n > 1 else float("nan")
    return {"n": n, "mean": m, "sd": sd, "se": sd / math.sqrt(n) if n > 1 else float("nan"), "min": min(v) if v else float("nan"),
            "max": max(v) if v else float("nan")}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--out")
    a = ap.parse_args()
    rows, S = [], {}
    rows.append("| config | runs | forward error mean ± SD (min-max) | turn error mean ± SD (min-max) | falls | arm max deviation (rad) |")
    rows.append("|---|---|---|---|---|---|")
    for cfg, label in CONFIGS:
        rs = load(a.dir, cfg)
        if not rs:
            continue
        fw = stats([r["forward"]["err_pct"] for r in rs if "forward" in r])
        tu = stats([r["turn"]["err_pct"] for r in rs if "turn" in r])
        falls = sum(1 for r in rs if not r.get("pass", True))
        dev = [r["arm_max_dev_rad"] for r in rs if "arm_max_dev_rad" in r]
        S[cfg] = (fw, tu)
        rows.append(f"| {label} | {fw['n']} | {fw['mean']:.1f} ± {fw['sd']:.1f} ({fw['min']:.1f}-{fw['max']:.1f}) % | "
                    f"{tu['mean']:.1f} ± {tu['sd']:.1f} ({tu['min']:.1f}-{tu['max']:.1f}) % | {falls}/{len(rs)} | "
                    f"{(f'max {max(dev):.3f}, mean {st.mean(dev):.3f}' if dev else '-')} |")
    rows.append("")
    base = S.get("05_walk_no_arm")
    for cfg, label in CONFIGS[1:]:
        if base and cfg in S:
            for k, name in ((0, "forward"), (1, "turn")):
                b, c = base[k], S[cfg][k]
                d = c["mean"] - b["mean"]
                se = math.sqrt(b["se"] ** 2 + c["se"] ** 2)
                rows.append(f"* {label} vs no arm, {name}: {d:+.1f} points (95 % ~ {d - 1.96 * se:+.1f} .. {d + 1.96 * se:+.1f}); "
                            f"acceptance +5 points: {'met' if d + 1.96 * se <= 5 else ('NOT met' if d - 1.96 * se > 5 else 'undecided')}")
    txt = "\n".join(rows)
    print(txt)
    if a.out:
        Path(a.out).write_text(txt + "\n")


if __name__ == "__main__":
    main()
