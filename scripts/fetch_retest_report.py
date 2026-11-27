#!/usr/bin/env python3
"""Report for scripts/fetch_retest.sh: per variant, per try (reply, can moved, robot->can) + per grasp attempt (estimate vs
ground truth, plan, centring, fingers, failure reason).

    python3 scripts/fetch_retest_report.py docs/results/data/fetch_retest/<stamp> [--out docs/results/phase9j_fetch_retest.md]
"""
import argparse
import json
import math
from pathlib import Path

WHAT = {"A": "teleport to the grasp_test pose + detector / servo pose (grasp pipeline only)",
        "B": "Nav2 approach + ground-truth object pose (approach only)",
        "C": "full fetch: search + Nav2 + detector / servo pose"}


def rows(path: Path) -> list:
    return [json.loads(x) for x in path.read_text().splitlines() if x.strip()] if path.exists() else []


def xy(s):
    try:
        a, b = (float(v) for v in str(s).split())
        return a, b
    except ValueError:
        return None


def outcome(reply: str) -> str:
    r = reply or ""
    return ("success" if r.startswith("I picked up") else "grasp_failed" if "couldn't get a grip" in r else
            "timeout" if r.startswith("TIMEOUT") else "estop" if "emergency stop" in r else "other")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    run = Path(a.run_dir)
    L = [f"# Fetch retest ({run.name}; scripts/fetch_retest.sh)", "",
         "Target: blue can, GT (-1.30, 0.30), 1.33 m from the spawn point; command \"pick up the blue can\"; single tries.", ""]
    summary = []
    for v in ("A", "B", "C"):
        tries = rows(run / v / "tries.jsonl")
        att = rows(run / v / "attempts.jsonl")
        if not tries:
            continue
        ok = sum(outcome(t["reply"]) == "success" for t in tries)
        summary.append(f"| {v} | {WHAT[v]} | {ok}/{len(tries)} | {len(att)} |")
        L += [f"## Variant {v}: {WHAT[v]}", "", "| try | result | wall s | can moved (m) | robot->can at start (m) | reply |", "|---|---|---|---|---|---|"]
        for t in tries:
            b, f, r0 = xy(t["can_before"]), xy(t["can_after"]), xy(t["robot_before"])
            moved = f"{math.dist(b, f):.3f}" if b and f else "-"
            d0 = f"{math.dist(b, r0):.2f}" if b and r0 else "-"
            L.append(f"| {t['try']} | {outcome(t['reply'])} | {t['wall_s']} | {moved} | {d0} | {t['reply'][:110]} |")
        L += ["", "| attempt | outcome | reason | est. source | robot->obj (m) | est. error vs GT (m) | plan | centre shift (m) | settle pre / grasp (rad) | fingers closed / lifted (m) | target in scene |",
              "|---|---|---|---|---|---|---|---|---|---|---|"]
        for x in att:
            cs = x.get("centre_shift_m")
            L.append(f"| {x['attempt']} | {x.get('outcome')} | {x.get('reason', '')} | {x.get('est_source')} | {x.get('dist_robot_obj_m')} | "
                     f"{x.get('est_err_m', '-')} | {'ok' if x.get('plan_ok') else x.get('plan_reason')} | "
                     f"{'-' if cs is None else f'{math.hypot(cs[0], cs[1]):.4f}'} | {x.get('settle_pre_rad', '-')} / {x.get('settle_grasp_rad', '-')} | "
                     f"{x.get('fingers_closed')} / {x.get('fingers_lifted')} | {x.get('target_in_scene')} |")
        L.append("")
    L[4:4] = ["| variant | what | picked up | grasp attempts |", "|---|---|---|---|", *summary, ""]
    txt = "\n".join(L)
    print(txt)
    if a.out:
        Path(a.out).write_text(txt + "\n")


if __name__ == "__main__":
    main()
