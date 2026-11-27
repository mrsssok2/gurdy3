#!/usr/bin/env python3
"""Report for scripts/fetch_retest.sh: per variant, per try (reply, can moved, robot->can) + per grasp attempt (estimate vs
ground truth, plan, centring, fingers, failure reason).

SUCCESS IS JUDGED FROM THE SIMULATION, not from the robot's reply (`gt_verdict`): the can (1) LIFTED off the floor (GT z > 3 cm;
runs before 2026-11-27 evening logged x / y only: then "moved with the arm" = >= 10 cm from its rest AND at the folded grasp
centre), (2) BETWEEN THE FINGERS after the lift (sim finger joints: both > 4 mm and opening >= 50 % of the can width), (3) STILL
HELD at the end (GT position within 10 cm of the folded grasp centre of the robot, 5 s after the reply). Stdlib only.

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


def xyz(s):
    try:
        v = tuple(float(x) for x in str(s).split())
        return v if len(v) >= 2 else None
    except ValueError:
        return None


def xy(s):
    v = xyz(s)
    return None if v is None else v[:2]


def try_attempts(run: Path, v: str, t: dict, att_all: list) -> list:
    """The grasp-attempt records of one try: variant C = one stack per try (run<k>/chat/attempts.jsonl), else the slice."""
    if v == "C":
        return rows(run / v / f"run{t['try']}" / "chat" / "attempts.jsonl")
    if "attempts" in t:
        return att_all[t["attempts"][0]:t["attempts"][1]]
    return att_all


FOLDED_GC = (0.127, 0.0, 0.124)        # base_link grasp centre of the `folded` pose (gurdy_core grasp_centre; tested)
CAN_W, FINGER_CLOSED, FLOOR_Z = 0.066, 0.004, 0.0


def _base(robot_xy, yaw, p):
    dx, dy = p[0] - robot_xy[0], p[1] - robot_xy[1]
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * dx + s * dy, -s * dx + c * dy)


def gt_verdict(t: dict, att: list) -> dict:
    """-> {'lifted', 'between_fingers', 'held_at_end', 'success', 'why', 'evidence'} from ground truth + sim finger joints."""
    b, f, r1 = xyz(t["can_before"]), xyz(t["can_after"]), xyz(t["robot_after"])
    last = att[-1] if att else {}
    yaw = (last.get("robot") or [0, 0, 0.0])[2]
    out = {"lifted": None, "between_fingers": None, "held_at_end": None}
    if b and f and r1:
        cb = _base(r1, yaw, f)
        d_fold = math.hypot(cb[0] - FOLDED_GC[0], cb[1] - FOLDED_GC[1])
        out["held_at_end"] = d_fold <= 0.10 and (len(f) < 3 or f[2] > FLOOR_Z + 0.05)
        if len(f) >= 3:
            out["lifted"] = f[2] > FLOOR_Z + 0.03
            out["evidence"] = f"can z {f[2]:.3f} m at the end, {d_fold:.3f} m from the folded grasp centre"
        else:
            out["lifted"] = math.dist(b[:2], f[:2]) >= 0.10 and d_fold <= 0.10
            out["evidence"] = (f"x/y only: can moved {math.dist(b[:2], f[:2]):.2f} m from its rest, ends {d_fold:.3f} m from the folded "
                               f"grasp centre (base {cb[0]:.3f}, {cb[1]:.3f})")
    fl = last.get("fingers_lifted")
    if fl:
        out["between_fingers"] = min(fl) > FINGER_CLOSED and sum(fl) >= 0.5 * CAN_W
    checks = [("lifted", "not lifted off the floor"), ("between_fingers", "not between the fingers after the lift"),
              ("held_at_end", "not held at the end")]
    out["success"] = all(out[k] for k, _ in checks)
    out["why"] = "" if out["success"] else "; ".join(w + (" (no data)" if out[k] is None else "") for k, w in checks if not out[k])
    if not out["success"] and last.get("reason"):
        out["why"] += f" [last attempt: {last['outcome']}: {last['reason']}]"
    if not att and t["reply"].startswith("TIMEOUT"):
        out["why"] = "timeout before any grasp attempt"
    return out


def unwrap(reply: str) -> str:
    r = reply or ""
    if r.startswith("{"):                 # runs before 2026-11-27 evening stored the raw /api/say JSON
        try:
            return json.loads(r).get("reply", r)
        except ValueError:
            pass
    return r


def outcome(reply: str) -> str:
    r = unwrap(reply)
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
        verdicts = [gt_verdict(t, try_attempts(run, v, t, att)) for t in tries]
        ok = sum(g["success"] for g in verdicts)
        claimed = sum(outcome(t["reply"]) == "success" for t in tries)
        summary.append(f"| {v} | {WHAT[v]} | {ok}/{len(tries)} | {claimed}/{len(tries)} | {len(att)} |")
        L += [f"## Variant {v}: {WHAT[v]}", "",
              "| try | success (sim GT) | wall s | failure reason | robot said | GT evidence |", "|---|---|---|---|---|---|"]
        for t, g in zip(tries, verdicts):
            L.append(f"| {t['try']} | {'yes' if g['success'] else 'no'} | {t['wall_s']} | {g['why'] or '-'} | {unwrap(t['reply'])[:90]} | "
                     f"{g.get('evidence', '-')} |")
        L += ["", "| attempt | outcome | reason | est. source | robot->obj (m) | est. error vs GT (m) | plan | centre shift (m) | settle pre / grasp (rad) | fingers closed / lifted (m) | target in scene |",
              "|---|---|---|---|---|---|---|---|---|---|---|"]
        for x in att:
            cs = x.get("centre_shift_m")
            L.append(f"| {x['attempt']} | {x.get('outcome')} | {x.get('reason', '')} | {x.get('est_source')} | {x.get('dist_robot_obj_m')} | "
                     f"{x.get('est_err_m', '-')} | {'ok' if x.get('plan_ok') else x.get('plan_reason')} | "
                     f"{'-' if cs is None else f'{math.hypot(cs[0], cs[1]):.4f}'} | {x.get('settle_pre_rad', '-')} / {x.get('settle_grasp_rad', '-')} | "
                     f"{x.get('fingers_closed')} / {x.get('fingers_lifted')} | {x.get('target_in_scene')} |")
        L.append("")
    L[4:4] = ["| variant | what | success (sim GT) | robot claimed | grasp attempts |", "|---|---|---|---|---|", *summary, ""]
    txt = "\n".join(L)
    print(txt)
    if a.out:
        Path(a.out).write_text(txt + "\n")


if __name__ == "__main__":
    main()
