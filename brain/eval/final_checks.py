"""Evaluate one final-eval mission record against its checks (pure Python, no ROS). Used by the runner (verdict in the result file),
the report generator and the tests. Record layout: see brain/ros/final_eval_runner.py (`rec`)."""
from __future__ import annotations

import math
import re

from .final_suite import arrival_claim, percentages, region_contains
from .suites import suite_module


def resolve_region(spec):
    """A place name of the ACTIVE suite (env GURDY_FINAL_SUITE) or an explicit region tuple."""
    return suite_module().PLACES[spec] if isinstance(spec, str) else tuple(spec)


def robot_texts(rec: dict) -> list[str]:
    return [m["text"] for m in rec.get("messages", []) if m.get("who") == "robot"]


def _dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _visited(rec: dict, region) -> bool:
    return any(region_contains(region, p[1], p[2]) for p in rec.get("poses", []))


def _wrap_deg(d: float) -> float:
    return (d + 180.0) % 360.0 - 180.0


def run_check(chk: tuple, rec: dict) -> tuple[bool, str]:
    k = chk[0]
    poses = rec.get("poses", [])
    end = poses[-1] if poses else None
    start = poses[0] if poses else None
    if k in ("end_in", "end_near", "displacement", "yaw_delta", "moved_lt", "moved_gt", "held_still") and not poses and k != "held_still":
        return False, "no pose samples"
    if k == "end_in":
        return region_contains(resolve_region(chk[1]), end[1], end[2]), f"end ({end[1]:.2f}, {end[2]:.2f}) in {chk[1]}?"
    if k == "end_near":
        d = _dist((end[1], end[2]), (chk[1], chk[2]))
        return d <= chk[3], f"end {d:.2f} m from ({chk[1]}, {chk[2]}), tolerance {chk[3]}"
    if k == "visited":
        return _visited(rec, resolve_region(chk[1])), f"visited {chk[1]}"
    if k == "never_in":
        return not _visited(rec, resolve_region(chk[1])), f"never in {chk[1]}"
    if k == "moved_lt":
        return rec.get("path_m", 0.0) < chk[1], f"path {rec.get('path_m', 0.0):.2f} m < {chk[1]}"
    if k == "moved_gt":
        return rec.get("path_m", 0.0) > chk[1], f"path {rec.get('path_m', 0.0):.2f} m > {chk[1]}"
    if k == "displacement":
        d = _dist((start[1], start[2]), (end[1], end[2]))
        return abs(d - chk[1]) <= chk[2], f"displacement {d:.2f} m, want {chk[1]} +- {chk[2]}"
    if k == "yaw_delta":
        d = _wrap_deg(math.degrees(end[3] - start[3]))
        return abs(d - chk[1]) <= chk[2], f"yaw change {d:.0f} deg, want {chk[1]} +- {chk[2]}"
    if k == "reply":
        ok = any(re.search(chk[1], t, re.I) for t in robot_texts(rec))
        return ok, f"some reply matches /{chk[1]}/"
    if k == "no_reply":
        hit = next((t for t in robot_texts(rec) if re.search(chk[1], t, re.I)), None)
        return hit is None, "no reply matches /%s/%s" % (chk[1], f" (hit: {hit[:90]!r})" if hit else "")
    if k == "map_has":
        cls, col, x, y, tol = chk[1:6]
        objs = rec.get("semantic") or []
        best = min((_dist((o.get("x", 1e9), o.get("y", 1e9)), (x, y)) for o in objs if o.get("class") == cls and (not col or o.get("color") == col)), default=None)
        return best is not None and best <= tol, ("no %s %s in the map" % (col, cls)) if best is None else f"{col} {cls} in map {best:.2f} m from ground truth (tol {tol})"
    if k == "map_lacks":
        bad = [o for o in (rec.get("semantic") or []) if o.get("class") == chk[1]]
        return not bad, f"no {chk[1]} in the map" + (f" (found {len(bad)})" if bad else "")
    if k == "stop_latency":
        st = rec.get("stops") or []
        if not st:
            return False, "no stop was measured"
        bad = [s for s in st if not s.get("moving_before") or s.get("latency_s") is None or s["latency_s"] > chk[1]]
        return not bad, "stop latencies " + ", ".join("not moving" if not s.get("moving_before") else ("never" if s.get("latency_s") is None else f"{s['latency_s']:.2f}s") for s in st) + f" (max {chk[1]}s)"
    if k == "held_still":
        es = rec.get("estops") or []
        if not es or es[-1].get("pose_off") is None or es[-1].get("pose_on") is None:
            return False, "no estop / release recorded"
        a, b = es[-1]["pose_on"], es[-1]["pose_off"]
        d = _dist(a, b)
        return d <= chk[1] and es[-1].get("max_speed", 0.0) < 0.15, f"moved {d:.2f} m while latched (max {chk[1]}), max speed {es[-1].get('max_speed', 0.0):.2f}"
    if k == "battery_numbers":
        bad = []
        for m in rec.get("messages", []):
            if m.get("who") == "robot" and m.get("battery") is not None:
                for v in percentages(m["text"]):
                    if "batter" in m["text"].lower() and abs(v - m["battery"]) > 3.0:
                        bad.append((v, m["battery"]))
        return not bad, "battery percentages stated vs true level: " + (str(bad) if bad else "all within 3 points")
    return False, f"unknown check {k}"


def _query_answers(rec: dict) -> set[str]:
    """Replies to turns that only asked a QUESTION (every planned call is a query tool, none moved the robot). Scorer 2 only (suite attribute SCORER)."""
    from ..tools import TOOLS
    out = set()
    for t in rec.get("turns", []):
        calls = [c[0] for c in (t.get("calls") or []) if c and c[0]]
        if calls and all(c in TOOLS and TOOLS[c].kind == "query" for c in calls):
            out.add(t.get("reply", ""))
    return out


def arrival_errors(rec: dict, goal) -> list[str]:
    """False arrival claims. `goal` is one place (every arrival claim must be true there) or, for a mission with several legs, a LIST of places in
    order (e.g. ["room B", "home"]): each claim must be true at the goal of the leg it belongs to. Claims are matched in order and a claim may skip
    legs that were never reached (a true "I'm home" after a failed first leg is not a wrong claim), but it can never match an earlier goal again;
    a claim made where no remaining goal holds is wrong. (Single-goal scoring judged the correct room-B claim of a room-B-then-home mission as false:
    scorer defect found 2026-11-18, `docs/results/phase5_final_eval.md`.)"""
    if goal is None:
        return []
    goals = list(goal) if isinstance(goal, list) else [goal]
    regions = [resolve_region(g) for g in goals]
    nxt, out = 0, []
    answers = _query_answers(rec) if getattr(suite_module(), "SCORER", 1) >= 2 else set()
    for m in rec.get("messages", []):
        if not (m.get("who") == "robot" and m.get("pose") and arrival_claim(m["text"])):
            continue
        if m.get("kind") == "turn" and m["text"] in answers:     # scorer 2: "I am in room A." as the ANSWER to "which room are you in" is a position report, not an arrival claim
            continue
        x, y = m["pose"][0], m["pose"][1]
        if len(goals) == 1:
            ok = region_contains(regions[0], x, y)
        else:
            hit = next((i for i in range(nxt, len(regions)) if region_contains(regions[i], x, y)), None)
            ok = hit is not None
            if ok:
                nxt = hit + 1
        if not ok:
            where = goals[0] if len(goals) == 1 else (" / ".join(map(str, goals[nxt:])) or "any remaining goal")
            out.append(f"said '{m['text'][:80]}' while at ({x:.1f}, {y:.1f}), not in {where}")
    return out


def evaluate(rec: dict) -> dict:
    """Verdict for one record: checks, wrong claims, success."""
    m = suite_module().BY_ID[rec["mission"]]
    results = []
    for chk in m["checks"]:
        ok, why = run_check(tuple(chk), rec)
        results.append({"check": list(chk), "ok": ok, "why": why})
    infra = rec.get("status") in ("sim_hang", "error", "infra_failed", "setup_failed")
    timed_out = rec.get("status") in ("timeout", "wall_timeout")       # wall_timeout: the whole attempt exceeded its wall-clock budget in every retry
    falls = int(rec.get("falls", 0))
    arr = arrival_errors(rec, m.get("goal"))
    ground = int(sum(t.get("grounding_final", 0) for t in rec.get("turns", [])))
    claim_fail = [r for r in results if not r["ok"] and (r["check"][0] in ("no_reply", "map_lacks", "battery_numbers"))]
    wrong = len(arr) + ground + len(claim_fail)
    success = (not infra) and (not timed_out) and falls == 0 and all(r["ok"] for r in results) and not arr
    return {"mission": rec["mission"], "success": success, "infra_failure": infra, "timed_out": timed_out, "falls": falls, "checks": results,
            "arrival_errors": arr, "grounding_violations": ground, "wrong_claims": wrong,
            "wrong_claim_details": arr + [r["why"] for r in claim_fail] + ([f"{ground} grounding violation(s) in final replies"] if ground else [])}
