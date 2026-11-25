"""Scores one incident-graph evaluation run (brain/eval/incident_suite.py, produced by brain/ros/incident_eval_runner.py).

  python3 -m brain.eval.incident_score brain/runs/incident_eval/<run>/result.json [--doc docs/results/phase6_incident_eval.md]

Two independent parts:
  1. GRAPH   the recorded events against the ground-truth world changes: every moved object found (right rooms, after the change), the removed object
             reported missing, and NO moved / missing report for an object that was not touched (false reports).
  2. ANSWERS the 20 typed questions: routing (right tool), content (regexes, clock windows, room sets) and grounding (no unsupported claim).
Pure functions over plain dicts (unit-tested with synthetic records); never re-scores frozen results of other suites."""
from __future__ import annotations

import argparse
import json
import re
from datetime import datetime
from pathlib import Path

from . import incident_suite as S

GREY_S = 90.0                       # rooms whose last visit is this close to the threshold may be listed either way


# ---------------------------------------------------------------- graph
def graph_score(gt: dict, events: list[dict]) -> dict:
    """`events`: the graph's events as dicts (etype, cls, color, room, wall_t, data). Ground truth: gt["ops"] with wall times."""
    out = {"moves": [], "removed": [], "false_reports": []}
    ops = {(o["cls"], o["color"]): o for o in gt["ops"]}
    for o in gt["ops"]:
        key = (o["cls"], o["color"])
        late = [e for e in events if (e.get("cls"), e.get("color") or "") == key and e["wall_t"] >= o["wall"] - 1.0]
        if o["op"] == "move":
            mv = [e for e in late if e["etype"] == "object_moved"]
            right = [e for e in mv if (e["data"].get("from") or {}).get("room") == o["from"][0] and (e["data"].get("to") or {}).get("room") == o["to"][0]]
            first = min((e["wall_t"] for e in right), default=None)
            out["moves"].append({"object": f"{o['color']} {o['cls']}", "found": bool(right), "rooms_right": bool(right), "reported": len(mv),
                                 "delay_s": None if first is None else round(first - o["wall"], 1)})
        else:
            ms = [e for e in late if e["etype"] == "object_missing"]
            out["removed"].append({"object": f"{o['color']} {o['cls']}".strip(), "found": bool(ms), "room": ms[0]["room"] if ms else None,
                                   "delay_s": None if not ms else round(min(e["wall_t"] for e in ms) - o["wall"], 1)})
    for e in events:
        if e["etype"] not in ("object_moved", "object_missing"):
            continue
        key = (e.get("cls"), e.get("color") or "")
        if key not in ops:
            out["false_reports"].append({"type": e["etype"], "object": f"{e.get('color') or ''} {e.get('cls')}".strip(), "room": e.get("room")})
        elif e["etype"] == "object_moved" and e["wall_t"] < ops[key]["wall"] - 1.0:
            out["false_reports"].append({"type": "moved_before_change", "object": f"{e.get('color') or ''} {e.get('cls')}".strip(), "room": e.get("room")})
    out["moves_found"] = sum(m["found"] for m in out["moves"])
    out["removed_found"] = sum(r["found"] for r in out["removed"])
    out["n_false"] = len(out["false_reports"])
    return out


# ---------------------------------------------------------------- answers
def clocks(reply: str) -> list[tuple[int, int]]:
    return [(int(h), int(m)) for h, m in re.findall(r"\b([01]?\d|2[0-3]):([0-5]\d)\b", reply)]


def _local_secs(wall: float) -> float:
    d = datetime.fromtimestamp(wall)
    return d.hour * 3600 + d.minute * 60 + d.second


def _in_window(h: int, m: int, lo: float, hi: float) -> bool:
    """HH:MM is minute-resolution: accept when the minute overlaps [lo, hi] (wall epoch seconds), same day."""
    t = h * 3600 + m * 60
    a, b = _local_secs(lo), _local_secs(hi)
    return a - 60 <= t <= b + 60 if a <= b else (t >= a - 60 or t <= b + 60)


def sweep(gt: dict, rnd: int, room: str) -> dict | None:
    return next((s for s in gt["rounds"].get(str(rnd), []) if s["room"] == room), None)


def unchecked_expectation(gt: dict, asked_wall: float, threshold_s: float) -> dict:
    """Rooms that were definitely / possibly / definitely not in the unchecked list when the question was asked. A room counts as 'last visited' at the end
    of its round-2 look-around; the room the robot is standing in at the end is always recent."""
    last = {s["room"]: s["end"] for s in gt["rounds"].get("2", [])}
    here = gt["rounds"]["2"][-1]["room"] if gt["rounds"].get("2") else None
    sure_old, grey, sure_new = [], [], []
    for r in S.ROUND1:
        if r == here or r not in last:
            sure_new.append(r) if r == here else sure_old.append(r)
            continue
        age = asked_wall - last[r]
        (sure_old if age > threshold_s + GREY_S else sure_new if age < threshold_s - GREY_S else grey).append(r)
    return {"definitely_unchecked": sure_old, "grey": grey, "definitely_recent": sure_new}


def listed_unchecked(reply: str) -> list[str]:
    head = re.split(r"checked recently", reply, flags=re.I)[0]
    if re.search(r"checked every room|every room", head, re.I):
        return []
    return re.findall(r"room [A-D]", head)


def check_answer(q: dict, reply: str, gt: dict, asked_wall: float) -> list[str]:
    """Problems with the content of one answer (empty list = pass)."""
    c, bad = q["check"], []
    for pat in c.get("all", []):
        if not re.search(pat, reply, re.I):
            bad.append(f"missing /{pat}/")
    for pat in c.get("none", []):
        if re.search(pat, reply, re.I):
            bad.append(f"must not say /{pat}/")
    if "time" in c:
        sw = sweep(gt, c["time"]["round"], c["time"]["room"])
        tol = c["time"]["tol_s"]
        cl = clocks(reply)
        if sw is None:
            bad.append("no ground truth for the look-around")
        elif not cl:
            bad.append("no clock time in the reply")
        elif not any(_in_window(h, m, sw["start"] - tol, sw["end"] + tol) for h, m in cl):
            bad.append(f"clock {cl[0][0]:02d}:{cl[0][1]:02d} outside the ground-truth window")
    if "stop" in c:
        sw = gt.get("stop")
        cl = clocks(reply)
        if not sw:
            bad.append("no ground truth for the stop")
        elif not cl:
            bad.append("no clock time in the reply")
        elif not any(_in_window(h, m, sw["wall"] - c["stop"]["tol_s"], sw["wall"] + c["stop"]["tol_s"]) for h, m in cl):
            bad.append("stop time does not match")
    if "unchecked" in c:
        exp = unchecked_expectation(gt, asked_wall, c["unchecked"]["threshold_s"])
        got = set(listed_unchecked(reply))
        bad += [f"{r} should be listed (not visited for {c['unchecked']['threshold_s']:.0f} s)" for r in exp["definitely_unchecked"] if r not in got]
        bad += [f"{r} was visited recently" for r in exp["definitely_recent"] if r in got]
    return bad


def score(result: dict) -> dict:
    gt = result["gt"]
    g = graph_score(gt, result.get("graph_events", []))
    rows = []
    for q in S.QUESTIONS:
        t = next((t for t in result.get("turns", []) if t.get("qid") == q["id"]), None)
        if t is None:
            rows.append({"id": q["id"], "category": q["category"], "text": q["text"], "ran": False, "routed": False, "content": False, "ok": False, "problems": ["not asked"]})
            continue
        tools = [c[0] for c in t.get("calls", [])]
        routed = q["expect_tool"] in tools
        problems = check_answer(q, t["reply"], gt, t.get("wall", gt.get("t_end", 0)))
        wrong_claim = [p for p in problems if p.startswith("must not say")]
        rows.append({"id": q["id"], "category": q["category"], "text": q["text"], "ran": True, "routed": routed, "tools": tools, "reply": t["reply"],
                     "content": not problems, "ok": routed and not problems, "problems": problems, "wrong_claims": len(wrong_claim),
                     "grounding_violations": t.get("grounding_final", 0), "llm_calls": t.get("llm_calls")})
    n = len(rows)
    cats: dict[str, list[dict]] = {}
    for r in rows:
        cats.setdefault(r["category"], []).append(r)
    return {"scenario": S.NAME, "n_questions": n, "answers_ok": sum(r["ok"] for r in rows), "routed": sum(r["routed"] for r in rows),
            "content_ok": sum(r["content"] for r in rows), "wrong_claims": sum(r.get("wrong_claims", 0) for r in rows),
            "grounding_violations": sum(r.get("grounding_violations", 0) for r in rows), "by_category": {k: [sum(x["ok"] for x in v), len(v)] for k, v in cats.items()},
            "graph": g, "rows": rows, "ops_verified": all(o.get("verified") for o in gt["ops"]), "falls": result.get("falls", 0)}


def report(s: dict, result: dict) -> str:
    g = s["graph"]
    L = [f"# Incident graph evaluation ({s['scenario']})", "",
         f"Run: {result.get('started', '?')} commit {result.get('commit', '?')}, model {result.get('model', '?')}, falls {s['falls']}, "
         f"ground-truth changes verified in Gazebo: {s['ops_verified']}.", "",
         "## Graph (events vs ground truth)", "",
         f"- moved objects found with the right rooms: {g['moves_found']} / {len(g['moves'])}; removed object reported missing: {g['removed_found']} / {len(g['removed'])}; "
         f"false moved / missing reports for untouched objects: {g['n_false']}", ""]
    L += [f"  - {m['object']}: found={m['found']} delay {m['delay_s']} s" for m in g["moves"]] + [f"  - {r['object']} (removed): found={r['found']} in {r['room']}, delay {r['delay_s']} s" for r in g["removed"]]
    L += [f"  - FALSE: {f['type']} {f['object']} {f['room']}" for f in g["false_reports"]]
    L += ["", "## Answers", "", f"**{s['answers_ok']} / {s['n_questions']}** correct (right tool AND right content); routed {s['routed']} / {s['n_questions']}, "
          f"content {s['content_ok']} / {s['n_questions']}, wrong claims {s['wrong_claims']}, grounding violations {s['grounding_violations']}.", "",
          "| category | ok | n |", "|---|---|---|"] + [f"| {k} | {a} | {b} |" for k, (a, b) in s["by_category"].items()]
    L += ["", "| id | question | tool | ok | problems |", "|---|---|---|---|---|"]
    for r in s["rows"]:
        L.append(f"| {r['id']} | {r['text']} | {','.join(r.get('tools', []))} | {'yes' if r['ok'] else 'NO'} | {'; '.join(r['problems'])} |")
    L += ["", "## Replies", ""] + [f"- **{r['id']}** {r['text']} -> {r.get('reply', '')}" for r in s["rows"]]
    return "\n".join(L) + "\n"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("result")
    ap.add_argument("--doc")
    a = ap.parse_args()
    result = json.load(open(a.result))
    s = score(result)
    md = report(s, result)
    Path(a.result).with_name("score.json").write_text(json.dumps(s, indent=1, default=str))
    Path(a.result).with_name("report.md").write_text(md)
    if a.doc:
        Path(a.doc).write_text(md)
    print(md.split("## Replies")[0])


if __name__ == "__main__":
    main()
