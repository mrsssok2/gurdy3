"""Gazebo VLN evaluation, the pure part (no ROS): the episode file, the ground-truth world, landmark seeding from GT, and the
GT-graded score of one episode + the aggregate + the markdown report. scripts/vln_eval.sh runs one episode per fresh stack
(brain/ros/vln_eval_runner.py writes one record per episode); `python3 -m brain.vln.gazebo_eval RUN_DIR` scores a run.

Grading (docs/results/vln_plan.md section 7):
  nav episode (expect = a reference plan): the reference is validated on the GROUND-TRUTH world from the robot's real start
    pose -> goal region, landmarks to pass, avoided rooms, reference length (door-point estimate). metrics.episode on the GT
    trajectory: success (final pose in the goal, nothing avoided entered, every landmark passed within 1.2 m, AND the robot
    said it arrived), SPL, oracle success, wrong room, avoid violation, false claims. A refusal of a nav episode = failure
    (counted as an over-refusal).
  answer episode (expect = {error: CODE}): the plan error / walk code must equal CODE and the robot must move <= 0.3 m.
  infra (no valid record: the stack never started, Gazebo hung, wall timeout): reported, kept OUT of the rates.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import yaml

from . import metrics as M
from .plan import validate
from .world import VLNWorld, room_names

ROOT = Path(__file__).resolve().parents[2]
ERROR_CODES = {"LANDMARK_NOT_FOUND", "AMBIGUOUS", "UNKNOWN_ROOM", "UNKNOWN_LANDMARK", "IMPOSSIBLE", "NO_FREE_SPOT", "UNREACHABLE"}
OK_STATUS = ("ok", "timeout")
NEAR_M = 1.2       # "stop near / at X" is reached within this distance of the GT landmark (stand-offs go to 1.0 m + Nav2's 0.12 m tolerance)


def _path(p) -> Path:
    p = Path(p)
    return p if p.is_absolute() else ROOT / p


def load_suite(path) -> dict:
    """-> {"name", "world", "map_name", "places", "gt", "episodes": [{id, text, limit_s, expect}]}; ValueError on any problem."""
    d = yaml.safe_load(Path(path).read_text()) or {}
    s = {"name": Path(path).stem, "world": d.get("world", "lab4_vln"), "map_name": d.get("map_name", "lab4_map"),
         "places": d.get("places") or f"maps/{d.get('world', 'lab4_vln')}.places.yaml",
         "gt": d.get("gt") or f"docs/results/data/{d.get('world', 'lab4_vln')}/{d.get('world', 'lab4_vln')}_gt.json", "episodes": []}
    eps = d.get("episodes") or []
    if not eps:
        raise ValueError("no episodes")
    seen = set()
    w = gt_world(s)
    for e in eps:
        eid, text, exp = e.get("id"), e.get("text"), e.get("expect")
        if not eid or not text:
            raise ValueError(f"episode without id / text: {e}")
        if eid in seen:
            raise ValueError(f"duplicate episode id {eid}")
        seen.add(eid)
        if not isinstance(exp, dict):
            raise ValueError(f"{eid}: no expect (a reference plan or {{error: CODE}})")
        if "error" in exp:
            if exp["error"] not in ERROR_CODES:
                raise ValueError(f"{eid}: unknown error code {exp['error']!r}")
        else:
            p = validate(json.dumps(exp), text, w)
            if not p.ok:
                raise ValueError(f"{eid}: the reference plan does not validate on the GT world: {p.error} {p.reply or p.notes}")
        s["episodes"].append({"id": eid, "text": text, "limit_s": float(e.get("limit_s", 900)), "expect": exp})
    return s


def gt_objects(suite) -> list:
    return [dict(o) for o in json.loads(_path(suite["gt"]).read_text())["objects"]]


def gt_world(suite, start=(0.0, 0.0, 0.0)) -> VLNWorld:
    """Rooms + names from the places file, doors + objects from the world's ground truth (sigma 0), robot at `start`."""
    places = yaml.safe_load(_path(suite["places"]).read_text())["places"]
    gt = json.loads(_path(suite["gt"]).read_text())
    rooms = {k: tuple(float(c) for c in v["rect"]) for k, v in places.items() if v.get("kind") == "room"}
    objs = [{"id": i + 1, "name": o["name"], "class": o["class"], "color": o.get("color") or "", "x": o["x"], "y": o["y"]}
            for i, o in enumerate(gt["objects"])]
    st = tuple(start) + (0.0,) * (3 - len(start))
    return VLNWorld(rooms, {k: tuple(v) for k, v in gt["doors"].items()}, objs, st[:3], names=room_names(places), places=tuple(places))


def seed_landmarks(gt: list, live: list, match_m: float = 1.0, sigma: float = 0.10) -> list:
    """Landmarks for the 'gt' mode: POSITIONS from the world's ground truth (with an assumed map sigma), CONFIRMATION from the
    live detector: an object's last_seen is that of a live track of the same class (same colour when both have one) within
    match_m, else -1 (never seen: the executor's re-observation check will not confirm it). Live tracks not in GT are dropped."""
    out = []
    for i, o in enumerate(gt):
        col = o.get("color") or ""
        cands = [t for t in live or [] if t.get("class") == o["class"] and (not col or not t.get("color") or t.get("color") == col)
                 and ((t["x"] - o["x"]) ** 2 + (t["y"] - o["y"]) ** 2) ** 0.5 <= match_m]
        seen = max((t.get("last_seen") if t.get("last_seen") is not None else -1.0 for t in cands), default=-1.0)
        out.append({"id": 1001 + i, "class": o["class"], "color": col, "x": o["x"], "y": o["y"], "sigma_m": sigma,
                    "last_seen": seen, "hits": 99, "name": o.get("name")})
    return out


def reference(ep, world, start):
    """The reference plan on the GT world from the robot's real start (x, y[, yaw]). For a final "stop at an object" the goal
    becomes {"kind": "near", landmark x / y, NEAR_M, room}: which free side the robot's planner picks (its landmark sigmas
    differ from GT's 0) is not graded; relation goals ("left of X", "between") keep the exact point (radius 0.5)."""
    w = VLNWorld(world.rooms, world.doors, world.objects, (float(start[0]), float(start[1]), 0.0), names=world.names, places=world.places)
    p = validate(json.dumps(ep["expect"]), ep["text"], w)
    last = (ep["expect"].get("steps") or [{}])[-1]
    if p.ok and last.get("action") == "stop" and "object" in last.get("target", {}) and p.waypoints[-1].get("landmark"):
        L = p.waypoints[-1]["landmark"]
        p.goal = {"kind": "near", "x": L["x"], "y": L["y"], "radius": NEAR_M, "room": p.goal["room"]}
    return p


def score_episode(ep, rec, world) -> dict:
    out = {"id": ep["id"], "text": ep["text"], "status": rec.get("status"), "reply": rec.get("reply")}
    if rec.get("status") not in OK_STATUS:
        return {**out, "kind": "infra", "error": rec.get("error")}
    traj = [tuple(p[:3]) for p in rec.get("gt_traj") or []]
    plan = rec.get("plan") or {}
    res = rec.get("result") or {}
    got = plan.get("error") if plan.get("error") else (res.get("code") if res and not res.get("ok") else None)
    if "error" in ep["expect"]:
        a = M.answer_episode(ep["expect"]["error"], got, traj)
        return {**out, "kind": "answer", **a}
    if not traj:
        return {**out, "kind": "nav", "success": False, "reason": "no ground-truth trajectory", "spl": 0.0, "wrong_room": False,
                "avoid_viol": False, "false_claims": 0, "path_len": 0.0, "ref_len": 0.0, "oracle": False, "refused": got}
    ref = reference(ep, world, traj[0][1:3])
    if not ref.ok:
        return {**out, "kind": "infra", "error": f"reference plan invalid from the start pose: {ref.error}"}
    e = M.episode(traj, ref.goal, world, ref.length_m, avoid=set(ref.avoid),
                  pass_landmarks=[w["landmark"] for w in ref.waypoints if w["kind"] == "pass" and "landmark" in w],
                  claims=res.get("claims") or [], arrived_claimed=bool(res.get("ok")))
    refused = plan.get("error") if plan.get("error") else None
    return {**out, "kind": "nav", **e, "refused": refused, "code": res.get("code"), "timeout": rec.get("status") == "timeout"}


def aggregate(scores: list) -> dict:
    nav = [s for s in scores if s["kind"] == "nav"]
    ans = [s for s in scores if s["kind"] == "answer"]
    agg = M.aggregate(nav, ans)
    agg.update({"n": len(scores), "infra_failures": sum(s["kind"] == "infra" for s in scores),
                "over_refusals": sum(1 for s in nav if s.get("refused")), "false_claims": sum(s.get("false_claims", 0) for s in nav),
                "mean_path_len": round(sum(s["path_len"] for s in nav) / len(nav), 3) if nav else None})
    return agg


def report_md(name: str, scores: list, agg: dict, meta: dict | None = None) -> str:
    L = [f"# VLN Gazebo run: {name}", ""]
    if meta:
        L += [", ".join(f"{k}: {v}" for k, v in meta.items()), ""]
    L += ["| metric | value |", "|---|---|"]
    rows = [("navigation episodes", agg.get("n_nav")), ("success rate", agg.get("success_rate")), ("SPL", agg.get("spl")),
            ("oracle success", agg.get("oracle_success")), ("mean path length (m)", agg.get("mean_path_len")),
            ("wrong-room rate", agg.get("wrong_room_rate")), ("avoid-violation rate", agg.get("avoid_violation_rate")),
            ("false claims", agg.get("false_claims")), ("over-refusals (nav episodes refused)", agg.get("over_refusals")),
            ("refusal episodes", agg.get("n_answer")), ("refusal accuracy", agg.get("answer_accuracy")),
            ("infrastructure failures (not in the rates)", agg.get("infra_failures"))]
    L += [f"| {k} | {'-' if v is None else v} |" for k, v in rows]
    L += ["", "| episode | kind | result | path / ref (m) | SPL | wrong room | avoid viol. | false claims | reply |", "|---|---|---|---|---|---|---|---|---|"]
    for s in scores:
        rep = (s.get("reply") or "").replace("|", "/")[:140]
        if s["kind"] == "infra":
            L.append(f"| {s['id']} | INFRA | {s.get('status')}: {(s.get('error') or '')[:100]} | | | | | | |")
        elif s["kind"] == "answer":
            L.append(f"| {s['id']} | refusal | {'OK' if s['answer_ok'] else 'WRONG'} (got {s['got']}, expected {s['expected']}, moved {s['moved_m']} m) | | | | | | {rep} |")
        else:
            res = "SUCCESS" if s["success"] else ("REFUSED " + s["refused"] if s.get("refused") else ("TIMEOUT" if s.get("timeout") else "FAIL"))
            L.append(f"| {s['id']} | nav | {res} | {s['path_len']} / {s['ref_len']} | {s['spl']} | {s['wrong_room']} | {s['avoid_viol']} | {s['false_claims']} | {rep} |")
    return "\n".join(L) + "\n"


def score_run(run_dir, suite_path) -> dict:
    run = Path(run_dir)
    s = load_suite(suite_path)
    w = gt_world(s)
    scores = []
    for ep in s["episodes"]:
        f = run / "episodes" / f"{ep['id']}.json"
        if not f.exists():
            continue
        try:
            rec = json.loads(f.read_text())
        except ValueError as e:
            rec = {"status": "unreadable", "error": str(e)}
        scores.append(score_episode(ep, rec, w))
    agg = aggregate(scores)
    meta = json.loads((run / "meta.json").read_text()) if (run / "meta.json").exists() else None
    (run / "scores.json").write_text(json.dumps(scores, indent=1) + "\n")
    (run / "summary.json").write_text(json.dumps(agg, indent=1) + "\n")
    (run / "report.md").write_text(report_md(s["name"], scores, agg, meta))
    return agg


if __name__ == "__main__":
    print(json.dumps(score_run(sys.argv[1], sys.argv[2])))
