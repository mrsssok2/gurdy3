"""VLN metrics from GROUND TRUTH (Gazebo /odom/ground_truth trajectory + the world's object list), docs/results/vln_plan.md
section 6. Pure Python.

Per episode (one instruction):
  success     the FINAL pose is in the goal region (room goal: inside the room rectangle; point goal: within `radius`)
              AND no avoided room was entered AND (if the instruction passes landmarks) every one was passed within
              PASS_CHECK. The robot must also have said it arrived (a silent stop in the right place is not a success).
  oracle      some pose along the trajectory was in the goal region (the robot was there at some point)
  path_len    travelled GT path length; ref_len = shortest feasible path (Nav2 compute_path on the GT map, through the
              required landmarks / doors, around avoided rooms) -> SPL term S * ref / max(path, ref)
  wrong_room  the final pose is in a room other than the goal's room (only episodes with a goal)
  avoid_viol  any pose inside an avoided room (the start room is exempt while the robot is still in it)
  false_claims the robot's claims that the ground truth contradicts: "arrived" / "I'm in room X" when the final pose is
              not there, "passed the X" when the trajectory never came within PASS_CHECK of it
  answer cases (expected NOT_FOUND / AMBIGUOUS / IMPOSSIBLE / UNKNOWN_ROOM): correct when the robot answered with that
              kind of reply and did not move more than 0.3 m
"""
from __future__ import annotations

import math
from statistics import mean

from .plan import PASS_CHECK


def path_length(traj) -> float:
    return sum(math.dist(a[1:3], b[1:3]) for a, b in zip(traj, traj[1:]))


def in_goal(x, y, goal, world) -> bool:
    if goal["kind"] == "room":
        return world.room_of(x, y) == goal["room"]
    return math.hypot(x - goal["x"], y - goal["y"]) <= goal.get("radius", 0.5)


def episode(traj, goal, world, ref_len: float, avoid=(), pass_landmarks=(), claims=(), arrived_claimed: bool = True) -> dict:
    """traj: [(t, x, y), ...] ground truth; goal: Plan.goal; pass_landmarks: [{x, y}]; claims: [{type: arrived|in_room|passed, ...}]."""
    if not traj:
        return {"success": False, "oracle": False, "path_len": 0.0, "ref_len": ref_len, "spl": 0.0, "wrong_room": False,
                "avoid_viol": False, "false_claims": 0, "reason": "no trajectory"}
    _, fx, fy = traj[-1]
    start_room = world.room_of(traj[0][1], traj[0][2])
    left_start = False
    viol = False
    for _, x, y in traj:
        r = world.room_of(x, y)
        left_start = left_start or (r != start_room and r is not None)
        if r in avoid and (r != start_room or left_start):
            viol = True
    passed = all(min(math.hypot(x - L["x"], y - L["y"]) for _, x, y in traj) <= PASS_CHECK for L in pass_landmarks)
    at = in_goal(fx, fy, goal, world)
    oracle = any(in_goal(x, y, goal, world) for _, x, y in traj)
    succ = at and not viol and passed and arrived_claimed
    plen = path_length(traj)
    spl = (ref_len / max(plen, ref_len)) if succ and ref_len > 0 else (1.0 if succ else 0.0)
    final_room = world.room_of(fx, fy)
    wrong = goal.get("room") is not None and final_room is not None and final_room != goal["room"]
    false = 0
    for c in claims:
        if c["type"] == "arrived" and not at:
            false += 1
        elif c["type"] == "in_room" and final_room != c["room"]:
            false += 1
        elif c["type"] == "passed" and min(math.hypot(x - c["x"], y - c["y"]) for _, x, y in traj) > PASS_CHECK:
            false += 1
    return {"success": succ, "oracle": oracle, "path_len": round(plen, 3), "ref_len": ref_len, "spl": round(spl, 4),
            "wrong_room": wrong, "avoid_viol": viol, "false_claims": false, "passed_all": passed}


def answer_episode(expected_code: str, got_code: str | None, traj) -> dict:
    moved = path_length(traj) if traj else 0.0
    return {"answer_ok": got_code == expected_code and moved <= 0.3, "moved_m": round(moved, 3), "expected": expected_code, "got": got_code}


def aggregate(nav: list, answers: list = ()) -> dict:
    n = len(nav)
    out = {"n_nav": n, "n_answer": len(answers)}
    if n:
        out.update({"success_rate": mean(e["success"] for e in nav), "spl": mean(e["spl"] for e in nav),
                    "oracle_success": mean(e["oracle"] for e in nav), "wrong_room_rate": mean(e["wrong_room"] for e in nav),
                    "avoid_violation_rate": mean(e["avoid_viol"] for e in nav), "false_claims": sum(e["false_claims"] for e in nav)})
    if answers:
        out["answer_accuracy"] = mean(a["answer_ok"] for a in answers)
    return {k: (round(v, 4) if isinstance(v, float) else v) for k, v in out.items()}
