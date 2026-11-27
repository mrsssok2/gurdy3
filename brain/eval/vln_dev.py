"""VLN DEV instruction set + offline eval of the plan step (instruction -> LLM plan -> validated, grounded plan).

    python3 -m brain.eval.vln_dev                     # the real model (project Ollama, gurdy-sft-400): NOT run yet
    python3 -m brain.eval.vln_dev --model qwen3:1.7b --out FILE.json
    python3 -m brain.eval.vln_dev --oracle            # each case's reference plan as the "model" (checks the validator)
    python3 -m brain.eval.vln_dev --mock [--oracle | --model M]   # END TO END through the agent on the lab4 MOCK backend (walks)

DEV set, NOT held-out: written by the assistant on 2026-11-27 while building brain/vln (the user writes the blind
held-out VLN suite). World = lab4 with all 10 big ground-truth objects in the semantic map, the robot at home (0, 0) in
room A. Each case: (id, text, reference plan = what a perfect model returns (None = {"kind": "none"}), expectation):
  {"rooms": [...], "goal": "room D" | ("point", "room D")}   valid plan: room sequence + goal
  {"error": CODE}                                            answered failure (LANDMARK_NOT_FOUND, AMBIGUOUS, IMPOSSIBLE, UNKNOWN_ROOM, NO_FREE_SPOT)
Room sequences are the GEOMETRICALLY shortest route through the door points (checked: A-B-D beats A-C-D by 0.13-1.9 m
for the room D goals except the toolbox), not a guess.
  "none"                                                     not a navigation instruction (planner returns None)
Only the PLAN is scored here (no walking). Gazebo metrics: brain/vln/metrics.py, docs/results/vln_plan.md section 6.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

from ..vln.planner import VLNPlanner
from ..vln.world import VLNWorld


def R(cls, color=None, in_room=None):
    return {"class": cls, **({"color": color} if color else {}), **({"in_room": in_room} if in_room else {})}


def S(action, **t):
    if "relation" in t:
        return {"action": action, "target": {"relation": t["relation"], "of": t["of"]}}
    return {"action": action, "target": t}


def GO(*steps, avoid=()):
    return {"kind": "go", "steps": list(steps), "avoid": list(avoid)}


A_, B_, C_, D_ = "room A", "room B", "room C", "room D"
DEV = [
    # ---- object goal (a landmark is the goal)
    ("o01", "go to the red barrel", GO(S("stop", object=R("barrel", "red"))), {"rooms": [A_, B_, D_], "goal": ("point", D_)}),
    ("o02", "take me to the mannequin", GO(S("stop", object=R("mannequin"))), {"rooms": [A_, B_, D_], "goal": ("point", D_)}),
    ("o03", "walk over to the extinguisher", GO(S("stop", object=R("fire_extinguisher"))), {"rooms": [A_, C_], "goal": ("point", C_)}),
    ("o04", "head to the blue box", GO(S("stop", object=R("box", "blue"))), {"rooms": [A_, B_], "goal": ("point", B_)}),
    ("o05", "go and stand at the toolbox", GO(S("stop", object=R("toolbox"))), {"rooms": [A_, C_, D_], "goal": ("point", D_)}),
    # ---- room goal by its contents
    ("w01", "go to the room with the red barrel", GO(S("enter", room_with=R("barrel", "red"))), {"rooms": [A_, B_, D_], "goal": D_}),
    ("w02", "take me into the room that has the extinguisher", GO(S("enter", room_with=R("fire_extinguisher"))), {"rooms": [A_, C_], "goal": C_}),
    ("w03", "go to the room with the blue barrel", GO(S("enter", room_with=R("barrel", "blue"))), {"rooms": [A_, B_], "goal": B_}),
    ("w04", "find your way to the room with the yellow box", GO(S("enter", room_with=R("box", "yellow"))), {"rooms": [A_, C_], "goal": C_}),
    ("w05", "go to the room with the cone", GO(S("enter", room_with=R("traffic_cone"))), {"error": "AMBIGUOUS"}),
    ("w06", "go to the room with the toolbox, but stay out of room C", GO(S("enter", room_with=R("toolbox")), avoid=[C_]),
     {"rooms": [A_, B_, D_], "goal": D_}),
    # ---- sequential
    ("q01", "go to room B, then room D", GO(S("enter", room=B_), S("enter", room=D_)), {"rooms": [A_, B_, D_], "goal": D_}),
    ("q02", "first walk into room C and then go to the toolbox", GO(S("enter", room=C_), S("stop", object=R("toolbox"))),
     {"rooms": [A_, C_, D_], "goal": ("point", D_)}),
    ("q03", "visit room B and afterwards stop next to the mannequin", GO(S("enter", room=B_), S("stop", relation="next_to", of=[R("mannequin")])),
     {"rooms": [A_, B_, D_], "goal": ("point", D_)}),
    ("q04", "go to room C, then to room D, and finish in room B", GO(S("enter", room=C_), S("enter", room=D_), S("enter", room=B_)),
     {"rooms": [A_, C_, D_, B_], "goal": B_}),
    # ---- landmark-based route ("past", "by", "through")
    ("p01", "go past the blue box into room D", GO(S("pass", object=R("box", "blue")), S("enter", room=D_)), {"rooms": [A_, B_, D_], "goal": D_}),
    ("p02", "walk past the extinguisher and into room D", GO(S("pass", object=R("fire_extinguisher")), S("enter", room=D_)),
     {"rooms": [A_, C_, D_], "goal": D_}),
    ("p03", "go past the cone in room A and then into room B", GO(S("pass", object=R("traffic_cone", in_room=A_)), S("enter", room=B_)),
     {"rooms": [A_, B_], "goal": B_}),
    ("p04", "go past the red box, then to the blue barrel", GO(S("pass", object=R("box", "red")), S("stop", object=R("barrel", "blue"))),
     {"rooms": [A_, B_], "goal": ("point", B_)}),
    ("p05", "go through room B to the red barrel", GO(S("pass", room=B_), S("stop", object=R("barrel", "red"))),
     {"rooms": [A_, B_, D_], "goal": ("point", D_)}),
    ("p06", "go past the barrel into room C", GO(S("pass", object=R("barrel")), S("enter", room=C_)), {"error": "AMBIGUOUS"}),
    # ---- relational goals
    # r01: seen from the A-B door, the left of the blue barrel is a 0.47 m gap to the B-D wall (the robot needs ~0.7 m): honest refusal
    # (end to end on the mock, whose map has 4 cm observation noise, the gap can come out just wide enough: an arrival is
    # accepted there only if the final pose is free in the TRUE occupancy, see run_mock / vln_plan.md section 11)
    ("r01", "stop to the left of the blue barrel", GO(S("stop", relation="left_of", of=[R("barrel", "blue")])), {"error": "NO_FREE_SPOT", "knife_edge": True}),
    ("r08", "stop to the left of the mannequin", GO(S("stop", relation="left_of", of=[R("mannequin")])), {"rooms": [A_, B_, D_], "goal": ("point", D_)}),
    ("r02", "wait to the right of the mannequin", GO(S("stop", relation="right_of", of=[R("mannequin")])), {"rooms": [A_, B_, D_], "goal": ("point", D_)}),
    ("r03", "go between the mannequin and the toolbox", GO(S("stop", relation="between", of=[R("mannequin"), R("toolbox")])),
     {"rooms": [A_, C_, D_], "goal": ("point", D_)}),
    # r04: ~0.10 m spare clearance to the wall (~2.5 sigma of the mock map): exact map -> arrives; noisy map (mock e2e,
    # sigma 0.04, sigma_k 3) -> a consistent NO_FREE_SPOT is the intended behaviour (accepted there, vln_plan.md section 12)
    ("r04", "stand behind the yellow box", GO(S("stop", relation="behind", of=[R("box", "yellow")])), {"rooms": [A_, C_], "goal": ("point", C_), "sigma_refusal_ok": True}),
    ("r09", "stand behind the mannequin", GO(S("stop", relation="behind", of=[R("mannequin")])), {"rooms": [A_, B_, D_], "goal": ("point", D_)}),
    ("r05", "go in front of the red barrel", GO(S("stop", relation="in_front_of", of=[R("barrel", "red")])), {"rooms": [A_, B_, D_], "goal": ("point", D_)}),
    ("r06", "go next to the extinguisher", GO(S("stop", relation="next_to", of=[R("fire_extinguisher")])), {"rooms": [A_, C_], "goal": ("point", C_)}),
    ("r07", "park between the red box and the cone", GO(S("stop", relation="between", of=[R("box", "red"), R("traffic_cone")])), {"error": "AMBIGUOUS"}),
    # ---- negations / constraints
    ("n01", "go to room D but avoid room B", GO(S("enter", room=D_), avoid=[B_]), {"rooms": [A_, C_, D_], "goal": D_}),
    ("n02", "take me to the mannequin without going through room C", GO(S("stop", object=R("mannequin")), avoid=[C_]),
     {"rooms": [A_, B_, D_], "goal": ("point", D_)}),
    ("n03", "go to the red barrel and stay out of room C", GO(S("stop", object=R("barrel", "red")), avoid=[C_]),
     {"rooms": [A_, B_, D_], "goal": ("point", D_)}),
    ("n04", "go to room B, don't go through room C", GO(S("enter", room=B_), avoid=[C_]), {"rooms": [A_, B_], "goal": B_}),
    ("n05", "go to room D avoiding room B and room C", GO(S("enter", room=D_), avoid=[B_, C_]), {"error": "IMPOSSIBLE"}),
    ("n06", "go to the blue barrel but avoid room B", GO(S("stop", object=R("barrel", "blue")), avoid=[B_]), {"error": "IMPOSSIBLE"}),
    ("n07", "go to room C without going through room A", GO(S("enter", room=C_), avoid=[A_]), {"rooms": [A_, C_], "goal": C_}),
    # ---- failures
    ("f01", "go to the chair", GO(S("stop", object=R("chair"))), {"error": "LANDMARK_NOT_FOUND"}),
    ("f02", "go to the green barrel", GO(S("stop", object=R("barrel", "green"))), {"error": "LANDMARK_NOT_FOUND"}),
    ("f03", "go to the cone", GO(S("stop", object=R("traffic_cone"))), {"error": "AMBIGUOUS"}),
    ("f04", "go to room E", GO(S("enter", room="room E")), {"error": "UNKNOWN_ROOM"}),
    ("f05", "go over to the box", GO(S("stop", object=R("box"))), {"error": "AMBIGUOUS"}),
    # ---- not navigation
    ("x01", "what is in room B?", None, "none"),
    ("x02", "find the red box", None, "none"),
    ("x03", "how much battery do I have", None, "none"),
    ("x04", "stop", None, "none"),
]


def world():
    return VLNWorld.lab4()


def check(p, exp) -> tuple[bool, str]:
    if exp == "none":
        return p is None, "" if p is None else f"expected no plan, got {p.error or 'a plan'}"
    if p is None:
        return False, "no plan (rejected / not routed)"
    if "error" in exp:
        return p.error == exp["error"], f"error {p.error} (expected {exp['error']})"
    if not p.ok:
        return False, f"error {p.error}: {p.reply}"
    g = exp["goal"]
    goal_ok = (p.goal["kind"] == "room" and p.goal["room"] == g) if isinstance(g, str) else (p.goal["kind"] == "point" and p.goal["room"] == g[1])
    ok = p.rooms == exp["rooms"] and goal_ok
    return ok, "" if ok else f"rooms {p.rooms} goal {p.goal} (expected {exp['rooms']} {g})"


class OracleLLM:
    """The reference plan of each case as the model's answer (tests the validator, not a model)."""

    def __init__(self):
        self.table = {t: json.dumps(pl if pl is not None else {"kind": "none"}) for _, t, pl, _ in DEV}

    def __call__(self, system, user):
        return self.table[json.loads(user.split(": ", 1)[1])]


def run(planner: VLNPlanner, verbose=True) -> dict:
    w = world()
    rows, n_ok, by = [], 0, {}
    for cid, text, _, exp in DEV:
        p = planner.plan(text, w)
        ok, why = check(p, exp)
        n_ok += ok
        g = cid[0]
        by.setdefault(g, [0, 0])
        by[g][0] += ok
        by[g][1] += 1
        rows.append({"id": cid, "text": text, "ok": ok, "why": why, "error": p.error if p else None,
                     "summary": p.summary if p and p.ok else None, "raw": planner.last_raw})
        if verbose and not ok:
            print(f"  WRONG {cid} {text!r}: {why}\n        raw {planner.last_raw!r}")
    names = {"o": "object goal", "w": "room with X", "q": "sequential", "p": "past / through", "r": "relational",
             "n": "negation", "f": "failure", "x": "not nav"}
    out = {"n": len(DEV), "ok": n_ok, "by_type": {names[k]: f"{v[0]}/{v[1]}" for k, v in by.items()},
           "calls": planner.calls, "failures": planner.failures, "rejected": planner.rejected, "rows": rows}
    if verbose:
        print({k: v for k, v in out.items() if k != "rows"})
    return out


def mock_agent(planner):
    """Agent (keyword planner for everything else) + lab4 mock backend, robot at home, all 10 objects already in the map
    (like the DEV world), VLN ON."""
    from ..agent.graph import Agent
    from ..agent.llm import KeywordLLM
    from ..mock import MockBackend, gazebo_lab4
    from ..tools import ToolLayer
    be = MockBackend(gazebo_lab4())
    be.x, be.y, be.yaw = 0.0, 0.0, 0.0
    for o in be.world.objects:
        be._sight(o)
    return Agent(ToolLayer(be), KeywordLLM(be.known_places), vln=planner), be


def run_mock(planner_factory, verbose=True) -> dict:
    """Every DEV case end to end on the mock: the agent's reply, the walk, and the metrics against GROUND TRUTH (the goal is
    re-computed from the true object positions with the reference plan, not from the robot's semantic map)."""
    from ..vln import metrics as M
    from ..vln.plan import validate
    gt_world = world()
    nav, answers, rows = [], [], []
    for cid, text, ref, exp in DEV:
        agent, be = mock_agent(planner_factory())
        reply = agent.say(text).reply
        res = agent.vln_last
        traj = res.trajectory if res else [(0.0, 0.0, 0.0), (be.now(), be.x, be.y)]
        row = {"id": cid, "text": text, "reply": reply, "code": res.code if res else None}
        if exp == "none":
            row["ok"] = res is None and agent.vln.last_plan is None or (agent.vln.last_plan is not None and not agent.vln.last_plan.ok
                                                                         and not agent.vln.last_plan.answered)
        elif "error" in exp:
            got = agent.vln.last_plan.error if agent.vln.last_plan else None
            if got is None and exp["error"] == "UNKNOWN_ROOM" and "there is no room" in reply.lower():
                got = "UNKNOWN_ROOM"                     # the agent's own room guard (brain/agent/rooms.py) answered first: same answer
            a = M.answer_episode(exp["error"], got, traj)
            if exp.get("knife_edge") and res is not None and res.ok and be.world.is_free(be.x, be.y):
                a = {**a, "answer_ok": True, "knife_edge_arrival": True}
            answers.append(a)
            row["ok"] = a["answer_ok"] and (res is None or a.get("knife_edge_arrival", False))
            row["knife_edge_arrival"] = a.get("knife_edge_arrival", False)
        elif exp.get("sigma_refusal_ok") and res is None and agent.vln.last_plan is not None and agent.vln.last_plan.error == "NO_FREE_SPOT" \
                and math.hypot(be.x, be.y) < 0.3:
            row.update(ok=True, sigma_refusal=True)         # borderline spot, noisy map: refused without moving (by design)
        else:
            g = validate(json.dumps(ref), text, gt_world)
            e = M.episode(traj, g.goal, gt_world, g.length_m, avoid=set(g.avoid),
                          pass_landmarks=[w["landmark"] for w in g.waypoints if w["kind"] == "pass" and "landmark" in w],
                          claims=res.claims if res else [], arrived_claimed=bool(res and res.ok))
            nav.append(e)
            row.update(ok=e["success"] and e["false_claims"] == 0, episode=e)
        rows.append(row)
        if verbose and not row["ok"]:
            print(f"  WRONG {cid} {text!r}: {reply!r} ({row.get('code')})")
    agg = M.aggregate(nav, answers)
    out = {"n": len(DEV), "ok": sum(r["ok"] for r in rows), **agg, "rows": rows}
    if verbose:
        print({k: v for k, v in out.items() if k != "rows"})
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", action="append", default=[])
    ap.add_argument("--oracle", action="store_true")
    ap.add_argument("--mock", action="store_true", help="end to end through the agent on the lab4 mock backend")
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    res = []
    if a.mock:
        fac = (lambda: VLNPlanner(OracleLLM())) if a.oracle else (lambda: VLNPlanner(model=(a.model or ["gurdy-sft-400"])[0]))
        res.append({"model": "oracle" if a.oracle else (a.model or ["gurdy-sft-400"])[0], "mode": "mock_e2e", **run_mock(fac)})
    elif a.oracle:
        res.append({"model": "oracle", **run(VLNPlanner(OracleLLM()))})
    else:
        for m in a.model or ["gurdy-sft-400"]:
            print(f"== {m}")
            pl = VLNPlanner(model=m)
            r = run(pl)
            if pl.calls and pl.failures == pl.calls:
                print(f"!! every call failed: is Ollama running at {pl.llm.host} with {m!r}?", file=sys.stderr)
            res.append({"model": m, **r})
    if a.out:
        Path(a.out).write_text(json.dumps(res, indent=1) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
