"""HELD-OUT incident-graph evaluation for the Gurdy 3.0 `lab4` world (Phase 6, temporal memory). COMMITTED BEFORE IT WAS RUN.

ONE continuous session in ONE Gazebo stack (the graph needs time to pass), driven by `brain/ros/incident_eval_runner.py`:

  round 1   the robot looks around every room (explore, one room after the other, order A B D C): every object is seen where it is
  user      typed "go to room B", 12 s later typed "stop" (a user stop with a cause), then typed "hello" (= the user leaves: the "while I was away" boundary)
  world     Gazebo `set_pose` moves two objects, `remove` deletes one (ground truth, logged with the wall time, verified by reading the pose back):
              red box        room A (-1.2,-1.3)  ->  room C (0.6, 4.9)
              red barrel     room D ( 5.1, 5.0)  ->  room B (4.0,-0.2)
              toolbox        room D ( 3.2, 5.3)  ->  removed
  round 2   the robot looks around every room again, order B D C A (so the red box is seen in C BEFORE room A is re-checked and the red barrel in B
            BEFORE room D is re-checked: both identity paths of the recorder are exercised)
  wait      300 sim seconds in the last room (for "which rooms have you not checked for 5 minutes")
  questions the 20 questions below, typed, one after the other (nothing else is typed in between)

Everything not listed under `world` stays untouched (blue box, both cones, mannequin, extinguisher, yellow box, blue barrel): a change reported for any
of them is a FALSE report. The question texts occur in no unit test, test set, SFT data or older suite (`check_overlap`, tested). Written by the
assistant that also wrote the question router, so the PHRASINGS ARE NOT INDEPENDENT of it (some are deliberately less regular: failures to understand
them are findings, not scorer errors). The expected answers are fixed here from the ground truth, not from the agent.

Checks per question (`check` dict, interpreted by brain/eval/incident_score.py):
  all / none  regexes (case-insensitive) that must / must not match the reply
  time        the reply must quote a clock time (HH:MM) inside a ground-truth window: (round, room) = start .. end of that room's look-around, +- tol_s
  unchecked   "which rooms ..." answer vs the rooms the robot was in at least / at most `threshold_s` before the question (a +- 90 s grey zone is allowed)
  stop        the clock time must be within tol_s of the typed stop
`expect_tool` is the tool the question should be routed to (reported separately: routing vs content)."""
from __future__ import annotations

import re

from .final_suite_lab4 import MAP_ID, MAP_NAME, PLACES_FILE, WORLD  # noqa: F401  (the same world / saved map / places as the lab4 held-out suites)

NAME = "incident_v1"
ROUND1 = ["room A", "room B", "room D", "room C"]
ROUND2 = ["room B", "room D", "room C", "room A"]
SWEEP_LIMIT_S = 300                     # sim seconds per room look-around (go there + 4 headings)
FINAL_WAIT_S = 300

# ground-truth world changes: model names are the SDF names (scripts/gen_world_lab4.py prefixes `l4_`); `room` = where the object was / is
OPS = [
    {"op": "move", "model": "l4_red_box", "cls": "box", "color": "red", "from": ("room A", -1.2, -1.3), "to": ("room C", 0.6, 4.9)},
    {"op": "move", "model": "l4_barrel_red", "cls": "barrel", "color": "red", "from": ("room D", 5.1, 5.0), "to": ("room B", 4.0, -0.2)},
    {"op": "remove", "model": "l4_toolbox", "cls": "toolbox", "color": "", "from": ("room D", 3.2, 5.3), "to": None},
]
CHANGED = {("box", "red"), ("barrel", "red"), ("toolbox", "")}
UNCHANGED_WORDS = r"blue box|yellow box|cone|mannequin|extinguisher|blue barrel"

SCENARIO = [
    ("round", 1, ROUND1),
    ("say", "go to room B"), ("wait", 12), ("stop", "stop"),
    ("say", "hello"), ("mark", "left"),
    ("world",),
    ("round", 2, ROUND2),
    ("wait", FINAL_WAIT_S),
]


def Q(qid, cat, text, tool, **check):
    return {"id": qid, "category": cat, "text": text, "expect_tool": tool, "check": check}


QUESTIONS = [
    # ---- what changed (6)
    Q("q01", "changes", "Did anything change while I was gone?", "what_changed",
      all=[r"red box", r"barrel", r"toolbox"], none=[UNCHANGED_WORDS]),
    Q("q02", "changes", "What has changed in room C since I left?", "what_changed",
      all=[r"red box"], none=[r"toolbox", r"barrel", r"nothing (has )?changed"]),
    Q("q03", "changes", "Anything new in room A since I left?", "what_changed",
      all=[r"red box"], none=[r"toolbox", r"barrel", r"nothing (has )?changed"]),
    Q("q04", "changes", "What has changed in room B since I left?", "what_changed",
      all=[r"barrel"], none=[r"red box", r"toolbox", r"nothing (has )?changed"]),
    Q("q05", "changes", "Did anything change in room D while I was away?", "what_changed",
      all=[r"toolbox", r"barrel"], none=[r"red box", r"nothing (has )?changed"]),
    Q("q06", "changes", "What changed in room Z since I left?", "what_changed",
      all=[r"(don't|do not) know a room|no such room|not a room"], none=[r"nothing (has )?changed", r"I haven't been in room Z"]),
    # ---- when / where last seen (4)
    Q("q07", "last_seen", "When did you last see the toolbox, and where was it?", "last_seen",
      all=[r"room D", r"not there|missing|gone"], time={"round": 1, "room": "room D", "tol_s": 240}),
    Q("q08", "last_seen", "When was the red box last seen?", "last_seen",
      all=[r"room C"], none=[r"room A"], time={"round": 2, "room": "room C", "tol_s": 240}),
    Q("q09", "last_seen", "When was the mannequin last spotted?", "last_seen",
      all=[r"room D"], time={"round": 2, "room": "room D", "tol_s": 240}),
    Q("q10", "last_seen", "When did you last see a chair?", "last_seen",
      all=[r"no record|never|haven't seen|have not seen"], none=[r"room [A-D]"]),
    # ---- has it moved (4)
    Q("q11", "history", "Has the red box been moved?", "object_history",
      all=[r"^\W*yes", r"room A", r"room C"]),
    Q("q12", "history", "Did the blue box move at all?", "object_history",
      all=[r"haven't recorded|no record|not moved|hasn't moved|did not move"], none=[r"^\W*yes"]),
    Q("q13", "history", "Has the red barrel shifted?", "object_history",
      all=[r"^\W*yes", r"room D", r"room B"]),
    Q("q14", "history", "Is the toolbox still where it was?", "object_history",
      all=[r"missing|not there|gone"], none=[r"^\W*yes"]),
    # ---- timeline / why (3)
    Q("q15", "timeline", "Give me a recap of what went on while I was out.", "timeline",
      all=[r"entered room"], none=[r"hello"]),
    Q("q16", "timeline", "Show me the event log for room B since I left.", "timeline",
      all=[r"room B"], none=[r"toolbox"]),
    Q("q17", "timeline", "Why did you stop earlier?", "incident_why",
      all=[r"told me to|you said|stop"], none=[r"no record"], stop={"tol_s": 150}),
    # ---- which rooms / when last checked (3)
    Q("q18", "rooms", "Which rooms haven't you looked at for 5 minutes?", "unchecked_rooms", unchecked={"threshold_s": 300}),
    Q("q19", "rooms", "When did you last check room C?", "unchecked_rooms",
      all=[r"room C"], time={"round": 2, "room": "room C", "tol_s": 300}),
    Q("q20", "rooms", "When did you last visit room Z?", "unchecked_rooms",
      all=[r"(don't|do not) know a room|no such room|not a room"], none=[r"no record of being in room Z"]),
]
BY_ID = {q["id"]: q for q in QUESTIONS}


def say_texts() -> list[str]:
    return [s[1] for s in SCENARIO if s[0] in ("say", "stop")] + [q["text"] for q in QUESTIONS]


def _norm(t: str) -> str:
    return re.sub(r"[^a-z0-9 ]", "", t.lower()).strip()


def check_overlap(root: str = ".") -> dict:
    """Questions that also occur (normalised) in an older suite, a test set, SFT data, voice prompts, my_commands or a unit test: must be {}."""
    import glob
    import os
    from . import final_suite_lab4 as base
    mine = {_norm(t): t for t in [q["text"] for q in QUESTIONS]}
    hits: dict[str, list[str]] = {}
    saved = base.say_texts
    base.say_texts = lambda: list(mine.values())                       # reuse the lab4 checker's file scan (testsets, SFT data, voice prompts, my_commands)
    try:
        for k, v in base.check_overlap(root).items():
            hits.setdefault(k, []).extend(v)
    finally:
        base.say_texts = saved
    from . import final_suite, final_suite_lab4_v2, final_suite_lab4_v3
    for mod in (base, final_suite_lab4_v2, final_suite_lab4_v3):
        for t in [s[1] for m in mod.SUITE for s in m["steps"] if s[0] in ("say", "stop")]:
            if _norm(t) in mine:
                hits.setdefault(mine[_norm(t)], []).append(mod.__name__.rsplit(".", 1)[-1])
    for f in glob.glob(os.path.join(root, "brain/tests/*.py")) + glob.glob(os.path.join(root, "web/tests/*.py")):
        if f.endswith("test_incident_suite.py"):
            continue
        txt = _norm(open(f).read())
        for n, t in mine.items():
            if n in txt:
                hits.setdefault(t, []).append(os.path.basename(f))
    return hits
