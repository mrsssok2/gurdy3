"""THIRD held-out incident evaluation (Phase 6c), same scenario, world and scorer as `incident_suite` / `incident_suite_v2` but NEW questions. COMMITTED BEFORE IT WAS RUN.

  own       20 questions with new phrasings, written after the Phase 6c router / semantics fixes and not run against the router before the evaluation. Still
            written by the assistant that wrote the router (not independent).
  external  10 questions written by the USER, added in a single final commit right before the run, after all code changes were committed (see EXTERNAL below).
            Honest note: the question list was part of the task message, so the assistant had seen it; it was not used for cue words or tests and the router
            was not run on it.
Expected answers are fixed here from the scenario ground truth (red box A -> C, red barrel D -> B, toolbox removed, everything else untouched), not from the agent.
Scorer: class with optional colour for the graph (`incident_score.use_suite("incident_suite_v3")`); v1 scoring stays exact."""
from __future__ import annotations

import glob
import os

from . import incident_suite as V1
from . import incident_suite_v2 as V2
from .incident_suite import (CHANGED, FINAL_WAIT_S, MAP_ID, MAP_NAME, OPS, PLACES_FILE, ROUND1, ROUND2, SCENARIO, SWEEP_LIMIT_S, UNCHANGED_WORDS, WORLD,  # noqa: F401
                             Q, X, _norm)

NAME = "incident_v3"
ASSUMED = r"(don't|do not) know a room|no such room|not a room|there is no room"

QUESTIONS = [
    # ---- what changed (6)
    Q("m01", "changes", "Did I miss anything while I was out?", "what_changed",
      all=[r"red box", r"barrel", r"toolbox"], none=[UNCHANGED_WORDS]),
    Q("m02", "changes", "Has room B been rearranged?", "what_changed",
      all=[r"barrel"], none=[r"red box", r"toolbox", r"nothing (has )?changed"]),
    Q("m03", "changes", "Is anything gone from room D?", "what_changed",
      all=[r"toolbox"], none=[r"red box", r"nothing (has )?changed"]),
    Q("m04", "changes", "Is everything in room A where it should be?", "what_changed",
      all=[r"red box"], none=[r"toolbox", r"barrel", r"^\W*yes"]),
    Q("m05", "changes", "What differs in room C compared with before?", "what_changed",
      all=[r"red box"], none=[r"toolbox", r"barrel", r"nothing (has )?changed"]),
    Q("m06", "changes", "Any objects that have vanished altogether?", "what_changed",
      all=[r"toolbox"], none=[UNCHANGED_WORDS]),
    # ---- when / where last seen (4)
    Q("m07", "last_seen", "When did the toolbox last show up on camera?", "last_seen",
      all=[r"toolbox", r"not there|missing|gone"], time={"round": 1, "room": "room D", "tol_s": 240}),
    Q("m08", "last_seen", "What's the most recent place you saw the red box?", "last_seen",
      all=[r"room C"], none=[r"room A"], time={"round": 2, "room": "room C", "tol_s": 240}),
    Q("m09", "last_seen", "Do you remember when you spotted the fire extinguisher?", "last_seen",
      all=[r"room C"], time={"round": 2, "room": "room C", "tol_s": 300}),
    Q("m10", "last_seen", "When did you last notice a wheelbarrow?", "last_seen",
      all=[r"no record|never|haven't seen|have not seen|don't know what"], none=[r"room [A-D]"]),
    # ---- has it moved (4)
    Q("m11", "history", "Is the red barrel in the same spot as at the start?", "object_history",
      all=[r"^\W*no", r"room D", r"room B"]),
    Q("m12", "history", "Has the blue barrel been shifted at all?", "object_history",
      all=[r"haven't recorded|no record|not moved|hasn't moved|did not move"], none=[r"^\W*yes"]),
    Q("m13", "history", "Did the toolbox get taken?", "object_history",
      all=[r"missing|not there|gone"], none=[r"^\W*yes"]),
    Q("m14", "history", "Is the red box staying in room A?", "object_history",
      all=[r"^\W*no", r"room C"]),
    # ---- timeline / why (3)
    Q("m15", "timeline", "Fill me in on the last stretch while I was gone.", "timeline",
      all=[r"entered room"], none=[r"hello"]),
    Q("m16", "timeline", "What went on in room D today?", "timeline",
      all=[r"room D"], none=[r"red box"]),
    Q("m17", "timeline", "What was the reason for you halting after I gave that order?", "incident_why",
      all=[r"told me to|you said|stop"], none=[r"no record"], stop={"tol_s": 150}),
    # ---- which rooms / when last checked (3)
    Q("m18", "rooms", "Which rooms have gone unvisited for more than five minutes?", "unchecked_rooms", unchecked={"threshold_s": 300}),
    Q("m19", "rooms", "How recently were you in room C?", "unchecked_rooms",
      all=[r"room C"], time={"round": 2, "room": "room C", "tol_s": 300}),
    Q("m20", "rooms", "Does room G exist?", "unchecked_rooms",
      all=[ASSUMED], none=[r"nothing", r"I haven't been in room G"]),
]

# EXTERNAL: 10 questions written by the user, added in the final commit before the run. Expectations fixed here from the scenario ground truth.
# z07 has no time word ("overdue"): the documented default applies (1 hour), the robot visited every room within the last hour, so no room should be listed.
EXTERNAL = [
    X("z01", "what's been shuffled around since this morning?", ("what_changed", "timeline"),
      all=[r"red box", r"barrel"], none=[UNCHANGED_WORDS, r"nothing (has )?changed"]),
    X("z02", "tell me only the things that actually changed", ("what_changed",),
      all=[r"red box", r"barrel", r"toolbox"], none=[UNCHANGED_WORDS, r"nothing (has )?changed"]),
    X("z03", "is the barrel still in room D?", ("object_history", "last_seen", "where_is"),
      all=[r"^\W*no", r"room B"], none=[r"^\W*yes"]),
    X("z04", "I think someone took something, did they?", ("what_changed", "timeline", "object_history"),
      all=[r"toolbox"], none=[UNCHANGED_WORDS]),
    X("z05", "what did room C look like before versus now?", ("what_changed",),
      all=[r"red box"], none=[r"toolbox", r"barrel", r"nothing (has )?changed"]),
    X("z06", "when did you last pass through room A?", ("unchecked_rooms", "timeline"),
      all=[r"room A"], time={"round": 2, "room": "room A", "tol_s": 900}),
    X("z07", "which rooms are overdue for a check?", ("unchecked_rooms",), unchecked={"threshold_s": 3600}),
    X("z08", "has the red box gone anywhere?", ("object_history", "last_seen"),
      all=[r"^\W*yes", r"room A", r"room C"]),
    X("z09", "is there anything in a room F?", ("unchecked_rooms", "where_am_i"),
      all=[ASSUMED], none=[r"nothing", r"^\W*yes"]),
    X("z10", "quick recap, nothing extra", ("timeline",),
      all=[r"room"], none=[r"no record|haven't recorded anything"]),
]
ALL_QUESTIONS = QUESTIONS + EXTERNAL
BY_ID = {q["id"]: q for q in ALL_QUESTIONS}


def say_texts() -> list[str]:
    return [s[1] for s in SCENARIO if s[0] in ("say", "stop")] + [q["text"] for q in ALL_QUESTIONS]


def check_overlap(root: str = ".") -> dict:
    """Questions that also occur (normalised) in the v1 incident questions, any older suite, test set, SFT data, voice prompts, my_commands or a unit test: must be {}."""
    from . import final_suite_lab4 as base, final_suite_lab4_v2, final_suite_lab4_v3
    mine = {_norm(q["text"]): q["text"] for q in ALL_QUESTIONS}
    hits: dict[str, list[str]] = {}
    for q in V1.ALL_QUESTIONS + V2.ALL_QUESTIONS:
        if _norm(q["text"]) in mine:
            hits.setdefault(mine[_norm(q["text"])], []).append("incident_suite")
    saved = base.say_texts
    base.say_texts = lambda: list(mine.values())
    try:
        for k, v in base.check_overlap(root).items():
            hits.setdefault(k, []).extend(v)
    finally:
        base.say_texts = saved
    for mod in (base, final_suite_lab4_v2, final_suite_lab4_v3):
        for t in [s[1] for m in mod.SUITE for s in m["steps"] if s[0] in ("say", "stop")]:
            if _norm(t) in mine:
                hits.setdefault(mine[_norm(t)], []).append(mod.__name__.rsplit(".", 1)[-1])
    for f in glob.glob(os.path.join(root, "brain/tests/*.py")) + glob.glob(os.path.join(root, "web/tests/*.py")):
        if os.path.basename(f) in ("test_incident_suite.py", "test_incident_suite_v2.py", "test_incident_suite_v3.py"):
            continue
        txt = _norm(open(f).read())
        for n, t in mine.items():
            if n in txt:
                hits.setdefault(t, []).append(os.path.basename(f))
    return hits
