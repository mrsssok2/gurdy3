"""SECOND held-out incident evaluation (Phase 6b), same scenario, world and scorer as `incident_suite` (incident_v1) but NEW questions. COMMITTED BEFORE IT WAS RUN.

  own       20 questions with new phrasings, written after the Phase 6b router work and NOT run against the router before the evaluation. Still written by the
            assistant that wrote the router (not independent); unusual wordings the router does not know are findings, not scorer errors.
  external  10 questions written by the USER (given in the Phase 6b task, `y01..y10`). The router was not run on them and is not tuned on them. NOTE: the
            assistant read the list before finishing the router's cue-word vocabulary (generic words only: change / different / moved / last seen / summary /
            unchecked / `is there a room X`), so even this set is not blind with respect to topics.
Expected answers are fixed here from the scenario ground truth (red box A -> C, red barrel D -> B, toolbox removed, everything else untouched), not from the
agent. The scenario (look around, "go to room B" / stop / hello, world changes, second look around, 300 s wait) is the one of incident_v1.
Scorer changes for FUTURE runs only (`incident_score`): objects are matched by class with the colour optional (the detector calls the toolbox `red toolbox`)."""
from __future__ import annotations

import glob
import os

from . import incident_suite as V1
from .incident_suite import (CHANGED, FINAL_WAIT_S, MAP_ID, MAP_NAME, OPS, PLACES_FILE, ROUND1, ROUND2, SCENARIO, SWEEP_LIMIT_S, UNCHANGED_WORDS, WORLD,  # noqa: F401
                             Q, X, _norm)

NAME = "incident_v2"
ASSUMED = r"(don't|do not) know a room|no such room|not a room|there is no room"

QUESTIONS = [
    # ---- what changed (6)
    Q("n01", "changes", "Has anything been shuffled around since I stepped out?", "what_changed",
      all=[r"red box", r"barrel", r"toolbox"], none=[UNCHANGED_WORDS]),
    Q("n02", "changes", "Is room A the way I left it?", "what_changed",
      all=[r"red box"], none=[r"toolbox", r"barrel", r"^\W*yes"]),
    Q("n03", "changes", "Any surprises in room B since I went out?", "what_changed",
      all=[r"barrel"], none=[r"red box", r"toolbox", r"nothing (has )?changed"]),
    Q("n04", "changes", "What's different in room C compared to before I left?", "what_changed",
      all=[r"red box"], none=[r"toolbox", r"barrel", r"nothing (has )?changed"]),
    Q("n05", "changes", "Did anything disappear?", "what_changed",
      all=[r"toolbox"], none=[UNCHANGED_WORDS]),
    Q("n06", "changes", "Tell me what is new in room D.", "what_changed",
      all=[r"toolbox", r"barrel"], none=[r"red box", r"nothing (has )?changed"]),
    # ---- when / where last seen (4)
    Q("n07", "last_seen", "Where did you see the red barrel most recently, and when?", "last_seen",
      all=[r"room B"], none=[r"room D"], time={"round": 2, "room": "room B", "tol_s": 240}),
    Q("n08", "last_seen", "How long ago was the mannequin spotted?", "last_seen",
      all=[r"room D"], time={"round": 2, "room": "room D", "tol_s": 240}),
    Q("n09", "last_seen", "Have you seen a toolbox lately?", "last_seen",
      all=[r"toolbox", r"not there|missing|gone|not since|no longer|haven't|have not"], time={"round": 1, "room": "room D", "tol_s": 240}),
    Q("n10", "last_seen", "When did you last spot a bicycle?", "last_seen",
      all=[r"no record|never|haven't seen|have not seen|don't know what"], none=[r"room [A-D]"]),
    # ---- has it moved (4)
    Q("n11", "history", "Did the red box end up somewhere new?", "object_history",
      all=[r"^\W*yes", r"room A", r"room C"]),
    Q("n12", "history", "Has the yellow box been touched?", "object_history",
      all=[r"haven't recorded|no record|not moved|hasn't moved|did not move"], none=[r"^\W*yes"]),
    Q("n13", "history", "Was the red barrel relocated?", "object_history",
      all=[r"^\W*yes", r"room D", r"room B"]),
    Q("n14", "history", "Is the red toolbox where I left it?", "object_history",
      all=[r"missing|not there|gone"], none=[r"^\W*yes"]),
    # ---- timeline / why (3)
    Q("n15", "timeline", "What went on while I was out of the room?", "timeline",
      all=[r"entered room"], none=[r"hello"]),
    Q("n16", "timeline", "Can you describe the latest events in room C?", "timeline",
      all=[r"room C"], none=[r"toolbox"]),
    Q("n17", "timeline", "What made you stop earlier on?", "incident_why",
      all=[r"told me to|you said|stop"], none=[r"no record"], stop={"tol_s": 150}),
    # ---- which rooms / when last checked (3)
    Q("n18", "rooms", "Which rooms haven't been visited in the past 4 minutes?", "unchecked_rooms", unchecked={"threshold_s": 240}),
    Q("n19", "rooms", "At what time were you last in room B?", "unchecked_rooms",
      all=[r"room B"], time={"round": 2, "room": "room B", "tol_s": 300}),
    Q("n20", "rooms", "Is there anything in room Q?", "unchecked_rooms",
      all=[ASSUMED], none=[r"nothing", r"I haven't been in room Q"]),
]

EXTERNAL = [
    X("y01", "has anything been taken away since you started?", ("what_changed", "timeline"),
      all=[r"toolbox"], none=[UNCHANGED_WORDS]),
    X("y02", "the barrel, where did it end up?", ("object_history", "last_seen", "where_is"),
      all=[r"room B"], none=[r"nothing"]),
    X("y03", "compare room A now with how it looked at the start", ("what_changed",),
      all=[r"red box"], none=[r"toolbox", r"barrel", r"nothing (has )?changed"]),
    X("y04", "list what moved and where to", ("what_changed",),
      all=[r"red box", r"room C", r"barrel", r"room B"], none=[UNCHANGED_WORDS, r"nothing (has )?changed"]),
    X("y05", "when was the last time you laid eyes on the toolbox?", ("last_seen", "object_history"),
      all=[r"toolbox", r"not there|missing|gone|not since|no longer|haven't|have not"], time={"round": 1, "room": "room D", "tol_s": 240}),
    X("y06", "anything different in room D?", ("what_changed",),
      all=[r"toolbox", r"barrel"], none=[r"red box", r"nothing (has )?changed"]),
    X("y07", "give me a quick summary of today's events", ("timeline",),
      all=[r"room"], none=[r"no record|haven't recorded anything"]),
    X("y08", "did the red box stay put?", ("object_history", "last_seen"),
      all=[r"room A", r"room C"], none=[r"^\W*yes"]),
    X("y09", "which room have you not checked in a while?", ("unchecked_rooms",), unchecked={"threshold_s": 600}),
    X("y10", "is there a room E?", ("unchecked_rooms", "where_am_i"),
      all=[r"no\b.*room E|there is no room E"], none=[r"^\W*yes"]),
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
    for q in V1.ALL_QUESTIONS:
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
        if os.path.basename(f) in ("test_incident_suite.py", "test_incident_suite_v2.py"):
            continue
        txt = _norm(open(f).read())
        for n, t in mine.items():
            if n in txt:
                hits.setdefault(t, []).append(os.path.basename(f))
    return hits
