"""BLIND incident suite v1 (Phase 6e). The 20 questions were written by the user (not by the author of the router / picker) and were not seen before this file was
committed. Wording is verbatim; only {room} / {object} were filled from the ground truth of the 6c run (scenario of v1: red box A -> C, red barrel D -> B, toolbox
removed from D, everything else untouched). Expected answers are fixed here from that ground truth. COMMITTED BEFORE ANY RUN. NEVER tune the router, picker or prompt on it.
Q16-Q20 are not incident questions: `expect_tool` "none" = no incident tool may be called, "any" = routing is not scored (content only).
Scorer: incident_score with `use_suite("incident_suite_blind_v1")`; `check["calls"]` = tools that must have been called."""
from __future__ import annotations

from . import incident_suite_v4 as V4
from .incident_suite import (CHANGED, FINAL_WAIT_S, MAP_ID, MAP_NAME, OPS, PLACES_FILE, ROUND1, ROUND2, SCENARIO, SWEEP_LIMIT_S, UNCHANGED_WORDS, WORLD,  # noqa: F401
                             Q)

NAME = "incident_blind_v1"
UNCH = r"blue box|yellow box|mannequin|extinguisher|blue barrel"
OBJ_TOOLS = ("last_seen", "object_history")

QUESTIONS = [
    Q("b01", "changes", "anything look different in room D since you last went in?", "what_changed",
      all=[r"toolbox|barrel"], none=[r"red box", UNCH, r"nothing (has )?changed"]),
    Q("b02", "last_seen", "when did you last spot the toolbox?", "last_seen",
      all=[r"toolbox", r"not there|missing|gone"], time={"round": 1, "room": "room D", "tol_s": 300}),
    Q("b03", "history", "has the red box moved around at all today?", "object_history",
      all=[r"room A", r"room C"], none=[r"^\W*no\b", r"haven't recorded the red box moving"]),
    Q("b04", "timeline", "walk me through what happened over the last ten minutes", "timeline",
      none=[r"toolbox", r"barrel", r"red box", UNCH]),
    Q("b05", "why", "why did you stop earlier?", "incident_why", all=[r"told me to|you said|stop"]),
    Q("b06", "rooms", "which rooms haven't you looked at yet?", "unchecked_rooms",
      all=[r"every room|all rooms|all four|no room"], none=[r"not checked for"]),
    Q("b07", "history", "is the red barrel still where it was?", "object_history", all=[r"^\W*no", r"room B"], none=[r"^\W*yes"]),
    Q("b08", "changes", "did someone mess with room A?", "what_changed", all=[r"red box"], none=[r"toolbox", r"barrel", r"nothing (has )?changed"]),
    Q("b09", "last_seen", "where'd the red box go?", OBJ_TOOLS, all=[r"room C"], none=[r"room A"]),
    Q("b10", "timeline", "give me a recap of your patrol", "timeline", all=[r"entered room"]),
    Q("b11", "rooms", "any rooms you skipped?", "unchecked_rooms", all=[r"every room|all rooms|all four|no room"], none=[r"not checked for"]),
    Q("b12", "why", "how come you paused after I told you to stop?", "incident_why", all=[r"told me to|you said|stop"]),
    Q("b13", "changes", "what's new around here in the last 10 minutes?", "what_changed",
      all=[r"nothing|no change|haven't recorded"], none=[r"red box", r"toolbox", r"barrel", UNCH]),
    Q("b14", "changes", "has anything been missing for more than five minutes?", ("what_changed", "last_seen", "object_history"),
      all=[r"toolbox"], none=[UNCH, r"red box|red barrel"]),
    Q("b15", "history", "tell me the story of the red box", "object_history", all=[r"room A", r"room C"]),
    # ---- not incident questions / honesty
    Q("b16", "honesty", "is room Q real?", "any", all=[r"there is no room Q|no such room|don't know a room"]),
    Q("b17", "negative", "go to room C", "none", all=[r"."], calls=["go_to"]),
    Q("b18", "negative", "what do you see right now?", "none", all=[r"."], calls=["describe_view"]),
    Q("b19", "negative", "how many legs do you have?", "none", all=[r"."], none=[r"toolbox|barrel|red box"]),
    Q("b20", "negative", "stop", "none", all=[r"."]),
]

EXTERNAL: list = []
ALL_QUESTIONS = QUESTIONS + EXTERNAL
BY_ID = {q["id"]: q for q in ALL_QUESTIONS}
