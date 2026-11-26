"""Doorway suite v1 for the Gurdy 3.0 `lab4` world (Phase 8a). COMMITTED BEFORE ANY NAV2 PARAMETER CHANGE; never tune on it afterwards.

Purpose: does Nav2 + the walking policy pass every lab4 doorway in both directions without a "path blocked" / "blocked or unknown space"
failure? 12 missions cover the 4 doorways x 2 directions (single hops, back-and-forth, the full ring both ways, two-hop goals) and 2 are
tight-turn cases (a door-to-door corner around the mannequin in room D, and a reversal ordered while the robot is in / next to a doorway).
Wording is plain and general (it is a navigation test, not a language test); the agent model is the normal one (gurdy-sft-400).

lab4 (scripts/gen_world_lab4.py): room A [-2,-2,2,2]  B [2,-2,6,2]  C [-2,2,2,6]  D [2,2,6,6]; RING A-B-D-C-A (no A-D / B-C doorway);
doorways 1.0 m wide, gap centres: A-B (2,-0.7)  B-D (4,2)  D-C (2,4.2)  C-A (-0.7,2). Robot starts at home (0,0) in room A, facing +x.
A hop between adjacent rooms has one shortest route (through their doorway). B<->C and A<->D are two hops; B<->C is shorter through D
(about 7.0 m vs 8.1 m through A), A<->D is about equal both ways, so for those only the end room is checked.

Checks: `visited` a doorway region (within 0.45 m of its gap centre: pose samples are taken while walking, the robot passes through the
gap), `end_in` the target room, and `no_reply` for the two blocked-path replies (the failure this suite is about).
"""
from __future__ import annotations

from .final_suite import mission, region_contains, arrival_claim, percentages  # noqa: F401
from .final_suite_lab4 import WORLD, MAP_ID, MAP_NAME, PLACES_FILE, PLACES as _LAB4_PLACES  # noqa: F401

SCORER = 2
DOOR_R = 0.45
DOORS = {"A-B": (2.0, -0.7), "B-D": (4.0, 2.0), "D-C": (2.0, 4.2), "C-A": (-0.7, 2.0)}
PLACES = dict(_LAB4_PLACES)
PLACES.update({f"door {k}": ("near", x, y, DOOR_R) for k, (x, y) in DOORS.items()})
BLOCKED = r"blocked|unknown space|gave up"


def resolve_region(spec):
    return PLACES[spec] if isinstance(spec, str) else tuple(spec)


def _hop(room_from_say: list, checks: list, mid: str, name: str, cat: str, limit: int, goal):
    return mission(mid, name, cat, limit, room_from_say, checks + [("no_reply", BLOCKED)], goal=goal)


SUITE = [
    # ---- one doorway, one direction
    _hop([("say", "walk to room B"), ("idle", 200)],
         [("visited", "door A-B"), ("end_in", "room B")], "dw_a_to_b", "A -> B (door A-B, forward)", "single", 240, "room B"),
    _hop([("say", "walk to room C"), ("idle", 200)],
         [("visited", "door C-A"), ("end_in", "room C")], "dw_a_to_c", "A -> C (door C-A, from A)", "single", 240, "room C"),
    # ---- one doorway, both directions
    _hop([("say", "walk to room B"), ("idle", 200), ("say", "walk back to room A"), ("idle", 200)],
         [("visited", "door A-B"), ("visited", "room B"), ("end_in", "room A")], "dw_b_to_a", "A -> B -> A (door A-B both ways)", "both",
         460, ["room B", "room A"]),
    _hop([("say", "walk to room C"), ("idle", 200), ("say", "walk back to room A"), ("idle", 200)],
         [("visited", "door C-A"), ("visited", "room C"), ("end_in", "room A")], "dw_c_to_a", "A -> C -> A (door C-A both ways)", "both",
         460, ["room C", "room A"]),
    _hop([("say", "walk to room B"), ("idle", 200), ("say", "now walk to room D"), ("idle", 200)],
         [("visited", "door B-D"), ("visited", "room B"), ("end_in", "room D")], "dw_b_to_d", "A -> B -> D (door B-D, from B)", "single",
         460, ["room B", "room D"]),
    _hop([("say", "walk to room C"), ("idle", 200), ("say", "now walk to room D"), ("idle", 200)],
         [("visited", "door D-C"), ("visited", "room C"), ("end_in", "room D")], "dw_c_to_d", "A -> C -> D (door D-C, from C)", "single",
         460, ["room C", "room D"]),
    _hop([("say", "walk to room B"), ("idle", 200), ("say", "now walk to room D"), ("idle", 200), ("say", "walk back to room B"), ("idle", 200)],
         [("visited", "room D"), ("visited", "door B-D"), ("end_in", "room B")], "dw_d_to_b", "A -> B -> D -> B (door B-D both ways)", "both",
         680, ["room B", "room D", "room B"]),
    _hop([("say", "walk to room C"), ("idle", 200), ("say", "now walk to room D"), ("idle", 200), ("say", "walk back to room C"), ("idle", 200)],
         [("visited", "room D"), ("visited", "door D-C"), ("end_in", "room C")], "dw_d_to_c", "A -> C -> D -> C (door D-C both ways)", "both",
         680, ["room C", "room D", "room C"]),
    # ---- the full ring, both directions (every doorway once)
    _hop([("say", "walk to room B"), ("idle", 200), ("say", "now walk to room D"), ("idle", 200), ("say", "now walk to room C"), ("idle", 200),
          ("say", "now walk to room A"), ("idle", 200)],
         [("visited", "door A-B"), ("visited", "door B-D"), ("visited", "door D-C"), ("visited", "door C-A"), ("end_in", "room A")],
         "dw_ring_fwd", "ring A-B-D-C-A", "ring", 900, ["room B", "room D", "room C", "room A"]),
    _hop([("say", "walk to room C"), ("idle", 200), ("say", "now walk to room D"), ("idle", 200), ("say", "now walk to room B"), ("idle", 200),
          ("say", "now walk to room A"), ("idle", 200)],
         [("visited", "door C-A"), ("visited", "door D-C"), ("visited", "door B-D"), ("visited", "door A-B"), ("end_in", "room A")],
         "dw_ring_rev", "ring A-C-D-B-A", "ring", 900, ["room C", "room D", "room B", "room A"]),
    # ---- two hops in one goal (route chosen by the planner)
    _hop([("say", "walk to room D"), ("idle", 340)],
         [("end_in", "room D")], "dw_a_to_d", "A -> D (two doorways, either side)", "two-hop", 380, "room D"),
    _hop([("say", "walk to room D"), ("idle", 340), ("say", "walk back to room A"), ("idle", 340)],
         [("visited", "room D"), ("end_in", "room A")], "dw_d_to_a", "A -> D -> A (two doorways each way)", "two-hop", 760, ["room D", "room A"]),
    # ---- tight turns
    _hop([("say", "walk to room B"), ("idle", 200), ("say", "now walk to room C"), ("idle", 340)],
         [("visited", "room B"), ("visited", "door B-D"), ("visited", "door D-C"), ("end_in", "room C")], "dw_tight_corner",
         "B -> C through D: door-to-door corner around the mannequin", "tight", 600, ["room B", "room C"]),
    _hop([("say", "walk to room C"), ("wait", 28), ("say", "change of plan, walk back to room A"), ("idle", 260)],
         [("visited", "door C-A"), ("end_in", "room A")], "dw_tight_reverse", "reversal ordered at / near door C-A", "tight", 340, "room A"),
]

BY_ID = {m["id"]: m for m in SUITE}


def check_ids() -> list[str]:
    return [m["id"] for m in SUITE]


def say_texts() -> list[str]:
    return [s[1] for m in SUITE for s in m["steps"] if s[0] in ("say", "stop")]


def door_lane_widths(door_w: float = 1.0, inscribed: float = 0.273, circumscribed: float = 0.340) -> dict:
    """Free lateral room for the footprint centre in a doorway (metres either side of the gap centre)."""
    return {"inscribed_lane": door_w / 2 - inscribed, "circumscribed_lane": door_w / 2 - circumscribed}
