"""Sample incident history for the PUBLIC demo (in-browser build and the server's demo mode).

A fresh demo world has no past, so the Timeline tab would start empty. This seeds a short, invented earlier patrol into the visitor's own in-memory incident
graph (written BEFORE the recorder starts, so it loads it like any history). Every event carries data["sample"] = True and the panel tags it "sample":
it is never presented as something this robot did. No real robot, no network, nothing leaves the page."""
from __future__ import annotations

from datetime import datetime

# (minutes ago, type, text, severity, room, cls, color, extra data, x, y, x2, y2, caused_by_type)
SAMPLE = [
    (58, "room_visited", "Entered room A.", "info", "room A", None, None, {}, 0.0, 0.0, None, None, None),
    (54, "object_moved", "The yellow box moved 1.5 m from where it was.", "warning", "room B", "box", "yellow",
     {"from": {"x": 3.2, "y": -2.4, "room": "room B"}, "to": {"x": 4.6, "y": -1.5, "room": "room B"}, "distance_m": 1.5}, 3.2, -2.4, 4.6, -1.5, None),
    (50, "room_visited", "Entered room B.", "info", "room B", None, None, {}, 3.4, 0.2, None, None, None),
    (46, "alert", "Battery low (22 %).", "warning", None, None, None, {"text": "Battery low (22 %)."}, None, None, None, None, None),
    (41, "no_go_set", "No-go zone set around the table.", "info", "room A", None, None, {"target": "the table"}, None, None, None, None, None),
    (36, "path_blocked", "The way to room B was blocked.", "warning", "room A", None, None, {"reason": "blocked"}, 1.9, 0.4, None, None, None),
    (35, "task_failed", "I could not get to room B: the path was blocked.", "error", "room A", None, None, {"reason": "blocked", "tool": "go_to"}, 1.9, 0.4, None, None, "path_blocked"),
    (30, "no_go_cleared", "No-go zone around the table removed.", "info", "room A", None, None, {"target": "the table"}, None, None, None, None, None),
    (24, "task_finished", "Back in room A.", "info", "room A", None, None, {"tool": "go_to"}, 0.0, 0.0, None, None, None),
]


def seed(store, now: float) -> int:
    """Write SAMPLE into `store` as events `now - minutes` ago (kept inside today and in order). Returns the number of events."""
    midnight = datetime.fromtimestamp(now).replace(hour=0, minute=0, second=0, microsecond=0).timestamp()
    ids: dict[str, int] = {}
    for k, (mins, etype, text, sev, room, cls, color, data, x, y, x2, y2, cause) in enumerate(SAMPLE):
        wall = min(max(now - mins * 60, midnight + 60 * (k + 1)), now - 1)
        eid = store.add_event(etype, text, 0.0, wall, sev, room=room, x=x, y=y, x2=x2, y2=y2, cls=cls, color=color, data={**data, "sample": True})
        if cause and cause in ids:
            store.add_edge(eid, ids[cause], "caused_by", 0.0, wall)
        ids[etype] = eid
    return len(SAMPLE)
