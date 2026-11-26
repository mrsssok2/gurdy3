"""Sample incident history for the PUBLIC demo (in-browser build and the server's demo mode), written for the lab4 world (rooms A-D, ring of doorways).

A fresh demo world has no past, so the Timeline tab would start empty. This seeds a short, invented earlier patrol into the visitor's own in-memory incident
graph (written BEFORE the recorder starts, so it loads it like any history). Every event carries data["sample"] = True and the panel tags it "sample":
it is never presented as something this robot did. No real robot, no network, nothing leaves the page. Objects and rooms are the lab4 ones
(docs/results/data/lab4/lab4_gt.json); the toolbox "moved" 1.3 m inside room D, the only changed object."""
from __future__ import annotations

from datetime import datetime

# (minutes ago, type, text, severity, room, cls, color, extra data, x, y, x2, y2, caused_by_type)
SAMPLE = [
    (62, "room_visited", "Entered room A.", "info", "room A", None, None, {}, 0.0, 0.0, None, None, None),
    (58, "room_visited", "Entered room B.", "info", "room B", None, None, {}, 3.2, -0.7, None, None, None),
    (53, "room_visited", "Entered room D.", "info", "room D", None, None, {}, 4.0, 2.6, None, None, None),
    (51, "object_moved", "The toolbox moved 1.3 m from where it was.", "warning", "room D", "toolbox", None,
     {"from": {"x": 4.4, "y": 4.4, "room": "room D"}, "to": {"x": 3.2, "y": 5.3, "room": "room D"}, "distance_m": 1.3}, 4.4, 4.4, 3.2, 5.3, None),
    (47, "alert", "Battery low (22 %).", "warning", None, None, None, {"text": "Battery low (22 %)."}, None, None, None, None, None),
    (43, "room_visited", "Entered room C.", "info", "room C", None, None, {}, 1.2, 3.8, None, None, None),
    (40, "no_go_set", "No-go zone set around the fire extinguisher.", "info", "room C", None, None, {"target": "the fire extinguisher"}, None, None, None, None, None),
    (36, "path_blocked", "The way from room C to room A was blocked.", "warning", "room C", None, None, {"reason": "blocked"}, -0.7, 2.0, None, None, None),
    (35, "task_failed", "I could not get back to room A: the path was blocked.", "error", "room C", None, None, {"reason": "blocked", "tool": "go_to"}, -0.7, 2.0, None, None, "path_blocked"),
    (30, "no_go_cleared", "No-go zone around the fire extinguisher removed.", "info", "room C", None, None, {"target": "the fire extinguisher"}, None, None, None, None, None),
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
