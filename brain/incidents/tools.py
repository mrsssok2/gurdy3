"""The incident tools. They are NOT in `brain.tools.TOOLS`: that dict is the planner prompt (the fine-tuned model was trained on exactly those 24
tools; adding to it would change the byte-identical system prompt). The agent routes incident questions deterministically, so the planner never
needs to see them. `ToolLayer.call` validates against these specs and dispatches to `backend.incident_query(name, args)`."""
from __future__ import annotations

from brain.tools.specs import OBJECT_REF, ToolSpec, _S, _obj

_SINCE = {"type": "string", "description": "today | session | away | all | clock:HH:MM | ago:<seconds>"}
INCIDENT_SPECS = [
    ToolSpec("what_changed", "What changed (objects moved, missing, new, no-go zones) in a room and / or since a time.", "query",
             _obj({"room": _S, "since": _SINCE})),
    ToolSpec("last_seen", "When and where an object was last seen.", "query", _obj({"object": OBJECT_REF}, ["object"])),
    ToolSpec("object_history", "Everything remembered about one object: sightings, moves, disappearances.", "query",
             _obj({"object": OBJECT_REF, "since": _SINCE}, ["object"])),
    ToolSpec("timeline", "Latest events, optionally for one room, one object and since a time.", "query",
             _obj({"since": _SINCE, "room": _S, "object": OBJECT_REF, "limit": {"type": "integer", "minimum": 1, "maximum": 50}})),
    ToolSpec("incident_why", "Why did the robot stop / fail / alert, from the recorded cause chain.", "query",
             _obj({"about": {"type": "string", "enum": ["stop", "failure", "alert", "last_event"]}, "at": {"type": "string"},
                   "event_id": {"type": "integer", "minimum": 1}}, ["about"])),
    ToolSpec("unchecked_rooms", "Which rooms the robot has not been in for a while (or when it last was in one).", "query",
             _obj({"older_than_s": {"type": "number", "minimum": 1}, "room": _S})),
]
INCIDENT_TOOLS = {s.name: s for s in INCIDENT_SPECS}
