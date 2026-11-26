"""Strict specs of the fetch tools (validated with brain.tools.schema.validate, like every other tool)."""
from __future__ import annotations

from ..tools.schema import validate
from .catalog import COLOURS, PICKABLE, TOO_BIG

KNOWN_CLASSES = sorted(set(PICKABLE) | set(TOO_BIG) | {"mannequin"})   # safety (not the schema) refuses the wrong ones
_OBJ = {"type": "object", "properties": {"class": {"type": "string", "enum": KNOWN_CLASSES},
                                         "color": {"type": "string", "enum": list(COLOURS)},
                                         "room": {"type": "string", "minLength": 1}}, "required": ["class"]}
FETCH_TOOLS = {
    "pick_up": {"type": "object", "properties": {"object": _OBJ, "room": {"type": "string", "minLength": 1}},
                "required": ["object"]},
    "bring_to": {"type": "object", "properties": {"target": {"type": "string", "enum": ["user", "home", "room"]},
                                                  "room": {"type": "string", "minLength": 1}}, "required": ["target"]},
    "place": {"type": "object", "properties": {"room": {"type": "string", "minLength": 1}}, "required": ["room"]},
    "hand_over": {"type": "object", "properties": {}, "required": []},
}


def check(tool: str, args: dict) -> list[str]:
    """Problems (empty = valid). bring_to(target=room) also needs `room`."""
    if tool not in FETCH_TOOLS:
        return [f"unknown fetch tool {tool!r}"]
    probs = validate(args, FETCH_TOOLS[tool])
    if not probs and tool == "bring_to" and args["target"] == "room" and not args.get("room"):
        probs.append("args.room: required when target is 'room'")
    if not probs and tool == "bring_to" and args["target"] != "room" and "room" in args:
        probs.append("args.room: only allowed when target is 'room'")
    return probs
