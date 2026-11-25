"""The 24 tool definitions (docs/phase5_design.md section 2): name, description, JSON schema, kind.

kind: "motion" (returns {task_id, state: started, estimate}; final result arrives as a task_finished event),
      "query"  (read-only, answers inline), "config" (changes stored configuration), "dialogue".
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ToolSpec:
    name: str
    description: str
    kind: str
    parameters: dict[str, Any]

    def as_llm_tool(self) -> dict[str, Any]:
        """OpenAI / Ollama style function definition."""
        return {"type": "function",
                "function": {"name": self.name, "description": self.description, "parameters": self.parameters}}


def _obj(props: dict | None = None, required: list | None = None, **extra: Any) -> dict:
    return {"type": "object", "properties": props or {}, "required": required or [], **extra}


_S = {"type": "string", "minLength": 1}
_CLASSES = ["box", "barrel", "fire_extinguisher", "traffic_cone", "toolbox", "mannequin", "chair"]
OBJECT_REF = _obj({"class": {"type": "string", "enum": _CLASSES}, "color": _S}, ["class"])
POINT = _obj({"x": {"type": "number"}, "y": {"type": "number"}}, ["x", "y"])
_DIRS4 = {"type": "string", "enum": ["forward", "backward", "left", "right"]}
_TURN = {"type": "string", "enum": ["left", "right"]}
_REF_A = {"object": OBJECT_REF, "object_id": {"type": "integer", "minimum": 1}}


def _target_props() -> dict:
    return {"place": _S, **_REF_A, "point": POINT}


_TARGET = ["place", "object", "object_id", "point"]

_SPECS = [
    # ---- motion and missions
    ToolSpec("walk", "Walk a short distance in a fixed direction relative to the robot.", "motion",
             _obj({"direction": _DIRS4, "distance_m": {"type": "number", "minimum": 0.1, "maximum": 3.0}},
                  ["direction", "distance_m"])),
    ToolSpec("turn", "Turn in place by an angle.", "motion",
             _obj({"direction": _TURN, "angle_deg": {"type": "number", "minimum": 5, "maximum": 360}},
                  ["direction", "angle_deg"])),
    ToolSpec("go_to", "Navigate to a named place, a known object, or a map point (give exactly one).", "motion",
             _obj({**_target_props(), "standoff_m": {"type": "number", "minimum": 0.3, "maximum": 3.0}},
                  exactly_one_of=_TARGET)),
    ToolSpec("find", "Find an object: look in the map first, else explore until it is seen.", "motion",
             _obj({"object": OBJECT_REF, "approach": {"type": "boolean"},
                   "max_time_s": {"type": "number", "minimum": 10, "maximum": 1200}}, ["object"])),
    ToolSpec("explore", "Explore unmapped areas and report what was found.", "motion",
             _obj({"max_time_s": {"type": "number", "minimum": 10, "maximum": 1800}, "area": _S})),
    ToolSpec("patrol", "Visit places in order, reporting new, missing and moved objects.", "motion",
             _obj({"places": {"type": "array", "items": _S, "minItems": 1, "maxItems": 12},
                   "loops": {"type": "integer", "minimum": 1, "maximum": 10}}, ["places"])),
    ToolSpec("inspect", "Go close to a known object and verify that it is still there.", "motion",
             _obj({**_REF_A}, exactly_one_of=["object", "object_id"])),
    ToolSpec("return_home", "Navigate back to the home place.", "motion", _obj()),
    ToolSpec("stop", "Stop immediately and cancel the current task.", "motion", _obj()),
    ToolSpec("mission_control", "Pause, resume or cancel the current mission.", "motion",
             _obj({"action": {"type": "string", "enum": ["pause", "resume", "cancel"]}}, ["action"])),
    ToolSpec("schedule", "Run another tool call after a delay.", "motion",
             _obj({"delay_s": {"type": "integer", "minimum": 1, "maximum": 86400},
                   "call": _obj({"tool": _S, "args": {"type": "object"}}, ["tool", "args"])},
                  ["delay_s", "call"])),
    # ---- questions
    ToolSpec("where_is", "Where is an object that the robot has seen?", "query", _obj({"object": OBJECT_REF}, ["object"])),
    ToolSpec("whats_nearby", "Objects and obstacles around the robot.", "query",
             _obj({"radius_m": {"type": "number", "minimum": 0.5, "maximum": 10}})),
    ToolSpec("where_am_i", "The robot's own position, heading and place.", "query", _obj()),
    ToolSpec("relation", "Spatial relation between two known objects.", "query",
             _obj({"a": OBJECT_REF, "b": OBJECT_REF}, ["a", "b"])),
    ToolSpec("path_check", "Can the robot reach a target, how far, and what is in the way?", "query",
             _obj(_target_props(), exactly_one_of=_TARGET)),
    ToolSpec("status", "Battery (simulated), legs, mode, current task, e-stop and active alerts.", "query", _obj()),
    ToolSpec("why", "Logged causes: why did the robot stop / fail / alert / refuse?", "query",
             _obj({"about": {"type": "string", "enum": ["stop", "last_task", "alert", "refusal"]}}, ["about"])),
    ToolSpec("recall", "Look up stored objects, missions or places from memory.", "query",
             _obj({"what": {"type": "string", "enum": ["object", "mission", "place"]},
                   "query": _obj({"class": {"type": "string", "enum": _CLASSES}, "color": _S,
                                  "since_s": {"type": "number", "minimum": 0}})}, ["what"])),
    ToolSpec("describe_view", "Describe what the camera sees now (visual description is unverified).", "query",
             _obj({"question": _S})),
    ToolSpec("report", "Summary of a finished mission.", "query", _obj({"mission_id": {"type": "integer", "minimum": 1}})),
    # ---- configuration and dialogue
    ToolSpec("no_go_zone", "Add, remove or list no-go zones.", "config",
             _obj({"action": {"type": "string", "enum": ["add", "remove", "list"]}, "place": _S,
                   "around_object_id": {"type": "integer", "minimum": 1},
                   "radius_m": {"type": "number", "minimum": 0.2, "maximum": 5}}, ["action"],
                  at_most_one_of=["place", "around_object_id"])),
    ToolSpec("save_place", "Save the current pose under a name.", "config", _obj({"name": _S}, ["name"])),
    ToolSpec("ask_user", "Ask the user a clarifying question; ends the turn.", "dialogue",
             _obj({"question": _S, "options": {"type": "array", "items": _S, "maxItems": 6}}, ["question"])),
]

TOOLS: dict[str, ToolSpec] = {s.name: s for s in _SPECS}
assert len(TOOLS) == 24


# Arguments the AGENT sets (deterministic routing) but the planner never sees and may not use: not in the prompt, stripped from a planner call
# by `route_plan`, accepted by validation. `find` + `in_place`: only an object seen / found INSIDE that place counts.
INTERNAL_ARGS: dict[str, tuple[str, ...]] = {"find": ("in_place",)}


def public_args(tool: str, args: dict) -> dict:
    """`args` without the agent-internal keys (what the JSON schema of the tool describes)."""
    hide = INTERNAL_ARGS.get(tool, ())
    return {k: v for k, v in args.items() if k not in hide} if hide else args
