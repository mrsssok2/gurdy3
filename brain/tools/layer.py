"""ToolLayer: the only entry point the agent uses to act. validate -> dispatch to backend -> envelope + focus.

Deterministic bookkeeping lives here, not in the LLM: the FOCUS slots (design 1.6) are updated from tool
results, and `state_block()` renders the small per-turn prompt block. `ask_user` is handled here (no backend).
"""
from __future__ import annotations

from typing import Any

from .backend import RobotBackend
from .schema import validate
from .specs import TOOLS, public_args
from .types import Result, fail, ok


class ToolLayer:
    def __init__(self, backend: RobotBackend):
        self.backend = backend
        self.focus: dict[str, Any] = {"object": None, "place": None, "last_task": None}
        self.pending_confirmation: dict | None = None
        self.log: list[dict] = []          # every call and result of the session (evaluation, grounding check)

    # ------------------------------------------------------------------ calling
    def call(self, name: str, args: dict[str, Any] | None = None) -> Result:
        args = {} if args is None else args
        spec = TOOLS.get(name)
        if spec is None:
            res = fail("UNKNOWN_TOOL", f"no tool named '{name}'", tools=sorted(TOOLS))
        elif (problems := validate(public_args(name, args), spec.parameters)):
            res = fail("INVALID_ARGS", "; ".join(problems), problems=problems)
        elif name == "ask_user":
            res = ok(question=args["question"], options=args.get("options", []), awaiting_user=True)
        else:
            res = getattr(self.backend, name)(**args)
        self._update_focus(name, args, res)
        self.log.append({"t": self.backend.now(), "tool": name, "args": args, "result": res.to_dict()})
        return res

    def schemas(self) -> list[dict]:
        """Tool list for the LLM prompt."""
        return [s.as_llm_tool() for s in TOOLS.values()]

    # ------------------------------------------------------------------ focus
    def _update_focus(self, name: str, args: dict, res: Result) -> None:
        if not res.ok:
            return
        d = res.data
        obj = None
        if name in ("where_is",) and d.get("matches"):
            obj = d["matches"][0]
        elif name in ("find", "inspect") and isinstance(d.get("object"), dict):
            obj = d["object"]
        elif name == "go_to" and d.get("target", {}).get("kind") == "object":
            obj = d["target"].get("object")
        if obj:
            self.focus["object"] = {k: obj.get(k) for k in ("id", "class", "color")}
        place = args.get("place") if name in ("go_to", "path_check") else None
        if place:
            self.focus["place"] = place
        if name in ("walk", "turn", "go_to", "find", "explore", "patrol", "inspect", "return_home"):
            self.focus["last_task"] = {"tool": name, "args": args, "state": d.get("state", "started")}

    def note_task_finished(self, event: dict) -> None:
        """Feed a `task_finished` event so FOCUS.last_task shows the outcome."""
        lt = self.focus.get("last_task")
        if lt:
            lt["state"] = event.get("data", {}).get("stopped_reason", "done")

    def state_block(self) -> str:
        """The small state block of design 1.6, regenerated every turn from backend + focus."""
        st = self.backend.status().data
        where = self.backend.where_am_i().data
        f = self.focus
        o = f["object"]
        obj = f"#{o['id']} {o['color'] or ''} {o['class']}".replace("  ", " ") if o else "none"
        lt = f["last_task"]
        last = f"{lt['tool']}({_fmt_args(lt['args'])}) {lt['state']}" if lt else "none"
        pend = self.pending_confirmation["tool"] if self.pending_confirmation else "none"
        task = st["task"]["tool"] if st.get("task") else "none"
        batt = "unknown" if st.get("battery_pct") is None else f"{st['battery_pct']:.0f}% (simulated)"
        return (f"STATE  mode={st['mode']} task={task} battery={batt} "
                f"pose={where['place']}\n"
                f"FOCUS  object={obj}   place={f['place'] or 'none'}   last_task={last}\n"
                f"PENDING confirmation: {pend}\n"
                f"KNOWN PLACES {', '.join(self.backend.known_places())}")


def _fmt_args(args: dict) -> str:
    def v(x: Any) -> str:
        if isinstance(x, dict):
            return " ".join(str(i) for i in x.values())
        return str(x)
    return ", ".join(v(x) for x in args.values())
