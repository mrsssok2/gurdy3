"""OFFLINE REPLAY evaluation of incident questions (Phase 6d): no Gazebo. Replays the recorded temporal graph of a finished evaluation run (its `stack/incidents.db`)
and asks a suite's questions through the REAL agent (router, tool picker, planner, templates), scored with the same checks against the same ground truth.

    python3 -m brain.eval.incident_replay RUN_DIR [--suite incident_suite_v3] [--model gurdy-sft-400] [--picker on|off] [--llm ollama|keyword] [--doc out.md] [--json out.json]

How it works: the run's database is COPIED (the original is never touched), opened with a clock that returns, for question i, the wall time at which the same question
index was asked in the live run, so every "N minutes ago" / clock time is what the live run would have said. The backend (`ReplayBackend`) has the graph, the map's places
and the object memory of the run; everything that needs the robot (walking, camera, battery ...) answers UNAVAILABLE, so a question the router misses and the planner sends
to `where_is` / `recall` sees the same stale object memory as live. Use it to measure routing / picker / template changes in minutes instead of a 1 h Gazebo run.

Limits (read before trusting a number): the graph is whatever the recorder of THAT run produced (a run recorded before a recorder fix keeps its old events); the planner path has no
live robot; suites other than the one the run asked are scored against the run's ground truth, which is valid only for suites with the same scenario (v1 / v2 / v3).
Held-out hygiene: replaying a held-out suite and then changing code to pass it turns it into a dev set; the report says which suite and run were replayed."""
from __future__ import annotations

import argparse
import importlib
import json
import shutil
import tempfile
import time
from pathlib import Path
from typing import Any

import yaml

from ..agent import Agent
from ..agent.llm import KeywordLLM
from ..incidents.backend import IncidentBackend
from ..incidents.store import IncidentStore
from ..tools import ToolLayer
from ..tools.types import fail, ok
from . import incident_score as SC


class ReplayBackend:
    """Graph + places + object memory of a finished run; nothing else works (UNAVAILABLE)."""

    def __init__(self, store: IncidentStore, places: list[str], objects: list[dict]):
        import threading
        self.store, self._places, self._objects = store, places, objects
        self._lock = threading.RLock()

    def now(self) -> float:
        return 0.0

    def known_places(self) -> list[str]:
        return list(self._places)

    def known_rooms(self) -> list[str]:
        return [p for p in self._places if p.lower().startswith("room")]

    def status(self):
        return ok(mode="idle", task=None, battery_pct=None, legs={}, estop=False)

    def where_am_i(self):
        return ok(x=0.0, y=0.0, heading_deg=0.0, place="room A", pose_known=True)

    def tick(self, force: bool = False) -> None:            # nothing to sample
        return None

    def incident_query(self, name: str, args: dict):
        return IncidentBackend.incident_query(self, name, args)         # same dispatch as live (shares store / known_rooms / tick / _lock)

    def recall(self, what: str, query: dict | None = None):
        if what != "object":
            return ok(records=[])
        query = query or {}
        recs = [{"id": o["id"], "class": o["cls"], "color": o["color"] or "", "x": o["x"], "y": o["y"], "place": o["room"], "age_s": 0.0, "state": "present"}
                for o in self._objects if o["state"] != "merged"
                and (not query.get("class") or o["cls"] == query["class"]) and (not query.get("color") or (o["color"] or "").lower() == query["color"].lower())]
        return ok(records=recs)

    def where_is(self, object: dict | None = None, object_id: int | None = None):          # noqa: A002  stale memory: what the live agent would have printed
        cls, color = (object or {}).get("class"), ((object or {}).get("color") or "").lower()
        hits = [o for o in self._objects if o["state"] != "merged" and o["cls"] == cls and (not color or (o["color"] or "").lower() == color)]
        if not hits:
            return fail("NOT_FOUND", f"I have not seen a {cls}.")
        return ok(matches=[{"id": o["id"], "class": o["cls"], "color": o["color"] or "", "place": o["room"], "distance_m": 0.0, "bearing_word": "ahead",
                            "last_seen_age_s": 0.0} for o in hits], found=True)

    def __getattr__(self, name: str):
        if name.startswith("_"):
            raise AttributeError(name)
        return lambda *a, **k: fail("UNAVAILABLE", f"{name} is not available in a replay")


def load(run_dir: Path, tmp: Path):
    result = json.load(open(run_dir / "result.json"))
    db = tmp / "incidents.db"
    shutil.copy(run_dir / "stack" / "incidents.db", db)
    clock = [time.time()]
    store = IncidentStore(db, result.get("map_id") or "lab4", lambda: clock[0])
    ss = store.events(etypes=["session_start"])
    if ss:
        store.started_wall = min(e["wall_t"] for e in ss)               # "since you started me" = the live run's start, not the replay's
    places = list((yaml.safe_load(open(run_dir / "stack" / "places.yaml")) or {}).get("places", {}))
    objects = [{"id": n["id"], "cls": n["cls"], "color": n["color"], "x": n["x"], "y": n["y"], "room": n["room"], "state": n["state"]} for n in store.nodes("object")]
    return result, store, clock, ReplayBackend(store, places, objects)


def replay(run_dir: str, suite: str = "incident_suite_v3", model: str = "gurdy-sft-400", picker: bool = True, llm: str = "ollama") -> dict:
    S = importlib.import_module(f"brain.eval.{suite}")
    SC.use_suite(suite)
    tmp = Path(tempfile.mkdtemp(prefix="replay_"))
    result, store, clock, be = load(Path(run_dir), tmp)
    if llm == "ollama":
        from ..agent.ollama_llm import OllamaLLM
        model_llm: Any = OllamaLLM(model, constrain=False)
        if not picker:
            model_llm.incident_picker = None
    else:
        model_llm = KeywordLLM(be.known_places)
        if picker:
            from ..incidents.picker import IncidentPicker
            model_llm.incident_picker = IncidentPicker(model=model)
    agent = Agent(ToolLayer(be), model_llm)
    live_walls = [t["wall"] for t in result["turns"] if t.get("qid")]
    turns = []
    for i, q in enumerate(S.ALL_QUESTIONS):
        clock[0] = live_walls[min(i, len(live_walls) - 1)] if live_walls else time.time()
        t0 = time.perf_counter()
        r = agent.say(q["text"])
        turns.append({"say": q["text"], "qid": q["id"], "reply": r.reply, "calls": [(c.get("tool"), c.get("args")) for c in r.calls], "verdicts": r.verdicts,
                      "llm_calls": r.llm_calls, "grounding_final": len((r.grounding or {}).get("violations", [])), "latency_s": round(time.perf_counter() - t0, 2),
                      "wall": clock[0], "started_motion": False})
        agent.gate.clear()
    out = {**{k: result[k] for k in ("gt", "falls", "started", "commit", "model") if k in result}, "turns": turns, "graph_events": result.get("graph_events", []),
           "replay": {"run_dir": str(run_dir), "suite": suite, "model": model, "picker": picker, "llm": llm}}
    shutil.rmtree(tmp, ignore_errors=True)
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--suite", default="incident_suite_v3")
    ap.add_argument("--model", default="gurdy-sft-400")
    ap.add_argument("--picker", default="on", choices=["on", "off"])
    ap.add_argument("--llm", default="ollama", choices=["ollama", "keyword"])
    ap.add_argument("--doc")
    ap.add_argument("--json")
    a = ap.parse_args()
    out = replay(a.run_dir, a.suite, a.model, a.picker == "on", a.llm)
    s = SC.score(out)
    md = SC.report(s, out).replace("# Incident graph evaluation", f"# OFFLINE REPLAY ({a.llm}, picker {a.picker}, run {a.run_dir})\n\n# Incident graph evaluation", 1)
    if a.json:
        Path(a.json).write_text(json.dumps(out, indent=1, default=str))
    if a.doc:
        Path(a.doc).write_text(md)
    for part, name in ((s, "own"), (s["external"], "external")):
        print(f"{name}: {part['answers_ok']}/{part['n_questions']}  routed {part['routed']}  content {part['content_ok']}  wrong claims {part['wrong_claims']}  "
              f"p50 {part['latency_p50_s']} s  LLM questions {part['llm_questions']}")


if __name__ == "__main__":
    main()
