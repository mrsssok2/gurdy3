"""Builders for the public demo: the mock world (Gazebo flat_lab layout) with the keyword planner (no LLM, no GPU, no robot).
`build_host` is the per-visitor factory: the server calls it once per browser session, so every visitor gets their own world, agent and chat."""
from __future__ import annotations


def make_agent(sample_history: bool = True):
    from brain.agent.graph import Agent
    from brain.agent.llm import KeywordLLM
    from brain.mock import MockBackend, gazebo_flat_lab
    from brain.tools import ToolLayer
    b = MockBackend(gazebo_flat_lab())            # the real Gazebo flat_lab layout, generated from its files
    be = b
    try:                                          # temporal memory (in RAM, per visitor); the browser build has it only if sqlite3 is available
        from brain.incidents.backend import IncidentBackend, virtual_wall
        from brain.incidents.store import IncidentStore
        wall = virtual_wall(b)
        store = IncidentStore(":memory:", b.world.map_id, wall)
        if sample_history:                        # an invented earlier patrol so the Timeline tab is not empty (tagged "sample", see web/api/sample_history.py)
            from .sample_history import seed
            seed(store, wall())
        be = IncidentBackend(b, store)
    except ImportError:
        pass
    return Agent(ToolLayer(be), KeywordLLM(be.known_places, smart=True))


def build_host():
    from brain.voice.host import AgentHost
    return AgentHost(make_agent(), sim_speed=3.0)          # mock world: virtual clock runs 3x wall time (the server starts the pump)
