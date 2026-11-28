"""Terminal demo: type commands, the robot moves in the mock world (virtual clock). Planner = keyword baseline
until a model server exists (5.3). Run: python3 -m brain.agent.cli   (needs langgraph on PYTHONPATH)
    python3 -m brain.agent.cli --vln [--vln-model gurdy-sft-400]   # lab4 mock world + VLN instructions (default OFF; needs Ollama)
    python3 -m brain.agent.cli --vln --vln-keyword                 # the same with the keyword parser only (no Ollama)"""
from __future__ import annotations

from ..mock import MockBackend
from ..tools import ToolLayer
from .graph import Agent
from .llm import KeywordLLM


def main(argv=None) -> None:
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--vln", action="store_true", help="VLN instructions (brain/vln) on the lab4 mock world; default off")
    ap.add_argument("--vln-model", default=None, help="Ollama model for the VLN plan (default GURDY_VLN_MODEL / gurdy-sft-400)")
    ap.add_argument("--vln-keyword", action="store_true", help="VLN with the keyword parser only (no Ollama)")
    a = ap.parse_args(argv)
    vln = None
    if a.vln:
        from ..mock import gazebo_lab4
        from ..vln.planner import VLNPlanner
        b = MockBackend(gazebo_lab4())
        b.x, b.y, b.yaw = 0.0, 0.0, 0.0
        vln = VLNPlanner(False) if a.vln_keyword else VLNPlanner(model=a.vln_model)
    else:
        b = MockBackend()
    agent = Agent(ToolLayer(b), KeywordLLM(b.known_places), vln=vln)
    print("Gurdy (mock world, keyword planner). Type a command; empty line = advance 5 s; 'quit' to exit.")
    while True:
        try:
            text = input("> ").strip()
        except EOFError:
            break
        if text == "quit":
            break
        if text:
            print("Gurdy:", agent.say(text).reply)
        else:
            b.advance(5)
        for m in agent.pump():
            print("Gurdy:", m)
        if not text:
            print(f"  t={b.t:.0f}s pose=({b.x:.2f},{b.y:.2f}) mission={agent.executor.state}")


if __name__ == "__main__":
    main()
