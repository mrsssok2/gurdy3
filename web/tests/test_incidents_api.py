"""Timeline panel API (web/api/incidents_api.py) and the map arrows (telemetry `moves`) on the mock robot with the incident graph."""
from __future__ import annotations

import unittest

from brain.agent.graph import Agent
from brain.agent.llm import KeywordLLM
from brain.incidents.backend import virtual_wall, with_incidents
from brain.mock import MockBackend, gazebo_flat_lab
from brain.tools import ToolLayer
from brain.voice.host import AgentHost
from web.api.incidents_api import answer
from web.api.telemetry import MockTelemetry


def make_host():
    mb = MockBackend(gazebo_flat_lab())
    be = with_incidents(mb, None, "lab", wall=virtual_wall(mb, 1_700_000_000.0))
    host = AgentHost(Agent(ToolLayer(be), KeywordLLM(be.known_places, smart=True)), sim_speed=1.0)
    return mb, be, host


def patrol(mb, be, *places):
    assert be.patrol(places=list(places)).ok
    while mb.task is not None:
        be.advance(0.5)
        be.poll_events()


class TestIncidentsApi(unittest.TestCase):
    def test_timeline_changes_and_arrows(self):
        mb, be, host = make_host()
        patrol(mb, be, "room A", "room B", "room A")
        mb.move_object(1, 3.6, 0.2)
        mb.remove_object(4)
        patrol(mb, be, "room B", "room A", "room B")
        tl = answer(be, "timeline", {"since": "session", "limit": "5"}, host.lock)
        self.assertTrue(tl["available"])
        self.assertLessEqual(len(tl["data"]["events"]), 5)
        self.assertGreater(tl["data"]["total"], 5)
        ch = answer(be, "changes", {"since": "session", "room": "room B"})
        self.assertIn("red box", ch["text"])
        self.assertIn("yellow box", ch["text"])
        one = answer(be, "timeline", {"since": "session", "class": "box", "color": "red", "limit": "50"})
        self.assertTrue(all(e.get("object") == "red box" for e in one["data"]["events"]))
        moves = MockTelemetry(host).snapshot()["moves"]
        self.assertEqual([m["object"] for m in moves], ["red box"])
        self.assertEqual((moves[0]["from"]["room"], moves[0]["to"]["room"]), ("room A", "room B"))

    def test_bad_input_and_no_timeline(self):
        mb, be, host = make_host()
        self.assertIn("error", answer(be, "timeline", {"since": "tomorrow"}))
        self.assertEqual(answer(be, "timeline", {"limit": "abc"})["data"]["events"][:0], [])
        self.assertEqual(answer(MockBackend(gazebo_flat_lab()), "timeline", {}), {"available": False})
        self.assertEqual(MockTelemetry(AgentHost(Agent(ToolLayer(MockBackend(gazebo_flat_lab())), KeywordLLM(lambda: []))), ).snapshot()["moves"], [])

    def test_server_route(self):
        import json
        import urllib.request
        from web.api.server import GurdyWebServer
        mb, be, host = make_host()
        srv = GurdyWebServer(host, MockTelemetry(host), token="t", port=0, bind="127.0.0.1")
        srv.start()
        try:
            req = urllib.request.Request(f"http://127.0.0.1:{srv.port}/api/timeline?since=session", headers={"Authorization": "Bearer t"})
            d = json.load(urllib.request.urlopen(req, timeout=5))
            self.assertTrue(d["available"])
            req = urllib.request.Request(f"http://127.0.0.1:{srv.port}/api/changes?since=today", headers={"Authorization": "Bearer t"})
            self.assertIn("text", json.load(urllib.request.urlopen(req, timeout=5)))
        finally:
            srv.close()


if __name__ == "__main__":
    unittest.main()
