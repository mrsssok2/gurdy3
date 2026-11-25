"""Incident tool picker (Phase 6d): strict validation of the model's choice, pre-filter, fallbacks, integration with the agent (template answers only)."""
from __future__ import annotations

import json
import re
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from test_incidents import LAB4, Rig, TestWhatChangedSemantics  # noqa: E402

from brain.agent import Agent  # noqa: E402
from brain.agent.llm import KeywordLLM  # noqa: E402
from brain.eval import incident_picker_dev as DEV  # noqa: E402
from brain.eval import incident_suite as V1, incident_suite_v2 as V2, incident_suite_v3 as V3  # noqa: E402
from brain.incidents import picker as P  # noqa: E402
from brain.tools import ToolLayer  # noqa: E402

ROOMS = ["room A", "room B", "room C", "room D"]


def stub(answer):
    """A picker whose 'model' answers `answer` (dict -> JSON, str -> as is, exception -> raised)."""
    def transport(system, user):
        if isinstance(answer, Exception):
            raise answer
        return answer if isinstance(answer, str) else json.dumps(answer)
    return P.IncidentPicker(transport=transport)


def norm(t):
    return re.sub(r"[^a-z0-9 ]", "", t.lower()).strip()


class TestValidation(unittest.TestCase):
    def v(self, raw, text):
        return P.validate(raw, text, ROOMS)

    def test_tool_must_be_allowed(self):
        for tool in ("none", "where_is", "go_to", "drop_table", None, 5, ["what_changed"]):
            self.assertIsNone(self.v({"tool": tool}, "what changed"), tool)
        self.assertIsNone(self.v("what_changed", "what changed"))
        self.assertIsNone(self.v(None, "what changed"))

    def test_room_must_be_on_the_map(self):
        self.assertEqual(self.v({"tool": "what_changed", "room": "room C"}, "anything odd in room C")["args"]["room"], "room C")
        self.assertEqual(self.v({"tool": "what_changed", "room": "c"}, "anything odd in room C")["args"]["room"], "room C")
        self.assertIsNone(self.v({"tool": "what_changed", "room": "room Z"}, "anything odd in room Z"))          # not on the map: never guessed
        self.assertNotIn("room", self.v({"tool": "what_changed", "room": "the kitchen"}, "anything odd in the kitchen")["args"])   # not a map room: dropped, never used
        self.assertIsNone(self.v({"tool": "what_changed", "room": "room A"}, "anything odd in room C"))          # contradicts the user's words
        self.assertNotIn("room", self.v({"tool": "what_changed", "room": "room A"}, "anything odd around here")["args"])   # a room the user did not name is dropped

    def test_object_comes_from_the_users_words_not_the_model(self):
        r = self.v({"tool": "last_seen", "object": {"class": "toolbox"}}, "when did you last notice the red crate")
        self.assertEqual(r["args"]["object"], {"class": "box", "color": "red"})
        self.assertIsNone(self.v({"tool": "last_seen"}, "when did you last notice the gizmo"))                    # no known object named
        self.assertIsNone(self.v({"tool": "object_history"}, "did the sofa move"))

    def test_object_tool_with_only_a_room_is_remapped(self):
        self.assertEqual(self.v({"tool": "last_seen"}, "how long since you were in room B")["tool"], "unchecked_rooms")
        self.assertEqual(self.v({"tool": "object_history"}, "has anything shifted in room B")["tool"], "what_changed")

    def test_times_are_parsed_from_the_text_not_trusted_from_the_model(self):
        r = self.v({"tool": "unchecked_rooms", "since": "ago:1", "older_than_s": 1}, "which areas stayed unvisited for longer than seven minutes")
        self.assertEqual(r["args"], {"older_than_s": 420.0})
        self.assertEqual(self.v({"tool": "what_changed"}, "anything different since I left")["args"]["since"], "away")
        self.assertEqual(self.v({"tool": "what_changed"}, "anything different lately")["args"]["since"], "ago:600")
        self.assertEqual(self.v({"tool": "what_changed"}, "anything different since 10")["args"]["since"], "clock:10:00")
        self.assertNotIn("since", self.v({"tool": "what_changed"}, "anything different")["args"])
        self.assertEqual(self.v({"tool": "unchecked_rooms", "older_than_s": 1}, "which rooms are neglected")["args"], {})   # the tool default, not the model's number

    def test_still_and_where_styles(self):
        r = self.v({"tool": "object_history", "still": True, "room": "room D"}, "is the red barrel still in room D")
        self.assertEqual(r["style"], {"still": True, "room": "room D"})
        self.assertEqual(self.v({"tool": "object_history"}, "where did the barrel end up")["style"], {"where": True})
        self.assertEqual(self.v({"tool": "incident_why", "about": "bogus"}, "why did you stop")["args"], {"about": "stop"})


class TestPrefilter(unittest.TestCase):
    def test_commands_are_not_candidates_but_information_requests_are(self):
        for t in ("go to room B", "find the red box", "stop", "turn around", "patrol the rooms", "walk forward"):
            self.assertFalse(P.might_be_incident(t), t)
        for t in ("walk me through your day", "run through what happened", "has anything changed", "did the box move", "what happened earlier", "tell me what is different"):
            self.assertTrue(P.might_be_incident(t), t)
        self.assertFalse(P.might_be_incident(""))
        self.assertFalse(P.might_be_incident("x " * 200))


class TestPickerNeverBreaksAnything(unittest.TestCase):
    def test_failures_return_none(self):
        for answer in (ConnectionError("down"), "not json at all", "{broken", '["a"]', "", {"tool": "none"}):
            pk = stub(answer)
            self.assertIsNone(pk.pick("has anything changed in room A", ROOMS), answer)
        self.assertEqual(stub(ConnectionError("x")).pick("has anything changed", ROOMS), None)

    def test_json_wrapped_in_text_is_still_read(self):
        pk = stub('Sure! {"tool": "timeline", "room": null} hope that helps')
        self.assertEqual(pk.pick("give me a recap", ROOMS)["tool"], "timeline")

    def test_not_asked_for_non_candidates(self):
        calls = []
        pk = P.IncidentPicker(transport=lambda s, u: calls.append(u) or '{"tool": "what_changed"}')
        self.assertIsNone(pk.pick("go to room B", ROOMS))
        self.assertEqual(calls, [])


class TestAgentIntegration(unittest.TestCase):
    def agent(self, rig, answer):
        llm = KeywordLLM(rig.ib.known_places)
        llm.incident_picker = stub(answer)
        return Agent(ToolLayer(rig.ib), llm)

    def test_picked_tool_answers_with_a_template_over_the_result(self):
        rig = TestWhatChangedSemantics().scenario()
        ag = self.agent(rig, {"tool": "what_changed", "room": None})
        r = ag.say("were any things relocated while nobody was around")      # no keyword pattern knows this wording
        self.assertEqual(r.calls[0]["tool"], "what_changed")
        self.assertIn("3 changes", r.reply)
        self.assertEqual(r.verdicts[0]["rule"], "incident_query")
        self.assertEqual(r.llm_calls, 0)

    def test_invalid_or_none_falls_back_to_the_keyword_routing(self):
        rig = TestWhatChangedSemantics().scenario()
        for answer in ({"tool": "none"}, {"tool": "what_changed", "room": "room Z"}, ConnectionError("down"), "garbage"):
            ag = self.agent(rig, answer)
            r = ag.say("what changed since the start?")                       # the exact patterns handle it without the picker
            self.assertIn("3 changes", r.reply, answer)

    def test_picker_cannot_make_the_robot_move_or_print_memory(self):
        rig = TestWhatChangedSemantics().scenario()
        ag = self.agent(rig, {"tool": "go_to", "room": "room A"})
        r = ag.say("did anything happen to the place")
        self.assertFalse([c for c in r.calls if c["tool"] == "go_to"])

    def test_unknown_room_is_answered_before_the_picker(self):
        rig = TestWhatChangedSemantics().scenario()
        ag = self.agent(rig, {"tool": "what_changed", "room": "room A"})
        self.assertIn("There is no room Q", ag.say("did anything happen in room Q").reply)


class TestHeldOut(unittest.TestCase):
    def test_few_shot_and_dev_do_not_overlap_the_held_out_sets_or_each_other(self):
        shots = {norm(m) for m in re.findall(r"^Q: (.+)$", P.PROMPT, re.M)}
        dev = {norm(t) for t, _, _ in DEV.DEV}
        held = {norm(q["text"]) for mod in (V1, V2, V3) for q in mod.ALL_QUESTIONS}
        self.assertEqual(shots & held, set())
        self.assertEqual(dev & held, set())
        self.assertEqual(shots & dev, set())
        self.assertEqual(len(DEV.DEV), 66)


if __name__ == "__main__":
    unittest.main()
