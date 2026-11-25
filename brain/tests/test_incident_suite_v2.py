"""incident_v2 (Phase 6b): well-formed, held out against every earlier set, colour-optional graph scoring for future runs only (v1 scoring unchanged)."""
import re
import unittest
from pathlib import Path

from brain.eval import incident_score as SC
from brain.eval import incident_suite as V1
from brain.eval import incident_suite_v2 as S

ROOT = Path(__file__).resolve().parents[2]
TOOLS = {"what_changed", "last_seen", "object_history", "timeline", "incident_why", "unchecked_rooms", "where_is", "where_am_i"}
CHECKS = {"all", "none", "time", "stop", "unchecked"}


class TestSuiteV2(unittest.TestCase):
    def tearDown(self):
        SC.use_suite("incident_suite")                    # never leave the scorer on v2 for other tests

    def test_shape(self):
        self.assertEqual((len(S.QUESTIONS), len(S.EXTERNAL), len(S.ALL_QUESTIONS)), (20, 10, 30))
        self.assertEqual(len({q["id"] for q in S.ALL_QUESTIONS}), 30)
        self.assertEqual(len({q["text"] for q in S.ALL_QUESTIONS}), 30)
        for q in S.ALL_QUESTIONS:
            tools = q["expect_tool"] if isinstance(q["expect_tool"], (tuple, list)) else (q["expect_tool"],)
            self.assertTrue(set(tools) <= TOOLS and set(q["check"]) <= CHECKS and q["check"], q["id"])
            for r in q["check"].get("all", []) + q["check"].get("none", []):
                re.compile(r)
        self.assertTrue(all(q["external"] for q in S.EXTERNAL))
        self.assertEqual({q["category"] for q in S.QUESTIONS}, {"changes", "last_seen", "history", "timeline", "rooms"})

    def test_held_out_against_everything_earlier(self):
        self.assertEqual(S.check_overlap(str(ROOT)), {})
        old = {re.sub(r"[^a-z0-9 ]", "", q["text"].lower()) for q in V1.ALL_QUESTIONS}
        self.assertFalse(old & {re.sub(r"[^a-z0-9 ]", "", q["text"].lower()) for q in S.ALL_QUESTIONS})

    def test_same_scenario_as_v1(self):
        self.assertEqual((S.SCENARIO, S.OPS, S.ROUND1, S.ROUND2), (V1.SCENARIO, V1.OPS, V1.ROUND1, V1.ROUND2))

    def test_graph_matching_is_colour_optional_only_in_v2(self):
        gt = {"ops": [{"op": "remove", "cls": "toolbox", "color": "", "wall": 100.0, "from": ("room D", 3.2, 5.3), "to": None},
                      {"op": "move", "cls": "box", "color": "red", "wall": 100.0, "from": ("room A", 0, 0), "to": ("room C", 0, 0)}]}
        ev = [{"etype": "object_missing", "cls": "toolbox", "color": "red", "room": "room D", "wall_t": 150.0, "data": {}},
              {"etype": "object_moved", "cls": "box", "color": "red", "room": "room C", "wall_t": 160.0,
               "data": {"from": {"room": "room A"}, "to": {"room": "room C"}}},
              {"etype": "object_missing", "cls": "box", "color": "blue", "room": "room B", "wall_t": 170.0, "data": {}}]
        v1 = SC.graph_score(gt, ev)
        self.assertEqual((v1["removed_found"], v1["n_false"]), (0, 2))          # the frozen v1 scoring: 'red toolbox' is not the toolbox
        SC.use_suite("incident_suite_v2")
        v2 = SC.graph_score(gt, ev)
        self.assertEqual((v2["removed_found"], v2["moves_found"], v2["n_false"]), (1, 1, 1))     # only the blue box is a false report


if __name__ == "__main__":
    unittest.main()
