"""incident_v3 (Phase 6c): well-formed, held out against every earlier set (v1, v2, tests, SFT data), same scenario as v1."""
import re
import unittest
from pathlib import Path

from brain.eval import incident_score as SC
from brain.eval import incident_suite as V1
from brain.eval import incident_suite_v2 as V2
from brain.eval import incident_suite_v3 as S

ROOT = Path(__file__).resolve().parents[2]
TOOLS = {"what_changed", "last_seen", "object_history", "timeline", "incident_why", "unchecked_rooms", "where_is", "where_am_i"}
CHECKS = {"all", "none", "time", "stop", "unchecked"}


def norm(t):
    return re.sub(r"[^a-z0-9 ]", "", t.lower()).strip()


class TestSuiteV3(unittest.TestCase):
    def tearDown(self):
        SC.use_suite("incident_suite")

    def test_shape(self):
        self.assertEqual(len(S.QUESTIONS), 20)
        self.assertEqual(len(S.ALL_QUESTIONS), len(S.QUESTIONS) + len(S.EXTERNAL))
        self.assertEqual(len({q["id"] for q in S.ALL_QUESTIONS}), len(S.ALL_QUESTIONS))
        for q in S.ALL_QUESTIONS:
            tools = q["expect_tool"] if isinstance(q["expect_tool"], (tuple, list)) else (q["expect_tool"],)
            self.assertTrue(set(tools) <= TOOLS and set(q["check"]) <= CHECKS and q["check"], q["id"])
            for r in q["check"].get("all", []) + q["check"].get("none", []):
                re.compile(r)
        self.assertEqual({q["category"] for q in S.QUESTIONS}, {"changes", "last_seen", "history", "timeline", "rooms"})
        self.assertTrue(all(q["external"] for q in S.EXTERNAL))

    def test_held_out_against_everything_earlier(self):
        self.assertEqual(S.check_overlap(str(ROOT)), {})
        old = {norm(q["text"]) for q in V1.ALL_QUESTIONS + V2.ALL_QUESTIONS}
        self.assertFalse(old & {norm(q["text"]) for q in S.ALL_QUESTIONS})

    def test_same_scenario(self):
        self.assertEqual((S.SCENARIO, S.OPS, S.ROUND1, S.ROUND2), (V1.SCENARIO, V1.OPS, V1.ROUND1, V1.ROUND2))


if __name__ == "__main__":
    unittest.main()
