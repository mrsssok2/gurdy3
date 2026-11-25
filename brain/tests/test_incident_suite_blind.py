import re
import unittest

from brain.eval import incident_score as SC
from brain.eval import incident_suite_blind_v1 as S


class TestBlind(unittest.TestCase):
    def tearDown(self):
        SC.use_suite("incident_suite")

    def test_shape(self):
        self.assertEqual(len(S.ALL_QUESTIONS), 20)
        self.assertEqual(len({q["id"] for q in S.ALL_QUESTIONS}), 20)
        for q in S.ALL_QUESTIONS:
            self.assertTrue(q["check"], q["id"])
            for r in q["check"].get("all", []) + q["check"].get("none", []):
                re.compile(r)

    def test_none_and_any_routing(self):
        SC.use_suite("incident_suite_blind_v1")
        gt = {"ops": [], "rounds": {}}
        res = {"turns": [{"qid": "b17", "reply": "ok", "calls": [("go_to", {})]}, {"qid": "b18", "reply": "ok", "calls": [("what_changed", {})]},
                         {"qid": "b16", "reply": "There is no room Q.", "calls": [("unchecked_rooms", {})]}]}
        rows = {r["id"]: r for r in SC._rows(res, S.QUESTIONS, gt)}
        self.assertTrue(rows["b17"]["ok"])
        self.assertFalse(rows["b18"]["routed"])
        self.assertTrue(rows["b16"]["ok"])


if __name__ == "__main__":
    unittest.main()
