"""The final-eval machinery can score the lab4 suite (env GURDY_FINAL_SUITE) and still scores the flat_lab suite by default."""
import os
import unittest

from brain.eval import final_checks
from brain.eval.suites import suite_module


class Select(unittest.TestCase):
    def setUp(self):
        self.prev = os.environ.get("GURDY_FINAL_SUITE")

    def tearDown(self):
        if self.prev is None:
            os.environ.pop("GURDY_FINAL_SUITE", None)
        else:
            os.environ["GURDY_FINAL_SUITE"] = self.prev

    def test_default_is_flat_lab(self):
        os.environ.pop("GURDY_FINAL_SUITE", None)
        self.assertIn("nav_room_b", suite_module().BY_ID)

    def test_lab4_end_in_room_d_and_claims(self):
        os.environ["GURDY_FINAL_SUITE"] = "final_suite_lab4"
        rec = {"mission": "l4_room_d", "status": "ok", "falls": 0, "turns": [],
               "poses": [[0, 0, 0, 0], [50, 4.0, 4.0, 0]],
               "messages": [{"who": "robot", "text": "I'm in room D.", "pose": [4.0, 4.0]}, {"who": "robot", "text": "Going to room D.", "pose": [0.0, 0.0]}]}
        v = final_checks.evaluate(rec)
        self.assertTrue(v["success"], v)
        rec["messages"][0]["pose"] = [0.5, 0.0]                    # claims arrival while still in room A
        self.assertEqual(final_checks.evaluate(rec)["wrong_claims"], 1)
        rec["poses"][-1] = [50, 0.5, 0.0, 0]
        self.assertFalse(final_checks.evaluate(rec)["success"])


if __name__ == "__main__":
    unittest.main()
