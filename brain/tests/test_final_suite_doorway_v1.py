"""The doorway_v1 suite (brain/eval/final_suite_doorway_v1.py) stays well-formed and its doorway regions match the lab4 world."""
import json
import os
import unittest
from pathlib import Path

from brain.eval import final_suite_doorway_v1 as S
from brain.eval import final_checks

ROOT = Path(__file__).resolve().parents[2]
CHECKS = {"end_in", "visited", "no_reply"}
STEPS = {"say", "wait", "idle"}


class DoorwayV1Suite(unittest.TestCase):
    def test_size_and_ids(self):
        self.assertEqual(len(S.SUITE), 14)
        self.assertEqual(len(S.SUITE), len({m["id"] for m in S.SUITE}))
        self.assertEqual(sum(m["category"] == "tight" for m in S.SUITE), 2)

    def test_steps_checks_places(self):
        for m in S.SUITE:
            for st in m["steps"]:
                self.assertIn(st[0], STEPS, m["id"])
            for c in m["checks"]:
                self.assertIn(c[0], CHECKS, m["id"])
                if c[0] in ("end_in", "visited"):
                    self.assertIn(c[1], S.PLACES, m["id"])
            self.assertIn(("no_reply", S.BLOCKED), m["checks"], m["id"])
            goals = m["goal"] if isinstance(m["goal"], list) else [m["goal"]]
            for g in goals:
                self.assertIn(g, S.PLACES, m["id"])

    def test_every_door_both_directions(self):
        seen = {k: 0 for k in S.DOORS}
        for m in S.SUITE:
            for c in m["checks"]:
                if c[0] == "visited" and c[1].startswith("door "):
                    seen[c[1][5:]] += 1
        self.assertTrue(all(v >= 2 for v in seen.values()), seen)

    def test_doors_match_world(self):
        gt = json.loads((ROOT / "docs/results/data/lab4/lab4_gt.json").read_text())
        for k, (x, y) in S.DOORS.items():
            gx, gy = gt["doors"][k]
            self.assertAlmostEqual(gx, x, places=2)
            self.assertAlmostEqual(gy, y, places=2)

    def test_checks_resolve_with_suite_active(self):
        os.environ["GURDY_FINAL_SUITE"] = "final_suite_doorway_v1"
        try:
            rec = {"poses": [[0, 0.0, 0.0, 0.0], [1, 2.0, -0.7, 0.0], [2, 4.0, 0.0, 0.0]], "messages": [{"who": "robot", "text": "I'm in room B."}]}
            for c in S.BY_ID["dw_a_to_b"]["checks"]:
                ok, why = final_checks.run_check(c, rec)
                self.assertTrue(ok, why)
            rec["messages"].append({"who": "robot", "text": "I couldn't reach room B: my path is blocked."})
            ok, _ = final_checks.run_check(("no_reply", S.BLOCKED), rec)
            self.assertFalse(ok)
        finally:
            os.environ.pop("GURDY_FINAL_SUITE", None)


if __name__ == "__main__":
    unittest.main()
