"""The held-out lab4 suite stays well-formed and really held out (see brain/eval/final_suite_lab4.py)."""
import json
import unittest
from pathlib import Path

from brain.eval import final_suite_lab4 as S

ROOT = Path(__file__).resolve().parents[2]
CHECKS = {"end_in", "end_near", "visited", "never_in", "moved_lt", "moved_gt", "displacement", "yaw_delta", "reply", "no_reply", "map_has",
          "map_lacks", "stop_latency", "held_still", "battery_numbers"}
STEPS = {"say", "wait", "idle", "stop", "estop", "release", "battery", "leg", "note"}


class Lab4Suite(unittest.TestCase):
    def test_size_and_unique_ids(self):
        self.assertTrue(28 <= len(S.SUITE) <= 36)
        self.assertEqual(len(S.SUITE), len({m["id"] for m in S.SUITE}))

    def test_steps_and_checks_known(self):
        for m in S.SUITE:
            for st in m["steps"]:
                self.assertIn(st[0], STEPS, m["id"])
            for c in m["checks"]:
                self.assertIn(c[0], CHECKS, m["id"])
                for a in c[1:2]:
                    if c[0] in ("end_in", "visited", "never_in"):
                        self.assertIn(a, S.PLACES, m["id"])
            goals = m["goal"] if isinstance(m["goal"], list) else [m["goal"]]
            for g in goals:
                self.assertTrue(g is None or g in S.PLACES, m["id"])

    def test_held_out(self):
        hits = {k: v for k, v in S.check_overlap(str(ROOT)).items() if k.lower() not in ("no", "yes")}   # bare yes/no answers are not commands
        self.assertEqual(hits, {})

    def test_world_matches_places(self):
        gt = json.loads((ROOT / "docs/results/data/lab4/lab4_gt.json").read_text())
        self.assertEqual(gt["bounds"], [-2.0, 6.0, -2.0, 6.0])
        for room, (kind, x0, y0, x1, y1) in {k: v for k, v in S.PLACES.items() if k.startswith("room")}.items():
            self.assertTrue(gt["bounds"][0] <= x0 < x1 <= gt["bounds"][1])
        # every object named in the checks exists in the world at that position
        objs = gt["objects"]
        for m in S.SUITE:
            for c in m["checks"]:
                if c[0] == "map_has":
                    cls, col, x, y, tol = c[1:6]
                    self.assertTrue(any(o["class"] == cls and (not col or o["color"] == col) and abs(o["x"] - x) < 0.01 and abs(o["y"] - y) < 0.01 for o in objs), m["id"])
                if c[0] == "map_lacks":
                    self.assertFalse(any(o["class"] == c[1] for o in objs) and m["id"] in ("l4_chair_where", "l4_find_chair"), m["id"])

    def test_limits_positive(self):
        for m in S.SUITE:
            self.assertGreater(m["limit_s"], 0)


if __name__ == "__main__":
    unittest.main()
