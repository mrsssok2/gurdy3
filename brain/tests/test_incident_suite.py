"""The incident-graph evaluation (brain/eval/incident_suite.py + incident_score.py): well-formed, held out, ground truth consistent with the lab4 world,
scorer correct on synthetic runs. NO question is ever put to an agent here (the suite is held out until the Gazebo run)."""
import math
import re
import unittest
from datetime import datetime
from pathlib import Path

from brain.eval import incident_score as SC
from brain.eval import incident_suite as S
from brain.eval.final_suite_lab4 import PLACES

ROOT = Path(__file__).resolve().parents[2]
TOOLS = {"what_changed", "last_seen", "object_history", "timeline", "incident_why", "unchecked_rooms"}
CHECKS = {"all", "none", "time", "stop", "unchecked"}


class TestSuiteShape(unittest.TestCase):
    def test_questions(self):
        self.assertEqual(len(S.QUESTIONS), 20)
        self.assertEqual(len({q["id"] for q in S.QUESTIONS}), 20)
        self.assertEqual(len({q["text"] for q in S.QUESTIONS}), 20)
        for q in S.QUESTIONS:
            self.assertIn(q["expect_tool"], TOOLS, q["id"])
            self.assertTrue(set(q["check"]) <= CHECKS, q["id"])
            for r in q["check"].get("all", []) + q["check"].get("none", []):
                re.compile(r)
            self.assertTrue(q["check"], q["id"])
        cats = {q["category"] for q in S.QUESTIONS}
        self.assertEqual(cats, {"changes", "last_seen", "history", "timeline", "rooms"})

    def test_external_set(self):
        self.assertEqual(len(S.EXTERNAL), 10)
        self.assertEqual(len(S.ALL_QUESTIONS), 30)
        self.assertEqual(len({q["id"] for q in S.ALL_QUESTIONS}), 30)
        for q in S.EXTERNAL:
            self.assertTrue(q["external"] and set(q["check"]) <= CHECKS and set(q["expect_tool"]) <= TOOLS | {"where_am_i"}, q["id"])

    def test_scenario_steps(self):
        kinds = [s[0] for s in S.SCENARIO]
        self.assertEqual(kinds, ["round", "say", "wait", "stop", "say", "mark", "world", "round", "wait"])
        self.assertEqual(sorted(S.ROUND1), sorted(S.ROUND2))
        self.assertNotEqual(S.ROUND1, S.ROUND2)

    def test_held_out(self):
        self.assertEqual(S.check_overlap(str(ROOT)), {})

    def test_two_moves_and_one_removal(self):
        self.assertEqual([o["op"] for o in S.OPS], ["move", "move", "remove"])


class TestGroundTruthWorld(unittest.TestCase):
    """The scenario's objects exist where the suite says, the targets are free, inside their room and clear of walls and doorways."""

    @classmethod
    def setUpClass(cls):
        from brain.ros.incident_eval_runner import model_sdf_pose
        cls.pose = staticmethod(model_sdf_pose)
        import json
        cls.gt = json.loads((ROOT / "docs/results/data/lab4/lab4_gt.json").read_text())

    def room_of(self, x, y):
        for n, v in PLACES.items():
            if n.startswith("room") and v[1] <= x <= v[3] and v[2] <= y <= v[4]:
                return n

    def test_sources_match_the_sdf(self):
        for o in S.OPS:
            x, y, z, yaw = self.pose(o["model"])
            self.assertAlmostEqual(x, o["from"][1], places=1, msg=o["model"])
            self.assertAlmostEqual(y, o["from"][2], places=1, msg=o["model"])
            self.assertEqual(self.room_of(x, y), o["from"][0])

    def test_targets_are_free_and_inside_the_room(self):
        others = []
        for o in S.OPS:
            others.append(self.pose(o["model"])[:2])
        import re as _re
        sdf = (ROOT / "ros2_ws/src/gurdy_gazebo/worlds/lab4.sdf").read_text()
        objs = {m.group(1): tuple(float(t) for t in m.group(2).split()[:2]) for m in _re.finditer(r'<model name="(l4_[a-z_0-9]+)"><static>true</static><pose>([^<]+)</pose>', sdf)}
        self.assertGreaterEqual(len(objs), 10)
        doors = [(2.0, -0.7), (4.0, 2.0), (2.0, 4.2), (-0.7, 2.0)]
        for o in S.OPS:
            if o["op"] != "move":
                continue
            room, x, y = o["to"]
            self.assertEqual(self.room_of(x, y), room)
            r = PLACES[room]
            self.assertTrue(r[1] + 0.5 <= x <= r[3] - 0.5 and r[2] + 0.5 <= y <= r[4] - 0.5, f"{o['model']} too close to a wall")
            for d in doors:
                self.assertGreater(math.hypot(x - d[0], y - d[1]), 0.9, f"{o['model']} blocks a doorway")
            for name, (ox, oy) in objs.items():
                if name != o["model"]:
                    self.assertGreater(math.hypot(x - ox, y - oy), 0.9, f"{o['model']} lands on {name}")

    def test_unchanged_objects_are_everything_else(self):
        names = {("box", "red"), ("barrel", "red"), ("toolbox", "")}
        self.assertEqual(S.CHANGED, names)


def wall_of(h, m, day=1_700_000_000.0):
    d = datetime.fromtimestamp(day).replace(hour=h, minute=m, second=0, microsecond=0)
    return d.timestamp()


def gt_synth():
    base = wall_of(10, 0)
    r1 = [{"room": r, "start": base + 100 * i, "end": base + 100 * i + 90} for i, r in enumerate(S.ROUND1)]
    r2 = [{"room": r, "start": base + 1000 + 100 * i, "end": base + 1000 + 100 * i + 90} for i, r in enumerate(S.ROUND2)]
    ops = [{**o, "wall": base + 900, "verified": True} for o in S.OPS]
    return {"ops": ops, "rounds": {"1": r1, "2": r2}, "stop": {"wall": base + 500}, "left": base + 600, "t_end": base + 2000}, base


class TestScorer(unittest.TestCase):
    def test_clocks_and_windows(self):
        self.assertEqual(SC.clocks("at 10:42 (3 minutes ago) and 9:05"), [(10, 42), (9, 5)])
        lo = wall_of(10, 0)
        self.assertTrue(SC._in_window(10, 1, lo, lo + 120))
        self.assertFalse(SC._in_window(10, 30, lo, lo + 120))

    def test_check_answer_time_and_words(self):
        gt, base = gt_synth()
        q = S.BY_ID["q07"]
        sw = SC.sweep(gt, 1, "room D")
        t = datetime.fromtimestamp(sw["start"] + 30).strftime("%H:%M")
        good = f"I last saw the toolbox in room D at {t} (3 minutes ago), but it was not there when I checked later."
        self.assertEqual(SC.check_answer(q, good, gt, base + 2000), [])
        self.assertTrue(SC.check_answer(q, good.replace(t, "03:17"), gt, base + 2000))
        self.assertTrue(SC.check_answer(q, f"I last saw the toolbox in room C at {t}, but it was not there.", gt, base + 2000))
        self.assertTrue(SC.check_answer(q, "I have no record of a toolbox.", gt, base + 2000))

    def test_wrong_claims_are_counted(self):
        gt, base = gt_synth()
        q = S.BY_ID["q01"]
        ok = "Today: 3 changes. The red box moved from room A to room C; the red barrel moved from room D to room B; the toolbox is missing from room D."
        self.assertEqual(SC.check_answer(q, ok, gt, base), [])
        self.assertIn("must not say", " ".join(SC.check_answer(q, ok + " The blue box moved too.", gt, base)))

    def test_history_yes_no(self):
        gt, base = gt_synth()
        self.assertEqual(SC.check_answer(S.BY_ID["q11"], "Yes. The red box moved from room A to room C at 10:20.", gt, base), [])
        self.assertTrue(SC.check_answer(S.BY_ID["q11"], "I haven't recorded the red box moving.", gt, base))
        self.assertEqual(SC.check_answer(S.BY_ID["q12"], "I haven't recorded the blue box moving. I have seen it 2 times.", gt, base), [])
        self.assertTrue(SC.check_answer(S.BY_ID["q12"], "Yes. The blue box moved.", gt, base))

    def test_unchecked_expectation_and_answer(self):
        gt, base = gt_synth()
        asked = gt["rounds"]["2"][-1]["end"] + 300
        exp = SC.unchecked_expectation(gt, asked, 300)
        self.assertEqual(exp["definitely_recent"], ["room A"])                 # the robot is standing in it
        self.assertIn("room B", exp["definitely_unchecked"])
        q = S.BY_ID["q18"]
        rooms = exp["definitely_unchecked"] + exp["grey"]
        good = "Not checked for 5 minutes: " + ", ".join(f"{r} (last at 10:00, 9 minutes ago)" for r in rooms) + ". Checked recently: room A."
        self.assertEqual(SC.check_answer(q, good, gt, asked), [])
        self.assertTrue(SC.check_answer(q, "I have checked every room less than 5 minutes ago.", gt, asked))
        self.assertTrue(SC.check_answer(q, "Not checked for 5 minutes: room A (last at 10:00, 9 minutes ago).", gt, asked))

    def test_stop_time(self):
        gt, base = gt_synth()
        t = datetime.fromtimestamp(base + 500).strftime("%H:%M")
        self.assertEqual(SC.check_answer(S.BY_ID["q17"], f"At {t} (30 minutes ago): I stopped because you told me to. You said \"stop\".", gt, base), [])
        self.assertTrue(SC.check_answer(S.BY_ID["q17"], "I have no record of a stop or a failure.", gt, base))

    def test_graph_score(self):
        gt, base = gt_synth()
        w = base + 1100

        def ev(t, cls, color, room, frm=None, to=None, at=w):
            return {"etype": t, "cls": cls, "color": color, "room": room, "wall_t": at, "data": {"from": {"room": frm}, "to": {"room": to}} if frm else {}}
        good = [ev("object_moved", "box", "red", "room C", "room A", "room C"), ev("object_moved", "barrel", "red", "room B", "room D", "room B"),
                ev("object_missing", "toolbox", "", "room D")]
        s = SC.graph_score(gt, good)
        self.assertEqual((s["moves_found"], s["removed_found"], s["n_false"]), (2, 1, 0))
        s2 = SC.graph_score(gt, good[:1] + [ev("object_missing", "box", "blue", "room B"), ev("object_moved", "traffic_cone", "", "room A", "room A", "room D")])
        self.assertEqual((s2["moves_found"], s2["removed_found"], s2["n_false"]), (1, 0, 2))
        early = SC.graph_score(gt, [ev("object_moved", "box", "red", "room C", "room A", "room C", at=base + 100)])
        self.assertEqual((early["moves_found"], early["n_false"]), (0, 1))                       # a move reported BEFORE the object was moved

    def test_score_and_report_end_to_end_on_a_synthetic_run(self):
        gt, base = gt_synth()
        turns = [{"qid": q["id"], "reply": "I have no record of anything.", "calls": [[q["expect_tool"], {}]], "wall": base + 1900, "grounding_final": 0, "llm_calls": 0}
                 for q in S.QUESTIONS]
        res = {"gt": gt, "turns": turns, "graph_events": [], "model": "x", "started": "t", "commit": "c"}
        s = SC.score(res)
        self.assertEqual(s["n_questions"], 20)
        self.assertEqual(s["routed"], 20)
        self.assertLess(s["answers_ok"], 20)                                      # an empty answer to every question must not pass
        self.assertIn("Incident graph evaluation", SC.report(s, res))
        self.assertEqual(s["external"]["n_questions"], 10)
        self.assertEqual(s["external"]["answers_ok"], 0)
        self.assertEqual(SC.score({**res, "turns": []})["answers_ok"], 0)


class TestRunner(unittest.TestCase):
    def test_runner_module_imports_without_ros(self):
        import brain.ros.incident_eval_runner as R
        self.assertTrue(callable(R.apply_op) and callable(R.main))

    def test_shell_script_is_valid_and_gated(self):
        txt = (ROOT / "scripts/incident_eval.sh").read_text()
        self.assertIn("check_overlap", txt)
        self.assertIn("docker ps", txt)
        self.assertIn("lab4_incident.sdf", txt)

    def test_eval_world_is_a_copy_with_three_dynamic_models(self):
        import subprocess, sys
        subprocess.run([sys.executable, str(ROOT / "scripts/make_incident_world.py")], check=True, capture_output=True)
        w = ROOT / "ros2_ws/src/gurdy_gazebo/worlds"
        a, b = (w / "lab4.sdf").read_text(), (w / "lab4_incident.sdf").read_text()
        self.assertEqual(b.count("<static>false</static>"), 3)
        self.assertEqual(b.replace("<static>false</static>", "<static>true</static>"), a)


if __name__ == "__main__":
    unittest.main()
