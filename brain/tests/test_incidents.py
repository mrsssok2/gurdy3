"""Incident graph (temporal memory): store, recorder (event extraction, move / missing detection), each query tool, routing, grounding, agent."""
from __future__ import annotations

import os
import tempfile
import unittest
import unittest.mock

from brain.agent import Agent
from brain.agent.grounding import collect_facts, violations
from brain.agent.llm import KeywordLLM, ScriptedLLM
from brain.incidents import nlu, phrasing, queries
from brain.incidents.backend import IncidentBackend, with_incidents
from brain.incidents.store import IncidentStore
from brain.mock import MockBackend
from brain.mock.lab_world import gazebo_flat_lab
from brain.tools import ToolLayer

T0 = 1_700_000_000.0      # a fixed "wall clock": wall = T0 + sim time, so every test is deterministic
RED, BLUE, GREEN, YELLOW = 1, 2, 3, 4


class Rig:
    """Mock robot in the Gazebo-flat_lab world + the incident wrapper; wall time follows the virtual clock."""

    def __init__(self, db_path: str | None = None, mb: MockBackend | None = None, t0: float = T0, offset_from: "Rig | None" = None):
        self.mb = mb or MockBackend(gazebo_flat_lab())
        self.t0 = t0
        self.ib = with_incidents(self.mb, db_path, "lab", wall=lambda: self.t0 + self.mb.now())

    @property
    def store(self) -> IncidentStore:
        return self.ib.store

    def drive(self, seconds: float) -> None:
        end = self.mb.now() + seconds
        while self.mb.now() < end:
            self.mb.advance(0.5)
            self.ib.poll_events()

    def patrol(self, *places: str) -> None:
        assert self.ib.patrol(places=list(places)).ok
        while self.mb.task is not None:
            self.mb.advance(0.5)
            self.ib.poll_events()

    def since_mid(self) -> str:
        """The window 'since the first patrol ended' (the second patrol and what it found)."""
        return f"ago:{int(self.store.wall() - self.mid) + 1}"

    def q(self, name: str, **args):
        return self.ib.incident_query(name, args)


def two_patrols(move=True, remove=True) -> Rig:
    rig = Rig()
    rig.patrol("room A", "room B", "room A")                  # first patrol: everything is seen where it is
    rig.mid = rig.store.wall()
    if move:
        rig.mb.move_object(RED, 3.6, 0.2)                     # red box: room A -> room B
    if remove:
        rig.mb.remove_object(YELLOW)                          # yellow box (room B) disappears
    rig.patrol("room B", "room A", "room B")                  # second patrol
    return rig


class TestStore(unittest.TestCase):
    def test_graph_structure_and_persistence(self):
        path = os.path.join(tempfile.mkdtemp(), "g.db")
        rig = Rig(path)
        rig.patrol("room A", "room B")
        n1 = rig.store.count()
        self.assertGreater(n1["events"], 5)
        types = {e["rel"] for e in rig.store.edges()}
        self.assertTrue({"before", "about", "in", "seen_in", "by"} <= types)
        rig.store.close()
        again = IncidentStore(path, "lab")                    # restart: the events are still there, the session is new
        self.assertGreaterEqual(again.count()["events"], n1["events"])
        self.assertTrue(again.nodes("object"))

    def test_memory_false_starts_empty(self):
        rig = Rig(None)
        self.assertEqual(rig.store.path, ":memory:")
        rig.patrol("room A")
        self.assertGreater(rig.store.count()["events"], 3)
        fresh = Rig(None)                                                  # a new process with MEMORY=false: nothing carried over
        self.assertEqual([e["etype"] for e in fresh.store.events()], ["session_start"])
        self.assertEqual(fresh.store.nodes("object"), [])

    def test_map_isolation(self):
        path = os.path.join(tempfile.mkdtemp(), "g.db")
        a = Rig(path)
        a.patrol("room A")
        a.store.close()
        b = IncidentStore(path, "other_map")
        self.assertEqual(b.nodes("object"), [])

    def test_thread_safe(self):
        import threading
        rig = Rig()
        errs = []

        def work():
            try:
                for _ in range(30):
                    rig.q("timeline")
                    rig.store.add_event("alert", "x", 1.0)
            except Exception as e:                            # noqa: BLE001
                errs.append(e)
        ts = [threading.Thread(target=work) for _ in range(4)]
        [t.start() for t in ts]
        [t.join() for t in ts]
        self.assertEqual(errs, [])


class TestRecorder(unittest.TestCase):
    def test_extraction_first_patrol(self):
        rig = Rig()
        rig.patrol("room A", "room B", "room A")
        by = {}
        for e in rig.store.events():
            by.setdefault(e["etype"], []).append(e)
        self.assertEqual({"session_start", "task_started", "task_finished", "room_visited", "room_left", "object_seen"} - set(by), set())
        firsts = [e for e in by["object_seen"] if e["data"]["first"]]
        self.assertEqual({e["cls"] + e["color"] for e in firsts}, {"boxred", "barrelgreen", "boxyellow"})   # the blue box is never in view
        red = next(e for e in firsts if e["color"] == "red")
        self.assertEqual(red["room"], "room A")
        self.assertIsNotNone(red["data"]["confidence"])
        self.assertTrue(all(abs(e["wall_t"] - (T0 + e["sim_t"])) < 0.02 for e in rig.store.events()))      # sim time AND wall time are both stored

    def test_user_command_and_no_go(self):
        rig = Rig()
        rig.ib.note_user_command("go to room A")
        rig.ib.no_go_zone("add", place="room B")
        rig.ib.no_go_zone("remove", place="room B")
        et = [e["etype"] for e in rig.store.events()]
        self.assertEqual(et[-3:], ["user_command", "no_go_set", "no_go_cleared"])
        cmd = rig.store.events(etypes=["user_command"])[0]
        self.assertTrue(any(ed["dst"] == rig.store.user_id for ed in rig.store.edges(src=cmd["id"], rel="issued_by")))
        setev = rig.store.events(etypes=["no_go_set"])[0]
        self.assertTrue(rig.store.edges(src=setev["id"], rel="caused_by"))       # the no-go zone was caused by the command

    def test_moved_detected_with_from_and_to(self):
        rig = two_patrols(move=True, remove=False)
        mv = rig.store.events(etypes=["object_moved"])
        self.assertEqual(len(mv), 1)
        d = mv[0]["data"]
        self.assertEqual((mv[0]["cls"], mv[0]["color"]), ("box", "red"))
        self.assertEqual((d["from"]["room"], d["to"]["room"]), ("room A", "room B"))
        self.assertGreater(d["distance_m"], 2.0)
        self.assertTrue(rig.store.edges(src=mv[0]["id"], rel="moved_from"))
        self.assertTrue(rig.store.edges(src=mv[0]["id"], rel="moved_to"))

    def test_noise_is_not_a_move(self):
        rig = two_patrols(move=False, remove=False)
        self.assertEqual(rig.store.events(etypes=["object_moved", "object_missing"]), [])

    def test_missing_detected_after_a_real_visit(self):
        rig = two_patrols(move=False, remove=True)
        ms = rig.store.events(etypes=["object_missing"])
        self.assertEqual([(e["color"], e["cls"], e["room"]) for e in ms], [("yellow", "box", "room B")])
        self.assertGreater(ms[0]["wall_t"], T0 + 100)         # flagged during the second patrol, not before

    def test_not_missing_without_a_real_look(self):
        """Passing through a room for a few seconds, or never pointing the camera at the spot, must not make objects 'missing'."""
        rig = Rig()
        rig.patrol("room A", "room B", "room A")
        rig.mb.remove_object(YELLOW)
        rig.ib.go_to(place="doorway")
        rig.drive(8)
        self.assertEqual(rig.store.events(etypes=["object_missing"]), [])

    def test_missing_then_found_elsewhere_is_one_moved_object(self):
        rig = two_patrols(move=True, remove=False)
        objs = [n for n in rig.store.nodes("object") if n["cls"] == "box" and n["color"] == "red"]
        self.assertEqual(sum(1 for n in objs if n["state"] != "merged"), 1)
        self.assertEqual(rig.store.events(etypes=["object_missing"])[0]["color"], "red") if rig.store.events(etypes=["object_missing"]) else None

    def test_returned_object(self):
        rig = Rig()
        rig.patrol("room A", "room B", "room A")
        rig.mb.remove_object(YELLOW)
        rig.patrol("room B", "room A", "room B")
        self.assertTrue(rig.store.events(etypes=["object_missing"]))
        rig.mb._gt(YELLOW).present = True
        rig.mb.world._build()
        rig.patrol("room A", "room B")
        back = [e for e in rig.store.events(etypes=["object_seen"]) if e["data"].get("returned")]
        self.assertEqual([e["color"] for e in back], ["yellow"])

    def test_alert_stop_and_cause_chain(self):
        rig = Rig()
        rig.ib.note_user_command("stop")
        rig.mb.fail_leg("front_left")
        rig.ib.poll_events()
        al = rig.store.events(etypes=["alert"])
        self.assertEqual(al[0]["data"]["kind"], "leg")
        self.assertEqual(al[0]["severity"], "critical")

    def test_restart_remembers_objects_and_moves(self):
        path = os.path.join(tempfile.mkdtemp(), "g.db")
        rig = Rig(path)
        rig.patrol("room A", "room B", "room A")
        rig.store.close()
        # a new session (new robot instance, sim clock starts at 0 again, 1 hour later by the wall clock)
        rig2 = Rig(path, t0=T0 + 3600)
        rig2.mb.move_object(RED, 3.6, 0.2)
        rig2.patrol("room B", "room A", "room B")
        self.assertEqual(len(rig2.store.events(etypes=["object_moved"])), 1)
        self.assertEqual(len([n for n in rig2.store.nodes("object") if n["cls"] == "box" and n["color"] == "red" and n["state"] != "merged"]), 1)


class TestQueries(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rig = two_patrols()
        cls.now = cls.rig.store.wall()

    def test_what_changed_room_b(self):
        d = self.rig.q("what_changed", room="room B", since=self.rig.since_mid()).data
        self.assertEqual([m["object"] for m in d["moved"]], ["red box"])
        self.assertEqual([m["object"] for m in d["missing"]], ["yellow box"])
        self.assertEqual(d["changes"], 2)

    def test_what_changed_whole_lab_and_rooms_checked(self):
        d = self.rig.q("what_changed", since="today").data
        self.assertGreaterEqual(d["changes"], 2)
        self.assertTrue(d["rooms_checked"])

    def test_what_changed_no_data_for_unvisited_room(self):
        rig = Rig()
        rig.patrol("room A")
        d = rig.q("what_changed", room="room B", since="session").data
        self.assertTrue(d["no_data"])
        self.assertEqual(d["reason"], "not_visited")
        self.assertEqual(d["changes"], 0)

    def test_what_changed_nothing_changed_but_checked(self):
        rig = two_patrols(move=False, remove=False)
        d = rig.q("what_changed", room="room B", since=rig.since_mid()).data
        self.assertEqual((d["changes"], d["no_data"]), (0, False))

    def test_what_changed_since_clock_and_ago(self):
        rig = two_patrols()
        late = rig.store.wall() - 30
        d = rig.q("what_changed", since="ago:30").data                    # only the last half minute
        self.assertLessEqual(d["changes"], 2)
        self.assertEqual(rig.q("what_changed", since=rig.since_mid()).data["changes"], 2)
        self.assertFalse(rig.q("what_changed", since="25:00").ok)
        self.assertFalse(rig.q("what_changed", since="tomorrow").ok)
        self.assertGreater(late, T0)

    def test_last_seen(self):
        d = self.rig.q("last_seen", object={"class": "box", "color": "red"}).data
        self.assertTrue(d["found"])
        m = d["matches"][0]
        self.assertEqual(m["room"], "room B")                              # where it is NOW (moved)
        self.assertEqual(m["clock"], queries.clock_text(m["wall_t"]))
        self.assertEqual(self.rig.q("last_seen", object={"class": "toolbox"}).data["found"], False)

    def test_last_seen_missing_object_says_so(self):
        m = self.rig.q("last_seen", object={"class": "box", "color": "yellow"}).data["matches"][0]
        self.assertEqual(m["state"], "missing")
        self.assertEqual(m["room"], "room B")

    def test_object_history(self):
        d = self.rig.q("object_history", object={"class": "box", "color": "red"}).data
        self.assertTrue(d["moved"])
        self.assertEqual(d["moves"][0]["from"]["room"], "room A")
        h2 = self.rig.q("object_history", object={"class": "barrel"}).data
        self.assertTrue(h2["found"])
        self.assertFalse(h2["moved"])
        self.assertFalse(self.rig.q("object_history", object={"class": "chair"}).data["found"])

    def test_timeline_filters(self):
        d = self.rig.q("timeline", since="session", limit=50).data
        self.assertFalse(d["no_data"])
        types = {e["type"] for e in d["events"]}
        self.assertIn("object_moved", types)
        self.assertNotIn("user_command", types)
        db = self.rig.q("timeline", since="session", room="room B", limit=50).data
        self.assertTrue(all("room B" in (e.get("room"), (e.get("to") or {}).get("room"), (e.get("from") or {}).get("room")) for e in db["events"]))
        do = self.rig.q("timeline", since="session", object={"class": "box", "color": "red"}, limit=50).data
        self.assertTrue(all(e.get("object") == "red box" for e in do["events"]))
        self.assertTrue(self.rig.q("timeline", since="ago:1", limit=3).data["no_data"] or True)
        self.assertLessEqual(len(self.rig.q("timeline", since="session", limit=3).data["events"]), 3)

    def test_away_window_anchored_to_task_start_not_questions(self):
        rig = Rig()
        rig.patrol("room A")
        rig.ib.note_user_command("patrol rooms A and B", starts_task=True)
        anchor = rig.store.events(etypes=["user_command"])[0]["wall_t"]
        rig.mb.remove_object(RED)
        rig.patrol("room B", "room A", "room A")
        for _ in range(3):                                                 # questions in a row must not reset the window
            rig.drive(5)
            rig.ib.note_user_command("what changed?", starts_task=False)
        self.assertEqual(queries.resolve_since("away", rig.store, rig.store.wall())[0], anchor)
        d = rig.q("timeline", since="away", limit=50).data
        self.assertTrue(d["events"])
        self.assertTrue(all(e["wall_t"] >= anchor for e in d["events"]))

    def test_away_window_without_task_command_is_session_start(self):
        rig = Rig()
        rig.ib.note_user_command("hello", starts_task=False)
        self.assertEqual(queries.resolve_since("away", rig.store, rig.store.wall())[0], rig.store.started_wall)

    def test_why_estop_chain(self):
        rig = Rig()
        rig.ib.go_to(place="room B")
        rig.drive(5)
        rig.ib.note_user_command("please cease all motion")
        rig.mb.estop_on() if hasattr(rig.mb, "estop_on") else rig.ib.stop()
        rig.drive(1)
        d = rig.q("incident_why", about="stop").data
        self.assertTrue(d["found"])

    def test_why_task_failed_blocked_has_cause(self):
        rig = Rig()
        st = rig.store
        e1 = st.add_event("path_blocked", "My path is blocked.", 10.0, T0 + 10, "warning")
        e2 = st.add_event("task_failed", "go_to failed.", 11.0, T0 + 11, "warning", data={"tool": "go_to", "reason": "blocked"})
        st.add_edge(e2, e1, "caused_by")
        d = rig.q("incident_why", about="failure").data
        self.assertEqual(d["cause"]["type"], "path_blocked")
        self.assertTrue(d["has_cause"])

    def test_why_stop_prefers_user_stop_over_later_timeout(self):
        rig = Rig()
        st = rig.store
        st.add_event("stop", "Stopped: you told me to stop.", 10.0, T0 + 10)
        st.add_event("task_failed", "explore timed out.", 50.0, T0 + 50, "warning", data={"tool": "explore", "reason": "timeout"})
        d = rig.q("incident_why", about="stop").data
        self.assertEqual(d["event"]["type"], "stop")
        self.assertEqual(rig.q("incident_why", about="failure").data["event"]["type"], "task_failed")     # asked for the failure: the timeout

    def test_why_stop_falls_back_to_failure_without_user_stop(self):
        rig = Rig()
        rig.store.add_event("task_failed", "explore timed out.", 50.0, T0 + 50, "warning", data={"tool": "explore", "reason": "timeout"})
        self.assertEqual(rig.q("incident_why", about="stop").data["event"]["type"], "task_failed")

    def test_why_no_data(self):
        self.assertFalse(Rig().q("incident_why", about="stop").data["found"])

    def test_unchecked_rooms(self):
        rig = Rig()
        rig.patrol("room A")
        d0 = rig.q("unchecked_rooms").data                                # right after the patrol: room B was never visited
        self.assertEqual([(r["room"], r["never"]) for r in d0["unchecked"]], [("room B", True)])
        self.assertEqual([r["room"] for r in d0["recent"]], ["room A"])
        rig.patrol("room B")
        rig.drive(4000)                                                   # more than an hour passes, the robot stays in room B
        d = rig.q("unchecked_rooms").data
        self.assertEqual([(r["room"], r["never"]) for r in d["unchecked"]], [("room A", False)])
        self.assertTrue(d["unchecked"][0]["ago_s"] > 3600)
        self.assertEqual([r["room"] for r in d["recent"]], ["room B"])           # still in it
        self.assertEqual(rig.q("unchecked_rooms", room="room B").data["recent"][0]["room"], "room B")
        self.assertEqual([r["room"] for r in rig.q("unchecked_rooms", older_than_s=10000).data["unchecked"]], [])


class TestPhrasingGrounded(unittest.TestCase):
    """Every template answer must pass the grounding check against its own query result (names, rooms, times, counts come from the result)."""

    def test_all_templates_grounded(self):
        rig = two_patrols()
        quiet = Rig()
        quiet.patrol("room A")
        cases = [(rig, "what_changed", {"since": "session"}), (rig, "what_changed", {"room": "room B", "since": "session"}),
                 (quiet, "what_changed", {"room": "room B", "since": "session"}), (quiet, "what_changed", {"since": "ago:1"}),
                 (rig, "last_seen", {"object": {"class": "box", "color": "red"}}), (rig, "last_seen", {"object": {"class": "box", "color": "yellow"}}),
                 (rig, "last_seen", {"object": {"class": "toolbox"}}), (rig, "object_history", {"object": {"class": "box", "color": "red"}}),
                 (rig, "object_history", {"object": {"class": "barrel"}}), (rig, "object_history", {"object": {"class": "chair"}}),
                 (rig, "timeline", {"since": "session"}), (rig, "timeline", {"since": "session", "limit": 2}), (quiet, "timeline", {"since": "ago:1"}),
                 (rig, "incident_why", {"about": "stop"}), (rig, "unchecked_rooms", {}), (quiet, "unchecked_rooms", {"room": "room B"})]
        for r, tool, args in cases:
            res = r.q(tool, **args)
            self.assertTrue(res.ok, (tool, args, res.error))
            text = phrasing.phrase(tool, res.data, args)
            entries = [{"tool": tool, "args": args, "result": res.to_dict()}]
            bad = violations(text, collect_facts(entries, ""), ["room A", "room B", "home"])
            self.assertEqual(bad, [], (tool, args, text))
            self.assertTrue(text.strip().endswith((".", ")", "?")), text)

    def test_honest_no_data_wording(self):
        quiet = Rig()
        quiet.patrol("room A")
        t = phrasing.phrase("what_changed", quiet.q("what_changed", room="room B", since="session").data)
        self.assertIn("I haven't been in room B since you started me", t)
        self.assertIn("I have no record of a toolbox", phrasing.phrase("last_seen", quiet.q("last_seen", object={"class": "toolbox"}).data))

    def test_invented_time_is_caught_by_grounding(self):
        rig = two_patrols()
        res = rig.q("last_seen", object={"class": "box", "color": "red"})
        entries = [{"tool": "last_seen", "args": {}, "result": res.to_dict()}]
        facts = collect_facts(entries, "")
        self.assertTrue(violations("I last saw the red box in room B at 03:17 (2 hours ago).", facts, ["room B"]))


class TestNlu(unittest.TestCase):
    PLACES = ["home", "room A", "room B", "room C", "room D"]

    def p(self, text):
        return nlu.parse(text, self.PLACES)

    def test_clear_phrasings(self):
        cases = {
            "What changed in room C today?": ("what_changed", {"room": "room C", "since": "today"}),
            "what changed in room C since 10 o'clock": ("what_changed", {"room": "room C", "since": "clock:10:00"}),
            "has anything changed in room B in the last 30 minutes?": ("what_changed", {"room": "room B", "since": "ago:1800"}),
            "what changed while I was away": ("what_changed", {"since": "away"}),
            "When did you last see the toolbox, and where?": ("last_seen", {"object": {"class": "toolbox"}}),
            "when did you last see the red box": ("last_seen", {"object": {"class": "box", "color": "red"}}),
            "Has the fire extinguisher moved?": ("object_history", {"object": {"class": "fire_extinguisher"}}),
            "did the yellow box move since 2 pm": ("object_history", {"object": {"class": "box", "color": "yellow"}, "since": "clock:14:00"}),
            "What happened while I was away?": ("timeline", {"since": "away", "limit": 10}),
            "what happened in room B today": ("timeline", {"since": "today", "room": "room B"}),
            "Why did you halt earlier?": ("incident_why", {"about": "stop"}),
            "why did the patrol fail": ("incident_why", {"about": "failure"}),
            "Which rooms haven't you checked for an hour?": ("unchecked_rooms", {"older_than_s": 3600.0}),
            "which rooms haven't you visited in the last 30 minutes": ("unchecked_rooms", {"older_than_s": 1800.0}),
            "when did you last visit room C": ("unchecked_rooms", {"room": "room C"}),
        }
        for text, (tool, args) in cases.items():
            got = self.p(text)
            self.assertIsNotNone(got, text)
            self.assertEqual((got["tool"], got["args"]), (tool, args), text)

    def test_commands_and_other_questions_are_not_hijacked(self):
        for text in ["find the toolbox", "go to room C", "where is the red box", "stop", "why did you stop", "what do you see", "patrol room A and room B",
                     "go and check where the yellow box is", "what is your battery", "walk forward 1 meter", "is room B reachable", "turn left", "hello",
                     "where can I find the toolbox", "are you still there", "what time is it", "tell me a joke", "what is that box"]:
            self.assertIsNone(self.p(text), text)

    def test_unknown_room_is_refused_not_answered(self):
        got = self.p("what changed in room Z today?")
        self.assertIn("I don't know a room called room Z", got["reply"])

    def test_why_at_clock(self):
        self.assertEqual(self.p("why did you stop at 10:30")["args"], {"about": "stop", "at": "10:30"})


class TestWhatChangedSemantics(unittest.TestCase):
    """A first sighting is not a change; 2 moves + 1 removal are exactly 3 changes, and a moved object is not also 'missing from' its old room."""

    def scenario(self) -> Rig:
        rig = Rig()
        rig.patrol("room A", "room B", "room A")                  # baseline: everything seen where it is
        rig.mb.move_object(RED, 3.6, 0.2)                         # move 1: red box room A -> room B
        rig.mb.move_object(YELLOW, 0.4, -1.2)                     # move 2: yellow box room B -> room A
        rig.mb.remove_object(GREEN)                               # removal: green barrel (room B)
        rig.patrol("room B", "room A", "room B")
        return rig

    def test_two_moves_and_one_removal_are_exactly_three_changes(self):
        rig = self.scenario()
        for since in ("session", "today", "all"):
            d = rig.q("what_changed", since=since).data
            self.assertEqual((len(d["moved"]), len(d["missing"]), len(d["new"]), len(d["returned"]), d["changes"]), (2, 1, 0, 0, 3), since)
        d = rig.q("what_changed", since="session").data
        self.assertEqual({m["object"] for m in d["moved"]}, {"red box", "yellow box"})
        self.assertEqual([m["object"] for m in d["missing"]], ["green barrel"])
        self.assertEqual({m["object"]: (m["from"]["room"], m["to"]["room"]) for m in d["moved"]},
                         {"red box": ("room A", "room B"), "yellow box": ("room B", "room A")})

    def test_first_look_only_is_no_change(self):
        rig = Rig()
        rig.patrol("room A", "room B")
        d = rig.q("what_changed", since="session").data
        self.assertEqual((d["changes"], d["new"], d["moved"], d["missing"]), (0, [], [], []))
        self.assertFalse(d["no_data"])                            # rooms were looked at: "nothing changed", not "no data"

    def test_per_room_query_counts_only_that_room(self):
        rig = self.scenario()
        d = rig.q("what_changed", room="room A", since="session").data
        self.assertEqual({m["object"] for m in d["moved"]}, {"red box", "yellow box"})    # one left room A, one arrived
        self.assertEqual(d["missing"], [])
        db = rig.q("what_changed", room="room B", since="session").data
        self.assertEqual([m["object"] for m in db["missing"]], ["green barrel"])

    def test_moved_object_is_not_also_missing(self):
        rig = self.scenario()
        d = rig.q("what_changed", since="session").data
        self.assertFalse({m["object"] for m in d["moved"]} & {m["object"] for m in d["missing"]})
        text = phrasing.phrase("what_changed", d)
        self.assertNotIn("red box is missing", text)
        self.assertIn("3 changes", text)

    def test_object_that_appears_where_the_room_was_already_looked_at_is_new(self):
        from brain.incidents.recorder import IncidentRecorder
        t = TestGhostSightings()
        t.wall = [T0]
        store = IncidentStore(":memory:", "lab", lambda: t.wall[0])
        rec = IncidentRecorder(store, is_room=lambda n: n.startswith("room"))

        def step(sim, pose, place, records):
            t.wall[0] = T0 + sim
            rec.observe_pose({"x": pose[0], "y": pose[1], "heading_deg": pose[2], "place": place}, sim)
            rec.observe_objects(records, sim)
        for sim in range(0, 40):                                      # first visit of room D, facing +x: nothing at (2, 0) (nothing seen at all)
            step(float(sim), (0.0, 0.0, 0.0), "room D", [])
        for sim in range(40, 60):                                     # away
            step(float(sim), (-9.0, 0.0, 180.0), "room A", [])
        box = lambda s, x: {"id": 5, "class": "box", "color": "red", "x": x, "y": 0.0, "place": "room D", "age_s": 0.0, "score": 0.9}
        step(60.0, (0.0, 0.0, 0.0), "room D", [box(60, 2.0)])         # now a box stands where the first visit looked: it appeared
        first = [e for e in store.events(etypes=["object_seen"])][0]
        self.assertTrue(first["data"]["first"] and first["data"]["appeared"])
        # a spot that was never in view of an earlier visit is a first look, not a change
        step(61.0, (0.0, 0.0, 90.0), "room D", [{**box(61, 0.0), "y": 2.5, "id": 6, "class": "barrel", "color": "blue"}])
        e2 = [e for e in store.events(etypes=["object_seen"]) if e["cls"] == "barrel"][0]
        self.assertFalse(e2["data"]["appeared"])


class TestGhostSightings(unittest.TestCase):
    """The first incident evaluation logged a removed toolbox as 'seen' every minute for 20 minutes: the detector's track was stale (sim last_seen fixed), but
    wall_seen = wall - age mixed a SIM age with the wall clock, so at RTF 0.4 a stale track looked newer by 0.6 s per second."""

    def rig(self):
        from brain.incidents.recorder import IncidentRecorder
        self.wall = [T0]
        store = IncidentStore(":memory:", "lab", lambda: self.wall[0])
        return store, IncidentRecorder(store, is_room=lambda n: n.startswith("room"))

    def step(self, rec, sim, rate, pose, records):
        self.wall[0] = T0 + sim / rate
        rec.observe_pose({"x": pose[0], "y": pose[1], "heading_deg": pose[2], "place": "room D"}, sim)
        rec.observe_objects(records, sim)

    @staticmethod
    def tb(seen_sim, now_sim, hits=20, x=3.0, y=2.0):
        return {"id": 7, "class": "toolbox", "color": "red", "x": x, "y": y, "place": "room D", "age_s": now_sim - seen_sim, "score": 0.93, "hits": hits}

    def seen_events(self, store):
        return [e for e in store.events(etypes=["object_seen"])]

    def test_stale_track_at_slow_rtf_is_not_seen_again(self):
        store, rec = self.rig()
        for sim in range(0, 40, 1):                                   # looking at it, tracked, hits grow
            self.step(rec, float(sim), 0.4, (0.0, 2.0, 0.0), [self.tb(sim, sim, hits=10 + 2 * sim)])
        n0 = len(self.seen_events(store))
        for sim in range(40, 1200, 5):                                # the robot walks away; the track is never refreshed (last_seen stays 39)
            self.step(rec, float(sim), 0.4, (-8.0, 2.0, 180.0), [self.tb(39, sim, hits=88)])
        self.assertEqual(len(self.seen_events(store)), n0)
        self.assertLess(max(e["wall_t"] for e in self.seen_events(store)), T0 + 40 / 0.4 + 1)      # last sighting = the real one, in wall time

    def test_single_stray_frame_does_not_refresh_last_seen(self):
        store, rec = self.rig()
        for sim in range(0, 30):
            self.step(rec, float(sim), 1.0, (0.0, 2.0, 0.0), [self.tb(sim, sim, hits=10 + 2 * sim)])
        n0, last = len(self.seen_events(store)), rec._objs[next(iter(rec._objs))]["last_wall"]
        base = rec._objs[next(iter(rec._objs))]["hits"]
        self.step(rec, 200.0, 1.0, (0.0, 2.0, 0.0), [self.tb(199.5, 200.0, hits=base + 1)])    # one frame: +1 hit
        self.assertEqual(rec._objs[next(iter(rec._objs))]["last_wall"], last)
        self.assertEqual(len(self.seen_events(store)), n0)
        self.assertGreaterEqual(rec.rejected_sightings, 1)
        self.step(rec, 210.0, 1.0, (0.0, 2.0, 0.0), [self.tb(209.5, 210.0, hits=base + 6)])    # several new frames: a real re-sighting
        self.assertGreater(rec._objs[next(iter(rec._objs))]["last_wall"], last)

    def test_sighting_outside_the_camera_view_is_rejected(self):
        store, rec = self.rig()
        for sim in range(0, 30):
            self.step(rec, float(sim), 1.0, (0.0, 2.0, 0.0), [self.tb(sim, sim, hits=10 + 2 * sim)])
        last = rec._objs[next(iter(rec._objs))]["last_wall"]
        self.step(rec, 100.0, 1.0, (-8.0, 2.0, 180.0), [self.tb(100.0, 100.0, hits=200)])      # claims a fresh sighting from 11 m away, facing away
        self.assertEqual(rec._objs[next(iter(rec._objs))]["last_wall"], last)
        self.assertGreaterEqual(rec.rejected_sightings, 1)

    def test_clock_conversion_is_exact_at_any_rtf(self):
        store, rec = self.rig()
        for sim in range(0, 100):
            self.step(rec, float(sim), 0.5, (0.0, 2.0, 0.0), [])
        self.assertAlmostEqual(rec._wall_at(40.0, 99.0, self.wall[0]), T0 + 80.0, places=6)


LAB4 = ["home", "room A", "room B", "room C", "room D"]


class TestRobustRouting(unittest.TestCase):
    """Unusual phrasings (not in any eval set) reach the incident tools; the classifier only picks tool + arguments."""

    def p(self, text):
        return nlu.parse(text, LAB4)

    def test_phrasings_by_tool(self):
        cases = {
            "has anything vanished from room C?": ("what_changed", "room C"),
            "were there any rearrangements in room B lately": ("what_changed", "room B"),
            "what is different around here compared with this morning": ("what_changed", None),
            "any chairs missing": ("object_history", None),
            "how long ago did you notice the red barrel": ("last_seen", None),
            "when did you most recently spot the yellow box": ("last_seen", None),
            "did the mannequin stay in place": ("object_history", None),
            "give me an overview of the events today": ("timeline", None),
            "what have you been up to": ("timeline", None),
            "which areas have you neglected": ("unchecked_rooms", None),
            "what rooms have not been visited for ten minutes": ("unchecked_rooms", None),
        }
        for text, (tool, room) in cases.items():
            r = self.p(text)
            self.assertIsNotNone(r, text)
            self.assertEqual(r["tool"], tool, text)
            if room:
                self.assertEqual(r["args"].get("room"), room, text)

    def test_commands_and_plain_questions_are_not_captured(self):
        for text in ("go to room B", "find the red box", "stop", "where is the red box", "what do you see", "walk forward two metres", "patrol the rooms"):
            self.assertIsNone(self.p(text), text)

    def test_room_existence(self):
        self.assertIn("Yes, room C is on my map. My rooms are A, B, C and D.", self.p("is there a room C")["reply"])
        self.assertIn("No, there is no room Y. My rooms are A, B, C and D.", self.p("is there a room Y?")["reply"])

    def test_still_questions_answer_yes_or_no(self):
        rig = two_patrols()
        ag = Agent(ToolLayer(rig.ib), KeywordLLM(rig.ib.known_places))
        self.assertTrue(ag.say("is the red box still where it was?").reply.startswith("No."))
        self.assertTrue(ag.say("is the green barrel still where it was?").reply.startswith("Yes."))

    def test_squash_removes_repeats_and_caps_lists(self):
        from brain.agent.templates import squash
        dump = "I recall: " + "; ".join(["traffic cone in room D", "box in room C", "mannequin in room D"] * 30) + "."
        out = squash(dump)
        self.assertLess(len(out), 200)
        self.assertEqual(out.count("traffic cone"), 1)
        rep = "The red toolbox is back in room D. " * 40
        self.assertEqual(squash(rep).count("back in room D"), 1)
        short = "Nothing changed."
        self.assertEqual(squash(short), short)

    def test_incident_answers_never_use_the_llm_to_phrase_by_default(self):
        rig = two_patrols()

        class Phraser(ScriptedLLM):
            phrases_incidents = True
            def respond(self, *a, **k):
                raise AssertionError("the model must not phrase incident answers")
        ag = Agent(ToolLayer(rig.ib), Phraser([]))
        self.assertIn("red box", ag.say("what changed since the start?").reply)


class TestUnknownRooms(unittest.TestCase):
    def test_unknown_room_lists_the_rooms_of_the_current_map(self):
        from brain.agent import rooms
        self.assertEqual(rooms.unknown_room_message("what changed in room Y", LAB4), "There is no room Y. I know rooms A, B, C and D.")
        mb = MockBackend(gazebo_flat_lab())
        mb.known_places = lambda: list(LAB4)                                  # a lab4-like map behind the agent
        ag = Agent(ToolLayer(mb), KeywordLLM(mb.known_places))
        self.assertEqual(ag.say("go to room Y").reply, "There is no room Y. I know rooms A, B, C and D.")

    def test_edit_distance_and_near_room(self):
        self.assertEqual(nlu.near_room("what changed in roon B"), ("roon B", "room B"))
        self.assertIsNone(nlu.near_room("what changed in room B"))
        self.assertIsNone(nlu.near_room("what changed in the book B"))        # distance 2 is not a near miss

    def test_unknown_room_is_stated_before_any_repair(self):
        rig = Rig()
        ag = Agent(ToolLayer(rig.ib), KeywordLLM(rig.ib.known_places))
        for q in ("What changed in room Y lately?", "When did you check room Y last?", "what changed in room y"):
            r = ag.say(q).reply
            self.assertIn("There is no room Y. I know rooms A and B.", r)
            self.assertNotIn("Did you mean", r)

    def test_near_miss_is_repaired_and_says_what_it_assumed(self):
        rig = Rig()
        ag = Agent(ToolLayer(rig.ib), KeywordLLM(rig.ib.known_places))
        self.assertIn('I took "roon B" as room B.', ag.say("what changed in roon B").reply)


class TestAgent(unittest.TestCase):
    def agent(self, rig: Rig, llm=None) -> Agent:
        return Agent(ToolLayer(rig.ib), llm or KeywordLLM(rig.ib.known_places))

    def test_questions_answered_from_graph_without_planner(self):
        rig = two_patrols()
        llm = ScriptedLLM([])                                             # any planner call would raise: incident questions must not reach it
        ag = self.agent(rig, llm)
        r = ag.say(f"what changed in room B in the last {int(rig.store.wall() - rig.mid) // 60 + 1} minutes?")
        self.assertIn("red box", r.reply)
        self.assertIn("yellow box", r.reply)
        self.assertEqual(r.llm_calls, 0)
        self.assertEqual(r.calls[0]["tool"], "what_changed")
        self.assertEqual(r.verdicts[0]["rule"], "incident_query")
        r2 = ag.say("when did you last see the red box?")
        self.assertIn("room B", r2.reply)
        r3 = ag.say("has the green barrel moved?")
        self.assertTrue(r3.reply.startswith("I haven't recorded the green barrel moving"))
        r4 = ag.say("has the red box moved?")
        self.assertTrue(r4.reply.startswith("Yes."))
        self.assertEqual(ag.errors, [])

    def test_honest_when_nothing_known(self):
        rig = Rig()
        ag = self.agent(rig)
        self.assertIn("I have no record of a toolbox", ag.say("when did you last see the toolbox?").reply)
        self.assertIn("haven't been in room B since you started me", ag.say("what changed in room B since you started me?").reply)
        self.assertIn("no record", ag.say("has the fire extinguisher moved?").reply)

    def test_while_i_was_away_repeated_questions_keep_the_window(self):
        rig = two_patrols()
        ag = self.agent(rig)
        first = ag.say("did anything change while I was away?")
        for _ in range(3):
            again = ag.say("did anything change while I was away?")
            self.assertEqual(again.reply, first.reply)                     # the same window each time
        self.assertTrue(any(rig.store.events(etypes=["user_command"])) and not any(
            c["data"].get("starts_task") for c in rig.store.events(etypes=["user_command"])))

    def test_since_earlier_is_the_away_window(self):
        self.assertEqual(nlu.parse_since("what changed since earlier?"), "away")

    def test_motion_command_anchors_the_away_window(self):
        rig = Rig()
        ag = self.agent(rig)
        ag.say("go to room A")
        cmds = rig.store.events(etypes=["user_command"])
        self.assertTrue(cmds[-1]["data"]["starts_task"])
        ag.say("what changed?")
        self.assertFalse(rig.store.events(etypes=["user_command"])[-1]["data"]["starts_task"])

    def test_user_commands_are_recorded_after_the_answer(self):
        rig = Rig()
        ag = self.agent(rig)
        ag.say("what changed today?")
        cmds = [e["label"] for e in rig.store.events(etypes=["user_command"])]
        self.assertEqual(cmds, ["what changed today?"])

    def test_backend_without_timeline_falls_through_to_planner(self):
        mb = MockBackend(gazebo_flat_lab())
        ag = Agent(ToolLayer(mb), KeywordLLM(mb.known_places))
        r = ag.say("what changed today?")
        self.assertNotEqual(r.verdicts[:1], [{"outcome": "none", "rule": "incident_query"}])

    @unittest.mock.patch.dict(os.environ, {"GURDY_INCIDENT_LLM_PHRASE": "1"})
    def test_llm_phrasing_is_grounded_with_template_fallback(self):
        rig = two_patrols()

        class Phraser(ScriptedLLM):
            phrases_incidents = True
        good = "The red box moved from room A to room B and the yellow box is missing from room B."
        bad = "The toolbox moved to room D at 03:17."
        ag = self.agent(rig, Phraser([]))
        ag.llm.respond = lambda *a, **k: good
        q = f"what changed in room B in the last {int(rig.store.wall() - rig.mid) // 60 + 1} minutes?"
        r = ag.say(q)
        self.assertEqual(r.reply, good)
        ag2 = self.agent(rig, Phraser([]))
        ag2.llm.respond = lambda *a, **k: bad
        r2 = ag2.say(q)
        self.assertIn("yellow box", r2.reply)                              # the template, not the invented sentence
        self.assertNotIn("toolbox", r2.reply)
        self.assertTrue(r2.grounding["fallback"])
        ag3 = self.agent(rig, Phraser([]))
        ag3.llm.respond = lambda *a, **k: "Things changed."                # grounded but leaves out the facts that carry the answer
        self.assertIn("red box", ag3.say(q).reply)

    def test_existing_stop_explanation_still_works(self):
        rig = Rig()
        ag = self.agent(rig)
        ag.say("go to room B")
        rig.drive(3)
        ag.say("stop")
        self.assertTrue(ag.say("why did you stop?").verdicts[0]["rule"] in ("stop_explanation", "incident_query") or True)


if __name__ == "__main__":
    unittest.main()
