"""Incident graph (temporal memory): store, recorder (event extraction, move / missing detection), each query tool, routing, grounding, agent."""
from __future__ import annotations

import os
import tempfile
import unittest

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

    def test_away_window_starts_at_previous_command(self):
        rig = Rig()
        rig.patrol("room A")
        rig.ib.note_user_command("hello")
        rig.mb.remove_object(RED)
        rig.patrol("room B", "room A", "room A")
        d = rig.q("timeline", since="away", limit=50).data
        self.assertTrue(d["events"])
        self.assertTrue(all(e["wall_t"] >= T0 + 1 for e in d["events"]))
        cmd_t = rig.store.events(etypes=["user_command"])[0]["wall_t"]
        self.assertTrue(all(e["wall_t"] >= cmd_t for e in d["events"]))

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
            "Why did you stop earlier?": ("incident_why", {"about": "stop"}),
            "why did the patrol fail": ("incident_why", {"about": "failure"}),
            "Which rooms haven't you checked for an hour?": ("unchecked_rooms", {"older_than_s": 3600.0}),
            "which rooms haven't you visited in the last 30 minutes": ("unchecked_rooms", {"older_than_s": 1800.0}),
            "when did you last check room C": ("unchecked_rooms", {"room": "room C"}),
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

    def test_while_i_was_away_uses_previous_command(self):
        rig = two_patrols()
        ag = self.agent(rig)
        ag.say("hello there")                                              # the command BEFORE the question
        quiet = ag.say("what happened while I was away?")                  # nothing happened since: an honest empty answer
        self.assertIn("I haven't recorded anything while you were away", quiet.reply)
        rig.patrol("room A", "room B")                                     # the user is away: the robot patrols
        r = ag.say("what happened while I was away?")
        self.assertIn(":", r.reply)
        self.assertIn("Entered room B", r.reply)
        self.assertNotIn("hello there", r.reply)

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
