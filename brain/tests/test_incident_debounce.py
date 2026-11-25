"""Phase 6e: graph noise debounce. `object_missing` needs a clear, consecutive look at the expected spot; 'back' needs two separated re-sightings."""
import unittest

from brain.incidents.recorder import IncidentRecorder
from brain.incidents.store import IncidentStore

T0 = 1_000_000.0


class Rig:
    def __init__(self):
        self.t = 0.0
        self.store = IncidentStore(":memory:", "lab", lambda: T0 + self.t)
        self.rec = IncidentRecorder(self.store)
        self.n = 0

    def pose(self, x, y, hdg, place="room A"):
        self.rec.observe_pose({"x": x, "y": y, "heading_deg": hdg, "place": place}, self.t)

    def box(self, hits, x=1.0, y=0.0, age=0.0):
        self.rec.observe_objects([{"id": 7, "class": "box", "color": "red", "x": x, "y": y, "place": "room A", "age_s": age, "hits": hits, "score": 0.9}], self.t)

    def run(self, seconds, hdg=0.0, place="room A", step=0.5, x=0.0):
        end = self.t + seconds
        while self.t < end:
            self.t += step
            self.pose(x, 0.0, hdg, place)

    def seen_box_in_room_A(self):
        """Baseline: the box at (1,0) is seen in room A, then the robot leaves for room B."""
        self.run(2.0)
        self.box(10)
        self.run(2.0)
        self.run(25.0, place="room B", x=8.0, hdg=180.0)

    def events(self, *t):
        return self.store.events(etypes=list(t))


class TestMissingDebounce(unittest.TestCase):
    def test_glimpse_from_the_doorway_is_not_missing(self):
        r = Rig()
        r.seen_box_in_room_A()
        r.run(1.5, hdg=0.0)                                   # 3 samples looking at the spot ...
        r.run(25.0, hdg=180.0, x=-3.0)                        # ... then the robot faces away for a 20 s visit
        self.assertEqual(r.events("object_missing"), [])

    def test_clear_look_makes_it_missing(self):
        r = Rig()
        r.seen_box_in_room_A()
        r.run(25.0, hdg=0.0)                                  # 25 s looking straight at the spot, nothing detected
        self.assertEqual([e["cls"] for e in r.events("object_missing")], ["box"])

    def test_interrupted_look_restarts_the_run(self):
        r = Rig()
        r.seen_box_in_room_A()
        for _ in range(4):                                    # 2 s looking, 1 s away, repeated: never 4 s in a row
            r.run(2.0, hdg=0.0)
            r.run(1.0, hdg=150.0)
        r.run(6.0, hdg=150.0)
        self.assertEqual(r.events("object_missing"), [])

    def test_seen_during_the_visit_is_never_missing(self):
        r = Rig()
        r.seen_box_in_room_A()
        r.run(10.0, hdg=0.0)
        r.box(20)
        r.run(20.0, hdg=0.0)
        self.assertEqual(r.events("object_missing"), [])


class TestReturnDebounce(unittest.TestCase):
    def missing_rig(self):
        r = Rig()
        r.seen_box_in_room_A()
        r.run(25.0, hdg=0.0)
        self.assertEqual(len(r.events("object_missing")), 1)
        return r

    def returned(self, r):
        return [e for e in r.events("object_seen") if e["data"].get("returned")]

    def test_one_frame_flicker_is_not_back(self):
        r = self.missing_rig()
        r.box(40)
        r.run(30.0, hdg=0.0)                                  # never detected again
        self.assertEqual(self.returned(r), [])
        self.assertEqual(r.rec._objs[next(iter(r.rec._objs))]["state"], "missing")

    def test_two_separated_resightings_are_back(self):
        r = self.missing_rig()
        r.box(40)
        r.run(1.5, hdg=0.0)
        r.box(60)
        r.run(1.0, hdg=0.0)
        self.assertEqual(len(self.returned(r)), 1)
        self.assertEqual(r.rec._objs[next(iter(r.rec._objs))]["state"], "present")

    def test_resightings_too_far_apart_start_over(self):
        r = self.missing_rig()
        r.box(40)
        r.run(20.0, hdg=0.0)
        r.box(60)                                             # first one expired: this one counts as a new first
        self.assertEqual(self.returned(r), [])


if __name__ == "__main__":
    unittest.main()
