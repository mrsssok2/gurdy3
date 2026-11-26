"""The lab4 mock world (public demo only): geometry equals the lab4 generator, rooms reachable through the doorways, flat_lab mocks untouched."""
import importlib.util
import json
import sys
import unittest
from pathlib import Path

from brain.mock import MockBackend, flat_lab, gazebo_flat_lab, gazebo_lab4
from brain.mock.lab_world import LAB4_GT

ROOT = Path(__file__).resolve().parents[2]


class Lab4World(unittest.TestCase):
    def test_rooms_objects_and_map(self):
        w = gazebo_lab4()
        self.assertEqual(w.map_id, "lab4")
        self.assertEqual(sorted(w.rooms), ["room A", "room B", "room C", "room D"])
        gt = json.loads(LAB4_GT.read_text())["objects"]
        self.assertEqual(len(w.objects), 10)
        for o, g in zip(w.objects, gt):
            self.assertEqual((o.cls, o.color, o.x, o.y), (g["class"], g["color"], g["x"], g["y"]))
            self.assertEqual(w.place_at(o.x, o.y), g["room"], g["name"])
        self.assertEqual({o.cls for o in w.objects}, {"box", "barrel", "traffic_cone", "fire_extinguisher", "toolbox", "mannequin"})

    def test_walls_match_the_generator(self):
        sys.path[:0] = [str(ROOT), str(ROOT / "gurdy_core"), str(ROOT / "scripts")]
        try:
            spec = importlib.util.spec_from_file_location("gen_world_lab4", ROOT / "scripts" / "gen_world_lab4.py")
            gen = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(gen)
        except ImportError as e:
            self.skipTest(f"generator needs the vision package: {e}")
        want = sorted((round(c[0] - s[0] / 2, 3), round(c[1] - s[1] / 2, 3), round(c[0] + s[0] / 2, 3), round(c[1] + s[1] / 2, 3)) for _, c, s in gen.walls())
        got = sorted((round(r.x0, 3), round(r.y0, 3), round(r.x1, 3), round(r.y1, 3)) for r in gazebo_lab4().walls)
        self.assertEqual(got, want)

    def test_ring_of_doorways(self):
        w = gazebo_lab4()
        for room in ("room B", "room C", "room D"):
            self.assertIsNotNone(w.plan((0.0, 0.0), w.places[room]), room)
        b = MockBackend(w)
        self.assertTrue(b.go_to(place="room D").ok)

    def test_other_mock_worlds_unchanged(self):
        self.assertEqual(sorted(flat_lab().rooms), ["room A", "room B"])
        self.assertEqual(sorted(gazebo_flat_lab().rooms), ["room A", "room B"])


if __name__ == "__main__":
    unittest.main()
