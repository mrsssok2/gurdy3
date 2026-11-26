"""The web/demo mock world must be the Gazebo flat_lab generated from its sources; the evaluation mock must stay what it was."""
import importlib.util
import re
from pathlib import Path

import yaml

from brain.agent.graph import Agent
from brain.agent.llm import KeywordLLM
from brain.mock import MockBackend, flat_lab, gazebo_flat_lab
from brain.tools import ToolLayer

ROOT = Path(__file__).resolve().parents[2]


def _gen():
    spec = importlib.util.spec_from_file_location("g", ROOT / "scripts/gen_world_flat_lab.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_objects_are_exactly_the_sdf_objects():
    g, w = _gen(), gazebo_flat_lab()
    names = {n: xy for n, _, xy, _, _ in g.OBJECTS}
    sdf = (ROOT / "ros2_ws/src/gurdy_gazebo/worlds/flat_lab.sdf").read_text()
    for n, (x, y) in names.items():                                   # the committed SDF has the same models at the same poses
        m = re.search(rf'<model name="{n}">\s*<static>true</static>\s*<pose>([-\d.]+) ([-\d.]+)', sdf)
        assert m and (float(m.group(1)), float(m.group(2))) == (x, y), n
    got = {(o.color, o.cls, o.x, o.y) for o in w.objects}
    assert got == {("red", "box", 1.0, 1.6), ("blue", "box", 1.8, -2.2), ("green", "barrel", 4.5, 1.6), ("yellow", "box", 4.6, -1.5)}
    assert len(w.obstacles) == 1 and w.obstacles[0].contains(0.9, -1.4)            # the table, an obstacle and no object
    assert not any(o.x == 0.9 for o in w.objects)


def test_rooms_doorway_places_from_the_yaml():
    w = gazebo_flat_lab()
    spec = yaml.safe_load((ROOT / "maps/flat_lab.places.yaml").read_text())["places"]
    assert {n: [r.x0, r.y0, r.x1, r.y1] for n, r in w.rooms.items()} == {n: v["rect"] for n, v in spec.items() if "rect" in v}
    assert w.places["doorway"] == (2.5, 0.5) and w.places["home"] == (0.0, 0.0)
    assert w.map_id == "flat_lab"
    # the doorway is a gap in the divider: free at x=2.5, y in [0, 1]; wall elsewhere
    assert w.is_free(2.5, 0.5) and not w.is_free(2.5, 1.5) and not w.is_free(2.5, -1.5)


def test_object_room_membership_matches_the_task_text():
    w = gazebo_flat_lab()
    where = {(o.color): w.place_at(o.x, o.y) for o in w.objects}
    assert where == {"red": "room A", "blue": "room A", "green": "room B", "yellow": "room B"}


def test_robot_can_drive_between_rooms_and_find_things():
    b = MockBackend(gazebo_flat_lab())
    agent = Agent(ToolLayer(b), KeywordLLM(b.known_places, smart=True))
    assert b.status().data["mode"] == "idle"
    r = b.go_to(place="room B")
    assert r.ok
    b.advance(200)
    assert b.where_am_i().data["place"] == "room B"
    r = b.find(object={"class": "box", "color": "yellow"})
    assert r.ok
    b.advance(200)
    assert any(o["color"] == "yellow" for o in b.recall("object").data["records"])
    assert agent is not None


def test_evaluation_mock_world_is_untouched():
    """flat_lab() keeps its 6 objects (barrel, fire extinguisher, mannequin ...): the eval sets depend on them."""
    w = flat_lab()
    assert sorted(o.cls for o in w.objects) == ["barrel", "box", "box", "box", "fire_extinguisher", "mannequin"]
    assert w.map_id == "mock_flat_lab"


def test_web_demo_uses_the_lab4_world():
    from web.api.demo import make_agent
    b = make_agent().layer.backend
    assert b.world.map_id == "lab4" and sorted(b.world.rooms) == ["room A", "room B", "room C", "room D"] and len(b.world.objects) == 10
