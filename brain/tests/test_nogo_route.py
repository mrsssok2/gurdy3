"""No-go zones for the WHOLE walk (held-out lab4 `l4_nogo_c_route`): room zones are rectangles, plans through a zone are re-routed through an
allowed room, a zone that appears mid-walk stops / re-routes the robot, an unchecked sweep step is checked before it starts."""
import math

from brain.ros import RosBackend
from brain.ros.fake_port import FakePort

# a ring-ish layout: A (home) left, M in the middle (forbidden), G on the right, V above the middle (the way round)
PLACES = """map_id: ring
places:
  home: {x: 0.0, y: 0.0, kind: waypoint}
  room A: {rect: [-1.5, -1.5, 1.5, 1.5], kind: room}
  room M: {rect: [2.0, -1.0, 3.0, 1.0], kind: room}
  room G: {rect: [4.5, -1.5, 7.0, 1.5], kind: room}
  room V: {rect: [2.0, 2.5, 4.0, 5.0], kind: room}
"""


def make(tmp_path, pose=(0.0, 0.0, 0.0)):
    f = tmp_path / "places.yaml"
    f.write_text(PLACES)
    port = FakePort(pose)
    return RosBackend(port, f), port


def run(b, port, secs, dt=0.2, track=None):
    for _ in range(int(secs / dt)):
        port.advance(dt)
        b.tick()
        if track is not None:
            track.append((port.x, port.y))
        if b.task is None:
            break


def in_rect(p, r):
    return r[0] <= p[0] <= r[2] and r[1] <= p[1] <= r[3]


M = [2.0, -1.0, 3.0, 1.0]


def test_room_zone_is_the_room_rectangle(tmp_path):
    b, p = make(tmp_path)
    b.no_go_zone("add", place="room M")
    z = b.no_go_zone("list").data["zones"][0]
    assert z["rect"] == M
    assert b._in_zone(2.05, 0.9) and b._in_zone(2.9, -0.95)         # corners of the room: a disc of radius 1 around the centre misses them
    assert not b._in_zone(1.9, 0.0)


def test_goal_behind_a_zone_is_reached_through_another_room(tmp_path):
    b, p = make(tmp_path)
    b.no_go_zone("add", place="room M")
    assert b.go_to(place="room G").ok
    track: list = []
    run(b, p, 200, track=track)
    assert not any(in_rect(t, M) for t in track), "walked through the forbidden room"
    assert in_rect((p.x, p.y), [4.5, -1.5, 7.0, 1.5])
    assert any(in_rect(t, [2.0, 2.5, 4.0, 5.0]) for t in track), "should have gone round through room V"


def test_no_way_round_is_refused(tmp_path):
    b, p = make(tmp_path)
    b.no_go_zone("add", place="room M")
    b.no_go_zone("add", place="room V")
    r = b.go_to(place="room G")
    assert not r.ok and r.code == "IN_NO_GO_ZONE"


def test_zone_added_while_walking_reroutes_or_stops(tmp_path):
    b, p = make(tmp_path)
    assert b.go_to(place="room G").ok
    track: list = []
    run(b, p, 6, track=track)                                         # on the straight line to G, before room M
    assert p.x > 0.5 and not in_rect((p.x, p.y), M)
    b.no_go_zone("add", place="room M")
    run(b, p, 200, track=track)
    assert not any(in_rect(t, M) for t in track)
    assert in_rect((p.x, p.y), [4.5, -1.5, 7.0, 1.5])                 # went round through V


def test_zone_added_while_walking_with_no_way_round_stops_and_says_why(tmp_path):
    b, p = make(tmp_path)
    b.no_go_zone("add", place="room V")
    assert b.go_to(place="room G").ok
    track: list = []
    run(b, p, 5, track=track)
    b.no_go_zone("add", place="room M")
    run(b, p, 100, track=track)
    assert b.task is None and not any(in_rect(t, M) for t in track)
    ev = [e for e in b.events if e["type"] == "task_failed"]
    assert ev and ev[-1]["data"]["stopped_reason"] == "no_go_zone"
    assert "no-go zone" in ev[-1]["text"]


def test_live_position_in_a_zone_stops_the_robot(tmp_path):
    b, p = make(tmp_path)
    assert b.go_to(place="room G").ok
    run(b, p, 3)
    b.zones.append({"id": 9, "name": "slip", "x": 0.0, "y": 0.0, "r": 0.1, "rect": [p.x + 0.3, -2, p.x + 2.0, 2]})   # unseen by the planner: appears ahead, no version bump
    b.zones_version += 1
    run(b, p, 100)
    assert b.task is None
    assert p.x < 4.5


def test_sweep_step_into_a_zone_room_is_left_out(tmp_path):
    b, p = make(tmp_path)
    b.no_go_zone("add", place="room M")
    steps = b._sweep_steps()
    assert all(s.get("name") != "room M" for s in steps if s["kind"] == "nav")


def test_walk_with_a_zone_is_cut_into_short_legs_along_the_checked_route():
    from brain.ros.backend import RosBackend
    b = RosBackend.__new__(RosBackend)
    b.p = type('P', (), {'map_cell': staticmethod(lambda x, y: 'free')})()
    b.zones = [{"id": 1, "x": 0.0, "y": 4.0, "r": 1.0, "rect": [-2.0, 2.0, 2.0, 6.0]}]
    pts = [(0.025 * i, 0.0) for i in range(321)]              # a straight 8 m plan
    path = {"length_m": 8.0, "points": pts}
    legs = b._waypoints(path)
    assert len(legs) == 3 and all(abs(x - 2.0 * (i + 1)) < 0.05 for i, (x, y) in enumerate(legs))      # 2, 4, 6 m (7.x m is too close to the goal)
    steps = b._nav_steps((8.0, 0.0), None, path, "room X")
    assert [s.get("via") for s in steps] == [True, True, True, None] and steps[-1]["name"] == "room X"
    b.zones = []
    assert b._waypoints(path) == []                           # no zone: one goal, as before
    assert len(b._nav_steps((8.0, 0.0), None, path, "room X")) == 1


def test_waypoints_skip_doorways_and_walls():
    from brain.ros.backend import RosBackend
    b = RosBackend.__new__(RosBackend)
    # a wall with a 1 m gap at x = 4: occupied for |y| > 0.5 on the line x = 4
    b.p = type('P', (), {'map_cell': staticmethod(lambda x, y: 'occupied' if abs(x - 4.0) < 0.05 and abs(y) > 0.5 else 'free')})()
    b.zones = [{"id": 1, "x": 0.0, "y": 4.0, "r": 1.0, "rect": [-2.0, 2.0, 2.0, 6.0]}]
    path = {"length_m": 8.0, "points": [(0.025 * i, 0.0) for i in range(321)]}
    xs = [round(x, 1) for x, _ in b._waypoints(path)]
    assert 4.0 not in xs and xs             # the doorway (x = 4) is not a waypoint, the next free point along the plan is
    steps = b._nav_steps((8.0, 0.0), None, path, "room X")
    assert steps[0]["length_m"] == 0.0 and steps[-1]["length_m"] == 8.0
    assert b._estimate(steps, (0.0, 0.0))["distance_m"] == 8.0
