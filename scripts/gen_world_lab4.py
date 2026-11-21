#!/usr/bin/env python3
"""Generate the `lab4` world (Gurdy 3.0): four rooms A-D in a ring, 8 x 8 m, 10 objects of classes the detector already knows.

    python3 scripts/gen_world_lab4.py            # writes worlds/lab4.sdf (+ meshes), maps/lab4.places.yaml, docs/results/data/lab4/lab4_gt.json
    python3 scripts/gen_world_lab4.py --check    # exit 1 if the committed files are out of date

Layout (metres, robot spawns at the origin facing +x, in the middle of room A; map frame = world frame):

    y=6  +---------------------+---------------------+
         |  room C             |  room D             |
         |  yellow box         :  mannequin  toolbox |     doorways (1.0 m wide, gap centre):
         |  fire extinguisher  : door C-D (2, 4.2)   |       A-B  x=2,  y=-0.7
    y=2  +--- door A-C (-0.7,2) ---+--- door B-D (4,2) ---+      B-D  y=2,  x=4.0
         |  room A             |  room B             |       D-C  x=2,  y=4.2
         |  R-> red box, cone  :  blue box, blue barrel      C-A  y=2,  x=-0.7
    y=-2 +---------------------+---------------------+
        x=-2                  x=2                   x=6

The rooms form a RING (A-B-D-C-A): there is no A-D or B-C doorway, so A to D is two hops. Same class in different rooms:
box (red A, blue B, yellow C), barrel (blue B, red D), traffic cone (A, D). No chair anywhere (the honesty missions ask for it).
Everything is taller than the lidar plane. Objects are placed away from walls and doorways (checked by `validate`).
"""
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "gurdy_core"))
sys.path.insert(0, str(ROOT / "scripts"))
import gen_world_flat_lab as flat  # noqa: E402
from vision import objects as O  # noqa: E402

WORLDS = ROOT / "ros2_ws/src/gurdy_gazebo/worlds"
OUT_SDF = WORLDS / "lab4.sdf"
OUT_PLACES = ROOT / "maps/lab4.places.yaml"
OUT_GT = ROOT / "docs/results/data/lab4/lab4_gt.json"

X0, X1, Y0, Y1 = -2.0, 6.0, -2.0, 6.0
DIV_X, DIV_Y = 2.0, 2.0
DOOR_W = 1.0
# doorways: (name, axis of the divider it cuts, gap centre along the divider)
DOORS = {"A-B": ("x", -0.7), "B-D": ("y", 4.0), "D-C": ("x", 4.2), "C-A": ("y", -0.7)}
ROOMS = {"room A": (X0, Y0, DIV_X, DIV_Y), "room B": (DIV_X, Y0, X1, DIV_Y),
         "room C": (X0, DIV_Y, DIV_X, Y1), "room D": (DIV_X, DIV_Y, X1, Y1)}

# (name, class, x, y, yaw, variant, color)
OBJECTS = [
    ("red_box", "box", -1.2, -1.3, 0.0, {"rgb": [0.85, 0.10, 0.10]}, "red"),
    ("cone_a", "traffic_cone", 1.3, 1.3, 0.0, {}, ""),
    ("blue_box", "box", 5.0, -1.2, 0.0, {"rgb": [0.10, 0.20, 0.85]}, "blue"),
    ("barrel_blue", "barrel", 4.9, 1.2, 0.0, {"rgb": [0.10, 0.30, 0.85]}, "blue"),
    ("yellow_box", "box", -1.3, 5.0, 0.0, {"rgb": [0.90, 0.80, 0.10]}, "yellow"),
    ("extinguisher", "fire_extinguisher", -1.4, 3.0, 0.0, {}, ""),
    ("barrel_red", "barrel", 5.1, 5.0, 0.0, {"rgb": [0.85, 0.12, 0.10]}, "red"),
    ("mannequin", "mannequin", 3.0, 3.0, 2.8, {"skin": [0.93, 0.80, 0.68], "rgb": [0.2, 0.3, 0.7], "rgb2": [0.3, 0.3, 0.3]}, ""),
    ("toolbox", "toolbox", 3.2, 5.3, 0.6, {"rgb": [0.85, 0.12, 0.10]}, ""),
    ("cone_d", "traffic_cone", 5.2, 3.0, 0.0, {}, ""),
]


def room_of(x, y):
    for n, (a, b, c, d) in ROOMS.items():
        if a <= x <= c and b <= y <= d:
            return n
    return None


def door_points():
    out = {}
    for k, (axis, c) in DOORS.items():
        out[k] = (DIV_X, c) if axis == "x" else (c, DIV_Y)
    return out


def walls():
    w, h = flat.WALL_T, flat.WALL_H
    out = [("wall_south", ((X0 + X1) / 2, Y0), (X1 - X0 + w, w)), ("wall_north", ((X0 + X1) / 2, Y1), (X1 - X0 + w, w)),
           ("wall_west", (X0, (Y0 + Y1) / 2), (w, Y1 - Y0)), ("wall_east", (X1, (Y0 + Y1) / 2), (w, Y1 - Y0))]
    # vertical divider x = DIV_X: lower half (y in [Y0, DIV_Y]) has the A-B door, upper half the D-C door
    for tag, lo, hi, door in (("low", Y0, DIV_Y, DOORS["A-B"][1]), ("high", DIV_Y, Y1, DOORS["D-C"][1])):
        g0, g1 = door - DOOR_W / 2, door + DOOR_W / 2
        out.append((f"div_x_{tag}_1", (DIV_X, (lo + g0) / 2), (w, g0 - lo)))
        out.append((f"div_x_{tag}_2", (DIV_X, (g1 + hi) / 2), (w, hi - g1)))
    # horizontal divider y = DIV_Y: left half (x in [X0, DIV_X]) has C-A, right half B-D
    for tag, lo, hi, door in (("left", X0, DIV_X, DOORS["C-A"][1]), ("right", DIV_X, X1, DOORS["B-D"][1])):
        g0, g1 = door - DOOR_W / 2, door + DOOR_W / 2
        out.append((f"div_y_{tag}_1", ((lo + g0) / 2, DIV_Y), (g0 - lo, w)))
        out.append((f"div_y_{tag}_2", ((g1 + hi) / 2, DIV_Y), (hi - g1, w)))
    return out


def validate():
    """Geometry sanity: objects inside a room, >= 0.3 m (+ radius) from walls, >= 0.9 m from doorway centres, >= 0.8 m from each other and the spawn."""
    errs, pts = [], door_points()
    for name, cls, x, y, yaw, var, col in OBJECTS:
        r = room_of(x, y)
        if r is None:
            errs.append(f"{name}: outside every room"); continue
        a, b, c, d = ROOMS[r]
        fw, fd, _ = O.footprint(cls, 1.0, var)
        rad = 0.5 * max(fw, fd)
        for dist, what in ((x - a, "west"), (c - x, "east"), (y - b, "south"), (d - y, "north")):
            if dist < 0.30 + rad:
                errs.append(f"{name}: {dist:.2f} m from the {what} wall of {r}")
        for k, (px, py) in pts.items():
            if math.hypot(x - px, y - py) < 0.9 + rad:
                errs.append(f"{name}: too close to doorway {k}")
        if math.hypot(x, y) < 0.8:
            errs.append(f"{name}: too close to the spawn")
    for i, o in enumerate(OBJECTS):
        for p in OBJECTS[i + 1:]:
            if math.hypot(o[2] - p[2], o[3] - p[3]) < 0.8:
                errs.append(f"{o[0]} / {p[0]} closer than 0.8 m")
    return errs


def build():
    base = flat.build()
    head = base[:base.index('    <model name="wall_south">')]
    head = head.replace('<world name="flat_lab">', '<world name="lab4">').replace(
        "GENERATED by scripts/gen_world_flat_lab.py - edit the script, not this file.",
        "GENERATED by scripts/gen_world_lab4.py - edit the script, not this file.").replace(
        "flat_lab: two rooms, one doorway, a few coloured objects (Phase 3 mapping / navigation).",
        "lab4: four rooms A-D in a ring (8 x 8 m), 10 objects (Gurdy 3.0).")
    parts = [head]
    for name, xy, (sx, sy) in walls():
        geom = f"<box><size>{sx:.3f} {sy:.3f} {flat.WALL_H}</size></box>"
        parts.append(flat._static_model(name, xy, flat.WALL_H / 2, geom, (0.85, 0.85, 0.80)))
    (WORLDS / "meshes").mkdir(exist_ok=True)
    meshes = {}
    gt = []
    parts.append("    <!-- objects: classes of the detector (vision/objects.py) -->\n")
    for name, cls, x, y, yaw, var, col in OBJECTS:
        mname = "l4_" + name
        parts.append(O.model_sdf(mname, O.parts(cls, 1.0, var), x, y, yaw, collision=True, mesh_dir=str(WORLDS / "meshes"), mesh_rel="meshes"))
        gt.append({"name": name, "class": cls, "x": x, "y": y, "color": col, "room": room_of(x, y)})
    parts.append("  </world>\n</sdf>\n")
    places = ["# Named places for the lab4 world (map frame = world frame: the robot spawns at the origin, in room A).",
              "# GENERATED by scripts/gen_world_lab4.py (doorways are not named places: 4 of them would be ambiguous; see lab4_gt.json). Rooms are rectangles [xmin, ymin, xmax, ymax] (goal = a free point near the centre).",
              "map_id: lab4", "places:", "  home: {x: 0.0, y: 0.0, kind: waypoint}"]
    for n, r in ROOMS.items():
        places.append(f"  {n}: {{rect: [{r[0]}, {r[1]}, {r[2]}, {r[3]}], kind: room}}")
    return "".join(parts), "\n".join(places) + "\n", json.dumps({"world": "lab4", "bounds": [X0, X1, Y0, Y1], "doors": {k: list(v) for k, v in door_points().items()},
                                                                    "objects": gt}, indent=1) + "\n"


if __name__ == "__main__":
    errs = validate()
    if errs:
        print("\n".join(errs)); sys.exit(2)
    sdf, places, gt = build()
    pairs = [(OUT_SDF, sdf), (OUT_PLACES, places), (OUT_GT, gt)]
    if "--check" in sys.argv:
        sys.exit(0 if all(p.exists() and p.read_text() == t for p, t in pairs) else 1)
    for p, t in pairs:
        p.parent.mkdir(parents=True, exist_ok=True); p.write_text(t)
        print(f"wrote {p}")
