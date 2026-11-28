"""The mock world that matches the REAL Gazebo flat_lab, generated from its sources (not a copy):

- walls, doorway, objects (positions, sizes, colours): `scripts/gen_world_flat_lab.py` (the SDF generator: walls(), OBJECTS, constants)
- rooms and named places: `maps/flat_lab.places.yaml` (the same file RosBackend uses)

`brain.mock.world.flat_lab()` is NOT this world: it is the richer world the evaluation sets (testset_v1 ...) were written for (barrel,
fire extinguisher, mannequin) and stays untouched. Use `gazebo_flat_lab()` for the web demo / dashboard, so what the page shows is what
the robot would find in Gazebo.

Object classes: the Gazebo lab has plain coloured boxes and one cylinder. `*_box` -> detector class `box` with its colour, the green
cylinder -> `barrel` (the nearest detector class; the real detector often calls it `box`), the table is an obstacle only (not a class).
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

import yaml

from .world import GtObject, Rect, World

ROOT = Path(__file__).resolve().parents[2]
GEN = ROOT / "scripts" / "gen_world_flat_lab.py"
PLACES = ROOT / "maps" / "flat_lab.places.yaml"
_CLASS_OF = {"cylinder": "barrel"}           # geometry -> detector class when the name has no class word
_NOT_OBJECTS = {"table"}                     # obstacles that are not detector classes


def _load_generator():
    spec = importlib.util.spec_from_file_location("gen_world_flat_lab", GEN)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _centre_rect(cx: float, cy: float, sx: float, sy: float) -> Rect:
    return Rect(cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2)


def _colour_of(name: str) -> str:
    return name.split("_")[0]


def gazebo_flat_lab() -> World:
    g = _load_generator()
    walls = [_centre_rect(c[0], c[1], s[0], s[1]) for _, c, s in g.walls()]
    obstacles: list[Rect] = []
    objects: list[GtObject] = []
    gid = 0
    for name, kind, (x, y), size, _rgb in g.OBJECTS:
        sx, sy = (size[0], size[1]) if kind == "box" else (2 * size[0], 2 * size[0])
        if name in _NOT_OBJECTS:
            obstacles.append(_centre_rect(x, y, sx, sy))
            continue
        gid += 1
        cls = "box" if name.endswith("_box") else _CLASS_OF.get(kind, kind)
        objects.append(GtObject(gid, cls, _colour_of(name), x, y, max(sx, sy) / 2))
    spec = yaml.safe_load(PLACES.read_text())
    rooms: dict[str, Rect] = {}
    places: dict[str, tuple[float, float]] = {}
    for pname, v in spec["places"].items():
        if "rect" in v:
            r = Rect(*[float(c) for c in v["rect"]])
            rooms[pname] = r
            places[pname] = r.centre
        else:
            places[pname] = (float(v["x"]), float(v["y"]))
    pad = 0.2
    bounds = Rect(g.X_MIN - pad, g.Y_MIN - pad, g.X_MAX + pad, g.Y_MAX + pad)
    return World(bounds=bounds, rooms=rooms, walls=walls, obstacles=obstacles, objects=objects, places=places,
                 map_id=spec.get("map_id", "flat_lab"))


# ---------------------------------------------------------------------------------------------------------------- lab4
LAB4_PLACES = ROOT / "maps" / "lab4.places.yaml"
LAB4_GT = ROOT / "docs" / "results" / "data" / "lab4" / "lab4_gt.json"
LAB4_DOOR_W = 1.0                          # scripts/gen_world_lab4.py DOOR_W
# half of the larger footprint side of each class (vision/objects.py footprint(), measured once; keeps this file free of the vision package)
_LAB4_RADIUS = {"box": 0.20, "traffic_cone": 0.18, "barrel": 0.28, "fire_extinguisher": 0.095, "mannequin": 0.28, "toolbox": 0.225,
                "chair": 0.30}            # chair: lab4_vln (0.42 m square collision box)


def lab4_walls(bounds: list[float], doors: dict[str, list[float]], div_x: float, div_y: float, thick: float) -> list[Rect]:
    """The lab4 wall rectangles, the same construction as scripts/gen_world_lab4.walls(): outer walls plus two dividers with a 1 m gap per half.
    `doors` = {name: [x, y] gap centre} (A-B / D-C cut the vertical divider, C-A / B-D the horizontal one)."""
    x0, x1, y0, y1 = bounds
    w = thick
    out = [_centre_rect((x0 + x1) / 2, y0, x1 - x0 + w, w), _centre_rect((x0 + x1) / 2, y1, x1 - x0 + w, w),
           _centre_rect(x0, (y0 + y1) / 2, w, y1 - y0), _centre_rect(x1, (y0 + y1) / 2, w, y1 - y0)]
    for lo, hi, door in ((y0, div_y, doors["A-B"][1]), (div_y, y1, doors["D-C"][1])):
        g0, g1 = door - LAB4_DOOR_W / 2, door + LAB4_DOOR_W / 2
        out += [_centre_rect(div_x, (lo + g0) / 2, w, g0 - lo), _centre_rect(div_x, (g1 + hi) / 2, w, hi - g1)]
    for lo, hi, door in ((x0, div_x, doors["C-A"][0]), (div_x, x1, doors["B-D"][0])):
        g0, g1 = door - LAB4_DOOR_W / 2, door + LAB4_DOOR_W / 2
        out += [_centre_rect((lo + g0) / 2, div_y, g0 - lo, w), _centre_rect((g1 + hi) / 2, div_y, hi - g1, w)]
    return out


def gazebo_lab4(variant: str = "lab4") -> World:
    """Mock of the Gazebo `lab4` world (Gurdy 3.0): four rooms A-D in a ring, doorways A-B, B-D, D-C, C-A, the 10 objects at their ground-truth
    positions. Generated from the committed lab4 files (`maps/lab4.places.yaml`, `docs/results/data/lab4/lab4_gt.json`, both written by
    scripts/gen_world_lab4.py); `gazebo_flat_lab()` and `flat_lab()` are untouched. Used by the public demo only."""
    import json
    g = _load_generator()
    gt_file, places_file = (LAB4_GT, LAB4_PLACES) if variant == "lab4" else \
        (ROOT / "docs" / "results" / "data" / variant / f"{variant}_gt.json", ROOT / "maps" / f"{variant}.places.yaml")   # lab4_vln: + a chair
    gt = json.loads(gt_file.read_text())
    spec = yaml.safe_load(places_file.read_text())
    rooms: dict[str, Rect] = {}
    places: dict[str, tuple[float, float]] = {}
    for pname, v in spec["places"].items():
        if "rect" in v:
            r = Rect(*[float(c) for c in v["rect"]])
            rooms[pname] = r
            places[pname] = r.centre
        else:
            places[pname] = (float(v["x"]), float(v["y"]))
    div_x, div_y = rooms["room B"].x0, rooms["room C"].y0
    x0, x1, y0, y1 = gt["bounds"]
    walls = lab4_walls(gt["bounds"], gt["doors"], div_x, div_y, g.WALL_T)
    objects = [GtObject(i + 1, o["class"], o["color"], o["x"], o["y"], _LAB4_RADIUS[o["class"]]) for i, o in enumerate(gt["objects"])]
    pad = 0.2
    w = World(bounds=Rect(x0 - pad, y0 - pad, x1 + pad, y1 + pad), rooms=rooms, walls=walls, objects=objects, places=places, map_id=spec.get("map_id", "lab4"))
    w.doors = {k: (float(v[0]), float(v[1])) for k, v in gt["doors"].items()}     # doorway centres (VLN routing: brain/vln/world.py)
    w.room_names = {str(n).lower(): k for k, v in spec["places"].items() if "rect" in v for n in v.get("names") or []}   # spoken names (VLN)
    return w
