#!/usr/bin/env python3
"""Generate `lab4_fetch` (Phase 9b): the lab4 world UNCHANGED (walls, doors, 10 big objects) + 8 small pickable objects.

    python3 scripts/gen_world_lab4_fetch.py            # writes worlds/lab4_fetch.sdf (+ meshes), maps/lab4_fetch.places.yaml,
                                                       # docs/results/data/lab4_fetch/lab4_fetch_gt.json
    python3 scripts/gen_world_lab4_fetch.py --check    # exit 1 if the committed files are out of date

lab4.sdf / lab4.places.yaml / lab4_gt.json are not touched (this script only imports gen_world_lab4).
The small objects (vision/small_objects.py) are DYNAMIC (mass, inertia, friction) and sit on the floor.
They are all BELOW the lidar plane (5-15 cm vs ~0.16 m): the SLAM map is the lab4 map (maps/lab4_map.yaml can be reused),
but Nav2 / the safety shield cannot see them -> placement keeps them off the walking routes (validate()).
Duplicated classes in different rooms / colours (can: blue A, red C; small_box: red A, blue D; mini_cone: orange B,
green D) for "the red can" / "the cube in room D" style missions.
"""
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "gurdy_core"))
sys.path.insert(0, str(ROOT / "scripts"))
import gen_world_lab4 as L4  # noqa: E402
from vision import objects as O  # noqa: E402
from vision import small_objects as SO  # noqa: E402

OUT_SDF = L4.WORLDS / "lab4_fetch.sdf"
OUT_PLACES = ROOT / "maps/lab4_fetch.places.yaml"
OUT_GT = ROOT / "docs/results/data/lab4_fetch/lab4_fetch_gt.json"

# (name, class, colour, x, y, yaw)
SMALL = [
    ("red_cube", "small_box", "red", 1.2, -1.4, 0.3),
    ("blue_can", "can", "blue", -1.3, 0.3, 0.0),
    ("orange_cone", "mini_cone", "orange", 3.2, -1.4, 0.0),
    ("red_mini_extinguisher", "mini_extinguisher", "red", 5.3, 0.0, 1.2),
    ("yellow_mini_toolbox", "mini_toolbox", "yellow", 0.9, 5.3, 0.5),
    ("red_can", "can", "red", 1.3, 2.6, 0.0),
    ("blue_cube", "small_box", "blue", 4.2, 5.4, -0.4),
    ("green_cone", "mini_cone", "lime", 5.3, 4.0, 0.0),
]
COLOUR_WORD = {"lime": "green", "dark_green": "green"}

# clearance rules (m). The robot must be able to stand with its back to the object, ~0.3 m away (grasp zone,
# docs/results/phase9a_arm_plan.md) with its 0.55 m long footprint, and walk past without kicking it.
MIN_WALL, MIN_BIG, MIN_SMALL, MIN_DOOR, MIN_ROUTE, MIN_SPAWN, MIN_ROOM_CENTRE = 0.55, 0.9, 0.6, 1.0, 0.45, 1.0, 0.8


def room_centres():
    return {n: ((a + c) / 2, (b + d) / 2) for n, (a, b, c, d) in L4.ROOMS.items()}


def routes():
    """Straight walking routes the robot is likely to take: room centre -> each of its doors, door -> door in a room."""
    doors, cen, segs = L4.door_points(), room_centres(), []
    for room, c in cen.items():
        ds = [p for k, p in doors.items() if room[-1] in k.split("-")]
        segs += [(c, p) for p in ds]
        segs += [(ds[i], ds[j]) for i in range(len(ds)) for j in range(i + 1, len(ds))]
    return segs


def _seg_dist(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / (ax * ax + ay * ay)))
    return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)


def validate():
    errs = []
    doors, cen, segs = L4.door_points(), room_centres(), routes()
    for name, cls, colour, x, y, yaw in SMALL:
        if cls not in SO.CATALOG or colour not in SO.CATALOG[cls]["variants"]:
            errs.append(f"{name}: unknown class / colour"); continue
        r = L4.room_of(x, y)
        if r is None:
            errs.append(f"{name}: outside every room"); continue
        a, b, c, d = L4.ROOMS[r]
        if min(x - a, c - x, y - b, d - y) < MIN_WALL:
            errs.append(f"{name}: {min(x - a, c - x, y - b, d - y):.2f} m from a wall of {r}")
        for bn, bcls, bx, by, *_ in L4.OBJECTS:
            if math.hypot(x - bx, y - by) < MIN_BIG:
                errs.append(f"{name}: {math.hypot(x - bx, y - by):.2f} m from {bn}")
        for k, p in doors.items():
            if math.hypot(x - p[0], y - p[1]) < MIN_DOOR:
                errs.append(f"{name}: too close to doorway {k}")
        for s in segs:
            if _seg_dist((x, y), *s) < MIN_ROUTE:
                errs.append(f"{name}: {_seg_dist((x, y), *s):.2f} m from the route {s}")
        if math.hypot(x, y) < MIN_SPAWN:
            errs.append(f"{name}: too close to the spawn")
        if math.hypot(x - cen[r][0], y - cen[r][1]) < MIN_ROOM_CENTRE:
            errs.append(f"{name}: too close to the centre of {r} (room goals)")
    for i, o in enumerate(SMALL):
        for p in SMALL[i + 1:]:
            if math.hypot(o[3] - p[3], o[4] - p[4]) < MIN_SMALL:
                errs.append(f"{o[0]} / {p[0]} closer than {MIN_SMALL} m")
    return errs


def build():
    sdf4, places4, gt4 = L4.build()
    sdf = sdf4.replace('<world name="lab4">', '<world name="lab4_fetch">').replace(
        "GENERATED by scripts/gen_world_lab4.py - edit the script, not this file.",
        "GENERATED by scripts/gen_world_lab4_fetch.py (lab4 + small pickable objects) - edit the script, not this file.")
    models, gt_small = [], []
    for name, cls, colour, x, y, yaw in SMALL:
        models.append(SO.model_sdf("f_" + name, cls, colour, x, y, yaw, mesh_dir=str(L4.WORLDS / "meshes"), mesh_rel="meshes"))
        m, com, _ = SO.mass_properties(cls)
        w, dpt, h = SO.CATALOG[cls]["size"]
        gt_small.append({"name": name, "class": cls, "color": COLOUR_WORD.get(colour, colour), "x": x, "y": y, "z": round(h / 2, 4),
                         "size": [w, dpt, h], "yaw": yaw, "mass": m, "room": L4.room_of(x, y), "pickable": True,
                         "model": "f_" + name})
    sdf = sdf.replace("  </world>", "    <!-- small pickable objects (Phase 9b, vision/small_objects.py; dynamic) -->\n"
                      + "".join(models) + "  </world>")
    gt = json.loads(gt4)
    gt["world"] = "lab4_fetch"
    gt["note"] = "big objects: lab4_gt.json (x, y only, static); small objects: 3D centre, size, mass (dynamic models)"
    for o in gt["objects"]:
        o["pickable"] = False
    gt["objects"] += gt_small
    places = places4.replace("map_id: lab4", "map_id: lab4_fetch").replace(
        "# Named places for the lab4 world", "# Named places for the lab4_fetch world (= lab4 + small objects; same walls, lab4 map reusable)").replace(
        "GENERATED by scripts/gen_world_lab4.py", "GENERATED by scripts/gen_world_lab4_fetch.py")
    return sdf, places, json.dumps(gt, indent=1) + "\n"


if __name__ == "__main__":
    errs = validate() + L4.validate()
    if errs:
        print("\n".join(errs)); sys.exit(2)
    sdf, places, gt = build()
    pairs = [(OUT_SDF, sdf), (OUT_PLACES, places), (OUT_GT, gt)]
    if "--check" in sys.argv:
        sys.exit(0 if all(p.exists() and p.read_text() == t for p, t in pairs) else 1)
    for p, t in pairs:
        p.parent.mkdir(parents=True, exist_ok=True); p.write_text(t)
        print(f"wrote {p}")
