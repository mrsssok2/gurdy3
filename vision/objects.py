"""Procedural object catalogue for the synthetic dataset and the test world (Phase 4).

Every class is built from SDF primitives (box / cylinder / sphere) plus one lathe mesh (traffic cone), so there is
NO third-party asset: all models are self-made for this repo and licensed MIT with it (see docs/vision_objects.md).
`parts(cls, scale, variant)` -> list of dicts {kind, size, pos, rgb}; positions are relative to the object origin,
which sits on the floor under the object centre. Pure Python, seedable via `random_variant`.
"""
from __future__ import annotations

import math

LICENSES = {c: "self-made procedural model (SDF primitives / generated OBJ), MIT with this repo"
            for c in ("box", "barrel", "fire_extinguisher", "traffic_cone", "toolbox", "mannequin", "chair")}

PLAIN_COLORS = [(0.85, 0.10, 0.10), (0.10, 0.20, 0.85), (0.90, 0.80, 0.10), (0.10, 0.65, 0.20), (0.90, 0.45, 0.05),
                (0.55, 0.20, 0.65), (0.9, 0.9, 0.9), (0.15, 0.15, 0.15)]
BARREL_COLORS = [(0.10, 0.30, 0.85), (0.85, 0.12, 0.10), (0.90, 0.75, 0.10), (0.15, 0.55, 0.25), (0.45, 0.45, 0.50)]
SKIN = [(0.93, 0.80, 0.68), (0.75, 0.55, 0.42), (0.55, 0.38, 0.28), (0.95, 0.95, 0.95)]
CLOTH = [(0.2, 0.3, 0.7), (0.7, 0.2, 0.2), (0.2, 0.5, 0.3), (0.3, 0.3, 0.3), (0.85, 0.75, 0.2)]
CARDBOARD = (0.65, 0.47, 0.25)


def random_variant(cls: str, rng) -> dict:
    """Colour/shape choices for one instance (JSON-serialisable)."""
    pick = lambda lst: list(lst[rng.integers(len(lst))])
    if cls == "box":
        plain = rng.random() < 0.45
        return {"plain": bool(plain), "rgb": pick(PLAIN_COLORS) if plain else list(CARDBOARD),
                "dims": [float(rng.uniform(0.25, 0.50)), float(rng.uniform(0.25, 0.50)), float(rng.uniform(0.2, 0.45))]}
    if cls == "barrel":
        return {"rgb": pick(BARREL_COLORS)}
    if cls == "toolbox":
        return {"rgb": pick([(0.85, 0.12, 0.10), (0.90, 0.75, 0.10), (0.1, 0.3, 0.8), (0.25, 0.25, 0.28)])}
    if cls == "mannequin":
        return {"skin": pick(SKIN), "rgb": pick(CLOTH), "rgb2": pick(CLOTH)}
    if cls == "chair":
        return {"rgb": pick([(0.35, 0.22, 0.10), (0.15, 0.15, 0.15), (0.75, 0.75, 0.78), (0.2, 0.4, 0.7)])}
    return {}


def _darker(rgb, f=0.55):
    return [c * f for c in rgb]


def cone_obj(r_base: float, r_top: float, h: float, n: int = 24, normals: bool = False, mtl: str | None = None) -> str:
    """Wavefront OBJ of a truncated cone (lathe), base centred on z = 0 (written next to the world by the scene builder).
    normals: per-vertex normals (side: smooth, caps: flat); mtl: name of a .mtl file next to the OBJ (material 'paint')
    so Gazebo uses the colour (2026-11-28: the plain OBJ rendered WHITE whatever the SDF <material> said)."""
    v, f = [], []
    for k in range(n):
        a = 2 * math.pi * k / n
        v.append((r_base * math.cos(a), r_base * math.sin(a), 0.0))
    for k in range(n):
        a = 2 * math.pi * k / n
        v.append((r_top * math.cos(a), r_top * math.sin(a), h))
    v += [(0.0, 0.0, 0.0), (0.0, 0.0, h)]
    for k in range(n):
        k2 = (k + 1) % n
        f.append((k + 1, k2 + 1, n + k2 + 1, n + k + 1))
        f.append((2 * n + 1, k2 + 1, k + 1))            # bottom cap (faces down)
        f.append((2 * n + 2, n + k + 1, n + k2 + 1))    # top cap
    head = "# generated\n" + (f"mtllib {mtl}\n" if mtl else "")
    vs = "".join(f"v {x:.5f} {y:.5f} {z:.5f}\n" for x, y, z in v)
    if not normals:
        return head + vs + ("usemtl paint\n" if mtl else "") + "".join("f " + " ".join(map(str, t)) + "\n" for t in f)
    slope = math.atan2(r_base - r_top, h)                # side normal tilts up by the cone's half angle
    vn = [(math.cos(2 * math.pi * k / n) * math.cos(slope), math.sin(2 * math.pi * k / n) * math.cos(slope), math.sin(slope))
          for k in range(n)] + [(0.0, 0.0, -1.0), (0.0, 0.0, 1.0)]
    ns = "".join(f"vn {x:.5f} {y:.5f} {z:.5f}\n" for x, y, z in vn)
    fs = []
    for k in range(n):
        k2 = (k + 1) % n
        fs.append(f"f {k + 1}//{k + 1} {k2 + 1}//{k2 + 1} {n + k2 + 1}//{k2 + 1} {n + k + 1}//{k + 1}")
        fs.append(f"f {2 * n + 1}//{n + 1} {k2 + 1}//{n + 1} {k + 1}//{n + 1}")
        fs.append(f"f {2 * n + 2}//{n + 2} {n + k + 1}//{n + 2} {n + k2 + 1}//{n + 2}")
    return head + vs + ns + ("usemtl paint\n" if mtl else "") + "\n".join(fs) + "\n"


def mtl_text(rgb) -> str:
    r, g, b = rgb
    return (f"# generated\nnewmtl paint\nKa {r:.3f} {g:.3f} {b:.3f}\nKd {r:.3f} {g:.3f} {b:.3f}\nKs 0.200 0.200 0.200\n"
            "Ns 10\nd 1.0\nillum 2\n")


def parts(cls: str, scale: float = 1.0, variant: dict | None = None) -> list:
    """SDF part list for a class. Dimensions in metres (before `scale`)."""
    v = variant or {}
    s = scale
    P = []

    def add(kind, size, pos, rgb):
        P.append({"kind": kind, "size": [x * s for x in size], "pos": [x * s for x in pos], "rgb": list(rgb)})

    if cls == "box":
        w, d, h = v.get("dims", [0.4, 0.4, 0.35])
        rgb = v.get("rgb", CARDBOARD)
        add("box", (w, d, h), (0, 0, h / 2), rgb)
        if not v.get("plain", False):
            add("box", (w * 1.01, 0.06, h * 1.01), (0, 0, h / 2), (0.85, 0.78, 0.55))   # tape strip
    elif cls == "barrel":
        r, h = 0.27, 0.88
        rgb = v.get("rgb", (0.1, 0.3, 0.85))
        add("cylinder", (r, h), (0, 0, h / 2), rgb)
        for z in (0.25, 0.62):
            add("cylinder", (r * 1.03, 0.04), (0, 0, z), _darker(rgb))
    elif cls == "fire_extinguisher":
        add("cylinder", (0.075, 0.38), (0, 0, 0.19), (0.88, 0.08, 0.08))
        add("cylinder", (0.03, 0.08), (0, 0, 0.42), (0.1, 0.1, 0.1))
        add("box", (0.14, 0.03, 0.03), (0.04, 0, 0.47), (0.1, 0.1, 0.1))
        add("cylinder", (0.012, 0.25), (0.08, 0, 0.30), (0.05, 0.05, 0.05))             # hose
        add("cylinder", (0.078, 0.06), (0, 0, 0.26), (0.92, 0.92, 0.92))                # label band
    elif cls == "traffic_cone":
        add("mesh_cone", (0.14, 0.03, 0.62), (0, 0, 0.03), (1.0, 0.42, 0.0))
        add("mesh_cone", (0.09, 0.065, 0.12), (0, 0, 0.30), (0.95, 0.95, 0.95))          # reflective band
        add("box", (0.36, 0.36, 0.03), (0, 0, 0.015), (0.12, 0.12, 0.12))
    elif cls == "toolbox":
        rgb = v.get("rgb", (0.85, 0.12, 0.1))
        add("box", (0.45, 0.20, 0.17), (0, 0, 0.085), rgb)
        add("box", (0.30, 0.02, 0.02), (0, 0, 0.23), (0.15, 0.15, 0.15))                # handle bar
        for sx in (-0.15, 0.15):
            add("box", (0.02, 0.02, 0.06), (sx, 0, 0.20), (0.15, 0.15, 0.15))
        add("box", (0.04, 0.205, 0.04), (0, 0, 0.14), _darker(rgb))                     # latch
    elif cls == "mannequin":
        skin, c1, c2 = v.get("skin", SKIN[0]), v.get("rgb", CLOTH[0]), v.get("rgb2", CLOTH[3])
        add("box", (0.40, 0.22, 0.55), (0, 0, 1.12), c1)                                # torso
        add("sphere", (0.11,), (0, 0, 1.55), skin)                                      # head
        add("cylinder", (0.045, 0.1), (0, 0, 1.43), skin)                               # neck
        for sy in (-0.12, 0.12):
            add("cylinder", (0.075, 0.82), (0, sy, 0.41), c2)                           # legs
            add("cylinder", (0.04, 0.55), (0, sy * 2.0, 1.12), skin)                    # arms
    elif cls == "chair":
        rgb = v.get("rgb", (0.35, 0.22, 0.1))
        add("box", (0.42, 0.42, 0.04), (0, 0, 0.45), rgb)                               # seat
        add("box", (0.04, 0.42, 0.45), (-0.19, 0, 0.70), rgb)                           # back
        for sx in (-0.19, 0.19):
            for sy in (-0.19, 0.19):
                add("box", (0.04, 0.04, 0.43), (sx, sy, 0.215), _darker(rgb, 0.8))
    else:
        raise KeyError(cls)
    return P


def footprint(cls: str, scale: float = 1.0, variant: dict | None = None) -> tuple:
    """(width_x, depth_y, height) bounding box of the parts (for placement spacing and collision boxes)."""
    xs, ys, zs = [], [], []
    for p in parts(cls, scale, variant):
        px, py, pz = p["pos"]
        if p["kind"] == "box":
            sx, sy, sz = p["size"]
        elif p["kind"] == "cylinder":
            sx = sy = 2 * p["size"][0]; sz = p["size"][1]
        elif p["kind"] == "sphere":
            sx = sy = sz = 2 * p["size"][0]
        else:  # mesh_cone: (r_base, r_top, h), origin at the base
            sx = sy = 2 * p["size"][0]; sz = p["size"][2]; pz += sz / 2
        xs.append((px - sx / 2, px + sx / 2)); ys.append((py - sy / 2, py + sy / 2)); zs.append((pz - sz / 2, pz + sz / 2))
    return (max(b for _, b in xs) - min(a for a, _ in xs), max(b for _, b in ys) - min(a for a, _ in ys),
            max(b for _, b in zs) - min(a for a, _ in zs))


def _geom(p, mesh_dir: str | None, name: str, mesh_rel: str | None = None, normals: bool = False):
    """normals=True writes per-vertex normals into the cone OBJ: without them Gazebo renders the mesh WHITE whatever the SDF
    <material> (2026-11-28, vision/cone_check.py). Default False = the meshes every dataset / world so far was made with."""
    k, sz = p["kind"], p["size"]
    if k == "box":
        return f"<box><size>{sz[0]:.4f} {sz[1]:.4f} {sz[2]:.4f}</size></box>", p["pos"]
    if k == "cylinder":
        return f"<cylinder><radius>{sz[0]:.4f}</radius><length>{sz[1]:.4f}</length></cylinder>", p["pos"]
    if k == "sphere":
        return f"<sphere><radius>{sz[0]:.4f}</radius></sphere>", p["pos"]
    path = f"{mesh_dir}/{name}.obj"
    with open(path, "w") as fh:
        fh.write(cone_obj(*sz, normals=normals))
    uri = f"{mesh_rel}/{name}.obj" if mesh_rel else f"file://{path}"      # relative = resolved against the world file
    return f"<mesh><uri>{uri}</uri></mesh>", p["pos"]


def model_sdf(name: str, part_list: list, x: float, y: float, yaw: float, label: int | None = None,
              collision: bool = False, mesh_dir: str | None = None, z: float = 0.0, mesh_rel: str | None = None) -> str:
    """SDF <model> for one object. `label` (>=1) adds the Gazebo label system so the boundingbox camera sees it."""
    vis = []
    for i, p in enumerate(part_list):
        geom, pos = _geom(p, mesh_dir, f"{name}_p{i}", mesh_rel)
        r, g, b = p["rgb"]
        vis.append(f"""        <visual name="v{i}"><pose>{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f} 0 0 0</pose><geometry>{geom}</geometry>
          <material><ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient><diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse><specular>0.1 0.1 0.1 1</specular></material></visual>
""")
    col = ""
    if collision:
        w = max(max(p["size"][0] if p["kind"] == "box" else 2 * p["size"][0] for p in part_list), 0.05)
        h = max(p["pos"][2] + (p["size"][2] if p["kind"] == "box" else p["size"][1] if p["kind"] == "cylinder" else
                               p["size"][0] if p["kind"] == "sphere" else p["size"][2]) / 2 for p in part_list)
        col = f'        <collision name="c"><pose>0 0 {h / 2:.3f} 0 0 0</pose><geometry><box><size>{w:.3f} {w:.3f} {h:.3f}</size></box></geometry></collision>\n'
    plugin = (f'      <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>{label}</label></plugin>\n'
              if label else "")
    return (f'    <model name="{name}"><static>true</static><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>\n'
            f'{plugin}      <link name="link">\n{col}{"".join(vis)}      </link>\n    </model>\n')


DISTRACTOR_KINDS = ("table", "bench", "stool", "pillar", "ball", "plank", "tall_crate", "stack")


def _rc(rng):
    return [float(x) for x in rng.uniform(0.1, 0.95, 3)]


def distractor_parts(kind: str, rng) -> list:
    """Unlabelled hard negatives (v2 dataset). Shapes are chosen so none is a cube-ish 0.25-0.5 m box (that is the `box`
    class) and none looks like a labelled class; they do cover the things a robot meets in a lab: tables, benches (a solid
    slab like flat_lab's table), stools / cylinders (any colour, like flat_lab's green cylinder), pillars, balls, planks."""
    u = lambda a, b: float(rng.uniform(a, b))
    P = []

    def add(k, size, pos, rgb):
        P.append({"kind": k, "size": list(size), "pos": list(pos), "rgb": list(rgb)})

    if kind == "table":
        w, d, h, c = u(0.5, 1.1), u(0.4, 0.8), u(0.45, 0.8), _rc(rng)
        add("box", (w, d, 0.04), (0, 0, h), c)
        for sx in (-1, 1):
            for sy in (-1, 1):
                add("box", (0.04, 0.04, h), (sx * (w / 2 - 0.03), sy * (d / 2 - 0.03), h / 2), [x * 0.7 for x in c])
    elif kind == "bench":
        w, d, h = u(0.55, 1.0), u(0.22, 0.36), u(0.28, 0.6)
        add("box", (w, d, h), (0, 0, h / 2), _rc(rng))
    elif kind == "stool":
        r, h = u(0.1, 0.3), u(0.2, 0.55)
        add("cylinder", (r, h), (0, 0, h / 2), _rc(rng))
    elif kind == "pillar":
        r, h = u(0.04, 0.18), u(0.6, 1.4)
        add("cylinder", (r, h), (0, 0, h / 2), _rc(rng))
    elif kind == "ball":
        r = u(0.08, 0.3)
        add("sphere", (r,), (0, 0, r), _rc(rng))
    elif kind == "plank":
        w, d, h = u(0.7, 1.4), u(0.06, 0.2), u(0.05, 0.5)
        add("box", (w, d, h), (0, 0, h / 2), _rc(rng))
    elif kind == "tall_crate":
        w, d, h = u(0.15, 0.3), u(0.15, 0.3), u(0.6, 1.2)
        add("box", (w, d, h), (0, 0, h / 2), _rc(rng))
    elif kind == "stack":                                       # two or three random primitives on top of each other
        z = 0.0
        for _ in range(int(rng.integers(2, 4))):
            if rng.random() < 0.5:
                w, d, h = u(0.1, 0.45), u(0.1, 0.45), u(0.08, 0.25)
                add("box", (w, d, h), (0, 0, z + h / 2), _rc(rng))
            else:
                r, h = u(0.05, 0.22), u(0.08, 0.3)
                add("cylinder", (r, h), (0, 0, z + h / 2), _rc(rng))
            z += h
    else:
        raise KeyError(kind)
    return P


def parts_radius(part_list: list) -> float:
    """Horizontal half-extent (for placement spacing)."""
    r = 0.0
    for p in part_list:
        px, py, _ = p["pos"]
        e = max(p["size"][:2]) / 2 if p["kind"] == "box" else p["size"][0]
        r = max(r, math.hypot(px, py) + e)
    return r
