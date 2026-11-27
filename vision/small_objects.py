"""Phase 9b: the five small PICKABLE objects (self-made SDF primitives, MIT with this repo).

    python3 vision/small_objects.py models     # writes ros2_ws/src/gurdy_gazebo/models/small_<class>_<colour>/ (model.sdf + model.config)

Each class: VISUAL parts (with colour, may include the lathe cone mesh of vision/objects.py) and COLLISION solids
(boxes / cylinders only: stable DART contacts; the cone is 3 stacked cylinders). Mass and inertia are COMPUTED from the
collision solids at uniform density (scaled to the class mass), never typed. Friction mu = mu2 = 1.0 on every collision
(gripper pads have 1.0 too), torsional friction for the twist while lifting.
Sizes / masses / grasp data must match gurdy_core.arm.PICKABLE (tested in vision/tests/test_small_objects.py).
Model origin = floor under the object centre (same convention as vision/objects.py).
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from vision import objects as O  # noqa: E402

MODELS_DIR = ROOT / "ros2_ws/src/gurdy_gazebo/models"
FRICTION = 1.0
TORSIONAL = 0.5
LICENSE = "self-made procedural model (SDF primitives / generated OBJ), MIT with this repo"

# bright, distinct colours; `variants` = colour names the dataset samples from (the detector must learn SHAPE, so every
# class appears in several colours, and colours are shared between classes)
COL = {"red": (0.90, 0.08, 0.08), "blue": (0.08, 0.30, 0.95), "green": (0.10, 0.75, 0.20), "yellow": (0.98, 0.85, 0.05),
       "orange": (1.00, 0.45, 0.02), "purple": (0.60, 0.20, 0.85), "pink": (0.98, 0.40, 0.70), "silver": (0.76, 0.77, 0.80),
       "lime": (0.65, 0.95, 0.10), "black": (0.06, 0.06, 0.07), "white": (0.95, 0.95, 0.95), "dark_green": (0.05, 0.40, 0.15)}


def _cat():
    c = {}
    # 5 cm cube, 50 g, top-down grasp
    c["small_box"] = {"mass": 0.050, "size": (0.05, 0.05, 0.05), "variants": ["red", "blue", "green", "yellow", "purple"],
                      "collision": [("box", (0.05, 0.05, 0.05), (0, 0, 0.025))],
                      "visual": lambda col: [("box", (0.05, 0.05, 0.05), (0, 0, 0.025), col)]}
    # mini toolbox 12 x 5 x 6 cm body + raised handle (bar 8 x 2 x 1 cm, top at 9.5 cm; 2.5 cm finger gap above the body)
    c["mini_toolbox"] = {"mass": 0.080, "size": (0.12, 0.05, 0.095), "variants": ["red", "yellow", "blue", "orange"],
                         "collision": [("box", (0.12, 0.05, 0.06), (0, 0, 0.03)),
                                       ("box", (0.01, 0.02, 0.025), (-0.035, 0, 0.0725)),
                                       ("box", (0.01, 0.02, 0.025), (0.035, 0, 0.0725)),
                                       ("box", (0.08, 0.02, 0.01), (0, 0, 0.09))],
                         "visual": lambda col: [("box", (0.12, 0.05, 0.06), (0, 0, 0.03), col),
                                                ("box", (0.122, 0.052, 0.006), (0, 0, 0.045), COL["black"]),   # lid line
                                                ("box", (0.01, 0.02, 0.025), (-0.035, 0, 0.0725), COL["black"]),
                                                ("box", (0.01, 0.02, 0.025), (0.035, 0, 0.0725), COL["black"]),
                                                ("box", (0.08, 0.02, 0.01), (0, 0, 0.09), COL["black"])]}
    # can r 3.3 cm, h 12 cm, 60 g, side grasp
    c["can"] = {"mass": 0.060, "size": (0.066, 0.066, 0.12), "variants": ["red", "blue", "green", "silver", "yellow"],
                "collision": [("cylinder", (0.033, 0.12), (0, 0, 0.06))],
                "visual": lambda col: [("cylinder", (0.033, 0.112), (0, 0, 0.06), col),
                                       ("cylinder", (0.031, 0.004), (0, 0, 0.002), COL["silver"]),
                                       ("cylinder", (0.031, 0.004), (0, 0, 0.118), COL["silver"])]}
    # mini cone: 8 x 8 x 0.8 cm base plate + cone r 3.5 -> 0.6 cm, total h 10 cm, 30 g, side grasp at 5 cm (4.4 cm wide)
    c["mini_cone"] = {"mass": 0.030, "size": (0.08, 0.08, 0.10), "variants": ["orange", "lime", "pink", "blue"],
                      "collision": [("box", (0.08, 0.08, 0.008), (0, 0, 0.004)),
                                    ("cylinder", (0.030, 0.031), (0, 0, 0.0235)),     # radius = mean radius of each third
                                    ("cylinder", (0.0205, 0.031), (0, 0, 0.0545)),
                                    ("cylinder", (0.0108, 0.030), (0, 0, 0.085))],
                      "visual": lambda col: [("box", (0.08, 0.08, 0.008), (0, 0, 0.004), COL["black"]),
                                             ("mesh_cone", (0.035, 0.006, 0.092), (0, 0, 0.008), col),
                                             ("cylinder", (0.0232, 0.010), (0, 0, 0.050), COL["white"])]}   # reflective band
    # mini extinguisher: body r 2.5 cm h 13 cm, valve r 1.2 cm h 1.2 cm, handle + nozzle; total h 15 cm, 80 g, side grasp
    c["mini_extinguisher"] = {"mass": 0.080, "size": (0.05, 0.05, 0.15), "variants": ["red", "dark_green", "blue"],
                              "collision": [("cylinder", (0.025, 0.13), (0, 0, 0.065)),
                                            ("cylinder", (0.012, 0.012), (0, 0, 0.136)),
                                            ("box", (0.03, 0.008, 0.008), (0.006, 0, 0.146))],
                              "visual": lambda col: [("cylinder", (0.025, 0.13), (0, 0, 0.065), col),
                                                     ("cylinder", (0.012, 0.012), (0, 0, 0.136), COL["silver"]),
                                                     ("box", (0.03, 0.008, 0.008), (0.006, 0, 0.146), COL["black"]),
                                                     ("cylinder", (0.0255, 0.025), (0, 0, 0.085), COL["white"])]}   # label band
    return c


CATALOG = _cat()


def _solid(kind, size, pos):
    """-> (volume, centre, inertia diag about own centre per unit mass) for a box (sx, sy, sz) / z-cylinder (r, h)."""
    if kind == "box":
        sx, sy, sz = size
        return sx * sy * sz, pos, ((sy * sy + sz * sz) / 12, (sx * sx + sz * sz) / 12, (sx * sx + sy * sy) / 12)
    r, h = size
    return math.pi * r * r * h, pos, ((3 * r * r + h * h) / 12, (3 * r * r + h * h) / 12, r * r / 2)


def mass_properties(cls: str):
    """Uniform density over the collision solids, scaled to the class mass -> (mass, com (3,), inertia diag (3,))."""
    c = CATALOG[cls]
    sol = [_solid(k, s, p) for k, s, p in c["collision"]]
    vol = sum(v for v, _, _ in sol)
    rho = c["mass"] / vol
    com = [sum(v * p[i] for v, p, _ in sol) / vol for i in range(3)]
    I = [0.0, 0.0, 0.0]
    for v, p, ic in sol:
        m = rho * v
        d = [p[i] - com[i] for i in range(3)]
        I[0] += m * (ic[0] + d[1] ** 2 + d[2] ** 2)
        I[1] += m * (ic[1] + d[0] ** 2 + d[2] ** 2)
        I[2] += m * (ic[2] + d[0] ** 2 + d[1] ** 2)
    return c["mass"], com, I          # off-diagonal terms are 0: every solid is centred on the z axis or symmetric in y


def _geom_sdf(kind, size):
    if kind == "box":
        return f"<box><size>{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}</size></box>"
    return f"<cylinder><radius>{size[0]:.4f}</radius><length>{size[1]:.4f}</length></cylinder>"


SURFACE = (f"<surface><friction><ode><mu>{FRICTION}</mu><mu2>{FRICTION}</mu2></ode>"
           f"<torsional><coefficient>{TORSIONAL}</coefficient></torsional></friction>"
           "<contact><ode><min_depth>0.0005</min_depth><max_vel>0.1</max_vel></ode></contact></surface>")


def model_sdf(name: str, cls: str, colour: str, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, label: int | None = None,
              static: bool = False, mesh_dir: str | None = None, mesh_rel: str | None = None, z: float = 0.0) -> str:
    """<model> of one small object. Dynamic by default (real mass / inertia / friction, picked by the gripper);
    `static=True` for dataset renders. `label` adds the gz Label system (boundingbox camera)."""
    c = CATALOG[cls]
    m, com, I = mass_properties(cls)
    vis = []
    for i, (kind, size, pos, rgb) in enumerate(c["visual"](COL[colour])):
        if kind == "mesh_cone":
            geom, _ = O._geom({"kind": "mesh_cone", "size": list(size), "pos": list(pos)}, mesh_dir, f"{name}_p{i}", mesh_rel)
        else:
            geom = _geom_sdf(kind, size)
        r, g, b = rgb
        vis.append(f'        <visual name="v{i}"><pose>{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f} 0 0 0</pose><geometry>{geom}</geometry>'
                   f'<material><ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient><diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>'
                   f'<specular>0.2 0.2 0.2 1</specular></material></visual>\n')
    col = "".join(f'        <collision name="c{i}"><pose>{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} 0 0 0</pose><geometry>{_geom_sdf(k, s)}</geometry>'
                  f'{SURFACE}</collision>\n' for i, (k, s, p) in enumerate(c["collision"]))
    inertial = (f'        <inertial><pose>{com[0]:.5f} {com[1]:.5f} {com[2]:.5f} 0 0 0</pose><mass>{m:.4f}</mass>'
                f'<inertia><ixx>{I[0]:.4e}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{I[1]:.4e}</iyy><iyz>0</iyz><izz>{I[2]:.4e}</izz></inertia></inertial>\n')
    plugin = (f'      <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>{label}</label></plugin>\n'
              if label else "")
    return (f'    <model name="{name}"><static>{str(static).lower()}</static><pose>{x:.3f} {y:.3f} {z + 0.0005:.4f} 0 0 {yaw:.4f}</pose>\n'
            f'{plugin}      <link name="link">\n{inertial}{col}{"".join(vis)}      </link>\n    </model>\n')


def write_model_dirs(out: Path = MODELS_DIR) -> list:
    """One Gazebo model directory per (class, colour): spawnable with `ros_gz_sim create -file .../model.sdf`."""
    written = []
    for cls, c in CATALOG.items():
        for colour in c["variants"]:
            d = out / f"small_{cls}_{colour}"
            (d / "meshes").mkdir(parents=True, exist_ok=True)
            body = model_sdf(f"small_{cls}_{colour}", cls, colour, mesh_dir=str(d / "meshes"), mesh_rel="meshes")
            (d / "model.sdf").write_text(f'<?xml version="1.0"?>\n<!-- GENERATED by vision/small_objects.py ({LICENSE}) -->\n'
                                         f'<sdf version="1.10">\n{body}</sdf>\n')
            (d / "model.config").write_text(
                f'<?xml version="1.0"?>\n<model><name>small_{cls}_{colour}</name><version>1.0</version>'
                f'<sdf version="1.10">model.sdf</sdf><author><name>Gurdy project (generated)</name></author>'
                f'<description>Phase 9b pickable object: {cls}, {colour}, {c["mass"]*1000:.0f} g. {LICENSE}</description></model>\n')
            if not any((d / "meshes").iterdir()):
                (d / "meshes").rmdir()
            written.append(d)
    return written


if __name__ == "__main__":
    if sys.argv[1:] == ["models"]:
        for d in write_model_dirs():
            print("wrote", d)
    else:
        print(__doc__)
