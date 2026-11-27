"""Random scene sampler + SDF world writer + camera/label geometry for the synthetic dataset (Phase 4).

`sample_scene(seed)` returns a JSON-serialisable dict (room, floor/wall appearance, lights, labelled objects,
unlabelled distractors, camera views). `world_sdf(scene)` turns it into a Gazebo Harmonic world with ONE static
camera model ("cam": rgbd_camera + boundingbox_camera at the same pose) that the generator moves between views.
Pure Python/numpy: tested without Gazebo.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from gurdy_core.vision.classes import CLASSES, GZ_LABEL_OFFSET
from vision import objects as O
from vision.textures import KINDS

WORLD_NAME = "synth"
N_TEXTURES = 12
SPLITS = (("train", 14), ("val", 3), ("test", 3))      # of every 20 consecutive scenes -> 70 / 15 / 15 %, scene-disjoint

DEFAULT_CAMERA = {"width": 320, "height": 240, "hfov": 1.047, "range": [0.10, 5.0], "pitch_down": 0.17}


def split_for_scene(idx: int) -> str:
    r = idx % 20
    acc = 0
    for name, n in SPLITS:
        acc += n
        if r < acc:
            return name
    return "train"


def _rand_rgb(rng, lo=0.15, hi=0.95):
    return [float(x) for x in rng.uniform(lo, hi, 3)]


def optical_from_world(point, cam_pos, cam_rpy) -> np.ndarray:
    """World point -> camera optical frame (x right, y down, z forward) for a Gazebo camera (x forward) at cam_pose."""
    r, p, y = cam_rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    R = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                  [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                  [-sp, cp * sr, cp * cr]])
    pl = R.T @ (np.asarray(point, float) - np.asarray(cam_pos, float))   # camera_link: x fwd, y left, z up
    return np.array([-pl[1], -pl[2], pl[0]])


def project(p_opt, hfov: float, w: int, h: int):
    """Optical-frame point -> pixel (u, v) or None if behind the camera."""
    if p_opt[2] <= 1e-6:
        return None
    f = (w / 2) / math.tan(hfov / 2)
    return f * p_opt[0] / p_opt[2] + w / 2, f * p_opt[1] / p_opt[2] + h / 2


def sample_scene(seed: int, n_views: int = 12, camera: dict | None = None, profile: str = "v1") -> dict:
    """profile "v1": the first dataset. "v2": more mannequins (class weight 3, one in half the scenes), views aimed at
    mannequins and at distractors, 2-10 distractors from distinct shape families (objects.DISTRACTOR_KINDS)."""
    cam = {**DEFAULT_CAMERA, **(camera or {})}
    rng = np.random.default_rng(seed)
    W, D = float(rng.uniform(4.0, 8.0)), float(rng.uniform(4.0, 8.0))
    sc = {"seed": seed, "profile": profile, "room": {"w": W, "d": D, "h": float(rng.uniform(1.8, 3.0))}, "camera": cam}
    tex = lambda: (int(rng.integers(N_TEXTURES)) if rng.random() < 0.65 else None)
    sc["floor"] = {"rgb": _rand_rgb(rng, 0.2, 0.85), "texture": tex()}
    sc["wall"] = {"rgb": _rand_rgb(rng, 0.3, 0.95), "texture": tex()}
    sc["ambient"] = float(rng.uniform(0.15, 0.6))
    sc["shadows"] = bool(rng.random() < 0.6)
    sc["sun"] = {"intensity": float(rng.uniform(0.4, 1.3)), "dir": [float(rng.uniform(-1, 1)), float(rng.uniform(-1, 1)),
                                                                     -float(rng.uniform(0.5, 1.2))], "rgb": [1.0, *[float(rng.uniform(0.8, 1.0)) for _ in range(2)]]}
    sc["lamps"] = [{"pos": [float(rng.uniform(-W / 2, W / 2)), float(rng.uniform(-D / 2, D / 2)), float(rng.uniform(1.2, 2.5))],
                    "intensity": float(rng.uniform(0.3, 1.0)), "rgb": _rand_rgb(rng, 0.7, 1.0)} for _ in range(int(rng.integers(0, 3)))]

    placed = []          # (x, y, radius)

    def place(r, tries=60):
        for _ in range(tries):
            x, y = float(rng.uniform(-W / 2 + 0.5, W / 2 - 0.5)), float(rng.uniform(-D / 2 + 0.5, D / 2 - 0.5))
            if all(math.hypot(x - a, y - b) > r + rr + 0.1 for a, b, rr in placed):
                placed.append((x, y, r))
                return x, y
        return None

    sc["objects"] = []
    v2 = profile == "v2"
    weights = np.array([3.0 if c == "mannequin" else 1.0 for c in CLASSES]) if v2 else np.ones(len(CLASSES))
    force_mannequin = v2 and rng.random() < 0.5
    for k in range(int(rng.integers(1, 7))):
        if force_mannequin and k == 0:
            cls = "mannequin"
        elif v2:
            cls = CLASSES[int(rng.choice(len(CLASSES), p=weights / weights.sum()))]
        else:                                                    # v1: the original random-number stream (reproducible dataset)
            cls = CLASSES[int(rng.integers(len(CLASSES)))]
        scale = float(rng.uniform(0.8, 1.2))
        var = O.random_variant(cls, rng)
        fp = O.footprint(cls, scale, var)
        xy = place(0.5 * max(fp[0], fp[1]))
        if xy:
            sc["objects"].append({"cls": cls, "name": f"obj{len(sc['objects'])}_{cls}", "x": xy[0], "y": xy[1],
                                  "yaw": float(rng.uniform(-math.pi, math.pi)), "scale": scale, "variant": var,
                                  "height": fp[2]})
    sc["distractors"] = []
    if v2:
        for i in range(int(rng.integers(2, 11))):
            kind = O.DISTRACTOR_KINDS[int(rng.integers(len(O.DISTRACTOR_KINDS)))]
            pl = O.distractor_parts(kind, rng)
            xy = place(O.parts_radius(pl))
            if xy:
                sc["distractors"].append({"name": f"dis{i}", "kind": kind, "x": xy[0], "y": xy[1], "yaw": float(rng.uniform(-3.1, 3.1)),
                                          "parts": pl, "part": pl[0]})
    for i in range(0 if v2 else int(rng.integers(0, 9))):
        kind = ("box", "cylinder", "sphere")[int(rng.integers(3))]
        size = {"box": [float(rng.uniform(0.1, 0.6)) for _ in range(3)], "cylinder": [float(rng.uniform(0.05, 0.3)), float(rng.uniform(0.2, 1.2))],
                "sphere": [float(rng.uniform(0.08, 0.3))]}[kind]
        r = 0.5 * max(size[0], size[1] if kind != "sphere" else size[0]) * (2 if kind != "box" else 1)
        xy = place(r)
        if xy:
            part = {"kind": kind, "size": size, "pos": [0, 0, (size[2] if kind == "box" else size[1] if kind == "cylinder" else size[0] * 2) / 2],
                    "rgb": _rand_rgb(rng)}
            sc["distractors"].append({"name": f"dis{i}", "x": xy[0], "y": xy[1], "yaw": float(rng.uniform(-3.1, 3.1)), "part": part})
    sc["views"] = sample_views(sc, rng, n_views)
    return sc


def sample_views(sc: dict, rng, n: int) -> list:
    """Camera poses: ~85 % aimed so a labelled object is in view (0.5-3.5 m away), the rest random (negatives)."""
    W, D = sc["room"]["w"], sc["room"]["d"]
    cam = sc["camera"]
    obstacles = [(o["x"], o["y"]) for o in sc["objects"]] + [(d["x"], d["y"]) for d in sc["distractors"]]
    views = []
    for _ in range(n):
        for _try in range(40):
            if sc.get("profile") == "v2" and sc["distractors"] and rng.random() < 0.25:
                o = sc["distractors"][int(rng.integers(len(sc["distractors"])))]
                th, dist = float(rng.uniform(-math.pi, math.pi)), float(rng.uniform(0.5, 3.0))
                x, y = o["x"] - dist * math.cos(th), o["y"] - dist * math.sin(th)
                yaw = th + float(rng.uniform(-0.35, 0.35))
            elif sc["objects"] and rng.random() < 0.85:
                mann = [q for q in sc["objects"] if q["cls"] == "mannequin"]
                if sc.get("profile") == "v2" and mann and rng.random() < 0.4:
                    o = mann[int(rng.integers(len(mann)))]
                else:
                    o = sc["objects"][int(rng.integers(len(sc["objects"])))]
                th, dist = float(rng.uniform(-math.pi, math.pi)), float(rng.uniform(0.5, 3.5))
                x, y = o["x"] - dist * math.cos(th), o["y"] - dist * math.sin(th)
                yaw = th + float(rng.uniform(-0.35, 0.35))
            else:
                x, y, yaw = float(rng.uniform(-W / 2, W / 2)), float(rng.uniform(-D / 2, D / 2)), float(rng.uniform(-math.pi, math.pi))
            if abs(x) > W / 2 - 0.3 or abs(y) > D / 2 - 0.3:
                continue
            if any(math.hypot(x - a, y - b) < 0.4 for a, b in obstacles):
                continue
            z = float(rng.uniform(0.10, 0.30))
            pitch = float(cam["pitch_down"] + rng.normal(0, 0.07))
            views.append({"pos": [x, y, z], "rpy": [float(rng.normal(0, 0.03)), pitch, yaw]})
            break
    return views


def gt_for_view(sc: dict, view: dict) -> list:
    """Ground-truth object centres in the camera optical frame (+ pixel of the centre) for objects in front of the camera."""
    out = []
    cam = sc["camera"]
    for o in sc["objects"]:
        p = optical_from_world([o["x"], o["y"], o["height"] / 2], view["pos"], view["rpy"])
        uv = project(p, cam["hfov"], cam["width"], cam["height"])
        if uv and 0 <= uv[0] < cam["width"] and 0 <= uv[1] < cam["height"]:
            out.append({"cls": o["cls"], "name": o["name"], "p_cam": [float(v) for v in p], "uv": [float(uv[0]), float(uv[1])],
                        "p_world": [o["x"], o["y"], o["height"] / 2]})
    return out


def _tex_ref(tex_dir: str | None, idx):
    if idx is None or tex_dir is None:
        return ""
    name = f"tex_{idx:02d}_{KINDS[idx % len(KINDS)]}.png"
    return f"<pbr><metal><albedo_map>{Path(tex_dir).resolve() / name}</albedo_map><roughness>0.9</roughness><metalness>0.0</metalness></metal></pbr>"


def _box_model(name, cx, cy, cz, sx, sy, sz, rgb, tex):
    r, g, b = rgb
    return (f'    <model name="{name}"><static>true</static><pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose><link name="link">'
            f'<visual name="v"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'
            f'<material><ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient><diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>{tex}</material></visual>'
            f'</link></model>\n')


def camera_model_sdf(cam: dict, pose, rate_hz: float = 10.0) -> str:
    w, h, fov, (near, far) = cam["width"], cam["height"], cam["hfov"], cam["range"]
    common = (f"<horizontal_fov>{fov}</horizontal_fov><image><width>{w}</width><height>{h}</height>%s</image>"
              f"<clip><near>{near}</near><far>{far}</far></clip>")
    return f"""    <model name="cam"><static>true</static><pose>{pose[0]:.3f} {pose[1]:.3f} {pose[2]:.3f} {pose[3]:.4f} {pose[4]:.4f} {pose[5]:.4f}</pose>
      <link name="link">
        <sensor name="rgbd" type="rgbd_camera"><topic>cam</topic><update_rate>{rate_hz}</update_rate><always_on>true</always_on>
          <camera>{common % "<format>R8G8B8</format>"}</camera></sensor>
        <sensor name="bbox" type="boundingbox_camera"><topic>cam/bbox</topic><update_rate>{rate_hz}</update_rate><always_on>true</always_on>
          <camera><box_type>visible_2d</box_type>{common % ""}</camera></sensor>
      </link>
    </model>
"""


def world_sdf(sc: dict, mesh_dir: str, tex_dir: str | None = None, with_camera: bool = True) -> str:
    """Complete world: room, lights, labelled objects, distractors, one movable camera model named `cam`."""
    W, D, H = sc["room"]["w"], sc["room"]["d"], sc["room"]["h"]
    s = sc["sun"]
    a = sc["ambient"]
    sun_c = [min(1.0, c * s["intensity"]) for c in s["rgb"]]      # gz light colours must be in [0, 1]
    parts = [f"""<?xml version="1.0"?>
<!-- GENERATED by vision/scene.py (seed {sc['seed']}) -->
<sdf version="1.10"><world name="{WORLD_NAME}">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <scene><ambient>{a:.2f} {a:.2f} {a:.2f} 1</ambient><background>0.5 0.5 0.55 1</background><shadows>{str(sc['shadows']).lower()}</shadows></scene>
    <light type="directional" name="sun"><cast_shadows>{str(sc['shadows']).lower()}</cast_shadows><pose>0 0 8 0 0 0</pose>
      <diffuse>{sun_c[0]:.2f} {sun_c[1]:.2f} {sun_c[2]:.2f} 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular><direction>{s['dir'][0]:.2f} {s['dir'][1]:.2f} {s['dir'][2]:.2f}</direction></light>
"""]
    for i, l in enumerate(sc["lamps"]):
        r, g, b = (min(1.0, c * l["intensity"]) for c in l["rgb"])       # gz light colours must be in [0, 1] (v3 scenes had 1.02)
        parts.append(f'    <light type="point" name="lamp{i}"><pose>{l["pos"][0]:.2f} {l["pos"][1]:.2f} {l["pos"][2]:.2f} 0 0 0</pose>'
                     f'<diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse><specular>0.05 0.05 0.05 1</specular><attenuation><range>8</range>'
                     f'<constant>0.4</constant><linear>0.1</linear><quadratic>0.02</quadratic></attenuation><cast_shadows>false</cast_shadows></light>\n')
    parts.append(_box_model("floor", 0, 0, -0.025, W + 0.4, D + 0.4, 0.05, sc["floor"]["rgb"], _tex_ref(tex_dir, sc["floor"]["texture"])))
    t, wt = 0.1, _tex_ref(tex_dir, sc["wall"]["texture"])
    for n, (cx, cy, sx, sy) in {"wn": (0, D / 2 + t / 2, W + 2 * t, t), "ws": (0, -D / 2 - t / 2, W + 2 * t, t),
                                "we": (W / 2 + t / 2, 0, t, D), "ww": (-W / 2 - t / 2, 0, t, D)}.items():
        parts.append(_box_model(n, cx, cy, H / 2, sx, sy, H, sc["wall"]["rgb"], wt))
    for o in sc["objects"]:
        cid = CLASSES.index(o["cls"]) + GZ_LABEL_OFFSET
        parts.append(O.model_sdf(o["name"], O.parts(o["cls"], o["scale"], o["variant"]), o["x"], o["y"], o["yaw"], label=cid, mesh_dir=mesh_dir))
    for d in sc["distractors"]:
        parts.append(O.model_sdf(d["name"], d.get("parts") or [d["part"]], d["x"], d["y"], d["yaw"], label=None, mesh_dir=mesh_dir))
    if with_camera:
        v0 = sc["views"][0] if sc["views"] else {"pos": [0, 0, 0.2], "rpy": [0, 0.17, 0]}
        parts.append(camera_model_sdf(sc["camera"], [*v0["pos"], *v0["rpy"]]))
    parts.append("  </world>\n</sdf>\n")
    return "".join(parts)


def yolo_lines(boxes: list, w: int, h: int, min_px: float = 6.0, min_area: float = 80.0) -> list:
    """[(class_id, x1, y1, x2, y2)] in pixels -> YOLO txt lines 'cls cx cy bw bh' (normalised); drops tiny/degenerate boxes."""
    out = []
    for cid, x1, y1, x2, y2 in boxes:
        x1, x2, y1, y2 = max(0.0, x1), min(float(w), x2), max(0.0, y1), min(float(h), y2)
        bw, bh = x2 - x1, y2 - y1
        if bw < min_px or bh < min_px or bw * bh < min_area:
            continue
        out.append(f"{cid} {(x1 + x2) / 2 / w:.6f} {(y1 + y2) / 2 / h:.6f} {bw / w:.6f} {bh / h:.6f}")
    return out
