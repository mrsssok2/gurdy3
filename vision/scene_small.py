"""Phase 9b dataset profiles "v3-body" / "v3-wrist": the v2 scenes (7 old classes, distractor families) PLUS the five small
pickable classes (vision/small_objects.py), for the 12-class v3 detector (gurdy_core.vision.classes.CLASSES_V3).

* v3-body:  robot body camera (robot.yaml depth_camera, 320 x 240, hfov 1.047), camera 0.10-0.30 m high like v1/v2;
            ~70 % of the views aimed at a small object 0.3-2.0 m away (smaller objects are < ~8 px beyond 2 m),
            ~20 % at an old-class object 0.5-3.5 m (keeps the old classes in view), ~10 % random.
* v3-wrist: wrist camera (arm.yaml wrist_camera, 320 x 240, hfov 1.2, 0.07-1.0 m). Views: top-down / oblique above a small
            object (camera 0.10-0.45 m above the floor, pitch 35-90 deg down) and low side views (0.05-0.15 m, the side-grasp
            approach). The two gripper fingers are part of the camera model (visual only, open 3-8 cm) at the arm's real
            geometry. NOTE: they are 2.5-6.5 cm ahead of the lens, i.e. inside the 0.07 m near clip, so Gazebo clips them
            (same in the arm model, so dataset and robot match); they appear only if the RGB near clip is lowered later.
Both: 2-6 small labelled objects (random class, colour from the class's variants, yaw), 0-5 small UNLABELLED distractors
of other shapes (balls, flat discs, thin rods, tall thin bottles, flat plates), the v2 lighting plus 30 % dim scenes and
20 % with a strong coloured lamp. Small objects are static in the dataset worlds (no physics needed for renders).
Pure Python / numpy (no Gazebo); world SDF via vision.scene.world_sdf for the v2 part.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml

from gurdy_core.vision.classes import CLASSES, CLASSES_V3, GZ_LABEL_OFFSET, SMALL_CLASSES
from vision import scene as S
from vision import small_objects as SO

ROOT = Path(__file__).resolve().parents[1]
PROFILES = ("v3-body", "v3-wrist")


def wrist_camera(arm_yaml: Path | None = None) -> dict:
    d = yaml.safe_load(open(arm_yaml or ROOT / "ros2_ws/src/gurdy_description/config/arm.yaml"))["wrist_camera"]
    return {"width": d["width"], "height": d["height"], "hfov": d["hfov"], "range": list(d["range"]), "pitch_down": 0.9,
            "wrist": True}


def _small_distractor(rng):
    k = ("ball", "disc", "rod", "bottle", "plate")[int(rng.integers(5))]
    rgb = [float(v) for v in rng.uniform(0.1, 0.95, 3)]
    if k == "ball":
        r = float(rng.uniform(0.015, 0.05))
        return [{"kind": "sphere", "size": [r], "pos": [0, 0, r], "rgb": rgb}], r
    if k == "disc":
        r, h = float(rng.uniform(0.03, 0.08)), float(rng.uniform(0.004, 0.015))
        return [{"kind": "cylinder", "size": [r, h], "pos": [0, 0, h / 2], "rgb": rgb}], r
    if k == "rod":
        L, t = float(rng.uniform(0.15, 0.35)), float(rng.uniform(0.008, 0.02))
        return [{"kind": "box", "size": [L, t, t], "pos": [0, 0, t / 2], "rgb": rgb}], L / 2
    if k == "bottle":      # taller and thinner than a can: r 1.2-2 cm, h 0.18-0.28 m (+ neck)
        r, h = float(rng.uniform(0.012, 0.02)), float(rng.uniform(0.18, 0.28))
        return [{"kind": "cylinder", "size": [r, h], "pos": [0, 0, h / 2], "rgb": rgb},
                {"kind": "cylinder", "size": [r * 0.45, 0.04], "pos": [0, 0, h + 0.02], "rgb": rgb}], r
    sx, sy, h = float(rng.uniform(0.08, 0.2)), float(rng.uniform(0.08, 0.2)), float(rng.uniform(0.003, 0.012))
    return [{"kind": "box", "size": [sx, sy, h], "pos": [0, 0, h / 2], "rgb": rgb}], 0.5 * math.hypot(sx, sy)


def sample_scene_v3(seed: int, n_views: int, profile: str, body_camera: dict | None = None,
                    wrist_cam: dict | None = None) -> dict:
    assert profile in PROFILES
    sc = S.sample_scene(seed, 0, body_camera, "v2")            # room, lights, old-class objects, distractor families
    rng = np.random.default_rng(seed + 7_000_003)
    sc["profile"] = profile
    if profile == "v3-wrist":
        sc["camera"] = {**S.DEFAULT_CAMERA, **(wrist_cam or wrist_camera())}
        sc["fingers_open"] = float(rng.uniform(0.03, 0.08))
    if rng.random() < 0.3:                                    # dim scene
        sc["ambient"] = float(rng.uniform(0.05, 0.15))
        sc["sun"]["intensity"] = float(rng.uniform(0.15, 0.5))
    if rng.random() < 0.2:                                    # strong coloured lamp
        W, D = sc["room"]["w"], sc["room"]["d"]
        sc["lamps"].append({"pos": [float(rng.uniform(-W / 3, W / 3)), float(rng.uniform(-D / 3, D / 3)), float(rng.uniform(0.8, 2.0))],
                            "intensity": float(rng.uniform(0.8, 1.5)), "rgb": [float(v) for v in rng.uniform(0.5, 1.0, 3)]})
    W, D = sc["room"]["w"], sc["room"]["d"]
    taken = [(o["x"], o["y"], 0.5 * max(o.get("height", 0.3), 0.3)) for o in sc["objects"]] + \
            [(d["x"], d["y"], 0.3) for d in sc["distractors"]]

    def place(r, tries=80):
        for _ in range(tries):
            x, y = float(rng.uniform(-W / 2 + 0.4, W / 2 - 0.4)), float(rng.uniform(-D / 2 + 0.4, D / 2 - 0.4))
            if all(math.hypot(x - a, y - b) > r + rr + 0.08 for a, b, rr in taken):
                taken.append((x, y, r))
                return x, y
        return None

    sc["small"] = []
    for _ in range(int(rng.integers(2, 7))):
        cls = SMALL_CLASSES[int(rng.integers(len(SMALL_CLASSES)))]
        cat = SO.CATALOG[cls]
        xy = place(0.5 * math.hypot(*cat["size"][:2]))
        if xy:
            sc["small"].append({"cls": cls, "name": f"sm{len(sc['small'])}_{cls}", "x": xy[0], "y": xy[1],
                                "yaw": float(rng.uniform(-math.pi, math.pi)),
                                "colour": cat["variants"][int(rng.integers(len(cat["variants"])))], "height": cat["size"][2]})
    sc["small_distractors"] = []
    for i in range(int(rng.integers(0, 6))):
        parts, r = _small_distractor(rng)
        xy = place(r)
        if xy:
            sc["small_distractors"].append({"name": f"sd{i}", "x": xy[0], "y": xy[1], "yaw": float(rng.uniform(-3.1, 3.1)), "parts": parts})
    sc["views"] = sample_views_v3(sc, rng, n_views)
    return sc


def _free(sc, x, y, r=0.12):
    W, D = sc["room"]["w"], sc["room"]["d"]
    if abs(x) > W / 2 - 0.15 or abs(y) > D / 2 - 0.15:
        return False
    obst = [(o["x"], o["y"], 0.3) for o in sc["objects"]] + [(d["x"], d["y"], 0.25) for d in sc["distractors"]]
    obst += [(o["x"], o["y"], 0.05) for o in sc["small"]]
    return all(math.hypot(x - a, y - b) > rr + r for a, b, rr in obst)


def sample_views_v3(sc, rng, n):
    views, wrist = [], sc["profile"] == "v3-wrist"
    for _ in range(n):
        for _try in range(60):
            u = rng.random()
            if sc["small"] and (wrist or u < 0.7):
                o = sc["small"][int(rng.integers(len(sc["small"])))]
                th = float(rng.uniform(-math.pi, math.pi))
                if wrist and rng.random() < 0.7:                       # above / oblique (top-down grasp, final approach)
                    z = float(rng.uniform(0.10, 0.45))
                    pitch = float(rng.uniform(math.radians(35), math.radians(90)))
                    h = z - o["height"] * 0.5
                    dist = max(0.0, h / math.tan(pitch)) + float(rng.normal(0, 0.03))
                elif wrist:                                            # low side view (side-grasp approach)
                    z, pitch, dist = float(rng.uniform(0.05, 0.15)), float(rng.uniform(0.0, 0.35)), float(rng.uniform(0.10, 0.45))
                else:                                                  # body camera
                    z, pitch, dist = float(rng.uniform(0.10, 0.30)), float(sc["camera"]["pitch_down"] + rng.normal(0, 0.07)), \
                        float(rng.uniform(0.3, 2.0))
                x, y = o["x"] - dist * math.cos(th), o["y"] - dist * math.sin(th)
                yaw = th + float(rng.uniform(-0.3, 0.3))
            elif sc["objects"] and u < 0.9:
                o = sc["objects"][int(rng.integers(len(sc["objects"])))]
                th, dist = float(rng.uniform(-math.pi, math.pi)), float(rng.uniform(0.5, 3.5))
                x, y = o["x"] - dist * math.cos(th), o["y"] - dist * math.sin(th)
                yaw, z = th + float(rng.uniform(-0.35, 0.35)), float(rng.uniform(0.10, 0.30))
                pitch = float(sc["camera"]["pitch_down"] + rng.normal(0, 0.07))
            else:
                W, D = sc["room"]["w"], sc["room"]["d"]
                x, y, yaw = float(rng.uniform(-W / 2, W / 2)), float(rng.uniform(-D / 2, D / 2)), float(rng.uniform(-math.pi, math.pi))
                z, pitch = float(rng.uniform(0.10, 0.30)), float(sc["camera"]["pitch_down"] + rng.normal(0, 0.07))
            if not _free(sc, x, y, 0.06 if wrist else 0.25):
                continue
            views.append({"pos": [x, y, z], "rpy": [float(rng.normal(0, 0.03)), pitch, yaw]})
            break
    return views


def gt_for_view_v3(sc, view):
    """Old-class + small-object centres in the optical frame (same format as vision.scene.gt_for_view)."""
    out = S.gt_for_view(sc, view)
    cam = sc["camera"]
    for o in sc["small"]:
        p = S.optical_from_world([o["x"], o["y"], o["height"] / 2], view["pos"], view["rpy"])
        uv = S.project(p, cam["hfov"], cam["width"], cam["height"])
        if uv and 0 <= uv[0] < cam["width"] and 0 <= uv[1] < cam["height"]:
            out.append({"cls": o["cls"], "name": o["name"], "p_cam": [float(v) for v in p], "uv": [float(uv[0]), float(uv[1])],
                        "p_world": [o["x"], o["y"], o["height"] / 2]})
    return out


def finger_visuals(opening: float) -> str:
    """Two gripper fingers in the CAMERA link frame (x forward, y left, z up): arm.yaml geometry, camera 1 cm behind the
    palm front and 2.2 cm above the jaw plane; fingers 2.5-6.5 cm ahead of the lens. Visual only."""
    out = []
    for i, s in enumerate((1, -1)):
        y = s * (opening / 2 + 0.004)
        out.append(f'<visual name="finger{i}"><pose>0.045 {y:.4f} -0.022 0 0 0</pose><geometry><box><size>0.04 0.008 0.012</size></box></geometry>'
                   f'<material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.85 0.85 0.85 1</diffuse></material></visual>')
    return "".join(out)


def world_sdf_v3(sc: dict, mesh_dir: str, tex_dir: str | None = None) -> str:
    base = S.world_sdf({**sc, "views": []}, mesh_dir, tex_dir, with_camera=False)
    parts = []
    for o in sc["small"]:
        cid = CLASSES_V3.index(o["cls"]) + GZ_LABEL_OFFSET
        parts.append(SO.model_sdf(o["name"], o["cls"], o["colour"], o["x"], o["y"], o["yaw"], label=cid, static=True, mesh_dir=mesh_dir))
    from vision import objects as O
    for d in sc["small_distractors"]:
        parts.append(O.model_sdf(d["name"], d["parts"], d["x"], d["y"], d["yaw"], label=None, mesh_dir=mesh_dir))
    v0 = sc["views"][0] if sc["views"] else {"pos": [0, 0, 0.2], "rpy": [0, 0.17, 0]}
    cam = S.camera_model_sdf(sc["camera"], [*v0["pos"], *v0["rpy"]])
    if sc["camera"].get("wrist"):
        cam = cam.replace('<link name="link">', '<link name="link">' + finger_visuals(sc.get("fingers_open", 0.06)), 1)
    return base.replace("  </world>", "".join(parts) + cam + "  </world>")


def n_classes(profile: str) -> int:
    return len(CLASSES_V3) if str(profile).startswith("v3") else len(CLASSES)


def class_names(profile: str):
    return CLASSES_V3 if str(profile).startswith("v3") else CLASSES
