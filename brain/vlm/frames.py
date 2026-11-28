"""Camera frames for scene questions: the image (PNG bytes, optional) + what the robot's own perception says about that frame
(the FACTS the VLM's answer is checked against) + where the camera looked.

Sources:
  * `frames_from_sightings(path)`: the detector's per-frame output saved by the Gazebo runs (`sightings.jsonl`, detector_node
    `detector_log`): one frame per timestamp; detections = class, colour, map position, pixel box; cam = camera pose in the map
    frame (x, y, z, qx, qy, qz, qw; optical frame, z forward). These runs saved no RGB, so image = None.
  * `frame_from_render(png, meta, scene)`: a frame rendered in Gazebo for the detector datasets (vision/generate_dataset.py):
    RGB + per-image GT (class, world position) + the scene's object colours (SDF rgb -> colour name). GT = a perfect detector.
"""
from __future__ import annotations

import json
import math
from collections import OrderedDict
from dataclasses import dataclass, field
from pathlib import Path

@dataclass
class Frame:
    t: float
    detections: list = field(default_factory=list)     # [{cls, color, x, y, z?, score?, box?, truncated?}]
    cam: tuple | None = None                            # (x, y, z, qx, qy, qz, qw) map frame, optical (z forward); or (x, y, yaw)
    image: bytes | None = None                          # PNG
    source: str = ""

    def cam_xy_yaw(self):
        """(x, y, yaw) of the camera's viewing direction in the map frame, or None."""
        c = self.cam
        if c is None:
            return None
        if len(c) == 3:
            return tuple(c)
        x, y, _, qx, qy, qz, qw = c
        fx = 2 * (qx * qz + qw * qy)                      # R @ (0, 0, 1): the optical z axis
        fy = 2 * (qy * qz - qw * qx)
        return (x, y, math.atan2(fy, fx))


def frames_from_sightings(path) -> list:
    """sightings.jsonl -> [Frame] in time order (detections with the same timestamp = one camera frame)."""
    by_t: "OrderedDict[float, Frame]" = OrderedDict()
    for line in Path(path).read_text().splitlines():
        if not line.strip():
            continue
        d = json.loads(line)
        f = by_t.setdefault(d["t"], Frame(t=d["t"], cam=tuple(d["cam"]) if d.get("cam") else None, source=str(path)))
        p = d.get("p") or [None, None, None]
        f.detections.append({"cls": d["cls"], "color": d.get("color") or "", "x": p[0], "y": p[1], "z": p[2], "score": d.get("score"),
                             "box": d.get("box"), "truncated": d.get("truncated", False), "range": d.get("range")})
    return list(by_t.values())


def rgb_to_colour(rgb) -> str:
    """An SDF colour (0-1 floats) -> one of the detector's colour words (gurdy_core.vision.colors)."""
    import numpy as np
    from gurdy_core.vision.colors import color_name
    px = np.clip(np.array(rgb[:3], dtype=float) * 255.0, 0, 255).astype(np.uint8).reshape(1, 1, 3)
    return color_name(np.repeat(np.repeat(px, 8, 0), 8, 1))


def frame_from_render(png, meta, scene=None) -> Frame:
    m = json.loads(Path(meta).read_text())
    colours = {}
    if scene is not None:
        sc = json.loads(Path(scene).read_text())
        for o in sc.get("objects", []):                  # big objects: the SDF colour of the variant (none = the class default)
            rgb = (o.get("variant") or {}).get("rgb")
            if rgb:
                colours[o["name"]] = rgb_to_colour(rgb)
        for o in sc.get("small", []):                    # small objects: a colour word
            if o.get("colour"):
                colours[o["name"]] = o["colour"]
    dets = []
    for g in m["gt"]:
        if g.get("kind", "object") not in ("object", "small"):
            continue
        w = g.get("p_world") or [None, None, None]
        dets.append({"cls": g["cls"], "color": colours.get(g.get("name"), ""), "x": w[0], "y": w[1], "z": w[2], "name": g.get("name")})
    cx, cy, _ = m["cam_pos"]
    yaw = m["cam_rpy"][2]
    return Frame(t=m.get("stamp_ns", 0) / 1e9, detections=dets, cam=(cx, cy, yaw), image=Path(png).read_bytes(), source=str(png))


def frames_from_saved(dir_) -> list:
    """Frames saved by a Gazebo run with `detector_frames:=DIR` (gurdy_core/vision/frame_saver.py): image + sidecar
    (camera pose, the detections of that frame in the sightings.jsonl fields) -> [Frame] in time order."""
    out = []
    for side in sorted(Path(dir_).glob("f_*.json")):
        d = json.loads(side.read_text())
        img = next((side.with_suffix(s) for s in (".jpg", ".png") if side.with_suffix(s).exists()), None)
        dets = [{"cls": x["cls"], "color": x.get("color") or "", "x": (x.get("p") or [None])[0], "y": (x.get("p") or [None, None])[1],
                 "z": (x.get("p") or [None, None, None])[2], "score": x.get("score"), "box": x.get("box"), "truncated": x.get("truncated", False)}
                for x in d.get("detections", [])]
        out.append(Frame(t=d["t"], detections=dets, cam=tuple(d["cam"]) if d.get("cam") else None,
                         image=img.read_bytes() if img else None, source=str(img or side)))
    return out
