"""Save body-camera RGB frames during Gazebo runs, for VLM replay (brain/vlm, docs/results/vlm_plan.md). Pure Python (numpy +
cv2 or PIL, whichever exists); used by gurdy_perception/detector_node when its `frame_dir` parameter is set (default "" = off).

Per saved frame, in `out_dir`:
  f_<t sim s, 3 decimals>.jpg   the RGB image (JPEG, quality 92; png=True for lossless)
  f_<t>.json                    {"t", "wh", "cam": [x, y, z, qx, qy, qz, qw] (camera optical frame in the map frame, or null),
                                 "detections": [{"cls", "score", "box", "color", "p", "truncated"}]}   (same fields as sightings.jsonl)
Limits: at most one frame per `every_s` SIM seconds, at most `max_frames` frames, and saving stops (once, logged by the caller)
when the disk has less than `min_free_gb` free -- a full disk crashed WSL once (2026-11-18).
"""
from __future__ import annotations

import json
import os
import shutil
from pathlib import Path

import numpy as np


def _encode(rgb: np.ndarray, png: bool, quality: int) -> bytes:
    try:
        import cv2
        ok, buf = cv2.imencode(".png" if png else ".jpg", np.ascontiguousarray(rgb[..., ::-1]),
                               [] if png else [cv2.IMWRITE_JPEG_QUALITY, quality])
        if ok:
            return buf.tobytes()
    except ImportError:
        pass
    import io
    from PIL import Image
    b = io.BytesIO()
    Image.fromarray(np.ascontiguousarray(rgb)).save(b, format="PNG" if png else "JPEG", quality=quality)
    return b.getvalue()


class FrameSaver:
    def __init__(self, out_dir, every_s: float = 1.0, max_frames: int = 3000, min_free_gb: float = 5.0, png: bool = False,
                 quality: int = 92, free_gb=None):
        self.dir = Path(out_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.every_s, self.max_frames, self.min_free_gb, self.png, self.quality = every_s, max_frames, min_free_gb, png, quality
        self.free_gb = free_gb or (lambda: shutil.disk_usage(self.dir).free / 1e9)
        self.last_t = None
        self.saved = 0
        self.stopped: str | None = None          # why saving stopped (max frames / disk), for the caller's log

    def maybe_save(self, t: float, rgb: np.ndarray, cam=None, detections=()) -> str | None:
        """-> the image path when this frame was saved, else None."""
        if self.stopped:
            return None
        if self.last_t is not None and t - self.last_t < self.every_s and t >= self.last_t:
            return None
        if self.saved >= self.max_frames:
            self.stopped = f"max_frames {self.max_frames} reached"
            return None
        if self.free_gb() < self.min_free_gb:
            self.stopped = f"less than {self.min_free_gb} GB free"
            return None
        self.last_t = t
        stem = f"f_{t:011.3f}"
        img = self.dir / (stem + (".png" if self.png else ".jpg"))
        tmp = img.with_suffix(img.suffix + ".tmp")
        tmp.write_bytes(_encode(rgb, self.png, self.quality))
        os.replace(tmp, img)
        side = {"t": round(float(t), 3), "wh": [int(rgb.shape[1]), int(rgb.shape[0])],
                "cam": [round(float(c), 4) for c in cam] if cam is not None else None, "detections": list(detections)}
        (self.dir / (stem + ".json")).write_text(json.dumps(side))
        self.saved += 1
        return str(img)
