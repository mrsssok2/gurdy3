"""describe_view(question): the scene-question tool (docs/results/vlm_plan.md section 2). Not in the planner's tool list (the
system prompt stays byte-identical); the agent routes scene questions here deterministically when `Agent(vlm=DescribeView(...))`.

  frame  = frame_source() -> brain.vlm.frames.Frame (image + this frame's detections + camera pose), at most max_age_s old
  facts  = this frame's detections + the backend's semantic-map objects inside the camera cone (range_m, +-half_fov_deg),
           de-duplicated (same class within 0.6 m); a detection without a position is kept (class only)
  path   = for "is the path to X clear": backend.path_check(place=X) + whether the route leaves in the camera's direction
  vlm    = vlm(image, question) -> raw text (client.OllamaVLM / ScriptedVLM); parsed by client.parse
  reply  = ground.answer(...), then ground.check_reply as a last guard (a sentence naming an unsupported object without a hedge
           -> the facts-only reply)
"""
from __future__ import annotations

import math
import time

from . import client as C
from . import ground as G
from .questions import classify


class DescribeView:
    def __init__(self, backend, vlm=None, frame_source=None, max_age_s: float = 2.0, range_m: float = 3.5, half_fov_deg: float = 45.0,
                 now=None):
        self.b, self.vlm, self.frame_source = backend, vlm, frame_source
        self.max_age_s, self.range_m, self.half_fov = max_age_s, range_m, math.radians(half_fov_deg)
        self.now = now or (lambda: backend.now() if callable(getattr(backend, "now", None)) else time.time())
        self.last: dict | None = None

    # ---- facts
    def _map_in_cone(self, cam) -> list:
        tracks = []
        if callable(getattr(self.b, "_tracks", None)):
            tracks = self.b._tracks()
        elif isinstance(getattr(self.b, "tracks", None), dict):
            tracks = [t for t in self.b.tracks.values() if t.get("state", "present") == "present"]
        out = []
        if cam is None:
            return out
        x, y, yaw = cam
        for t in tracks:
            d = math.hypot(t["x"] - x, t["y"] - y)
            b = math.atan2(math.sin(math.atan2(t["y"] - y, t["x"] - x) - yaw), math.cos(math.atan2(t["y"] - y, t["x"] - x) - yaw))
            if d <= self.range_m and abs(b) <= self.half_fov:
                out.append({"cls": t["class"], "color": t.get("color") or "", "x": t["x"], "y": t["y"], "source": "map", "id": t.get("id")})
        return out

    def facts(self, frame) -> list:
        cam = frame.cam_xy_yaw() if frame is not None else None
        out = [{**d, "source": "detector"} for d in (frame.detections if frame else [])]
        for m in self._map_in_cone(cam):
            dup = next((f for f in out if f["cls"] == m["cls"] and (f.get("x") is None or math.hypot(f["x"] - m["x"], f["y"] - m["y"]) <= 0.6)), None)
            if dup is None:
                out.append(m)
            elif dup.get("x") is None:                    # a 2D detection: the map gives it a position and a colour
                dup.update(x=m["x"], y=m["y"], color=dup.get("color") or m["color"])
        return out

    def _path(self, q, cam) -> dict:
        place = q.get("place")
        if not place:
            return {"reachable": None}
        known = {p.lower(): p for p in self.b.known_places()} if callable(getattr(self.b, "known_places", None)) else {}
        names = getattr(self.b, "room_names", None) or {}
        key = place.lower().removeprefix("the ").strip()
        real = known.get(key) or known.get(f"room {key}") or names.get(key)
        if real is None:
            return {"unknown_place": f"the {key}" if not key.startswith("room ") else key}
        label = real + (f" (the {key})" if key in names else "")
        r = self.b.path_check(place=real)
        if not r.ok:
            return {"reachable": None, "label": label}
        d = r.data
        in_view = False
        tgt = d.get("target") or {}
        if cam is not None and tgt.get("x") is not None:
            pts = d.get("points") or [(tgt["x"], tgt["y"])]
            px, py = next(((a, b) for a, b in pts if math.hypot(a - cam[0], b - cam[1]) > 0.5), pts[-1])
            b = math.atan2(math.sin(math.atan2(py - cam[1], px - cam[0]) - cam[2]), math.cos(math.atan2(py - cam[1], px - cam[0]) - cam[2]))
            in_view = abs(b) <= self.half_fov
        return {"reachable": bool(d.get("reachable")), "blocked_by": d.get("blocked_by"), "label": label, "in_view": in_view}

    # ---- the tool
    def __call__(self, question: str, q: dict | None = None) -> dict:
        q = q or classify(question)
        if q is None:
            return {"routed": False}
        frame = self.frame_source() if self.frame_source else None
        stale = frame is None or (frame.t is not None and self.now() - frame.t > self.max_age_s)
        cam = (frame.cam_xy_yaw() if frame else None) or (self.b._pose() if callable(getattr(self.b, "_pose", None)) else None)
        rec = {"routed": True, "question": question, "kind": q["kind"], "q": q, "frame_t": frame.t if frame else None, "stale": stale}
        if stale and q["kind"] != "path":
            v = G.Verdict("no_view", "I don't have a current camera view, so I can't answer that right now.")
            rec.update(status=v.status, reply=v.reply, verdict=v, facts=[], vlm_raw=None)
            self.last = rec
            return rec
        facts = self.facts(None if stale else frame)
        path = self._path(q, cam) if q["kind"] == "path" else None
        raw, vlm, why = None, None, "isn't available"
        if self.vlm is not None and not stale and frame.image:
            try:
                raw = self.vlm(frame.image, question)
                vlm = C.parse(raw)
                why = "gave no usable answer" if vlm is None else why
            except Exception as e:                       # noqa: BLE001  model down / timeout -> facts only
                why = "isn't available"
                rec["vlm_error"] = repr(e)
        elif self.vlm is not None and (stale or not frame.image):
            why = "had no image"
        v = G.answer(q, facts, vlm, path, why)
        extra = [w for k, w, c in v.vlm_only if k == "outside"]
        bad = G.check_reply(v.reply, facts)
        if bad:                                          # a template slipped: never speak an unsupported object
            v = G.answer(q, facts, None, path, "gave an answer I couldn't check")
            rec["guard"] = bad
        rec.update(status=v.status, reply=v.reply, verdict=v, facts=facts, vlm_raw=raw, vlm=vlm, path=path, extra_nouns=extra)
        self.last = rec
        return rec


def ros_frame_source(extras, backend, image_max_age_s: float = 2.0):
    """RosExtras (web/api/ros_extras.py: latest /camera/image as JPEG + /detector/boxes) + the robot pose -> Frame.
    Boxes have class + score only: their position / colour come from the semantic map (DescribeView.facts). NOT run in ROS yet."""
    from .frames import Frame

    def src():
        c = extras.camera()
        if c is None:
            return None
        pose = backend._pose()
        dets = [{"cls": b["name"], "color": "", "x": None, "y": None, "score": b.get("score"), "box": b.get("box")} for b in c.get("boxes", [])]
        return Frame(t=backend.now(), detections=dets, cam=tuple(pose) if pose else None, image=c["jpeg"], source="ros")
    return src
