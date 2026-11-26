"""Synthetic depth images for tests (numpy ray casting, no Gazebo).

Scene in the map/world frame: floor plane z = 0 plus solids
  {"kind": "box", "centre": (x, y, z), "size": (sx, sy, sz), "yaw": rad}
  {"kind": "cylinder", "centre": (x, y, z), "radius": r, "height": h}       (vertical axis)
Depth image value = optical-frame z of the first hit (like the Gazebo depth camera / ROS 32FC1), NaN = no return
(out of [near, far]). Also returns an id image (-1 floor / nothing, i = solid index) and the tight 2D box of each solid
(the "perfect YOLO box").
"""
from __future__ import annotations

import math

import numpy as np

from .transforms import apply_T, inv_T, pixel_rays, project


def _hit_box(o, d, s):
    """Ray o + t d (N, 3) against a box centred at the origin (box frame), half sizes s/2 -> t (inf = miss)."""
    h = np.asarray(s, float) / 2.0
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1.0 / d
        t1, t2 = (-h - o) * inv, (h - o) * inv
    tmin = np.nanmax(np.minimum(t1, t2), axis=1)
    tmax = np.nanmin(np.maximum(t1, t2), axis=1)
    hit = (tmax >= np.maximum(tmin, 0.0)) & (tmin > 1e-9)
    return np.where(hit, tmin, np.inf)


def _hit_cyl(o, d, r, hh):
    """Vertical cylinder at the origin, radius r, half height hh."""
    a = d[:, 0] ** 2 + d[:, 1] ** 2
    b = 2 * (o[:, 0] * d[:, 0] + o[:, 1] * d[:, 1])
    c = o[:, 0] ** 2 + o[:, 1] ** 2 - r * r
    disc = b * b - 4 * a * c
    t = np.full(len(d), np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        ts = (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a)
    zs = o[:, 2] + ts * d[:, 2]
    side = (disc >= 0) & (a > 1e-12) & (ts > 1e-9) & (np.abs(zs) <= hh)
    t = np.where(side, ts, t)
    with np.errstate(divide="ignore", invalid="ignore"):
        tc = (hh * np.sign(o[:, 2]) - o[:, 2]) / d[:, 2]        # cap facing the ray origin
    pc = o + tc[:, None] * d
    cap = (np.abs(o[:, 2]) > hh) & (tc > 1e-9) & (pc[:, 0] ** 2 + pc[:, 1] ** 2 <= r * r)
    return np.where(cap & (tc < t), tc, t)


def _corners(s):
    if s["kind"] == "box":
        sx, sy, sz = (v / 2 for v in s["size"])
        c, n = math.cos(s.get("yaw", 0.0)), math.sin(s.get("yaw", 0.0))
        loc = [(a, b, e) for a in (-sx, sx) for b in (-sy, sy) for e in (-sz, sz)]
        return np.array([(s["centre"][0] + c * a - n * b, s["centre"][1] + n * a + c * b, s["centre"][2] + e) for a, b, e in loc])
    r, hh = s["radius"], s["height"] / 2
    ang = np.linspace(0, 2 * math.pi, 24, endpoint=False)
    return np.array([(s["centre"][0] + r * math.cos(a), s["centre"][1] + r * math.sin(a), s["centre"][2] + e)
                     for a in ang for e in (-hh, hh)])


def render(K, width: int, height: int, T_map_opt, solids, near: float = 0.10, far: float = 5.0, floor: bool = True,
           noise: float = 0.0, rng=None):
    """-> (depth (H, W) float32 with NaN for no return, ids (H, W) int, boxes {index: (x1, y1, x2, y2) or None})."""
    us, vs = np.meshgrid(np.arange(width, dtype=float), np.arange(height, dtype=float))
    rays_opt = pixel_rays(K, us.ravel(), vs.ravel())                  # z = 1
    R, o = np.asarray(T_map_opt)[:3, :3], np.asarray(T_map_opt)[:3, 3]
    d = rays_opt @ R.T                                                # map-frame directions, scaled so optical z = 1
    best = np.full(len(d), np.inf)
    ids = np.full(len(d), -1)
    if floor:
        with np.errstate(divide="ignore", invalid="ignore"):
            tf = -o[2] / d[:, 2]
        best = np.where((tf > 1e-9) & np.isfinite(tf), tf, best)
    for i, s in enumerate(solids):
        c = np.asarray(s["centre"], float)
        if s["kind"] == "box":
            yaw = s.get("yaw", 0.0)
            cy, sy = math.cos(-yaw), math.sin(-yaw)
            Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            t = _hit_box(((o - c) @ Rz.T)[None, :].repeat(len(d), 0), d @ Rz.T, s["size"])
        else:
            t = _hit_cyl((o - c)[None, :].repeat(len(d), 0), d, s["radius"], s["height"] / 2)
        closer = t < best
        best = np.where(closer, t, best)
        ids = np.where(closer, i, ids)
    depth = best.copy()                                               # t is optical z because ray z = 1
    if noise and rng is not None:                                     # sigma = noise * z^2 (stereo-like growth)
        fin = np.isfinite(depth)
        depth[fin] = depth[fin] + rng.normal(0.0, 1.0, int(fin.sum())) * noise * depth[fin] ** 2
    bad = ~np.isfinite(depth) | (depth < near) | (depth > far)
    depth[bad] = np.nan
    ids[bad] = -1
    depth, ids = depth.reshape(height, width).astype(np.float32), ids.reshape(height, width)
    boxes = {}
    for i, s in enumerate(solids):
        ys, xs = np.nonzero(ids == i)
        if len(xs) == 0:
            boxes[i] = None
            continue
        boxes[i] = (float(xs.min()), float(ys.min()), float(xs.max() + 1), float(ys.max() + 1))   # visible-pixel tight box
    return depth, ids, boxes


def projected_box(K, width, height, T_map_opt, solid):
    """2D box of the solid's projected corners (includes occluded parts), clipped to the image; None if behind."""
    p = apply_T(inv_T(T_map_opt), _corners(solid))
    p = p[p[:, 2] > 1e-3]
    if len(p) == 0:
        return None
    uv = project(K, p)
    x1, y1 = np.clip(uv.min(0), 0, [width, height])
    x2, y2 = np.clip(uv.max(0), 0, [width, height])
    return (float(x1), float(y1), float(x2), float(y2)) if x2 > x1 and y2 > y1 else None
