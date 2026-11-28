"""Object yaw from one wrist RGB-D frame (numpy, no ROS), for the top-grasp wrist roll (gurdy_core.grasp_test.best_roll).

    est = estimate_yaw(depth, K, T_base_opt, "mini_toolbox", xy_hint=servo_obs.xy)
    est.yaw          base_link yaw of the object's long (x) axis, in [0, period); None for symmetric classes / failure
    est.confidence   0..1 (shape distinctness x point count x not cut by the image border)

Steps: depth -> base_link points -> RANSAC floor plane (gurdy_core.low_obstacles.fit_floor, same as the visual servo) ->
floor frame (plane = z 0, minimal rotation, so floor yaw = base_link yaw up to the plane tilt) -> the object's points
(height 1 cm .. class height + 3 cm, within the object's footprint radius + 3 cm of `xy_hint`, else of the median of all
raised points) -> the TOP LAYER, then per class:
  * small_box (square, period 90 deg): minimum-area rectangle of the top-face points over a 0.5 deg grid. Confidence from
    how distinct the minimum is: a full square face gives area(45 deg) / area(0 deg) ~1.4 with the 2-98 pct extents
    (2 untrimmed), a blob without corners ~1.
  * mini_toolbox (period 180 deg): the HANDLE bar (8 x 2 cm, top 9.5 cm, along the long axis): principal axis of the
    points above 8 cm. Confidence from the elongation (bar: std ratio ~4). Without handle points (cut off / occluded):
    the body top (12 x 5 cm at 6 cm) with the same principal-axis rule (method "body").
  * can, mini_cone, mini_extinguisher: rotationally symmetric -> yaw None, confidence 1, method "symmetric".
The top layer touching the image border (partial view) halves the confidence and sets `partial`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from . import low_obstacles as LO
from .arm import PICKABLE

SYMMETRIC = ("can", "mini_cone", "mini_extinguisher")
PERIOD = {"small_box": math.pi / 2, "mini_toolbox": math.pi}
HANDLE_MIN_H = 0.08                # m above the floor: only the handle bar of the mini toolbox (body top 0.06, posts below 0.085)
BODY_BAND = (0.045, 0.068)         # m: the mini toolbox body top (lid at 0.06)


@dataclass
class YawParams:
    stride: int = 1
    near: float = 0.07
    far: float = 1.0
    min_h: float = 0.01
    top_band: float = 0.012
    margin: float = 0.03
    min_points: int = 12
    full_points: int = 60          # points for full count confidence
    border: int = 3                # px
    grid_deg: float = 0.5


@dataclass
class YawEstimate:
    yaw: float | None
    confidence: float
    method: str                    # rect | handle | body | symmetric | none
    period: float | None = None
    n_points: int = 0
    partial: bool = False
    reason: str = ""

    def to_dict(self):
        r = lambda v: None if v is None else round(float(v), 4)    # noqa: E731
        return {"yaw": r(self.yaw), "confidence": r(self.confidence), "method": self.method, "period": r(self.period),
                "n_points": self.n_points, "partial": self.partial, "reason": self.reason}


def circ_err(a: float, b: float, period: float) -> float:
    """|a - b| modulo the symmetry period (0 .. period / 2)."""
    return abs((a - b + period / 2) % period - period / 2)


def _floor_T(n, d):
    n = np.asarray(n, float) / np.linalg.norm(n)
    z = np.array([0.0, 0.0, 1.0])
    v, c = np.cross(n, z), float(n @ z)
    R = np.eye(3)
    if np.linalg.norm(v) > 1e-12:
        Kx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R = np.eye(3) + Kx + Kx @ Kx / (1 + c)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, [0.0, 0.0, d]
    return T


def min_area_rect_yaw(xy: np.ndarray, period: float = math.pi / 2, grid_deg: float = 0.5):
    """-> (yaw in [0, period), area_min, area_max) of the bounding rectangle over the yaw grid (2-98 pct extents)."""
    best, amin, amax = 0.0, math.inf, 0.0
    for yaw in np.arange(0.0, period, math.radians(grid_deg)):
        c, s = math.cos(yaw), math.sin(yaw)
        u, v = xy @ np.array([c, s]), xy @ np.array([-s, c])
        a = float(np.ptp(np.percentile(u, [2, 98])) * np.ptp(np.percentile(v, [2, 98])))
        if a < amin:
            best, amin = float(yaw), a
        amax = max(amax, a)
    return best, amin, amax


def principal_yaw(xy: np.ndarray):
    """-> (yaw of the major axis in [0, pi), std ratio major / minor)."""
    c = xy - xy.mean(0)
    w, V = np.linalg.eigh(np.cov(c.T))
    ax = V[:, 1]
    ratio = math.sqrt(max(w[1], 1e-12) / max(w[0], 1e-12))
    return math.atan2(ax[1], ax[0]) % math.pi, ratio


def _ramp(x, lo, hi):
    return float(min(max((x - lo) / (hi - lo), 0.0), 1.0))


def estimate_yaw(depth, K, T_base_opt, cls: str, xy_hint=None, prm: YawParams | None = None) -> YawEstimate:
    prm = prm or YawParams()
    if cls in SYMMETRIC:
        return YawEstimate(None, 1.0, "symmetric", None, reason="rotationally symmetric: no yaw needed")
    if cls not in PERIOD:
        return YawEstimate(None, 0.0, "none", reason=f"no yaw model for {cls}")
    period = PERIOD[cls]
    lp = LO.LowObstacleParams(stride=prm.stride, min_range=prm.near, max_range=prm.far, floor_band=0.08)
    p_opt, uv = LO.depth_to_points(depth, K, lp)
    if len(p_opt) < 50:
        return YawEstimate(None, 0.0, "none", period, reason="no depth points")
    pts = LO.apply_T(T_base_opt, p_opt)
    n, d, fitted, _ = LO.fit_floor(pts, lp)
    if not fitted:
        return YawEstimate(None, 0.0, "none", period, reason="no floor fit")
    T_fb = _floor_T(n, d)
    f = LO.apply_T(T_fb, pts)
    size = PICKABLE[cls]["size"]
    raised = (f[:, 2] >= prm.min_h) & (f[:, 2] <= size[2] + 0.03)
    if raised.sum() < prm.min_points:
        return YawEstimate(None, 0.0, "none", period, int(raised.sum()), reason="no object points above the floor")
    if xy_hint is not None:
        c = LO.apply_T(T_fb, np.array([[xy_hint[0], xy_hint[1], 0.0]]))[0, :2]
    else:
        c = np.median(f[raised, :2], axis=0)
    rad = 0.5 * math.hypot(size[0], size[1]) + prm.margin
    obj = raised & (np.hypot(f[:, 0] - c[0], f[:, 1] - c[1]) <= rad)
    if obj.sum() < prm.min_points:
        return YawEstimate(None, 0.0, "none", period, int(obj.sum()), reason="too few points near the object")
    H, W = depth.shape[:2]

    def cut(mask):
        u, v = uv[mask, 0], uv[mask, 1]
        b = prm.border
        return bool(u.min() < b or v.min() < b or u.max() > W - 1 - b or v.max() > H - 1 - b)

    if cls == "small_box":
        top = float(np.percentile(f[obj, 2], 98))
        layer = obj & (f[:, 2] >= top - prm.top_band)
        if layer.sum() < prm.min_points:
            return YawEstimate(None, 0.0, "none", period, int(layer.sum()), reason="too few top-face points")
        yaw, amin, amax = min_area_rect_yaw(f[layer, :2], period, prm.grid_deg)
        shape_c = _ramp(amax / max(amin, 1e-12), 1.1, 1.35)      # full square face, 2-98 pct extents: ~1.4
        method = "rect"
    else:
        layer = obj & (f[:, 2] >= HANDLE_MIN_H)
        method = "handle"
        lo_r, hi_r = 2.0, 3.5                                     # bar 8 x 2 cm: std ratio ~4
        if layer.sum() < prm.min_points:
            layer = obj & (f[:, 2] >= BODY_BAND[0]) & (f[:, 2] <= BODY_BAND[1])
            method = "body"
            lo_r, hi_r = 1.4, 2.2                                 # body 12 x 5 cm: std ratio 2.4
        if layer.sum() < prm.min_points:
            return YawEstimate(None, 0.0, "none", period, int(layer.sum()), reason="no handle or body-top points")
        yaw, ratio = principal_yaw(f[layer, :2])
        shape_c = _ramp(ratio, lo_r, hi_r)
    partial = cut(layer)
    conf = shape_c * _ramp(int(layer.sum()), prm.min_points, prm.full_points) * (0.5 if partial else 1.0)
    # floor frame -> base_link yaw (the floor transform has no yaw; a tilt changes it only to second order)
    R = np.linalg.inv(T_fb)[:3, :3]
    ax = R @ np.array([math.cos(yaw), math.sin(yaw), 0.0])
    yb = math.atan2(ax[1], ax[0]) % period
    return YawEstimate(yb, conf, method, period, int(layer.sum()), partial,
                       "top face cut by the image border" if partial else "")


def standing_check(depth, K, T_base_opt, xy_base, radius: float, band=(0.005, 0.025), prm: YawParams | None = None,
                   floor=None, min_visible: int = 20) -> tuple:
    """(points of something STANDING on the floor at base-frame `xy_base` (height band just above the floor, within
    `radius`) or None, why). A lifted object (bottom > 2.5 cm up) or an empty spot gives ~0.
    floor: a known plane (n, d) in base_link (e.g. the servo's fit before the grasp: the robot has not moved since); without
    it the floor is fitted in this view. None (UNKNOWN, never 'gone') when the spot is not visible: fewer than `min_visible`
    points of the floor or of an object there -- e.g. the held object fills the wrist camera's view (2026-11-28 smoke: the
    in-view floor fit failed behind a held can; with a known floor a hidden spot would otherwise read 0 = 'lifted')."""
    prm = prm or YawParams()
    lp = LO.LowObstacleParams(stride=prm.stride, min_range=prm.near, max_range=prm.far, floor_band=0.08)
    p_opt, _ = LO.depth_to_points(depth, K, lp)
    if len(p_opt) < 50:
        return None, f"only {len(p_opt)} valid depth points (view blocked / too close)"
    pts = LO.apply_T(T_base_opt, p_opt)
    if floor is not None:
        n, d = np.asarray(floor[0], float), float(floor[1])
    else:
        n, d, fitted, _ = LO.fit_floor(pts, lp)
        if not fitted:
            return None, f"no floor fit in this view ({len(p_opt)} points)"
    T_fb = _floor_T(n, d)
    f = LO.apply_T(T_fb, pts)
    c = LO.apply_T(T_fb, np.array([[xy_base[0], xy_base[1], 0.0]]))[0, :2]
    near = np.hypot(f[:, 0] - c[0], f[:, 1] - c[1]) <= radius
    visible = near & (f[:, 2] >= -0.01) & (f[:, 2] <= band[1])
    if visible.sum() < min_visible:
        return None, f"spot not visible ({int(visible.sum())} floor / object points within {radius:.2f} m)"
    m = near & (f[:, 2] >= band[0]) & (f[:, 2] <= band[1])
    return int(m.sum()), ""


def standing_points(depth, K, T_base_opt, xy_base, radius: float, band=(0.005, 0.025), prm: YawParams | None = None,
                    floor=None) -> int | None:
    """standing_check without the reason (None = unknown: no floor / spot not visible)."""
    return standing_check(depth, K, T_base_opt, xy_base, radius, band, prm, floor)[0]
