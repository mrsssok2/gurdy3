"""Phase 8c wiring: one detector box + depth image -> a 3D sighting in the map frame (numpy, no ROS).

Used by `gurdy_perception/detector_node` when `position_method:=object3d` (default stays the Phase 4 one-pixel method).

    floor = FloorTracker(FloorParams(floor_z=-0.11))       # map floor z: base_link is z 0 in map/odom, floor 0.11 below
    plane = floor.update(depth, K, T_map_opt, t)           # once per frame: RANSAC floor plane in the MAP frame
    s = locate(depth, box, K, T_map_opt, "box", plane)     # -> Sighting3D (map frame) or None (-> pixel fallback)

Steps of `locate`:
  1. T_floor_map from the fitted plane (normal -> +z, plane -> z 0; the minimal rotation, so floor x/y = map x/y up to
     the plane tilt). object3d.estimate runs in that floor frame with floor_z = 0, so floor removal, "on the floor"
     and the bottom snap use the MEASURED floor, not a constant.
  2. class shape prior (SHAPES): rectangle fit for box / toolbox (+ small box classes), fixed-radius circle fit for
     barrel / fire extinguisher (+ can / mini extinguisher); cone, chair, mannequin, mini cone use the ray method.
  3. the result goes back to the map frame (centre, covariance rotated) and the heights are also kept relative to the
     floor (height, top above the floor), which is what the arm / fetch code needs.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .. import low_obstacles as LO
from .classes import SIZE_PRIOR, SIZE_PRIOR_SMALL
from .object3d import Object3D, Object3DParams, estimate
from .transforms import apply_T, inv_T, pixel_rays

ALL_PRIORS = {**SIZE_PRIOR, **SIZE_PRIOR_SMALL}
_BOX = ("box", "toolbox", "small_box", "mini_toolbox")
_CYL = ("barrel", "fire_extinguisher", "can", "mini_extinguisher")
# classes without one size: the rectangle size comes from the visible extents, clipped to this range (vision/objects.py
# draws `box` dims from U(0.25, 0.50); the semantic_lab boxes are 0.25-0.40). A fixed 0.40 prior put the 0.30 red box
# 6.5 cm off (median, synthetic, all 12 planned stops), worse than the ray method.
SIZE_RANGE = {"box": (0.25, 0.50)}


def shape_prior(cls: str):
    """Class footprint for object3d.estimate: {"kind": "box", "size": (w, d)} / {"kind": "cylinder", "radius": r} / None."""
    if cls not in ALL_PRIORS:
        return None
    w, d, _ = ALL_PRIORS[cls]
    if cls in _BOX:
        return {"kind": "box", "size": (w, d), **({"size_range": SIZE_RANGE[cls]} if cls in SIZE_RANGE else {})}
    if cls in _CYL:
        return {"kind": "cylinder", "radius": 0.5 * min(w, d)}
    return None


def radius_of(cls: str) -> float | None:
    if cls not in ALL_PRIORS:
        return None
    w, d, _ = ALL_PRIORS[cls]
    return 0.5 * min(w, d)


# ------------------------------------------------------------------ floor plane
def floor_transform(n, d) -> np.ndarray:
    """T_floor_x: rotates the plane normal n (frame x) onto +z and moves the plane n.p + d = 0 to z = 0 (minimal rotation,
    no yaw). Same as gurdy_core.visual_servo.floor_transform (kept here so the vision path does not import the arm code)."""
    n = np.asarray(n, float) / np.linalg.norm(n)
    z = np.array([0.0, 0.0, 1.0])
    v, c = np.cross(n, z), float(n @ z)
    if np.linalg.norm(v) < 1e-12:
        R = np.eye(3)
    else:
        Kx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R = np.eye(3) + Kx + Kx @ Kx / (1 + c)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [0.0, 0.0, d]
    return T


def plane_z_at(plane, x: float, y: float) -> float:
    """z of the plane n.p + d = 0 above map (x, y)."""
    n, d = plane
    return float(-(d + n[0] * x + n[1] * y) / n[2])


@dataclass
class FloorParams:
    floor_z: float = -0.11          # prior floor z in the target frame (map/odom: base_link 0, measured floor -0.11)
    fit: bool = True                # False: always the prior plane (flat floor at floor_z)
    stride: int = 4                 # pixel step for the floor fit (320x240 -> 80x60)
    max_range: float = 3.0          # m optical z: far floor is noisy and nearly parallel to the rays
    band: float = 0.06              # RANSAC candidates within this of the prior plane (z)
    tol: float = 0.012              # inlier distance
    max_tilt: float = 0.10          # rad vs level (the map floor is level; TF already removes the body tilt)
    max_offset: float = 0.05        # m: |fitted z - prior z| under the camera above this -> fit rejected
    min_points: int = 150
    hold_s: float = 3.0             # keep using the last good fit this long when a frame has no usable floor
    smooth: float = 0.5             # EMA weight of a new fit (0: never move, 1: take the newest)


@dataclass
class FloorTracker:
    prm: FloorParams = field(default_factory=FloorParams)
    plane: tuple | None = None       # last good fitted plane (n, d) in the target frame
    t_fit: float = -1e9
    n_fit: int = 0
    n_prior: int = 0
    source: str = "prior"            # "fit" | "held" | "prior" for the plane returned last

    @property
    def prior(self) -> tuple:
        return np.array([0.0, 0.0, 1.0]), -self.prm.floor_z

    def fit_frame(self, depth, K, T_map_opt):
        """-> (n, d) fitted in the target frame, or None."""
        p = self.prm
        H, W = depth.shape[:2]
        vs, us = np.mgrid[0:H:p.stride, 0:W:p.stride]
        z = depth[vs, us].astype(float)
        ok = np.isfinite(z) & (z >= 0.1) & (z <= p.max_range)
        if ok.sum() < p.min_points:
            return None
        pts = apply_T(T_map_opt, pixel_rays(K, us[ok].astype(float), vs[ok].astype(float)) * z[ok, None])
        lp = LO.LowObstacleParams(floor_prior_z=p.floor_z, floor_band=p.band, floor_tol=p.tol, floor_max_tilt=p.max_tilt,
                                  min_floor_points=p.min_points)
        n, d, fitted, _ = LO.fit_floor(pts, lp)
        if not fitted:
            return None
        cam = np.asarray(T_map_opt, float)[:3, 3]
        if abs(plane_z_at((n, d), cam[0], cam[1]) - p.floor_z) > p.max_offset:
            return None
        return np.asarray(n, float), float(d)

    def update(self, depth, K, T_map_opt, t: float) -> tuple:
        """Plane (n, d) to use for this frame; `source` says where it came from."""
        if not self.prm.fit:
            self.source = "prior"
            self.n_prior += 1
            return self.prior
        f = self.fit_frame(depth, K, T_map_opt)
        if f is not None:
            if self.plane is None or self.prm.smooth >= 1.0:
                self.plane = f
            else:
                a = self.prm.smooth
                n = (1 - a) * self.plane[0] + a * f[0]
                n = n / np.linalg.norm(n)
                cam = np.asarray(T_map_opt, float)[:3, 3]
                z = (1 - a) * plane_z_at(self.plane, cam[0], cam[1]) + a * plane_z_at(f, cam[0], cam[1])
                self.plane = (n, float(-(n[0] * cam[0] + n[1] * cam[1] + n[2] * z)))
            self.t_fit, self.source = t, "fit"
            self.n_fit += 1
            return self.plane
        if self.plane is not None and t - self.t_fit <= self.prm.hold_s:
            self.source = "held"
            return self.plane
        self.source = "prior"
        self.n_prior += 1
        return self.prior


# ------------------------------------------------------------------ one sighting
@dataclass
class Sighting3D:
    cls: str
    centre: np.ndarray               # map frame
    cov: np.ndarray                  # 3x3 map frame
    top_z: float                     # map frame
    bottom_z: float                  # map frame
    floor_z: float                   # map z of the floor plane under the centre
    height: float                    # top - bottom (above the floor when on_floor)
    width: float                     # extent perpendicular to the viewing bearing
    yaw: float | None                # rectangle fit yaw (map frame, modulo the shape period) or None
    method: str                      # "fit" | "ray"
    range: float                     # horizontal camera -> nearest surface
    obj: Object3D                    # floor-frame result (flags, n_points, ...)

    @property
    def top_known(self) -> bool:
        return self.obj.top_known

    @property
    def width_known(self) -> bool:
        return self.obj.width_known

    @property
    def truncated(self) -> dict:
        return self.obj.truncated

    def as_object3d(self, weight_scale: float = 1.0) -> Object3D:
        """Map-frame Object3D for Track3D fusion (cov inflated by 1 / weight_scale, e.g. edge-truncated boxes)."""
        o = Object3D(self.centre.copy(), self.top_z, self.bottom_z, self.width, self.height, self.range,
                     self.cov / max(weight_scale, 1e-3), self.obj.n_points, self.obj.valid_frac, self.obj.on_floor,
                     dict(self.obj.truncated))
        o.fit_yaw = self.yaw
        return o

    def to_dict(self) -> dict:
        r = lambda v: None if v is None else round(float(v), 4)    # noqa: E731
        return {"c": [r(v) for v in self.centre], "sigma": [r(math.sqrt(max(self.cov[i, i], 0.0))) for i in range(3)],
                "top_z": r(self.top_z), "bottom_z": r(self.bottom_z), "floor_z": r(self.floor_z), "height": r(self.height),
                "width": r(self.width), "yaw": r(self.yaw), "method": self.method, "range": r(self.range),
                "n": self.obj.n_points, "on_floor": self.obj.on_floor, "top_known": self.top_known,
                "width_known": self.width_known}


def locate(depth, box, K, T_map_opt, cls: str, plane=None, prm: Object3DParams | None = None, use_shape: bool = True):
    """One detection -> Sighting3D in the frame of T_map_opt, or None (too few object points: caller falls back).
    plane = (n, d) of the floor in that frame (FloorTracker.update); None = level floor at z 0."""
    n, d = plane if plane is not None else (np.array([0.0, 0.0, 1.0]), 0.0)
    T_fm = floor_transform(n, d)
    T_fo = T_fm @ np.asarray(T_map_opt, float)
    p = prm or Object3DParams()
    if p.floor_z != 0.0:
        p = Object3DParams(**{**p.__dict__, "floor_z": 0.0})
    shape = shape_prior(cls) if use_shape else None
    o = estimate(depth, box, K, T_fo, p, radius_prior=radius_of(cls), shape=shape)
    if o is None:
        return None
    T_mf = inv_T(T_fm)
    R = T_mf[:3, :3]
    c = apply_T(T_mf, o.centre[None, :])[0]
    cov = R @ o.cov @ R.T
    fz = plane_z_at((n, d), c[0], c[1])
    # heights along the floor normal; in the map frame: floor z under the object + height above the floor
    top, bottom = fz + o.top_z, fz + o.bottom_z
    yaw = None
    if o.fit_yaw is not None:
        ax = R @ np.array([math.cos(o.fit_yaw), math.sin(o.fit_yaw), 0.0])
        yaw = math.atan2(ax[1], ax[0])
    return Sighting3D(cls, c, cov, top, bottom, fz, o.height, o.width, yaw, "fit" if shape else "ray", o.near_range, o)


def size_of(cls: str, height: float | None) -> list:
    """(w, d, h) to publish: footprint = class prior (the shape fit assumes it; for SIZE_RANGE classes like `box` it is only
    the nominal size, NOT measured), height = measured when known."""
    w, d, h = ALL_PRIORS.get(cls, (0.3, 0.3, 0.3))
    return [w, d, h if height is None else height]


def yaw_mean(yaws, period: float) -> float | None:
    """Circular mean of yaws modulo `period` (pi for rectangles, pi/2 for squares). -> value in [0, period)."""
    if not yaws:
        return None
    k = 2 * math.pi / period
    s = sum(math.sin(k * y) for y in yaws)
    c = sum(math.cos(k * y) for y in yaws)
    if abs(s) < 1e-12 and abs(c) < 1e-12:
        return None
    return (math.atan2(s, c) / k) % period


def yaw_period(cls: str) -> float:
    w, d, _ = ALL_PRIORS.get(cls, (1.0, 1.0, 1.0))
    return math.pi / 2 if abs(w - d) < 1e-6 else math.pi
