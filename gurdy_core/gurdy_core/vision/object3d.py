"""YOLO box + depth image -> 3D object estimate in the map frame (centre, top, size, covariance). numpy, no ROS.

Phase 8c (picking prep). The Phase 4 path (`geometry.box_depth` -> one back-projected pixel + class radius prior) keeps
only x, y in the semantic map. This module uses ALL valid depth pixels inside the box:

  1. pixels in the box (optionally shrunk by `inner_margin`; default 0, it cuts width / top), every `stride`-th pixel
  2. back-project with K (depth value = optical z) -> optical frame -> map frame with T_map_opt (TF at the image stamp)
  3. drop floor points (map z < floor_z + floor_tol; flat floor assumed)
  4. depth clustering: sort by depth, keep the run that contains the near seed (5th percentile) with no gap > `gap`
     (background through gaps / beside the object is farther and separated by a jump)
  5. measure on the object points: top z (98th pct), bottom z (2nd pct, snapped to the floor if it reaches it), width
     perpendicular to the viewing bearing (1-99 pct, corrected), nearest surface along the bearing on the centre line
  6. centre xy: with a class shape prior, fit it (rectangle of known size over a yaw grid / circle of known radius) to
     the visible points; without one, nearest surface + radius along the bearing (radius = half measured width, capped
     by radius_prior). Centre z = middle of bottom..top
  Truncation flags come from the box AND from the object's own pixels reaching the image border.
  7. covariance from a depth-noise model (along the bearing), lateral / vertical pixel + noise terms

Flags say when a value is only a bound (box touches the image border: width / top are lower bounds).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .transforms import apply_T, pixel_rays


@dataclass
class Object3DParams:
    inner_margin: float = 0.0       # fraction of the box cut off on every side (depth clustering removes background bleed)
    stride: int = 2                 # pixel step
    near: float = 0.10
    far: float = 5.0
    floor_z: float = 0.0
    floor_tol: float = 0.02         # m above the floor counted as floor
    on_floor_tol: float = 0.04      # bottom within this of floor_tol -> object stands on the floor
    gap: float = 0.06               # m depth jump that separates object from background
    seed_pct: float = 5.0
    max_extent: float = 1.5         # m: cluster never deeper than this from the seed
    min_points: int = 12
    edge_margin: float = 3.0        # px: box this close to the border = truncated
    depth_sigma0: float = 0.005     # depth noise model sigma(z) = s0 + k z^2 (m); Gazebo default has no depth noise
    depth_sigma_k: float = 0.003
    radius_sigma: float = 0.03      # uncertainty of the 'centre = surface + radius' assumption


@dataclass
class Object3D:
    centre: np.ndarray              # map frame (x, y, z)
    top_z: float
    bottom_z: float
    width: float                    # extent perpendicular to the viewing bearing (m)
    height: float
    near_range: float               # horizontal distance camera -> nearest surface along the bearing
    cov: np.ndarray                 # 3x3, map frame
    n_points: int
    valid_frac: float               # valid depth pixels / pixels sampled
    on_floor: bool
    truncated: dict = field(default_factory=dict)
    fit_yaw: float | None = None    # rectangle fit yaw (box shape prior), else None

    @property
    def top_known(self) -> bool:
        return not self.truncated.get("top", False)

    @property
    def width_known(self) -> bool:
        return not (self.truncated.get("left", False) or self.truncated.get("right", False))

    def to_dict(self) -> dict:
        r = lambda v: round(float(v), 4)    # noqa: E731
        return {"x": r(self.centre[0]), "y": r(self.centre[1]), "z": r(self.centre[2]), "top_z": r(self.top_z),
                "bottom_z": r(self.bottom_z), "width": r(self.width), "height": r(self.height), "range": r(self.near_range),
                "sigma": [r(math.sqrt(max(self.cov[i, i], 0))) for i in range(3)], "n_points": self.n_points,
                "on_floor": self.on_floor, "top_known": self.top_known, "width_known": self.width_known}


def truncation(box, w: int, h: int, margin: float) -> dict:
    return {"left": box[0] < margin, "right": box[2] > w - margin, "top": box[1] < margin, "bottom": box[3] > h - margin}


def box_points(depth: np.ndarray, box, K, T_map_opt, prm: Object3DParams):
    """-> (map-frame points (N, 3), their optical depths (N,), sampled pixel count, pixel coords (N, 2) u, v).
    Invalid depth dropped."""
    H, W = depth.shape[:2]
    x1, y1, x2, y2 = box
    mx, my = (x2 - x1) * prm.inner_margin, (y2 - y1) * prm.inner_margin
    xa, xb = int(max(0, math.floor(x1 + mx))), int(min(W, math.ceil(x2 - mx)))
    ya, yb = int(max(0, math.floor(y1 + my))), int(min(H, math.ceil(y2 - my)))
    if xb <= xa or yb <= ya:
        return np.zeros((0, 3)), np.zeros(0), 0, np.zeros((0, 2))
    us, vs = np.meshgrid(np.arange(xa, xb, prm.stride), np.arange(ya, yb, prm.stride))
    us, vs = us.ravel(), vs.ravel()
    z = depth[vs, us].astype(float)
    n = len(z)
    ok = np.isfinite(z) & (z >= prm.near) & (z <= prm.far)
    p_opt = pixel_rays(K, us[ok], vs[ok]) * z[ok, None]
    uv = np.stack([us[ok], vs[ok]], axis=1).astype(float)
    return apply_T(T_map_opt, p_opt) if len(p_opt) else np.zeros((0, 3)), z[ok], n, uv


def depth_cluster(z: np.ndarray, gap: float, seed_pct: float, max_extent: float) -> np.ndarray:
    """Boolean mask of the depth run containing the near seed with no consecutive gap > `gap`."""
    if len(z) == 0:
        return np.zeros(0, bool)
    order = np.argsort(z)
    zs = z[order]
    seed = float(np.percentile(zs, seed_pct))
    i = int(np.searchsorted(zs, seed))
    i = min(i, len(zs) - 1)
    lo = i
    while lo > 0 and zs[lo] - zs[lo - 1] <= gap:
        lo -= 1
    hi = i
    while hi < len(zs) - 1 and zs[hi + 1] - zs[hi] <= gap and zs[hi + 1] - zs[lo] <= max_extent:
        hi += 1
    mask = np.zeros(len(z), bool)
    mask[order[lo:hi + 1]] = True
    return mask


def fit_circle_fixed_r(xy: np.ndarray, r: float, c0, iters: int = 15) -> np.ndarray:
    """Least-squares centre of a circle of KNOWN radius r through the visible arc points (Gauss-Newton from c0)."""
    c = np.asarray(c0, float).copy()
    for _ in range(iters):
        d = xy - c
        n = np.linalg.norm(d, axis=1)
        n = np.where(n < 1e-9, 1e-9, n)
        res = n - r
        J = -d / n[:, None]
        A = J.T @ J
        if np.linalg.cond(A) > 1e8:
            break
        step = -np.linalg.solve(A, J.T @ res)
        c = c + step
        if np.linalg.norm(step) < 1e-6:
            break
    return c


def _rect_boundary_dist(q: np.ndarray, hx: float, hy: float) -> np.ndarray:
    """|distance| of points q (box frame, centred) to the rectangle boundary."""
    ax, ay = np.abs(q[:, 0]) - hx, np.abs(q[:, 1]) - hy
    outside = np.hypot(np.maximum(ax, 0), np.maximum(ay, 0))
    inside = np.minimum(-ax, -ay)
    return np.where((ax > 0) | (ay > 0), outside, inside)


def fit_rectangle(xy: np.ndarray, size, cam_xy, step_deg: float = 2.0):
    """Centre + yaw of a rectangle of KNOWN size (sx, sy) whose camera-facing sides contain the visible points.
    For each yaw on a grid, the side facing the camera fixes the centre along that axis (robust 2nd / 98th pct of the points);
    the yaw with the smallest median point-to-boundary distance wins. -> (centre (2,), yaw, cost)."""
    hx, hy = size[0] / 2.0, size[1] / 2.0
    period = math.pi / 2 if abs(hx - hy) < 1e-6 else math.pi
    best = (None, 0.0, math.inf)
    cam = np.asarray(cam_xy, float)
    for yaw in np.arange(0.0, period, math.radians(step_deg)):
        c, s_ = math.cos(yaw), math.sin(yaw)
        R = np.array([[c, s_], [-s_, c]])                     # map -> box axes
        q = xy @ R.T
        qc = R @ cam
        lo, hi = np.percentile(q, 2, axis=0), np.percentile(q, 98, axis=0)
        cu = lo[0] + hx if qc[0] < 0.5 * (lo[0] + hi[0]) else hi[0] - hx
        cv = lo[1] + hy if qc[1] < 0.5 * (lo[1] + hi[1]) else hi[1] - hy
        cost = float(np.median(_rect_boundary_dist(q - [cu, cv], hx, hy)))
        if cost < best[2]:
            best = (R.T @ np.array([cu, cv]), float(yaw), cost)
    return best


def _continues_above(depth, uv, z, prm: Object3DParams, rows: int = 8, frac: float = 0.3) -> bool:
    """True when pixels at the object's depth continue in the strip just above its top pixels: the detector box was cut
    short or the object leaves the image there, so the measured top is only a lower bound."""
    v_top = int(uv[:, 1].min())
    if v_top - rows < 0:
        return True
    cols = np.unique(uv[:, 0].astype(int))
    strip = depth[v_top - rows:v_top, cols].astype(float)
    zlo, zhi = float(z.min()) - prm.gap, float(z.max()) + prm.gap
    same = np.isfinite(strip) & (strip >= zlo) & (strip <= zhi)
    return bool(same.mean() > frac)


def estimate(depth: np.ndarray, box, K, T_map_opt, prm: Object3DParams | None = None, radius_prior: float | None = None,
             shape: dict | None = None):
    """One detection -> Object3D, or None (too few object points).
    shape (class size prior, optional): {"kind": "box", "size": (sx, sy)} or {"kind": "cylinder", "radius": r}. With it the
    xy centre comes from fitting that footprint to the visible points (handles any yaw, and elongated objects seen end-on);
    without it: nearest surface + radius along the viewing ray (radius = half measured width, capped by radius_prior)."""
    prm = prm or Object3DParams()
    H, W = depth.shape[:2]
    pts, z, n_sampled, uv = box_points(depth, box, K, T_map_opt, prm)
    valid_frac = len(z) / n_sampled if n_sampled else 0.0
    keep = pts[:, 2] >= prm.floor_z + prm.floor_tol if len(pts) else np.zeros(0, bool)
    pts, z, uv = pts[keep], z[keep], uv[keep]
    if len(pts) < prm.min_points:
        return None
    m = depth_cluster(z, prm.gap, prm.seed_pct, prm.max_extent)
    pts, z, uv = pts[m], z[m], uv[m]
    if len(pts) < prm.min_points:
        return None
    cam = np.asarray(T_map_opt, float)[:3, 3]
    rel = pts[:, :2] - cam[:2]
    b = np.median(rel, axis=0)
    b = b / (np.linalg.norm(b) or 1.0)                      # horizontal viewing bearing
    perp = np.array([-b[1], b[0]])
    along, lat = rel @ b, rel @ perp
    l_lo, l_hi = np.percentile(lat, [1, 99])
    width = float(l_hi - l_lo) / 0.98                       # undo the 1 % trimming on each side (uniform spread)
    mid = 0.5 * (l_lo + l_hi)
    band = np.abs(lat - mid) <= 0.2 * width                 # nearest surface ON the object's centre line (a face seen at an
    near = float(np.percentile(along[band] if band.sum() >= 5 else along, 10))   # angle is nearer at one side)
    top = float(np.percentile(pts[:, 2], 98))
    bottom = float(np.percentile(pts[:, 2], 2))
    on_floor = bottom <= prm.floor_z + prm.floor_tol + prm.on_floor_tol
    if on_floor:
        bottom = prm.floor_z
    radius = 0.5 * width if radius_prior is None else min(0.5 * width, radius_prior)
    cxy = cam[:2] + b * (near + radius) + perp * mid
    fit_yaw = None
    if shape is not None and shape.get("kind") == "cylinder":
        cxy = fit_circle_fixed_r(pts[:, :2], float(shape["radius"]), cxy)
    elif shape is not None and shape.get("kind") == "box":
        c_fit, fit_yaw, _ = fit_rectangle(pts[:, :2], shape["size"][:2], cam[:2])
        cxy = c_fit
    centre = np.array([cxy[0], cxy[1], 0.5 * (top + bottom)])
    zc = float(np.median(z))
    s_along = math.hypot(prm.depth_sigma0 + prm.depth_sigma_k * zc * zc, prm.radius_sigma)
    s_lat = 0.01 + 0.004 * zc
    s_z = 0.01 + 0.004 * zc
    Rb = np.array([[b[0], perp[0], 0.0], [b[1], perp[1], 0.0], [0.0, 0.0, 1.0]])
    cov = Rb @ np.diag([s_along ** 2, s_lat ** 2, s_z ** 2]) @ Rb.T
    tr = truncation(box, W, H, prm.edge_margin)
    e = prm.edge_margin + prm.stride                         # object pixels reaching the border = cut off there
    tr = {"left": tr["left"] or uv[:, 0].min() < e, "right": tr["right"] or uv[:, 0].max() > W - 1 - e,
          "top": tr["top"] or uv[:, 1].min() < e or _continues_above(depth, uv, z, prm),
          "bottom": tr["bottom"] or uv[:, 1].max() > H - 1 - e}
    o = Object3D(centre, top, bottom, width, top - bottom, near, cov, int(len(pts)), float(valid_frac), bool(on_floor), tr)
    o.fit_yaw = fit_yaw
    return o


# ------------------------------------------------------------------ fusion over sightings (static object)
CHI2_3D_99 = 11.34


@dataclass
class Track3D:
    """Information-filter fusion of one static object's sightings (centre), robust size / top from the sightings."""
    info: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))
    info_vec: np.ndarray = field(default_factory=lambda: np.zeros(3))
    tops: list = field(default_factory=list)
    widths: list = field(default_factory=list)
    heights: list = field(default_factory=list)
    hits: int = 0
    rejected: int = 0
    min_cov_floor: float = 1e-4     # m^2 added to each sighting's covariance (correlated errors: do not get over-confident)

    @property
    def centre(self):
        return np.linalg.solve(self.info, self.info_vec) if self.hits else None

    @property
    def cov(self):
        return np.linalg.inv(self.info) if self.hits else None

    def mahalanobis2(self, obs: Object3D) -> float:
        if not self.hits:
            return 0.0
        d = obs.centre - self.centre
        S = self.cov + obs.cov
        return float(d @ np.linalg.solve(S, d))

    def update(self, obs: Object3D, gate: float = CHI2_3D_99) -> bool:
        """Fuse one sighting; False (rejected) when it is outside the gate (after the first 3 sightings)."""
        if self.hits >= 3 and self.mahalanobis2(obs) > gate:
            self.rejected += 1
            return False
        C = obs.cov + np.eye(3) * self.min_cov_floor
        Ci = np.linalg.inv(C)
        self.info = self.info + Ci
        self.info_vec = self.info_vec + Ci @ obs.centre
        if obs.top_known:
            self.tops.append(obs.top_z)
            self.heights.append(obs.height)
        if obs.width_known:
            self.widths.append(obs.width)
        self.hits += 1
        return True

    def summary(self) -> dict:
        c = self.centre
        med = lambda v: float(np.median(v)) if v else None    # noqa: E731
        return {"x": float(c[0]), "y": float(c[1]), "z": float(c[2]), "top_z": med(self.tops), "height": med(self.heights),
                "width": med(self.widths), "sigma": [float(math.sqrt(self.cov[i, i])) for i in range(3)], "hits": self.hits,
                "rejected": self.rejected}


# ------------------------------------------------------------------ accuracy vs ground truth
def gt_top_z(g: dict) -> float:
    """GT object dict with z (centre) and size [sx, sy, sz] (or height)."""
    h = g["size"][2] if "size" in g else g["height"]
    return float(g["z"]) + h / 2


def compare_3d(found: list, gt: list, max_dist: float = 0.6) -> dict:
    """found / gt: dicts with class, x, y, z (+ top_z / height / width; gt: size or height). Greedy nearest same-class
    match in xy within max_dist. -> per-GT rows + summary (median / p90 / max of each error)."""
    used, rows = set(), []
    for g in gt:
        best, bd = None, max_dist
        for i, f in enumerate(found):
            if i in used or f.get("class") != g.get("class"):
                continue
            d = math.hypot(f["x"] - g["x"], f["y"] - g["y"])
            if d <= bd:
                best, bd = i, d
        row = {"name": g.get("name", g.get("class")), "class": g.get("class"), "found": best is not None}
        if best is not None:
            used.add(best)
            f = found[best]
            row["xy_err"] = bd
            row["z_err"] = abs(f["z"] - g["z"])
            row["xyz_err"] = math.sqrt(bd * bd + row["z_err"] ** 2)
            if f.get("top_z") is not None:
                row["top_err"] = abs(f["top_z"] - gt_top_z(g))
            if f.get("height") is not None and ("size" in g or "height" in g):
                row["height_err"] = abs(f["height"] - (g["size"][2] if "size" in g else g["height"]))
        rows.append(row)

    def stats(k):
        v = sorted(r[k] for r in rows if k in r)
        if not v:
            return None
        return {"n": len(v), "median": float(np.median(v)), "p90": float(np.percentile(v, 90)), "max": float(v[-1])}

    return {"rows": rows, "recall": sum(r["found"] for r in rows) / len(rows) if rows else float("nan"),
            "false": len(found) - len(used),
            **{k: stats(k) for k in ("xy_err", "z_err", "xyz_err", "top_err", "height_err")}}
