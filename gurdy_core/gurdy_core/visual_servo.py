"""Phase 9e: visual servoing for the final approach of a pick (wrist RGB-D, numpy only, no ROS).

The robot stands with its back to the object (arm in `ready`, wrist camera looking down behind it, Phase 9a). Loop:
    observe:  wrist depth -> base_link points -> RANSAC floor plane (gurdy_core.low_obstacles.fit_floor; the floor is
              NOT assumed at a fixed height: measured at map z = -0.11 m on 2026-11-26, but the body pitches / rolls and
              the stance height varies) -> floor frame (plane = z 0) -> object box (detector box, or a depth blob above
              the floor nearest the image centre) -> Phase 8c object3d.estimate with the class shape prior -> object
              centre in base_link + plausibility (height vs the class, else 'implausible' = probably an occluder)
    servo:    error = object xy - the class's grasp-zone centre (Phase 9a reach maps); done when |error| <= 2 cm, or
              <= 4 cm and the arm's grasp planner reaches it from here (arm adjustment instead of another step);
              else a holonomic body step of min(|error|, 6 cm) toward it (robot moves by +error), at most 3 steps.
    failures: TARGET_LOST (not seen, no prior / still not seen after stepping toward the prior), OCCLUDED (implausible
              estimate twice, one sideways step in between), NOT_ALIGNED (still > tolerance after 3 steps), TIMEOUT.
              'too close' (object under / at the rear legs) needs no special case: the step points away from the robot.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import low_obstacles as LO
from .vision import object3d as O3

# class -> (shape prior for object3d.estimate or None, radius prior, height m, grasp-zone centre in base_link)
# zone centres from docs/results/data/phase9a/arm_reach.md (largest inscribed disc; y set to 0, tested feasible)
CLASS_INFO = {
    "small_box": ({"kind": "box", "size": (0.05, 0.05)}, 0.025, 0.05, (-0.27, 0.0)),
    "mini_toolbox": ({"kind": "box", "size": (0.12, 0.05)}, 0.025, 0.095, (-0.28, 0.0)),
    "can": ({"kind": "cylinder", "radius": 0.033}, 0.033, 0.12, (-0.35, 0.0)),
    "mini_cone": (None, 0.03, 0.10, (-0.35, 0.0)),          # a cone is no cylinder: ray method with a radius prior
    "mini_extinguisher": ({"kind": "cylinder", "radius": 0.025}, 0.025, 0.15, (-0.35, 0.0)),
}
# arm pose the robot holds while servoing (config/arm.yaml poses): top-grasp classes (zone ~0.27 m behind) use servo_view,
# side-grasp classes (zone 0.35 m behind, taller) servo_view_far so that their top face stays inside the image
VIEW_POSE = {"small_box": "servo_view", "mini_toolbox": "servo_view", "can": "servo_view_far", "mini_cone": "servo_view_far",
             "mini_extinguisher": "servo_view_far"}


@dataclass
class ServoParams:
    tol: float = 0.02               # m: done
    tol_arm: float = 0.04           # m: done if the arm can reach it from here (arm adjustment)
    max_step: float = 0.06          # m per body step
    max_tries: int = 3              # body steps
    timeout_s: float = 60.0
    min_step: float = 0.005         # below this a body step is not commanded
    height_tol: float = 0.03        # m: |measured height - class height| above this = implausible (occluder)
    near: float = 0.07
    far: float = 1.0
    blob_min_h: float = 0.012       # m above the floor for the depth-blob box
    blob_min_px: int = 25
    top_view_deg: float = 50.0      # camera elevation above which the top-face centroid replaces the side-view fit
    top_band: float = 0.012         # m below the measured top counted as the top face


@dataclass
class Observation:
    ok: bool
    reason: str = ""                # "" | no_points | no_blob | implausible | no_floor
    xy: tuple | None = None         # object centre in base_link (x, y)
    centre: np.ndarray | None = None
    height: float | None = None
    box: tuple | None = None
    floor: tuple | None = None      # (n, d) in base_link
    partial: bool = False           # the object is cut by the image border: position only approximate


@dataclass
class ServoResult:
    ok: bool
    code: str                       # ALIGNED | ALIGNED_ARM | TARGET_LOST | OCCLUDED | NOT_ALIGNED | TIMEOUT
    steps: int = 0
    residual: float | None = None
    obs: Observation | None = None
    log: list = field(default_factory=list)


# ------------------------------------------------------------------ geometry
def floor_transform(n, d) -> np.ndarray:
    """T_floor_base: rotates the fitted floor normal n onto +z and moves the plane n.p + d = 0 to z = 0."""
    n = np.asarray(n, float) / np.linalg.norm(n)
    z = np.array([0.0, 0.0, 1.0])
    v, c = np.cross(n, z), float(n @ z)
    if np.linalg.norm(v) < 1e-12:
        R = np.eye(3)
    else:
        K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R = np.eye(3) + K + K @ K / (1 + c)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [0.0, 0.0, d]                     # a point on the plane: n.p = -d -> after rotation its z = -d -> +d
    return T


def box_from_depth(depth, K, T_floor_opt, prm: ServoParams):
    """Bounding box of the connected blob of pixels standing > blob_min_h above the floor that is nearest the image
    centre (works without a detector for the small classes). -> (x1, y1, x2, y2) or None."""
    H, W = depth.shape
    s = 2
    fx, fy, cx, cy = K
    vs, us = np.mgrid[0:H:s, 0:W:s]
    z = depth[vs, us].astype(float)
    ok = np.isfinite(z) & (z >= prm.near) & (z <= prm.far)
    P = np.stack([(us - cx) / fx * z, (vs - cy) / fy * z, z], -1)
    h = (P.reshape(-1, 3) @ T_floor_opt[:3, :3].T + T_floor_opt[:3, 3])[:, 2].reshape(z.shape)
    m = ok & (h > prm.blob_min_h)
    if m.sum() < prm.blob_min_px / 4:
        return None
    lab = -np.ones(m.shape, int)
    comps = []
    for i0, j0 in zip(*np.nonzero(m)):                   # flood fill on the decimated grid
        if lab[i0, j0] >= 0:
            continue
        k = len(comps)
        stack, pix = [(i0, j0)], []
        lab[i0, j0] = k
        while stack:
            i, j = stack.pop()
            pix.append((i, j))
            for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if 0 <= a < m.shape[0] and 0 <= b < m.shape[1] and m[a, b] and lab[a, b] < 0 \
                        and abs(z[a, b] - z[i, j]) < 0.03:
                    lab[a, b] = k
                    stack.append((a, b))
        comps.append(np.array(pix))
    comps = [c for c in comps if len(c) * 4 >= prm.blob_min_px]
    if not comps:
        return None
    ctr = np.array([m.shape[0] / 2, m.shape[1] / 2])
    best = min(comps, key=lambda c: float(np.linalg.norm(c.mean(0) - ctr)))
    return (float(best[:, 1].min() * s), float(best[:, 0].min() * s), float(best[:, 1].max() * s + s), float(best[:, 0].max() * s + s))


def observe(depth, K, T_base_opt, cls: str, box=None, prm: ServoParams | None = None) -> Observation:
    prm = prm or ServoParams()
    shape, rprior, h_cls, _ = CLASS_INFO[cls]
    lp = LO.LowObstacleParams(stride=2, min_range=prm.near, max_range=prm.far, floor_band=0.08)
    p_opt, _ = LO.depth_to_points(depth, K, lp)
    if len(p_opt) < 50:
        return Observation(False, "no_points")
    pts = LO.apply_T(T_base_opt, p_opt)
    n, d, fitted, _ = LO.fit_floor(pts, lp)
    if not fitted:
        return Observation(False, "no_floor")
    T_fb = floor_transform(n, d)
    T_fo = T_fb @ T_base_opt
    box = box or box_from_depth(depth, K, T_fo, prm)
    if box is None:
        return Observation(False, "no_blob", floor=(n, d))
    est = O3.estimate(depth, box, K, T_fo, O3.Object3DParams(near=prm.near, far=prm.far, floor_z=0.0, stride=1, min_points=10),
                      radius_prior=rprior, shape=shape)
    if est is None:
        return Observation(False, "no_points", box=box, floor=(n, d))
    centre_f = est.centre
    cam_f = T_fo[:3, 3]
    elev = math.atan2(cam_f[2] - est.centre[2], math.hypot(*(est.centre[:2] - cam_f[:2])))
    Hh, Ww = depth.shape
    partial = bool(box[0] < 3 or box[1] < 3 or box[2] > Ww - 3 or box[3] > Hh - 3)      # cut by the image border
    top_view = elev > math.radians(prm.top_view_deg)
    if top_view:
        # looking down: the whole top face is seen, so the 8c side-view fits (points on the outline) do not apply;
        # the xy centroid of the highest band of the object's own points is the centre of a convex object
        o3p = O3.Object3DParams(near=prm.near, far=prm.far, floor_z=0.0, stride=1)
        pts, z, _, uv = O3.box_points(depth, box, K, T_fo, o3p)
        keep = pts[:, 2] >= o3p.floor_tol
        pts, z, uv = pts[keep], z[keep], uv[keep]
        if len(pts) >= 10:
            m = O3.depth_cluster(z, o3p.gap, o3p.seed_pct, o3p.max_extent)
            pts, uv = pts[m], uv[m]
            top = float(np.percentile(pts[:, 2], 98))
            sel = pts[:, 2] >= top - prm.top_band
            band, buv = pts[sel], uv[sel]
            if len(band) >= 5:
                centre_f = np.array([band[:, 0].mean(), band[:, 1].mean(), est.centre[2]])
                # what matters looking down is whether the TOP FACE is complete (a tall object's lower body may leave the image)
                partial = bool(buv[:, 0].min() < 3 or buv[:, 1].min() < 3 or buv[:, 0].max() > Ww - 4 or buv[:, 1].max() > Hh - 4)
    c_base = LO.apply_T(np.linalg.inv(T_fb), centre_f)
    obs = Observation(True, "", (float(c_base[0]), float(c_base[1])), c_base, est.height, box, (n, d), partial=partial)
    # looking down, the height (top above the fitted floor) is measured whenever the object is fully in the image; the
    # 8c 'top' flag (pixels at the object's depth above the box) misfires there because that is the floor
    height_known = (top_view and not partial) or (not top_view and est.top_known)
    too_tall = est.height > h_cls + prm.height_tol          # even a cut view's height is a lower bound: conclusive
    if too_tall or (height_known and abs(est.height - h_cls) > prm.height_tol):
        obs.ok, obs.reason = False, "implausible"
    return obs


# ------------------------------------------------------------------ servo loop
class VisualServo:
    """robot interface: wrist_view() -> (depth, K, T_base_opt); step(dx, dy) (body step in base_link, m); now() (s).
    arm_reachable(cls, xy) -> bool is optional (arm adjustment)."""

    def __init__(self, robot, prm: ServoParams | None = None, arm_reachable=None):
        self.r, self.prm, self.arm_ok = robot, prm or ServoParams(), arm_reachable

    def _step(self, v, res, why):
        n = float(np.hypot(*v))
        if n < self.prm.min_step:
            return
        if n > self.prm.max_step:
            v = np.asarray(v) * self.prm.max_step / n
        self.r.step(float(v[0]), float(v[1]))
        res.log.append({"t": round(self.r.now(), 1), "step": [round(float(v[0]), 4), round(float(v[1]), 4)], "why": why})

    def run(self, cls: str, prior_xy=None, box_fn=None) -> ServoResult:
        p = self.prm
        target = np.array(CLASS_INFO[cls][3])
        res = ServoResult(False, "NOT_ALIGNED")
        t0 = self.r.now()
        prior = None if prior_xy is None else np.array(prior_xy, float)
        lost = occl = 0
        while True:
            if self.r.now() - t0 > p.timeout_s:
                res.code = "TIMEOUT"
                return res
            depth, K, T = self.r.wrist_view()
            obs = observe(depth, K, T, cls, box_fn(depth) if box_fn else None, p)
            res.obs = obs
            res.log.append({"t": round(self.r.now(), 1), "obs": obs.reason or "ok", "xy": obs.xy})
            if not obs.ok and obs.reason == "implausible":
                occl += 1
                if occl > 1 or res.steps >= p.max_tries:
                    res.code = "OCCLUDED"
                    return res
                self._step((0.0, 0.03 if (obs.xy or (0, 0))[1] <= 0 else -0.03), res, "occluded: sideways")
                res.steps += 1
                continue
            if not obs.ok:
                lost += 1
                if prior is None or lost > 2 or res.steps >= p.max_tries:
                    res.code = "TARGET_LOST"
                    return res
                e = prior - target                                  # step toward where the object should be
                self._step(e, res, "lost: toward prior")
                moved = np.asarray(res.log[-1]["step"]) if "step" in res.log[-1] else np.zeros(2)
                prior = prior - moved
                res.steps += 1
                continue
            lost = 0
            prior = np.array(obs.xy)
            err = np.array(obs.xy) - target
            res.residual = float(np.hypot(*err))
            if res.residual <= p.tol and not obs.partial:
                res.ok, res.code = True, "ALIGNED"
                return res
            if res.residual <= p.tol_arm and not obs.partial and self.arm_ok and all(
                    self.arm_ok(cls, (obs.xy[0] + dx, obs.xy[1] + dy)) for dx, dy in ((0, 0), (0.01, 0), (-0.01, 0), (0, 0.01), (0, -0.01))):
                # the arm reaches it from here, with 1 cm margin for the estimate error: no more body steps
                res.ok, res.code = True, "ALIGNED_ARM"
                return res
            if res.steps >= p.max_tries:
                res.code = "NOT_ALIGNED"
                return res
            self._step(err, res, "align")
            res.steps += 1
