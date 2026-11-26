"""Simple kinematic robot for the visual-servo tests: holonomic body steps with execution error, wrist camera from the
arm model (ready pose), synthetic depth from gurdy_core.vision.synth_depth (world floor z = 0, base_link 0.11 m above)."""
import math

import numpy as np

from gurdy_core import arm as A
from gurdy_core.vision.synth_depth import render
from gurdy_core.vision.transforms import T_from_xyz_rpy

SOLIDS = {   # class -> solid template (world, standing on the floor)
    "small_box": lambda x, y, yaw: dict(kind="box", centre=(x, y, 0.025), size=(0.05, 0.05, 0.05), yaw=yaw),
    "mini_toolbox": lambda x, y, yaw: [dict(kind="box", centre=(x, y, 0.03), size=(0.12, 0.05, 0.06), yaw=yaw),
                                       *[dict(kind="box", centre=(x + math.cos(yaw) * dx, y + math.sin(yaw) * dx, z), size=sz, yaw=yaw)
                                         for dx, z, sz in ((-0.035, 0.0725, (0.01, 0.02, 0.025)), (0.035, 0.0725, (0.01, 0.02, 0.025)),
                                                           (0.0, 0.09, (0.08, 0.02, 0.01)))]],   # raised handle (vision/small_objects.py)
    "can": lambda x, y, yaw: dict(kind="cylinder", centre=(x, y, 0.06), radius=0.033, height=0.12),
    "mini_cone": lambda x, y, yaw: dict(kind="cylinder", centre=(x, y, 0.05), radius=0.022, height=0.10),
    "mini_extinguisher": lambda x, y, yaw: dict(kind="cylinder", centre=(x, y, 0.075), radius=0.025, height=0.15),
}


class SimRobot:
    def __init__(self, obj_cls, obj_rel, yaw_obj=0.4, step_gain=(0.9, 1.1), step_noise=0.005, seed=0, base_z=0.11,
                 tilt=(0.0, 0.0), extra=(), depth_noise=0.0):
        self.a = A.load_arm()
        from gurdy_core.visual_servo import VIEW_POSE
        self.q = self.a.poses[VIEW_POSE[obj_cls]]
        self.T_bo = A.T_base_wrist_optical(self.a, self.q)
        cam = self.a.cam
        f = (cam["width"] / 2) / math.tan(cam["hfov"] / 2)
        self.K, self.W, self.H = (f, f, cam["width"] / 2, cam["height"] / 2), cam["width"], cam["height"]
        self.rng = np.random.default_rng(seed)
        self.x, self.y, self.yaw = 0.0, 0.0, 0.3
        self.base_z, self.tilt, self.t = base_z, tilt, 0.0
        self.gain, self.noise, self.dn = step_gain, step_noise, depth_noise
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        ox, oy = obj_rel
        self.obj = (self.x + c * ox - s * oy, self.y + s * ox + c * oy)
        o = SOLIDS[obj_cls](*self.obj, yaw_obj)
        self.solids = (o if isinstance(o, list) else [o]) + [e(self) if callable(e) else e for e in extra]

    def T_world_base(self):
        return T_from_xyz_rpy((self.x, self.y, self.base_z), (self.tilt[0], self.tilt[1], self.yaw))

    def wrist_view(self):
        self.t += 0.5
        Tw = self.T_world_base() @ self.T_bo
        d, _, _ = render(self.K, self.W, self.H, Tw, self.solids, near=0.07, far=1.0, noise=self.dn, rng=self.rng)
        return d, self.K, self.T_bo

    def step(self, dx, dy):
        g = self.rng.uniform(*self.gain)
        dx, dy = dx * g + self.rng.normal(0, self.noise), dy * g + self.rng.normal(0, self.noise)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x, self.y = self.x + c * dx - s * dy, self.y + s * dx + c * dy
        self.t += math.hypot(dx, dy) / 0.05 + 2.0

    def now(self):
        return self.t

    def obj_rel(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        ex, ey = self.obj[0] - self.x, self.obj[1] - self.y
        return np.array([c * ex + s * ey, -s * ex + c * ey])
