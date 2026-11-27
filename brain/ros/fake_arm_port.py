"""FakeArmPort: test double of ArmPort over brain.ros.fake_port.FakePort (no ROS). Kinematic arm (goals reached exactly
after `seconds`), synthetic wrist depth (gurdy_core.vision.synth_depth: world floor z 0, base_link 0.11 m above), and
contact-like grasp rules:

  * squeezing (effort < 0) holds an object when the grasp centre (gurdy_core.arm.fk 'Gc') is within 1.5 cm in xy and 2 cm
    in height of the object's grasp point; the fingers stop at half the grip width; otherwise they close to 0
  * a held object follows the grasp centre; opening (effort > 0) puts it down where it is (on the floor)
  * faults: miss {names} (the fingers slide off), slip {names} (falls out on the lift), drop_carry {names} (falls out
    after 0.5 m of walking), drop_silent {names} (falls out after 0.5 m but the fingers stay at its width: jammed, only
    ground truth sees it), jam (bool: the gripper does not open), user_takes (bool, default True: while presenting with a light grip the user takes it after 2 s)
World objects: [{name, class, color, x, y, yaw}] (world = map frame); their semantic-map entries are kept in FakePort.objects
(id = index + 1, so the fetch robot's names are obj1, obj2, ...). `unseen` names are NOT in the semantic map until the fake
explorer reveals them (FakePort.reveal, during an explore action). `estop_at`: latch the e-stop at that sim time.
"""
from __future__ import annotations

import math

import numpy as np

from gurdy_core import arm as A
from gurdy_core.vision.synth_depth import render
from gurdy_core.vision.transforms import T_from_xyz_rpy

BASE_Z = 0.11


def _solids(o):
    x, y, yaw, cls, z0 = o["x"], o["y"], o.get("yaw", 0.0), o["class"], o.get("z0", 0.0)
    if cls == "small_box":
        return [dict(kind="box", centre=(x, y, z0 + 0.025), size=(0.05, 0.05, 0.05), yaw=yaw)]
    if cls == "mini_toolbox":
        c, s = math.cos(yaw), math.sin(yaw)
        return [dict(kind="box", centre=(x, y, z0 + 0.03), size=(0.12, 0.05, 0.06), yaw=yaw),
                *[dict(kind="box", centre=(x + c * dx, y + s * dx, z0 + z), size=sz, yaw=yaw)
                  for dx, z, sz in ((-0.035, 0.0725, (0.01, 0.02, 0.025)), (0.035, 0.0725, (0.01, 0.02, 0.025)),
                                    (0.0, 0.09, (0.08, 0.02, 0.01)))]]
    r, h = {"can": (0.033, 0.12), "mini_cone": (0.022, 0.10), "mini_extinguisher": (0.025, 0.15)}.get(cls, (0.05, 0.2))
    return [dict(kind="cylinder", centre=(x, y, z0 + h / 2), radius=r, height=h)]


class FakeArmPort:
    def __init__(self, fake, objects: list[dict], faults: dict | None = None, moveit: bool = True, user=(0.0, 0.0),
                 unseen=(), estop_at: float | None = None):
        self.f, self.faults, self.mg = fake, faults or {}, moveit
        self.a = A.load_arm()
        self.q = np.array(self.a.poses["folded"], float)
        self.objs = [dict(o, id=i + 1) for i, o in enumerate(objects)]
        self.unseen = set(unseen)
        self.estop_at = estop_at
        fake.reveal = [(20.0 + 10 * k, self._entry(o)) for k, o in enumerate(o for o in self.objs if o["name"] in self.unseen)]
        self.finger = (0.04, 0.04)
        self.held = None
        self.held_since_xy = None
        self.present_t = None
        self.user = user
        self.speed_limits: list = []
        self.base_cruise = fake.cruise
        self.moves: list = []
        self.scene: list = []                     # MoveIt scene calls: ("floor", ...) / ("target", cls, xy, on)
        self.sag = np.asarray(self.faults.get("sag", np.zeros(5)), float)   # constant joint offset of the effort-controlled arm (rad)
        cam = self.a.cam
        fx = (cam["width"] / 2) / math.tan(cam["hfov"] / 2)
        self.K, self.W, self.H = (fx, fx, cam["width"] / 2, cam["height"] / 2), cam["width"], cam["height"]
        self._sync()

    # ---- helpers
    @staticmethod
    def _entry(o):
        return {"id": o["id"], "class": o["class"], "color": o.get("color", ""), "x": o["x"], "y": o["y"],
                "z": 0.05, "hits": 20, "score": 0.9, "last_seen": o.get("seen", 0.0)}

    def _sync(self):
        revealed = {e["id"] for e in self.f.objects}
        self.unseen -= {o["name"] for o in self.objs if o["id"] in revealed}
        self.f.objects = [self._entry(o) for o in self.objs if o["name"] not in self.unseen]

    def _T_world_base(self):
        x, y, yaw = self.f.pose()
        return T_from_xyz_rpy((x, y, BASE_Z), (0.0, 0.0, yaw))

    def _gc_world(self, q=None):
        g = A.fk(self.a, self.q if q is None else q)["Gc"]
        return (self._T_world_base() @ np.array([*g, 1.0]))[:3]

    def _follow(self):
        if self.held is not None:
            g = self._gc_world()
            o = self.held
            o["x"], o["y"] = float(g[0]), float(g[1])
            o["z0"] = max(0.0, float(g[2]) - A.grasp_point(A.PICKABLE[o["class"]]))

    def _put_down(self):
        o, self.held = self.held, None
        if o is not None:
            o["z0"], o["seen"] = 0.0, self.f.now()
        self._sync()
        return o

    # ---- ArmPort
    def arm_ready(self):
        return True

    def moveit_available(self):
        return self.mg

    def arm_q(self):
        return [float(v) for v in self.q]

    def move_arm(self, qs, seconds, use_moveit=False):
        self.moves.append({"t": self.f.now(), "n": len(qs), "moveit": bool(use_moveit)})
        z0 = self._gc_world()[2]
        self.f.advance(seconds)
        self.q = np.array(qs[-1], float) + self.sag            # where the arm really settles
        if self.held is not None and self.held["name"] in self.faults.get("slip", set()) and self._gc_world()[2] - z0 > 0.02:
            self._put_down()
            self.finger = (0.0, 0.0)
        self._follow()
        return True

    def gripper(self, effort):
        if effort > 0 and self.faults.get("jam") and self.held is not None:
            return                                                  # jammed: stays closed on the object
        if effort > 0:
            self._put_down()
            self.finger, self.present_t = (0.04, 0.04), None
            return
        if self.held is not None:
            self.present_t = self.f.now() if abs(effort) < 2.0 else None      # light grip = presenting to the user
            return
        g = self._gc_world()
        for o in self.objs:
            p = A.PICKABLE[o["class"]]
            zg = A.grasp_point(p)
            if math.hypot(g[0] - o["x"], g[1] - o["y"]) <= 0.015 and abs(g[2] - zg) <= 0.02 and o["name"] not in self.faults.get("miss", set()):
                self.held, self.held_since_xy = o, self.f.pose()[:2]
                self.finger = (p["grip_w"] / 2 - 0.001,) * 2
                return
        self.finger = (0.0, 0.0)

    def fingers(self):
        return self.finger

    def wrist_view(self):
        Tw = self._T_world_base() @ A.T_base_wrist_optical(self.a, self.q)
        solids = [s for o in self.objs if o is not self.held for s in _solids(o)]
        d, _, _ = render(self.K, self.W, self.H, Tw, solids, near=0.07, far=1.0)
        return d, self.K, A.T_base_wrist_optical(self.a, self.q)

    def set_speed_limit(self, mps):
        self.speed_limits.append(mps)
        self.f.cruise = self.base_cruise if mps is None else min(self.base_cruise, mps)

    def settle(self, q, tol=0.003, timeout=3.0):
        self.f.advance(0.5)
        return float(np.max(np.abs(self.q - np.asarray(q, float))))

    def add_floor(self, margin=0.005):
        self.scene.append(("floor", margin))
        return {"added": bool(self.mg), "fitted": True, "z0": -0.108}

    def target_in_scene(self, cls, xy_base, on=True):
        self.scene.append(("target", cls, tuple(round(float(v), 4) for v in xy_base), bool(on)))
        return bool(self.mg)

    def wait(self, seconds):
        self.f.advance(seconds)
        if self.estop_at is not None and self.f.now() >= self.estop_at:
            self.f.set_estop(True)
        if self.unseen:
            self._sync()
        self._follow()
        if self.held is not None:
            x, y = self.f.pose()[:2]
            walked = math.hypot(x - self.held_since_xy[0], y - self.held_since_xy[1])
            if self.held["name"] in self.faults.get("drop_carry", set()) and walked > 0.5:
                self._put_down()
                self.finger = (0.0, 0.0)
            elif self.held["name"] in self.faults.get("drop_silent", set()) and walked > 0.5:
                self._put_down()                                    # fingers keep their width
            elif self.present_t is not None and self.faults.get("user_takes", True) and self.f.now() - self.present_t >= 2.0:
                o, self.held = self.held, None
                o["x"], o["y"], o["z0"] = self.user[0], self.user[1], 0.0
                self.objs.remove(o)                                 # with the user: no longer in the world
                self._sync()
                self.finger = (0.0, 0.0)
