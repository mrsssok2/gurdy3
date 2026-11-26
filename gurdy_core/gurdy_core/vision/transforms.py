"""Rigid transforms and camera intrinsics for 3D object positions (numpy, no ROS).

Conventions (same as ROS / the URDF):
  * 4x4 homogeneous matrices `T_a_b` map points from frame b into frame a:  p_a = T_a_b @ p_b.
  * quaternions are (x, y, z, w) like geometry_msgs; rpy = fixed-axis roll, pitch, yaw (URDF `rpy`): R = Rz(y) Ry(p) Rx(r).
  * optical frame: z forward, x right, y down. camera_link: x forward (gz cameras look along +x of the link).
Chain on Gurdy (gurdy.urdf.xacro): base_link -> camera_link (xyz = sensors.depth_camera.pos, rpy = (0, pitch_down, 0))
-> camera_optical_frame (rpy = (-pi/2, 0, -pi/2)).
"""
from __future__ import annotations

import math

import numpy as np


def rpy_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quat_to_R(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        raise ValueError("zero quaternion")
    s = 2.0 / n
    return np.array([[1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
                     [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
                     [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


def R_to_quat(R) -> tuple:
    R = np.asarray(R, float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2.0 * math.sqrt(tr + 1.0)
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    q = q / np.linalg.norm(q)
    return tuple(float(v) for v in (q if q[3] >= 0 else -q))


def make_T(R=None, t=(0.0, 0.0, 0.0)) -> np.ndarray:
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = np.asarray(R, float)
    T[:3, 3] = np.asarray(t, float)
    return T


def T_from_tq(t, q) -> np.ndarray:
    """geometry_msgs Transform fields (translation xyz, rotation xyzw) -> 4x4. A TF lookup_transform(target, source)
    result gives T_target_source."""
    return make_T(quat_to_R(q), t)


def T_from_xyz_rpy(xyz, rpy) -> np.ndarray:
    return make_T(rpy_to_R(*rpy), xyz)


def inv_T(T) -> np.ndarray:
    T = np.asarray(T, float)
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def compose(*Ts) -> np.ndarray:
    out = np.eye(4)
    for T in Ts:
        out = out @ np.asarray(T, float)
    return out


def apply_T(T, pts) -> np.ndarray:
    """Points (3,) or (N, 3) -> same shape in the target frame."""
    p = np.asarray(pts, float)
    T = np.asarray(T, float)
    if p.ndim == 1:
        return T[:3, :3] @ p + T[:3, 3]
    return p @ T[:3, :3].T + T[:3, 3]


def rotate_cov(T, cov) -> np.ndarray:
    R = np.asarray(T, float)[:3, :3]
    return R @ np.asarray(cov, float) @ R.T


OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)          # camera_link -> camera_optical_frame


def T_base_optical(cam: dict) -> np.ndarray:
    """robot.yaml `sensors.depth_camera` (pos, pitch_down) -> T_base_link_camera_optical_frame, as in gurdy.urdf.xacro."""
    T_bc = T_from_xyz_rpy(cam["pos"], (0.0, float(cam.get("pitch_down", 0.0)), 0.0))
    return T_bc @ T_from_xyz_rpy((0.0, 0.0, 0.0), OPTICAL_RPY)


def T_map_base(x: float, y: float, yaw: float, z: float = 0.0, roll: float = 0.0, pitch: float = 0.0) -> np.ndarray:
    """Robot pose in the map (planar pose + optional body height / tilt) -> T_map_base_link."""
    return T_from_xyz_rpy((x, y, z), (roll, pitch, yaw))


# ------------------------------------------------------------------ intrinsics
def intrinsics_from_fov(width: int, height: int, hfov: float, centre: str = "half") -> tuple:
    """Pinhole (fx, fy, cx, cy) for square pixels. centre "half": (w/2, h/2); "pixel": ((w-1)/2, (h-1)/2).
    Which one Gazebo's camera_info uses is NOT verified yet: the ROS node must use /camera/camera_info (K), this is for
    tests and offline tools only."""
    fx = (width / 2.0) / math.tan(hfov / 2.0)
    if centre == "half":
        return fx, fx, width / 2.0, height / 2.0
    return fx, fx, (width - 1) / 2.0, (height - 1) / 2.0


def K_from_camera_info(k9) -> tuple:
    return float(k9[0]), float(k9[4]), float(k9[2]), float(k9[5])


def pixel_rays(K, us, vs) -> np.ndarray:
    """Pixel coordinates -> optical-frame ray directions with z = 1 (so depth-image value z gives the point z * ray)."""
    fx, fy, cx, cy = K
    us, vs = np.asarray(us, float), np.asarray(vs, float)
    return np.stack([(us - cx) / fx, (vs - cy) / fy, np.ones_like(us)], axis=-1)


def project(K, pts_optical) -> np.ndarray:
    """Optical-frame points (N, 3) with z > 0 -> pixel (N, 2) (u, v)."""
    p = np.atleast_2d(np.asarray(pts_optical, float))
    fx, fy, cx, cy = K
    return np.stack([fx * p[:, 0] / p[:, 2] + cx, fy * p[:, 1] / p[:, 2] + cy], axis=1)
