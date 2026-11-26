"""Phase 8c convention checks (read-only, run in a live stack with perception): camera_info K vs intrinsics_from_fov, and floor pixels
back-projected through the REAL TF (map <- depth frame_id) as z-depth and as ray range -> map z. Prints JSON."""
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, sys.argv[1])
import transforms as T  # noqa: E402

W, H, HFOV = 320, 240, 1.047


class C(Node):
    def __init__(self):
        super().__init__("p8c_conv_check")
        self.info, self.depth = None, []
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(CameraInfo, "/camera/camera_info", lambda m: setattr(self, "info", m), qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth_image", self.on_depth, qos_profile_sensor_data)

    def on_depth(self, m):
        if len(self.depth) >= 30:
            return
        try:
            tr = self.tf.lookup_transform("map", m.header.frame_id, rclpy.time.Time())
        except Exception:
            return
        a = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width).copy()
        t, q = tr.transform.translation, tr.transform.rotation
        self.depth.append((a, T.T_from_tq((t.x, t.y, t.z), (q.x, q.y, q.z, q.w)), m.header.frame_id))


def main():
    rclpy.init()
    n = C()
    end = time.time() + 60
    while time.time() < end and (n.info is None or len(n.depth) < 30):
        rclpy.spin_once(n, timeout_sec=0.1)
    res = {"n_frames": len(n.depth)}
    if n.info is not None:
        K = T.K_from_camera_info(n.info.k)
        res.update(camera_info_K=K, size=[n.info.width, n.info.height], frame=n.info.header.frame_id, distortion=list(n.info.d),
                   fov_half=T.intrinsics_from_fov(W, H, HFOV, "half"), fov_pixel=T.intrinsics_from_fov(W, H, HFOV, "pixel"))
    if n.depth and n.info is not None:
        K = T.K_from_camera_info(n.info.k)
        vs, us = np.mgrid[180:240:4, 40:280:8]                 # lower image band = floor in front of the robot (objects excluded by z < 0.1 below)
        rays = T.pixel_rays(K, us.ravel(), vs.ravel())
        zs_depth, zs_range, per_row = [], [], {}
        for a, Tm, fid in n.depth:
            d = a[vs.ravel(), us.ravel()]
            ok = np.isfinite(d) & (d > 0.05)
            for kind, pts in (("zdepth", rays[ok] * d[ok, None]), ("range", rays[ok] / np.linalg.norm(rays[ok], axis=1)[:, None] * d[ok, None])):
                z = T.apply_T(Tm, pts)[:, 2]
                (zs_depth if kind == "zdepth" else zs_range).append(z)
            z = T.apply_T(Tm, rays[ok] * d[ok, None])[:, 2]
            for v, zz in zip(vs.ravel()[ok], z):
                per_row.setdefault(int(v), []).append(float(zz))
        zd, zr = np.concatenate(zs_depth), np.concatenate(zs_range)
        pct = lambda z: [round(float(x), 4) for x in np.percentile(z, [5, 50, 95])]  # noqa: E731
        res.update(depth_frame=n.depth[0][2], floor_z_as_zdepth_p5_50_95=pct(zd), floor_z_as_range_p5_50_95=pct(zr), n_pixels=int(zd.size),
                   floor_z_by_row_median_zdepth={k: round(float(np.median(v)), 4) for k, v in sorted(per_row.items())})
    print(json.dumps(res, indent=1))
    n.destroy_node()
    rclpy.try_shutdown()


main()
