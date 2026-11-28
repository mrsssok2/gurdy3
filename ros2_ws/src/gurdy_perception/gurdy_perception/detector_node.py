"""detector_node: RGB-D image -> detections -> 3D in the map frame -> semantic map (markers + JSON).

    /camera/image (rgb8) + /camera/depth_image (32FC1) + /camera/camera_info
        -> ONNX detector (gurdy_core.vision.detector)
        -> per detection: robust box depth -> back-project -> object centre (+ class radius prior)
        -> TF camera_optical_frame -> target_frame (map) at the IMAGE stamp
        -> SemanticMap (de-duplication, confirmation after min_hits sightings)
    out:  /detector/detections      vision_msgs/Detection3DArray  (this frame's detections, target frame)
          /semantic_map/markers     visualization_msgs/MarkerArray (confirmed objects: cube + label)
          /semantic_map/objects     std_msgs/String JSON (transient_local; for find_object / go_to)
          /detector/debug_image     sensor_msgs/Image (only with debug_image:=true)

Time: the node does NOT use /clock; it takes sim time from the image stamps (same convention as policy_node), so TF
lookups use the image stamp with `tf_timeout` and need the sim clock on the TF side only.
Edge-truncated detections are mapped with a lower weight (`truncated_weight`).

Position method (parameter `position_method`):
  * `pixel` (DEFAULT, Phase 4, unchanged): one robust box depth, one back-projected pixel + class radius prior.
  * `object3d` (Phase 8c): all depth pixels of the box -> floor removal against the floor plane fitted in the target
    frame each frame (`floor_z` prior -0.11 m = measured map floor; `floor_fit`) -> background removal -> class shape
    fit (rectangle / circle; `object3d_shape_fit`) -> 3D centre + covariance + top / height / width / yaw, fused per
    track (Track3D). A detection for which object3d finds too few object points falls back to the pixel method.
    /detector/detections then carries the measured centre, size and yaw; the semantic map JSON gets top_z, height,
    size, yaw, sigma_xyz, ... (gurdy_core.vision.semantic_map.Track.summary3d).
"""
import json
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from gurdy_core.vision.classes import COLORS_ALL as COLORS, SIZE_PRIOR_ALL as SIZE_PRIOR, radius_prior
from gurdy_core.vision.colors import color_name
from gurdy_core.vision.detector import OnnxDetector
from gurdy_core.vision.geometry import (K_from_camera_info, backproject, box_depth, surface_to_centre, transform_point, truncation_flags,
                                        visible_centre_u)
from gurdy_core.vision.locate3d import FloorParams, FloorTracker, locate, plane_z_at, size_of
from gurdy_core.vision.object3d import Object3DParams
from gurdy_core.vision.transforms import R_to_quat, T_from_tq
from gurdy_core.vision.semantic_map import SemanticMap

DEFAULT_MODEL = "det_v31_s.onnx"  # 12 classes (Phase 9b v3.1, 2026-11-28); rollback: model:=det_v3_s.onnx (or det_v2_s.onnx, 7 classes; names follow the ONNX)


def image_to_numpy(m: Image) -> np.ndarray:
    """sensor_msgs/Image (rgb8 or 32FC1) -> numpy without cv_bridge."""
    if m.encoding in ("rgb8", "bgr8"):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step // 3, 3)[:, :m.width]
        return a[..., ::-1] if m.encoding == "bgr8" else a
    if m.encoding == "32FC1":
        return np.frombuffer(m.data, np.float32).reshape(m.height, m.step // 4)[:, :m.width]
    if m.encoding == "16UC1":
        return np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, :m.width].astype(np.float32) / 1000.0
    raise ValueError(f"unsupported image encoding {m.encoding}")


def _stamp(h) -> float:
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector_node")
        P = self.declare_parameter
        P("model", ""); P("providers", "auto"); P("conf", 0.40); P("iou", 0.5)
        P("target_frame", "map"); P("max_range", 4.0); P("min_range", 0.25); P("max_rate_hz", 5.0)
        P("min_hits", 8); P("edge_margin", 3.0); P("tf_timeout", 0.2)
        # ttl: an UNCONFIRMED track is forgotten this long (sim s) after its last sighting; confirmed tracks stay. 30 s (was
        # 10, 2026-11-28): a small object seen in two short scan passes (6 + 5 sightings, 29 s apart) never reached 8 hits;
        # replay of the day's fetch logs: 30 s confirms it with no extra false track (vision/replay_fetch_sightings.py)
        P("ttl", 30.0)
        P("truncated_weight", 0.25)      # position weight of edge-truncated sightings (1 = like a full box, 0 = skip them)
        P("min_span", 2.0); P("min_mean_score", 0.6); P("max_scatter", 0.5)   # track confirmation (semantic_map.py)
        P("dominance_ratio", 4.0); P("dominance_radius", 0.35)   # a sparse track on top of a >= 4x better seen one is a class confusion
        P("log_path", "")                # JSON-lines log of every sighting, for offline replay of the confirmation rules
        P("frame_dir", "")               # save body RGB frames + sidecar (camera pose, this frame's detections) for VLM replay; "" = off
        P("frame_every_s", 1.0); P("frame_max", 3000)            # (gurdy_core/vision/frame_saver.py)
        P("debug_image", False); P("save_path", "")
        P("position_method", "pixel")    # pixel (Phase 4, default) | object3d (Phase 8c)
        P("floor_z", -0.11); P("floor_fit", True); P("floor_hold_s", 3.0)   # floor plane prior / RANSAC fit in target_frame
        P("object3d_stride", 2); P("object3d_shape_fit", True)
        model = self.get_parameter("model").value or DEFAULT_MODEL
        if not os.path.isabs(model) and not os.path.exists(model):
            model = os.path.join(os.environ.get("GURDY_REPO", os.path.expanduser("~/gurdy2")), "models", "detector", model)
        g = lambda k: self.get_parameter(k).value
        self.det = OnnxDetector(model, g("providers"), conf=g("conf"), iou=g("iou"))
        self.get_logger().info(f"detector {os.path.basename(model)} on {self.det.provider}, input {self.det.hw}")
        self.map = SemanticMap(min_hits=g("min_hits"), ttl=g("ttl"), min_span=g("min_span"), min_mean_score=g("min_mean_score"),
                               max_scatter=g("max_scatter"),
                               dominance_ratio=g("dominance_ratio"), dominance_radius=g("dominance_radius"))
        self.log = open(g("log_path"), "w") if g("log_path") else None
        self.frames = None
        if g("frame_dir"):
            from gurdy_core.vision.frame_saver import FrameSaver
            self.frames = FrameSaver(g("frame_dir"), every_s=float(g("frame_every_s")), max_frames=int(g("frame_max")))
            self.get_logger().info(f"saving RGB frames to {g('frame_dir')} (every {g('frame_every_s')} s sim, at most {g('frame_max')})")
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.K = None
        self.last_t = -1e9
        self.n = {"frames": 0, "tf_fail": 0, "depth_fail": 0, "used": 0, "truncated": 0, "o3d": 0, "o3d_fallback": 0}
        self.method = g("position_method")
        if self.method not in ("pixel", "object3d"):
            raise ValueError(f"position_method must be pixel or object3d, not {self.method!r}")
        self.floor = FloorTracker(FloorParams(floor_z=g("floor_z"), fit=g("floor_fit"), hold_s=g("floor_hold_s")))
        self.o3p = Object3DParams(stride=int(g("object3d_stride")), far=float(g("max_range")) + 1.0)
        self.get_logger().info(f"position method {self.method}" + (
            f" (floor prior z {g('floor_z')}, fit {g('floor_fit')}, shape fit {g('object3d_shape_fit')})" if self.method == "object3d" else ""))
        self.create_subscription(CameraInfo, "camera/camera_info", self.on_info, qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([Subscriber(self, Image, "camera/image", qos_profile=qos_profile_sensor_data),
                                                 Subscriber(self, Image, "camera/depth_image", qos_profile=qos_profile_sensor_data)],
                                                queue_size=5, slop=0.06)
        self.sync.registerCallback(self.on_images)
        self.pub_det = self.create_publisher(Detection3DArray, "detector/detections", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "semantic_map/markers", 1)
        self.pub_json = self.create_publisher(String, "semantic_map/objects", QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.pub_dbg = self.create_publisher(Image, "detector/debug_image", 1) if g("debug_image") else None
        self.pub_boxes = self.create_publisher(String, "detector/boxes", 1)      # 2D boxes of the last processed frame (dashboard overlay)
        self.create_timer(5.0, self.log_stats)

    def on_info(self, m: CameraInfo):
        self.K = K_from_camera_info(m.k)

    def on_images(self, rgb_m: Image, depth_m: Image):
        if self.K is None:
            return
        t = _stamp(rgb_m.header)
        if t - self.last_t < 1.0 / self.get_parameter("max_rate_hz").value:
            return
        self.last_t = t
        self.n["frames"] += 1
        rgb, depth = image_to_numpy(rgb_m), image_to_numpy(depth_m)
        dets = self.det(np.ascontiguousarray(rgb))
        self.pub_boxes.publish(String(data=json.dumps({
            "stamp": t, "w": int(rgb.shape[1]), "h": int(rgb.shape[0]),
            "boxes": [{"name": d.name, "score": round(float(d.score), 3), "box": [round(float(v), 1) for v in d.box]} for d in dets]})))
        frame = self.get_parameter("target_frame").value
        try:
            tr = self.tf.lookup_transform(frame, rgb_m.header.frame_id, rgb_m.header.stamp,
                                          Duration(seconds=self.get_parameter("tf_timeout").value))
        except Exception as e:                                   # noqa: BLE001  (extrapolation / missing frame)
            self.n["tf_fail"] += 1
            if self.n["tf_fail"] in (1, 20, 200):
                self.get_logger().warn(f"TF {frame} <- {rgb_m.header.frame_id} unavailable: {e}")
            return
        t_xyz = [tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z]
        q = [tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w]
        out = Detection3DArray()
        out.header.stamp, out.header.frame_id = rgb_m.header.stamp, frame
        H, W = rgb.shape[:2]
        em, rmin, rmax = self.get_parameter("edge_margin").value, self.get_parameter("min_range").value, self.get_parameter("max_range").value
        use3d = self.method == "object3d" and len(dets) > 0
        if use3d:
            T_map_opt = T_from_tq(t_xyz, q)
            plane = self.floor.update(depth, self.K, T_map_opt, t)
        frame_dets = []                         # this frame's detections for the frame saver (VLM replay facts)
        for d in dets:
            x1, y1, x2, y2 = d.box
            z, _ = box_depth(depth, d.box)
            if z is None or not rmin <= z <= rmax:
                self.n["depth_fail"] += 1
                continue
            trunc = truncation_flags(d.box, W, H, em)
            u = visible_centre_u(d.box, W, H, z, self.K[0], 0.5 * max(SIZE_PRIOR[d.name][:2]), em)
            if u is None:                       # wider than the view: no usable position
                continue
            v = (y1 + y2) / 2
            p_opt = surface_to_centre(backproject(u, v, z, self.K), radius_prior(d.name))
            s3 = None
            if use3d:
                s3 = locate(depth, d.box, self.K, T_map_opt, d.name, plane, self.o3p,
                            use_shape=self.get_parameter("object3d_shape_fit").value)
                self.n["o3d" if s3 is not None else "o3d_fallback"] += 1
            p = s3.centre if s3 is not None else transform_point(p_opt, t_xyz, q)
            det = Detection3D()
            det.header = out.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id, hyp.hypothesis.score = d.name, d.score
            hyp.pose.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            if s3 is not None:
                for i in range(3):
                    for j in range(3):
                        hyp.pose.covariance[6 * i + j] = float(s3.cov[i, j])
            det.results.append(hyp)
            det.bbox.center.position = hyp.pose.pose.position
            if s3 is not None:
                det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = (float(v_) for v_ in size_of(d.name, s3.height if s3.top_known else None))
                if s3.yaw is not None:
                    qx, qy, qz, qw = R_to_quat(np.array([[np.cos(s3.yaw), -np.sin(s3.yaw), 0], [np.sin(s3.yaw), np.cos(s3.yaw), 0], [0, 0, 1]]))
                    o = det.bbox.center.orientation
                    o.x, o.y, o.z, o.w = qx, qy, qz, qw
            else:
                det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = SIZE_PRIOR[d.name]
            out.detections.append(det)
            truncated = any(trunc.values())
            wscale = self.get_parameter("truncated_weight").value if truncated else 1.0
            if wscale > 0:
                crop = rgb[int(y1 + 0.25 * (y2 - y1)):int(y2 - 0.25 * (y2 - y1)) + 1, int(x1 + 0.25 * (x2 - x1)):int(x2 - 0.25 * (x2 - x1)) + 1]
                col = color_name(crop)
                frame_dets.append({"cls": d.name, "score": round(float(d.score), 4), "box": [round(float(c), 1) for c in d.box], "color": col,
                                   "p": [round(float(c), 4) for c in p], "truncated": truncated})
                self.map.update(d.name, p, d.score, t, range_m=float(np.linalg.norm(p_opt)), color=col, weight_scale=wscale, obs3d=s3)
                self.n["used"] += 1
                self.n["truncated"] += int(truncated)
                if self.log:
                    rec = {"t": t, "cls": d.name, "score": round(float(d.score), 4), "p": [round(float(c), 4) for c in p],
                           "range": round(float(np.linalg.norm(p_opt)), 3), "color": col, "truncated": truncated}
                    if use3d:     # 3D sighting + the Phase 4 pixel estimate of the same box (offline comparison) + camera pose
                        rec["o3d"] = s3.to_dict() if s3 is not None else None
                        rec["p_pixel"] = [round(float(c), 4) for c in transform_point(p_opt, t_xyz, q)]
                        rec["box"] = [round(float(c), 1) for c in d.box]
                        rec["cam"] = [round(float(c), 4) for c in t_xyz + q]
                        rec["floor"] = {"src": self.floor.source, "z": round(plane_z_at(plane, t_xyz[0], t_xyz[1]), 4)}
                    self.log.write(json.dumps(rec) + "\n")
        self.pub_det.publish(out)
        self.publish_map(rgb_m.header.stamp, frame)
        if self.frames is not None and not self.frames.stopped:
            self.frames.maybe_save(t, rgb, t_xyz + q, frame_dets)
            if self.frames.stopped:
                self.get_logger().warn(f"frame saving stopped: {self.frames.stopped} ({self.frames.saved} frames saved)")

    def publish_map(self, stamp, frame):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for tr in self.map.confirmed():
            w, d, h = SIZE_PRIOR[tr.cls]
            c = COLORS[tr.cls]
            p, zc, yaw = tr.xyz, h / 2, None
            if tr.has3d:                                      # measured: centre z, height (if the top was seen), yaw
                s3 = tr.summary3d()
                w, d, h = s3["size"]
                zc, yaw = float(p[2]), s3["yaw"]
            m = Marker()
            m.header.stamp, m.header.frame_id, m.ns, m.id, m.type = stamp, frame, "object", tr.id, Marker.CUBE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(p[0]), float(p[1]), zc
            m.pose.orientation.w = 1.0
            if yaw is not None:
                m.pose.orientation.z, m.pose.orientation.w = float(np.sin(yaw / 2)), float(np.cos(yaw / 2))
            m.scale.x, m.scale.y, m.scale.z = w, d, h
            m.color.r, m.color.g, m.color.b, m.color.a = c[0], c[1], c[2], 0.55
            arr.markers.append(m)
            t = Marker()
            t.header, t.ns, t.id, t.type = m.header, "label", tr.id, Marker.TEXT_VIEW_FACING
            t.pose.position.x, t.pose.position.y, t.pose.position.z = float(p[0]), float(p[1]), zc + h / 2 + 0.15
            t.scale.z = 0.12
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = f"{tr.color + ' ' if tr.color else ''}{tr.cls} #{tr.id} ({tr.hits})"
            arr.markers.append(t)
        self.pub_markers.publish(arr)
        self.pub_json.publish(String(data=self.map.to_json()))

    def log_stats(self):
        self.get_logger().info(f"frames {self.n['frames']}, map updates {self.n['used']} ({self.n['truncated']} truncated), depth rejects {self.n['depth_fail']}, "
                               f"tf failures {self.n['tf_fail']}, confirmed objects {len(self.map.confirmed())}"
                               + (f", object3d {self.n['o3d']} / pixel fallback {self.n['o3d_fallback']}, floor fits {self.floor.n_fit} "
                                  f"(prior {self.floor.n_prior}, last {self.floor.source})" if self.method == "object3d" else ""))

    def save(self):
        if self.log:
            self.log.close()
        path = self.get_parameter("save_path").value
        if path:
            with open(path, "w") as fh:
                fh.write(self.map.to_json())
            self.get_logger().info(f"semantic map saved to {path}")


def main():
    rclpy.init()
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.save()
    node.destroy_node()
    rclpy.try_shutdown()
