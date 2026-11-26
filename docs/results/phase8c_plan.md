# Phase 8c plan: 3D object positions from the depth camera (picking prep)

Status 2026-11-26: plan plus offline code and tests, on branch `phase8c-3d`. **Nothing has run in Gazebo.** The robot
model and launch files are NOT changed (another session is testing them).

## 1. What exists today (read first)

**Camera: Gurdy already has an RGB-D camera.** It is defined in `robot.yaml` `sensors.depth_camera`,
`gurdy.urdf.xacro` (`camera_link`, `camera_optical_frame`) and `gurdy.gazebo.xacro` (gz `rgbd_camera`).

| item | value |
|---|---|
| mount | `base_link` + (0.15, 0, 0.02) m, pitched down 0.17 rad (9.7°). base_link ≈ 0.11 m above the floor when standing, so the lens is ≈ 0.13 m high |
| image | 320 × 240, 15 Hz, hfov 1.047 rad (60°) → vfov 46.8°, fx = fy ≈ 277 px |
| depth | 0.10–5.0 m clip, 32FC1 `/camera/depth_image`, registered with `/camera/image` (same sensor), `/camera/camera_info`, `/camera/points` |
| noise | **none** in the SDF (only the LiDAR and IMU have noise) |
| frames | `camera_link` (x forward) → `camera_optical_frame` (z forward, x right, y down), gz `gz_frame_id` = optical frame |

**Vision path today (Phase 4).**
* `gurdy_perception/detector_node`: YOLO (`det_v2_s`, ONNX) boxes. Each box goes through `geometry.box_depth`, which
  takes ONE robust depth (25th percentile of the box centre). Then it back-projects one pixel (the box centre, or
  `visible_centre_u` for edge-truncated boxes) with K from `camera_info`, pushes back by the class radius prior
  (`SIZE_PRIOR`), and transforms to `map` by TF at the image stamp. The result goes to `SemanticMap` (associates in xy,
  inverse-variance mean, confirmation and dominance rules).
* What is lost: the map is **2D in practice**. z is averaged but never used or evaluated. Size, height and top come only
  from the class prior, and markers are drawn at the prior height. There is no per-sighting covariance beyond a
  range-only sigma.
* Measured accuracy: semantic_lab xy median 0.033 m, all 11 objects within 0.015–0.107 m (`docs/results/phase4b.md`).
* GT: `docs/results/data/phase4/semantic_lab_gt.json` has **x, y only**.

## 2. Camera: keep the body camera, add noise; a wrist camera comes with the arm

Geometry check of the current mount (same numbers as the code):
* View from 33.1° below to 13.7° above horizontal. The floor is visible from 0.20 m ahead of the lens.
* **The top of an object of height h is in view only beyond r = (h − 0.13) / tan 13.7°.**

| object (height) | top visible from |
|---|---|
| toolbox (0.24 m) | 0.45 m |
| box (0.30–0.35 m) | 0.7–0.9 m |
| fire extinguisher (0.49 m) | 1.5 m |
| barrel (0.88 m) | 3.1 m (effectively never at useful range) |

So the body camera is fine for **finding** objects and their xy, and for the height of small objects. For picking (the
last 0.3 m) it sees only the lower part of anything taller than about 0.2 m.

Proposal (to decide with you; nothing edited yet):
1. **Now:** keep the body camera as it is (mount, 320 × 240, 15 Hz, 0.1–5 m), so detection, navigation and the RAM/RTF
   budget stay unchanged. Add only **depth noise** to the gz sensor so the Gazebo numbers are not optimistic: Gaussian,
   stddev ~0.005 m, to be confirmed. gz-sensors applies `<noise>` per camera; whether it reaches the depth image of an
   `rgbd_camera` must be checked in the sim.
2. **With the arm (Extras):** a second RGB-D camera on the wrist / gripper, looking at the grasp area: 0.07–1.0 m,
   320 × 240 (or 424 × 240), 10–15 Hz, only on while picking (rendering costs RTF). Same code path (`object3d.estimate`
   with that camera's K and TF).
3. **Not recommended:** pitching the body camera further down (0.35 rad). It would see tops at grasp range, but loses
   the far view that detection and the semantic map use.

## 3. Box + depth → 3D point in the map frame (implemented, `gurdy_core/gurdy_core/vision/object3d.py`)

1. **Pixels.** All valid depth pixels in the YOLO box (stride 2), depth range 0.10–5 m.
2. **Back-projection.** `p_opt = z · K⁻¹ [u, v, 1]`. The depth image value is the optical **z** (not the ray length);
   that convention must be confirmed in Gazebo, see 5.
3. **Transform.** `p_map = T_map_opt · p_opt`, with `T_map_opt` from TF `lookup_transform(map, camera_optical_frame,
   image stamp)` (already done in `detector_node`). Helpers in `gurdy_core/gurdy_core/vision/transforms.py`: rpy /
   quaternion, `T_from_tq` for a TF result, `T_base_optical(robot.yaml cam)` = the URDF chain, `T_map_base(x, y, yaw, z,
   roll, pitch)` for ground-truth or replay poses, and `intrinsics_from_fov` for tests only. The node keeps using
   `camera_info` K.
4. **Floor removal.** Points with map z < 0.02 m are dropped (flat floor assumed).
5. **Background removal.** Depth clustering: sort the depths and keep the run containing the near seed (5th percentile)
   with no jump > 0.06 m. Background seen through or beside the object is farther and separated by a jump.
6. **Measurements.** top z (98th percentile), bottom (2nd percentile, snapped to the floor if it touches it), height,
   width perpendicular to the viewing bearing (1–99 percentile, corrected), and the nearest surface on the object's
   centre line.
7. **Centre xy.**
   * **With the class shape prior** (the detector knows the class): fit the class footprint to the visible points. Boxes
     use a rectangle of known size over a 2° yaw grid; the side facing the camera fixes the centre, and the yaw with the
     smallest median point-to-boundary distance wins. Cylinders use a fixed-radius least-squares circle. This handles any
     yaw and elongated objects seen end-on, and also gives the object's yaw (useful for grasping).
   * **Without it:** nearest surface + radius along the viewing ray (as in Phase 4).
8. **Centre z** = middle of bottom..top.
9. **Flags.** top / left / right are marked unknown (lower bounds) when the object's own pixels reach the image border,
   or when pixels at the object's depth continue just above the box (detector box cut short).
10. **Covariance** (map frame): along the bearing, `hypot(0.005 + 0.003 z², 0.03)`; lateral and vertical `0.01 + 0.004 z`.

**Filtering / tracking** (`Track3D`, same file):
* Information-filter fusion of the 3D centre over sightings. Each sighting's covariance gets a floor (correlated errors),
  and there is a χ² gate (3 dof, 99 %) after 3 sightings.
* top / height are the median over sightings with the top visible; width is the median over sightings not cut at the
  sides.
* Plan for integration (Gazebo step, not done): `SemanticMap` keeps its xy association and confirmation rules, and each
  `Track` gets a `Track3D`. `detector_node` calls `object3d.estimate(depth, box, K, T_map_opt, shape=class prior)` and
  falls back to the Phase 4 path when it returns None. `/detector/detections` then carries the measured centre and size,
  and the JSON gets `z`, `top_z`, `height`, `width`, `yaw` and sigma. Map-level dedup remains xy.
* The shape prior comes from `SIZE_PRIOR` (box, barrel, extinguisher, toolbox). Cone, chair and mannequin are not
  cylinders or rectangles: they use the ray method until a better model exists.

## 4. Offline results (synthetic depth images, `scripts/object3d_synth_eval.py`)

Setup:
* Camera = robot.yaml mount; one object per image; random robot pose; object 0.8–3.5 m away within ±20° of the view
  centre; random object yaw; 75 sightings per class and condition.
* `yolo` = the tight box with each edge moved by N(0, 4 px). Depth noise σ = 0.003 · z².
* Perfect geometry, flat floor, no rendering artefacts: these are **upper bounds** for Gazebo, not Gazebo numbers.

| class (prior) | xy method | xy error median / p90 (cm) | top visible | top error median / p90 (cm) |
|---|---|---|---|---|
| box 0.40 × 0.40 × 0.35 | ray (Phase 4 style) | 6.4 / 8.1 | 49 / 75 | 0.7 / 1.2 |
| box | **fit** | **1.8 / 4.5** | 49 / 75 | 0.7 / 1.2 |
| toolbox 0.45 × 0.20 × 0.20 | ray | 10.1 / 14.8 | 62 / 75 | 0.6 / 1.2 |
| toolbox | **fit** | **2.3 / 5.8** | 62 / 75 | 0.6 / 1.2 |
| fire extinguisher r 0.08, h 0.5 | ray | 2.2 / 4.6 | 40 / 75 | 1.0 / 1.5 |
| fire extinguisher | **fit** | **0.1 / 0.7** | 40 / 75 | 1.0 / 1.5 |
| barrel r 0.275, h 0.9 | ray | 3.0 / 5.9 | 0 / 75 (never) | - |
| barrel | **fit** | **0.0 / 0.2** | 0 / 75 | - |

Noise-free tight boxes with the fit: box 0.2 / 0.5 cm, toolbox 0.3 / **17.6** cm. The toolbox p90 is the end-on
ambiguity: a 0.20 m end face can be fitted on either side when the long side is not visible. The noisy runs hide it
(2.3 / 5.8). It needs a second view; `Track3D` fusion of two views is tested. Before the top-continuity rule, 5–18
jittered barrel boxes per condition claimed a visible top (errors of 20–50 cm). Now 0.

Tests (`gurdy_core/tests/test_object3d.py`, 29 tests, ~2 s, numpy only) cover:
* transforms: round trip, URDF chain direction, intrinsics/projection, floor back-projects to z = 0 from any pose
* cube, cylinder, rotated box: centre, top and size; the same map estimate from 4 robot poses
* background: a wall behind / 3 cm behind is rejected; noise
* truncation: barrel top truncated; box cut short → top unknown
* floor-only or empty box → None; covariance grows with range
* end-on toolbox: bias with the ray method, solved by the fit or by two-view fusion
* rectangle fit at 5 yaws, circle fit, χ²-gated fusion with outlier rejection, GT comparison
3D ground truth for semantic_lab: `vision/make_gt3d.py` → `docs/results/data/phase8c/semantic_lab_gt3d.json` (centre x, y,
z, size, yaw from the same part lists the world is generated from; the world file is untouched).

## 5. Measuring accuracy against Gazebo ground truth (tomorrow)

1. **Convention checks (first, 10 min).** In `semantic_lab` with `perception:=true`:
   * Compare `/camera/camera_info` K to `intrinsics_from_fov` (cx = w/2 or (w−1)/2?).
   * Floor pixels back-projected through TF must give map z ≈ 0. That checks z-depth vs ray range, the optical-frame
     TF, and base_link height. A wrong convention shows as a floor that bends or tilts.
   * Check whether `<noise>` reaches the depth image.
2. **Ground-truth poses.** Query the gz world for the 11 model poses once
   (`gz service -s /world/semantic_lab/scene/info` or a pose topic) and compare them to `semantic_lab_gt3d.json`. They
   should match; static models.
3. **Perception error isolated from localisation.** Run with `target_frame:=odom` and ground-truth odometry (spawn at
   the origin, so odom = world). Drive a fixed waypoint script: for each object, stand at 0.6 / 1.0 / 1.5 / 2.5 m at 3
   bearings (−20°, 0°, +20° of the view) and stand still 2 s. Log every sighting: per-sighting estimate, class, range,
   flags, robot GT pose.
4. **Metrics** (`object3d.compare_3d` on fused tracks, plus per-sighting statistics). Per class and range bin:
   * xy, z and xyz error; top error (only when `top_known`); height error
   * median / p90 / max, recall, false tracks
   * ray vs fit vs Phase 4 on the same log (replay, like `vision/replay_semantic_map.py`)
5. **Then with SLAM** (`target_frame:=map`, slam_toolbox): the extra error is localisation.
6. **Walking run** (no stops): body sway and timing (TF at image stamp) effects.
7. **Proposed targets** (to agree; for a gripper with ~±2 cm tolerance): static, GT odom, ≤ 1.5 m:
   * xy median ≤ 2 cm, p90 ≤ 4 cm for box, toolbox and extinguisher
   * top error median ≤ 1.5 cm when the top is visible
   * no sighting may report a top as known when it is cut off

## 6. Honest limits of the offline part

* **Idealised synthetic depth:** exact geometry, no edge artefacts, no reflections/holes, flat floor, Gaussian
  z²-noise. Gazebo will be worse. A real sensor worse again.
* **Shape fit needs a correct class and size prior.** Wrong class means a wrong centre. Classes without a simple
  footprint (cone, chair, mannequin) use the ray method. `SIZE_PRIOR` sizes differ from the semantic_lab models (e.g.
  box prior 0.40 m vs actual 0.25–0.40 m), so for the Gazebo run the prior should be the class prior, AND we report what
  a size mismatch costs.
* **Occluders:** an object in front of the target inside the box is taken as the target (nearest-cluster rule). Objects
  touching each other or a wall at a shallow angle can merge into one cluster.
* **Floor assumed flat at z = 0** in the map frame. Slopes or a SLAM map with a z offset break floor removal.
* **Low camera:** the tops of objects taller than ~0.2 m are not visible at grasp range (see 2). Picking those needs the
  wrist camera.
* **The covariance model is a guess.** It is to be calibrated from the Gazebo errors.

## 7. Ready now vs needs Gazebo

Ready (offline, tested):
* the box + depth → 3D math
* shape fits
* truncation flags
* covariance
* `Track3D` fusion
* transform helpers
* GT comparison
* synthetic renderer and eval script
* 3D GT file

Needs Gazebo:
* convention checks (K, z-depth, TF, noise)
* wiring `object3d` into `detector_node` / `SemanticMap` (perception code; robot model and launch untouched)
* the depth-noise model edit (robot model: after the other session)
* waypoint accuracy runs (GT odom, then SLAM, then walking)
* calibrating the covariance
* the wrist-camera spec when the arm exists
