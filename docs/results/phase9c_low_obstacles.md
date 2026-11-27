# Phase 9c: low obstacles (below the LiDAR plane) from the body depth camera

Status 2026-11-26: design plus offline code and tests, on branch `phase9a-arm`. **Nothing has run in Gazebo.**
Defaults are unchanged:
* `nav2_params.yaml` is untouched; the new layer is a separate overlay
* `low_obstacle_node` is in no launch file
* the shield change is a patch against `phase8b-shield`, with `use_depth_obstacles` default false

> **Update (Phase 9f, `docs/results/phase9f_small_items.md`):** the wrist RGB-D is now a second source (`wrist:=true`,
> own self filter incl. the held object, own costmap sources), so backing up to an object is no longer blind; a held
> object blinds ~50 % of the wrist view (inside the near clip). The floor was measured at map z = -0.11 m.

## 1. Problem

The 2D LiDAR scans one plane about 0.16 m above the floor. Everything lower is invisible to SLAM, the Nav2 costmaps,
the collision monitor and the Phase 8b safety shield: the Phase 9b pickable objects (5–15 cm), cables, steps, a fallen
toolbox. The robot walks into them; the legs kick them.

## 2. Design

```
/camera/depth_image (32FC1, optical z) + camera_info + /joint_states [+ TF odom <- base_link]
  -> low_obstacle_node (gurdy_navigation; logic: gurdy_core/low_obstacles.py)
       1. depth -> points (stride 2, 0.15-1.8 m) -> base_link (robot.yaml camera pose, URDF chain)
       2. self filter: body box + leg capsules from /joint_states (gurdy_core.kinematics FK) [+ Phase 9a arm capsules]
       3. floor: RANSAC plane per frame near the prior z = -0.11, tilt <= 10 deg, least-squares refit
       4. obstacles = points 0.025-0.45 m above that plane; clearing = floor points (decimated)
       5. memory (odom frame, 10 s, forgotten when seen through) for the shield
  -> /low_obstacles/marking  (PointCloud2, base_link)   -> local costmap low_obstacle_layer (marking) + collision monitor
  -> /low_obstacles/clearing (PointCloud2, base_link)   -> local costmap low_obstacle_layer (clearing ray traces)
  -> /low_obstacles/shield   (PointCloud2, base_link, current + memory) -> safety_shield_node (use_depth_obstacles)
```

**Why each step:**
* **Floor fit per frame, not a fixed height.** The body pitches and rolls a few degrees while walking. At 1.8 m a
  0.05 rad tilt moves the floor by 9 cm: a fixed threshold either marks the far floor as an obstacle or hides real
  5 cm objects. Tested both ways: 3 tilts, the fit gives 0 false points, and the fixed threshold gets > 50 floor points
  wrong. If too few floor points are seen (a wall right ahead), the prior plane is used and a warning is logged.
* **Self filter.**
  * Swinging legs can enter the lower image.
  * The Phase 9a arm hangs in front of the camera only while picking; folded it stays above and behind the lens.
  * The leg capsules come from the measured joint angles, not the nominal stance.
* **Height window 2.5–45 cm.**
  * 2.5 cm sits above floor noise and fit error; a flat 1 cm sheet is not an obstacle (tested).
  * 45 cm covers everything the body and folded arm can hit; higher things are also in the LiDAR plane.
* **Range 1.8 m.** Depth noise grows with z², and a 5 cm cube is only ~8 px at 1.8 m. At 0.15 m/s that is still
  ~10 s of warning.
* **Two clouds for the costmap.** An `ObstacleLayer` clears only along rays to the points it receives. With the floor
  removed, nothing would ever clear a mark. The floor points as a clearing-only source let a cell be freed once the
  camera sees the floor behind it.

**Nav2** (overlay `ros2_ws/src/gurdy_navigation/config/nav2_low_obstacles_overlay.yaml`, applied with
`gurdy_navigation.param_overlay.apply_overlay(params, "low_obstacles")`):
* **Local costmap first.** It gets a new `low_obstacle_layer` (`nav2_costmap_2d::ObstacleLayer`) between the LiDAR
  obstacle layer and inflation, so low obstacles are inflated too. It is a rolling 4 × 4 m window in odom, so marks
  persist as the robot walks past, and sources `sensor_frame: camera_optical_frame`.
* **Global costmap and planner stay LiDAR-only at first.** Marks of moved or picked objects would otherwise stay in the
  map. The local MPPI controller avoids low obstacles; the global path may still go through one and the controller
  bends around it.
* **Collision monitor:** a `pointcloud` source on `/low_obstacles/marking`.
* **Height windows are deliberately wide** (−0.3..0.6 m in the global frame), because the node already filters. Whether
  the floor is at z = 0 or −0.11 in odom must be confirmed in Gazebo.
* A launch flag `low_obstacles:=true` (node plus overlay before `patch_nav2_params`) is not added yet: launch files stay
  untouched while the other session tests them.

**Safety shield** (patch `docs/results/data/phase9c/safety_shield_depth.patch` for the shield node on `phase8b-shield`,
checked to apply cleanly):
* Parameters `use_depth_obstacles` (default **false**), `depth_topic` `/low_obstacles/shield`, `depth_timeout` 0.5 s.
* The depth xy points are added to the LiDAR points of every scan update, so the same CBF treats them like LiDAR returns.
  The shield ignores points within 2 cm of the footprint (its self-hit rule).
* **Stale depth is ignored**, not a stop: the LiDAR fail-safe stays, the status shows `depth_stale`, and a missing
  camera must not freeze the robot. A strict mode (stale depth → zero speed) can be added if Gazebo shows it is needed.
* Tested with the real 8b `SafetyShield` code: a 5 cm cube 0.42 m ahead → state `active`, forward speed reduced.
* The memory goes into the shield input, so an obstacle the robot has walked up to keeps limiting it after it leaves
  the view.

## 3. Field of view: what the camera cannot see

The camera is hfov 60°, vfov 46.8°, pitched 9.7° down, lens 0.13 m above the floor at x = +0.15:

| region | seen? |
|---|---|
| ahead, ±30°, 0.35–1.8 m from base_link | yes (floor visible from x = 0.35 m; footprint front at 0.27 m) |
| the last ~8 cm in front of the footprint (x 0.27–0.35) | **no** (below the image) |
| sides and rear (300° of 360°) | **no** |

What this means:
* **Walking forward** (Nav2's normal mode): obstacles are seen from 1.8 m and remembered when they pass under the
  image edge. The costmap keeps its marks; the shield gets the 10 s memory.
* **Turning in place:** objects at the sides are seen only after the turn brings them into the view. Objects seen before
  the turn stay remembered.
* **Strafing (vy) and walking backwards are blind** for low obstacles, unless the area was seen in the last 10 s (memory)
  or is still marked in the local costmap.
  * MPPI omni can choose vy or reverse motion. Mitigation for the Gazebo test: raise MPPI `PreferForwardCritic` and limit
    vy when the overlay is on. Better (later): a "seen recently" mask that limits lateral and backward speed where the
    camera has not looked.
  * The Phase 9a pick **backs up to the object**. There the wrist camera (ready pose, looking down behind the robot)
    should feed the same pipeline: same code, its K and TF.
* Objects under the body or between the legs are not covered: never seen, or seen > 10 s ago.

## 4. Offline results (`gurdy_core/tests/test_low_obstacles.py`, synthetic depth, 13 pass + 1 run with the 8b code)

| case | result |
|---|---|
| floor only | 0 obstacle points, plane z = −0.11 recovered (< 0.1 mm), > 100 clearing points |
| body tilted 3 deg (3 roll / pitch combinations) | 0 obstacle points with the fit; the fixed threshold gets > 50 floor points wrong |
| depth noise σ = 0.003 z² (3 seeds) | ≤ 3 false 2 cm cells per frame |
| 5 cm cube at 0.8 m | detected; all points inside the cube footprint ± 1 cm; height 5 cm ± 6 mm; nearest face ± 2 cm |
| cube + tilt + noise | ≥ 3 cells at the cube, ≤ 3 elsewhere |
| own leg (raised tibia) in view | an obstacle without the self filter, 0 points with the leg capsule |
| 1 cm sheet / 4 cm box | ignored / detected |
| obstacle at 2.5 m | ignored at max_range 1.8, seen with 3.0 |
| memory | kept when out of view (90° turn), forgotten when seen through, ttl 10 s |
| shield input | (x, y) tuples, 2 cm grid; with the 8b SafetyShield: cube 0.42 m ahead → active, speed reduced |

Compute (host CPython, numpy): **15 ms per frame**, 24 ms with the 12 leg capsules (stride 2, ~11k valid points).
At 5 Hz that is ~12 % of one core.

## 5. Needs Gazebo tomorrow

1. **Conventions:** depth = optical z (as assumed; the 8c plan check); `camera_info` K; floor z in the odom frame →
   final height window of the overlay.
2. **`low_obstacle_node` alone** in `lab4_fetch` (`nav:=slam`, perception off):
   * markers / clouds on the small objects
   * false points while walking: count per minute on a free floor; target < 1 cell / frame average
   * self-hits from the swinging legs; the leg capsule radius may need tuning
   * CPU at RTF < 1
3. **Overlay on:** Nav2 goals across rooms with small objects on the path. Measure collisions with small objects
   (contact / object displacement in Gazebo), goal success and time vs overlay off. Watch for marks that are never
   cleared, which block a doorway.
4. **Shield patch on** (merge 8b first): teleop straight at a small object, sideways at one (expected to fail: blind), and
   at one after turning (memory).
5. **Arm:** `arm:=true` arm capsules in the self filter while picking. The wrist camera into the same pipeline for the
   backing-up final approach.

## 6. Files

| file | what |
|---|---|
| `gurdy_core/gurdy_core/low_obstacles.py` | depth → points, floor RANSAC, self filter, leg capsules, grid, shield input, memory |
| `gurdy_core/tests/test_low_obstacles.py` | 14 tests (synthetic renderer inside) |
| `ros2_ws/src/gurdy_navigation/gurdy_navigation/low_obstacle_node.py` | ROS node (entry point added; package.xml + sensor_msgs / sensor_msgs_py). Syntax-checked only |
| `ros2_ws/src/gurdy_navigation/config/nav2_low_obstacles_overlay.yaml` | Nav2 overlay |
| `ros2_ws/src/gurdy_navigation/gurdy_navigation/param_overlay.py` | merge helper |
| `ros2_ws/src/gurdy_navigation/test/test_param_overlay.py` | default unchanged; overlay content; composition with `patch_nav2_params`, skipped on numpy ≥ 2 |
| `docs/results/data/phase9c/safety_shield_depth.patch` | shield node change (apply on `phase8b-shield`) |

## 7. Honest limits

* **Synthetic depth only:** perfect geometry, Gaussian z² noise, flat floor. The Gazebo depth camera has no noise
  configured either (Phase 8c finding), so Gazebo is optimistic too.
* **Flat floor assumed:** the RANSAC plane must be the dominant near-floor surface. Ramps and steps are not handled; a
  step down (a hole) is not detected at all.
* **Blind sides and rear** (§3). Low obstacles are avoided reliably only while walking forward.
* **Thin or dark objects:** cables thinner than ~1 cm, or below 2.5 cm in height, are not obstacles here.
* **Not integrated:** the node, overlay and shield patch have not been run together, and there is no launch flag yet.
