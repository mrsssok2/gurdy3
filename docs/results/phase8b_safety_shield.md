# Phase 8b: LiDAR safety shield (control barrier function)

Status 2026-11-26 (updated in the evening): merged into `main` (61290c4), built and tested in the ROS image, and run **live in Gazebo**
(lab4, headless, one run each, section 6). Off by default (`sim.launch.py shield:=false`). The default wiring is unchanged (6.6).
Sections 3-4 are the offline results from before the live run.

## 1. Where it sits in the cmd_vel chain

Before (unchanged when `shield:=false`):

```
Nav2: controller -> velocity_smoother -> /cmd_vel_smoothed -> cmd_vel_min_speed -> /cmd_vel_min -> collision_monitor -┐
agent (brain/ros RclpyPort, relmotion) -> /cmd_vel_smoothed  (same Nav2 tail: min-speed lift + collision monitor)       ├-> /cmd_vel
teleop -> /cmd_vel directly (no collision monitor!)                                                                     ┘
/cmd_vel -> [estop_gate_node -> /cmd_vel_safe if estop_gate:=true] -> gait_node | policy_node (also obeys latched /estop)
```

With `shield:=true`:

```
/cmd_vel -> safety_shield_node -> /cmd_vel_shielded -> [estop_gate_node -> /cmd_vel_safe] -> gait_node | policy_node
```

* The shield is the **last motion filter** and sees every source: Nav2, the agent and teleop. Teleop bypasses the
  collision monitor today; it does not bypass the shield.
* The **E-STOP gate stays after the shield**, closest to the controller. A latched stop beats everything. The shield
  cannot produce motion from a zero command (see 3), so it can never fight a stop.
* The collision monitor is untouched. Nav2 still slows/stops itself with it, and the shield is a second, independent layer.
* Launch wiring (`gurdy_bringup/launch/sim.launch.py`): the controller reads `/cmd_vel_safe` if the gate is on, else
  `/cmd_vel_shielded` if the shield is on, else `/cmd_vel`. The gate reads `/cmd_vel_shielded` when the shield is on.
  With both flags false nothing changes (checked for all 4 combinations).
* The shield needs a `/scan`. It exists with `nav:=slam|localization`: `scan_self_filter_node` turns `/scan_raw` into
  `/scan`. With `shield:=true` and no scan, the robot does not move (fail safe).

### Shield vs Nav2 collision monitor

| | collision_monitor (`nav2_params.yaml`) | safety shield |
|---|---|---|
| inputs | only Nav2/agent commands (`/cmd_vel_min`) | everything on `/cmd_vel`, incl. teleop |
| rule | `FootprintApproach` polygon: simulate the command 2 s ahead, scale the whole twist so it does not hit within `time_before_collision` | CBF per obstacle point: `dh/dt >= -alpha*h`, `h` = clearance to the footprint minus `d_safe` |
| output | one scale factor for the whole command (or stop) | **minimal change** (QP): only the component toward the obstacle is cut, sliding along a wall / through a doorway keeps its speed |
| near an obstacle | stops (scale -> 0) | approach speed shrinks smoothly to 0 at `d_safe`; retreat and tangential motion pass |
| rotation | footprint simulated with the twist | lever-arm term: a corner swinging into a wall limits `wz` |
| no data | `source_timeout` 1 s -> stop | `scan_timeout` 0.5 s -> zero; no scan ever -> zero |

## 2. Design

Code: `gurdy_core/gurdy_core/safety_shield.py` (pure Python, stdlib only, no ROS) and
`ros2_ws/src/gurdy_safety/gurdy_safety/safety_shield_node.py` (ROS wrapper).

* **Barrier.** For each LiDAR point `p` (body frame): `h = dist(p, footprint) - d_safe`. The footprint is the Nav2
  footprint from robot.yaml (`footprint_polygon`, margin 0.04), a hexagon of 0.546 × 0.57 m, corner radius 0.34 m. The
  point's apparent velocity is `p_dot = (-vx + wz*py, -vy - wz*px)`. The CBF condition `n·p_dot >= -alpha*h` is linear
  in the command `u = (vx, vy, wz)`: `a·u >= b` with `a = (-nx, -ny, nx*py - ny*px)` and `b = -alpha*max(h, 0)`.
* **Inside `d_safe`** (`h < 0`) the bound is capped at "no further approach" (`b = 0`), not "push back". So `u = 0` is
  always feasible, a zero/stop command always passes unchanged, and the shield never moves the robot by itself.
* **QP.** Minimise `||S^-1 (u - u_nom)||^2` subject to all constraints and `|u_i| <= limit_i`, with `S` = robot.yaml
  command limits (0.15, 0.10, 0.60). That makes m/s and rad/s comparable. It is solved with Dykstra's alternating
  projections (exact projection onto the intersection), then verified. Any numerical leftover is scaled toward 0.
  With no binding constraint the command passes bit-for-bit.
* **Size.** Only the nearest point per 10° sector within `d_safe + 0.6 m` is used, so there are at most 36 constraints.
  Compute measured on this host (CPython 3.14): 0.19 ms per scan update + filter in the doorway, 0.74 ms with a
  360-ray wall scan.
* **Self-hits.** Points closer than `self_margin` = 0.02 m to the footprint (or inside it) are ignored. That is the same
  rule as `scan_self_filter_node`, because the femurs cross the LiDAR plane. The node therefore also works on
  `/scan_raw`.
* **Fail safe.** No scan yet, or the newest scan is older than `scan_timeout` (0.5 s, wall clock) -> zero output.
* **Timing.** On every input command, and again at 20 Hz with the newest scan while the last command is younger than
  `cmd_timeout` (0.5 s). A sparse producer is therefore still stopped by an obstacle that appears later. After that
  nothing is published and the controller's own command timeout applies.
* **Status.** `/shield/status` (std_msgs/String, JSON) at 5 Hz and on every state change. Fields: `state`
  (pass | active | stale | no_scan), `active`, `reduction` (0 = untouched, 1 = removed; scaled-norm change / scaled-norm
  command), `min_clearance`, `n_constraints`, `n_self_hits`, `scan_age`, `cmd_in`, `cmd_out`.
* **Parameters.** `d_safe` 0.10 m, `alpha` 2.0 1/s, `self_margin` 0.02, `influence` 0.6, `footprint_margin` 0.04,
  `scan_timeout` 0.5, `cmd_timeout` 0.5, `rate_hz` 20, `scale_only` false, plus topic names. Limits, footprint and
  LiDAR position come from robot.yaml.
* **`scale_only`** (option, off): every axis may only shrink toward 0, so no sideways or reverse component is ever
  added. Measured below: it stalls at a door jamb, so the default is the full QP.

## 3. Results (2D kinematic simulator)

Simulator: `gurdy_core/gurdy_core/shield_sim.py`. Wall segments, holonomic body velocity, command latency 0.05 s,
first-order velocity lag τ 0.2 s, 360-beam LiDAR at 10 Hz (0.15–8 m), control at 20 Hz, physics at 100 Hz. Clearance
is the exact footprint-to-wall distance (0 = collision). Scenarios: `gurdy_core/gurdy_core/shield_scenarios.py`. The
doorway is the flat_lab divider (x = 2.5, 0.1 m thick, 1.0 m gap). Each scenario was also run in a sweep that is harder
than nominal: latency 0.1 s, τ 0.4 s, LiDAR noise 0.01 m (robot.yaml), 3 seeds.
Reproduce: `python3 scripts/shield_sim_report.py` (stdlib only, ~1 min). Tests: `gurdy_core/tests/test_safety_shield.py`.

| scenario | what | collision without | collision with | min clearance without (m) | min clearance with (m) | path without / with (m) | sweep: collisions / runs, worst min clearance (m) |
|---|---|---|---|---|---|---|---|
| `wall_full_speed` | teleop full speed (0.15 m/s) straight at a wall 1.23 m ahead | YES | no | 0.000 | 0.090 | 2.22 / 1.14 | 0 / 3, 0.070 |
| `wall_reverse_strafe` | full-speed strafe right at a wall 0.62 m to the side | YES | no | 0.000 | 0.093 | 1.18 / 0.52 | 0 / 3, 0.082 |
| `corner_diagonal` | diagonal (vx 0.15, vy 0.10) into a corner | YES | no | 0.000 | 0.091 | 3.56 / 1.06 | 0 / 3, 0.074 |
| `turn_near_wall` | turn in place at 0.6 rad/s, side 0.115 m from a wall | no | no | 0.060 | 0.100 | 0.00 / 0.07 | 0 / 3, 0.101 |
| `turn_tight` | turn in place, side 0.045 m from a wall: the corners (0.34 m) would hit it | YES | no | 0.000 | 0.045 | 0.00 / 0.14 | 0 / 3, 0.045 |
| `drive_turn_along_wall` | vx 0.15 + wz 0.3 toward a wall 0.165 m to the left | YES | no | 0.000 | 0.091 | 1.47 / 0.62 | 0 / 3, 0.076 |
| `doorway_centre` | flat_lab doorway, straight through the centre, full speed | no | no | 0.215 | 0.215 | 2.96 / 2.96 | 0 / 3, 0.215 |
| `doorway_offset` | same, path 0.15 m off-centre (0.065 m from the jamb, inside d_safe) | no | no | 0.065 | 0.096 | 2.96 / 2.96 | 0 / 3, 0.090 |
| `doorway_diagonal` | approach the doorway at an angle, turn through it | no | no | 0.191 | 0.191 | 4.55 / 4.55 | 0 / 3, 0.186 |
| `sudden_obstacle` | box appears at t = 3 s, 0.50 m ahead of the front, robot at 0.15 m/s | YES | no | 0.000 | 0.090 | 1.77 / 0.82 | 0 / 3, 0.072 |
| `sudden_obstacle_close` | box appears at t = 3 s only 0.05 m ahead (inside d_safe) | YES | no | 0.000 | **0.012** | 1.77 / 0.45 | 0 / 3, 0.012 |
| `stale_scan` | LiDAR stops publishing at t = 2 s while driving | no | no | (free space) | (free space) | 1.17 / 0.37 | 0 / 3 |
| `no_scan` | no scan ever arrives | no | no | (free space) | (free space) | 0.72 / **0.00** | 0 / 3 |

**Shielded collisions: 0 / 52 runs** (13 scenarios × (nominal + 3 sweep seeds)). Every scenario the shield is meant to
prevent collides without it, and the tests assert that (so the cases are not too easy).

Normal driving (doorway pass time = when the robot centre crosses the divider):

| doorway scenario | without shield (s) | with shield (s) | slowdown | goal reached | `scale_only` |
|---|---|---|---|---|---|
| `doorway_centre` | 10.25 | 10.25 | +0.0 % | yes | crosses, 10.25 s |
| `doorway_offset` | 10.25 | 10.45 | +2.0 % | yes | **stalls at the jamb** |
| `doorway_diagonal` | 16.15 | 16.15 | +0.0 % | yes | crosses, 16.15 s |

In the off-centre pass the QP adds a small sideways component away from the jamb, so the robot slides through. That
raises clearance from 0.065 to 0.096 m at a 2 % time cost. `scale_only` can only cut the forward speed and stalls there.

Stale scan: the last scan arrives at 1.9 s and the output is zero from t = 2.45 s. The robot then drifts 0.036 m (velocity lag).

## 4. Honest limits

* **Sim-only, offline-only.** 2D kinematic model: no contacts, no slip, no body sway, holonomic and instant except for
  the latency/lag model. **Not yet run in Gazebo or on the real stack.** The node is syntax-checked. Its rclpy test
  (`gurdy_safety/test/test_safety_nodes.py`) and the robot.yaml footprint-parity test only run in the ROS image; neither
  has been run yet (no Docker this session).
* **`d_safe` is a target, not a hard bound.** With lag the robot ends ~1 cm inside it (min 0.090 m for `d_safe` 0.10);
  in the slower sweep ~3 cm inside (0.070 m). A real stop margin needs `d_safe` + the speed × (latency + lag). The policy's
  real lag in Gazebo is not measured for this.
* **Obstacles that appear inside the stopping distance** cannot be fully avoided by any velocity filter.
  `sudden_obstacle_close` ends at 0.012 m. That is no collision in this model, but a real one could touch.
* **2D LiDAR only.** One horizontal plane ~0.16 m above ground. Anything below or above it (low steps, table tops, cables,
  overhangs) is invisible. Walking pitch/roll can tilt the plane into the ground at range (`docs/phase3_navigation.md`).
* **Self-hit rule hides very close obstacles.** A real obstacle within 0.02 m of the footprint looks like a leg and is
  ignored, same as `scan_self_filter_node`.
* **Footprint = feet hull**, and the legs swing outside it during a stride. The CBF treats it as rigid and convex.
* **Constraints use the latest scan only.** No motion compensation between scans (10 Hz, ≤ 0.015 m at 0.15 m/s) and no
  tracking of moving obstacles. A moving obstacle is handled only as fast as the scan rate allows.
* **Policy dead zone.** The shield can output small speeds (e.g. 0.04 m/s) that the RL policy does not follow from rest
  (dead zone ~0.03 m/s; the Nav2 min-speed lift sits upstream). Near walls the robot may then stop a little early. A
  lift after the shield would be unsafe, so none is added.
* **Not yet measured:** interaction with Nav2 MPPI + collision monitor in the loop (two layers both slowing may cost more
  than 2 % in tight places), CPU of the node at RTF < 1, status message rate.

## 5. Next (not done)

1. ROS image: `colcon build`, `colcon test --packages-select gurdy_safety`, and the footprint-parity test.
2. Gazebo, when nothing else runs: `sim.launch.py controller:=policy nav:=slam shield:=true`, then teleop at a wall, an
   agent `walk forward 3 m` at a wall, and Nav2 through the flat_lab doorway (with vs without shield). Log `/shield/status`.
3. Choose `d_safe` / `alpha` from the measured Gazebo stopping distance.


## 6. Live Gazebo results (2026-11-26, lab4, headless, one run each)

Setup: `scripts/shield_live_check.sh off|on` (localization on `lab4_map`, policy `terrain_dr2`, ground-truth odom, Nav2, no perception).
Direct-velocity tests go through `scripts/shield_probe.py`. It publishes `/cmd_vel` at 20 Hz, which is the teleop path: it bypasses Nav2's
collision monitor and only the shield stands in the way. Clearance comes from **ground truth**: the `/odom/ground_truth` pose against the
lab4 wall boxes from `scripts/gen_world_lab4.py`. Two polygons are used. `nav` is the Nav2 footprint (margin 0.04), the one the shield keeps
`d_safe` away from. `feet` is the convex hull of the feet incl. foot radius; touching means `feet` <= 0. Data: `docs/results/data/phase8b_live/{off,on}/`.
RTF (3 windows at start): off 0.72-0.81, on 0.21-0.57.

Build / tests after the merge: `colcon build` clean (10 packages), `gurdy_safety` tests 3/3, `gurdy_core/tests/test_safety_shield.py` 31/31,
`brain/tests/test_rclpy_port_smoke.py` 3/3, brain suite 949 passed (3 skipped = the ROS-only tests above).

### 6.1 Full speed straight at a wall (a)
The robot stands at (0, 0.5) facing +x. The command is vx 0.15 m/s for 30 s toward the A-B divider (solid at y 0.2-2.0; feet clearance 1.2 m at the start).

| | value |
|---|---|
| max speed (GT) | 0.158 m/s |
| shield first `active` | nav clearance 0.179 m, at 0.108 m/s |
| robot still (< 0.01 m/s) | 1.35 s later, nav clearance 0.123 m |
| **final clearance (stopping distance)** | **nav 0.116 m** (target d_safe 0.10), feet 0.148 m; shield's own LiDAR estimate 0.097 m |
| touched | **no** (min feet clearance 0.148 m) |

The robot stops **1.6 cm outside** `d_safe`, on the safe side. The policy does not follow the shield's last few cm/s (dead zone, section 4),
so it stops a little early. Offline the robot ended 1 cm inside. **No tuning:** the measured distance meets the target with margin, and
nothing touched. The 3 cm gap between the LiDAR estimate (0.097) and ground truth (0.116 / 0.123) comes from the body sway and pitch of walking.

### 6.2 Diagonal into a corner, then turning near the walls (b)
The robot starts at (-0.09, 4.55) in room C. The command is (vx 0.15, vy 0.10) for 30 s toward the NE corner (2, 6) (east wall = D-C divider
above the door, north outer wall). It then turns in place at +0.6 rad/s for 15 s and -0.6 rad/s for 15 s.

| phase | min nav clearance (GT) | min feet clearance | end | touched |
|---|---|---|---|---|
| corner (diagonal) | **0.095 m** | 0.132 m | stopped at (1.556, 5.566), nav 0.1005 m, speed 0.006 m/s | no |
| turn +0.6 rad/s | 0.1005 m | 0.136 m | turned 172° (shield let the turn through, slowing it while a corner swung toward a wall) | no |
| turn -0.6 rad/s | 0.117 m | 0.157 m | turned back 168° | no |

In the corner the robot slid along until both walls were at about d_safe, ending 0.5 mm outside it. The minimum during the slide was 5 mm
inside d_safe. Turning next to two walls never brought the footprint closer than d_safe.

### 6.3 Nav2 through doorways, with vs without the shield (c)
The same two goals on two fresh stacks: home -> room D centre (4, 4, two doorways), then -> room C (-0.1, 4.5, door D-C).

| goal | without shield: time, GT path, recoveries | with shield: time, GT path, recoveries | mean speed off / on |
|---|---|---|---|
| home -> (4, 4) | 142.9 s, 13.75 m, 1 | 192.9 s, 18.90 m, 1 | 0.096 / 0.098 m/s |
| (4, 4) -> (-0.1, 4.5) | 94.9 s, 8.99 m, 0 | 131.0 s, 12.81 m, 0 | 0.095 / 0.098 m/s |

Both succeeded with and without the shield, final error 0.05-0.10 m. **With the shield the legs took 35-38 % longer, but the robot walked
just as fast.** The extra time comes from the ground-truth path being 37-42 % longer. GT path length includes the body sway of walking, and
the route (via room B or room C) is the planner's choice. During both legs the shield was `active` in only **about 2 %** of its 5 Hz status
samples. This run cannot separate run-to-run variance from a shield effect. One run each is not enough: the overnight-style repeat is
needed before claiming a slowdown or none. (Offline: 0-2 %.)

### 6.4 Agent mission "take me to room D" with the shield (d)
`SUITE=final_suite_lab4 ONLY=l4_room_d EXTRA_LAUNCH_ARGS="shield:=true" scripts/p5_final_eval.sh` (the normal final-eval stack: perception,
e-stop gate, Nav2, monitors, explorer, agent gurdy-sft-400 typed; run dir `brain/runs/phase8b_live/agent_room_d`): **PASS**, 1 attempt. The robot
ended at (3.90, 3.97) in room D after 193.4 s sim / 424 s wall, 18.5 m GT path, 0 falls, 0 wrong claims. Replies: "Going to room D, 7.8 metres."
then "I have arrived, 0.1 metres from the goal." Without the shield, lab4 v1 `l4_room_d` passed 3/3 before (2026-11-22, older code, so not a timing baseline).
**Finding:** in this stack (detector on, more CPU load) the shield went `stale` **9 times during the walk** (at 14, 19, 19, 25, 95, 119, 120, 120 and
410 s). Each lasted 0.0-0.3 s, so the scan was late for more than 0.5 s and the output was zeroed for that time. The mission still succeeded,
but these are short unwanted stops. `scan_timeout` (0.5 s) was not changed tonight (only `d_safe` / gain were in scope, and they need no change).
Next: measure the `/scan` inter-arrival time under the full load, then consider `scan_timeout` 1.0 s (equal to the collision monitor's `source_timeout`).

### 6.5 Stale scan (e)
The robot drives at vx -0.10 m/s into free space. The `/scan` producer (`scan_self_filter_node`) is killed at t = 4.0 s and restarted at t = 8.0 s.

| event | t (s) |
|---|---|
| scan producer killed | 4.00 |
| shield output exactly zero | **4.55** (0.55 s = scan_timeout 0.5 s + one 20 Hz tick) |
| robot still (< 0.01 m/s, GT) | 5.31 (0.76 s after the zero) |
| scan back, shield output moving again | 9.45 |

State `stale` while the scan was missing, `pass` afterwards. Spurious `stale` events in about 15 min of running: 5 during start-up (before
the first scans), then 2 single blips of < 0.1 s (140 s and 840 s into the run, no measurable effect). Cause unknown, probably a late scan
while the CPU was loaded (RTF dipped to 0.21).

### 6.6 `shield:=false` is unchanged
The launch expressions for all 4 flag combinations are the same as before when `shield:=false`. With `estop_gate:=true` (final eval /
overnight run) the controller reads `/cmd_vel_safe`; the gate reads `/cmd_vel` (the remap is a no-op). With both false the controller
reads `/cmd_vel`. No shield node is started. Checked: the `off` stack's launch log has no `safety_shield_node`, and its Nav2 goals succeeded.
`scripts/p5_final_eval.sh` gained `EXTRA_LAUNCH_ARGS` (empty by default, so it adds nothing) to run 6.4.

### 6.7 Live limits
- The shield went stale under the full mission load (6.4): about 9 short zero-velocity blips in 7 min. Not tuned.
- One run per test. Single walls and one corner of lab4. No obstacle appearing suddenly, no moving obstacles, and no teleop while Nav2 is active.
- "Touch" is judged with the feet hull. The swinging legs reach outside it and are not modelled. Gazebo contacts were not read.
- `/shield/status` was echoed without `--full-length`, so the JSON was truncated. Only `state` survived (used in 6.3 and 6.5); `reduction` was not usable.
- The Nav2 time comparison (6.3) is confounded by path length. It is not evidence for or against a slowdown.

## 7. Phase 8c convention checks (read-only, measured in the 6.4 stack)
Plan: `docs/results/phase8c_plan.md` on branch `phase8c-3d` (not merged; nothing there was changed). The probe
`docs/results/data/phase8b_live/p8c/conv_check.py` uses that branch's `gurdy_core/vision/transforms.py` read from git.
`docker exec` ran it inside the running final-eval container, so no extra stack was started. It read 30 depth frames + `/camera/camera_info`
and looked up the real TF `map <- camera_optical_frame` for each frame. Output: `.../p8c/out2.json`.

| check | result |
|---|---|
| `/camera/camera_info` K vs `intrinsics_from_fov(320, 240, hfov 1.047)` | **identical to the `"half"` convention**: fx = fy = 277.19, cx = 160.0, cy = 120.0 (not 159.5 / 119.5); distortion all 0; frame `camera_optical_frame` |
| depth value = z-depth or ray range? | **z-depth**. Floor pixels (rows 180-236, cols 40-280) back-projected as z-depth give per-row medians -0.1089 to -0.1092 m (flat). As ray range they come out 1 cm higher (median -0.099) |
| floor at map z ≈ 0? | **No: floor at map z ≈ -0.11 m** (p5 / p50 / p95 = -0.120 / -0.108 / -0.102 while walking). TF `map -> base_link` z = -0.003: the 2D localisation puts base_link at z = 0, so the floor sits one body height (≈ 0.11 m) below map z = 0 |

Consequence for 8c (not fixed, read-only): the plan's floor removal "map z < 0.02 m" and its "floor back-projects to z = 0" test assume a
floor at z = 0. In this stack everything is 0.11 m lower. Either add the body height to the map z (use the standing base_link height from
robot.yaml or the GT odom z) or remove the floor relative to the measured floor plane. The `<noise>` check was not done.
