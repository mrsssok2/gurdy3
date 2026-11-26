# Phase 8b: LiDAR safety shield (control barrier function)

Status 2026-11-26: implemented and tested **offline only**: unit tests plus a 2D kinematic simulator. **Not run in Gazebo,
not run live.** Branch `phase8b-shield`. Off by default (`sim.launch.py shield:=false`).

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
