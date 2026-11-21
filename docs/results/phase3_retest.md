# Phase 3 Gazebo re-test (run AFTER terrain_dr2 training has finished; one heavy workload at a time)

Code state: `cmd_vel_min_speed_node` (small Nav2 commands are lifted to 0.09 m/s / 0.23 rad/s, direction kept; ≤ the policy deadband 0.015 m/s / 0.05 rad/s → exact 0),
smoother deadband 0, collision_monitor input `cmd_vel_min`, goal tolerance xy 0.12 m / yaw 0.2 rad. EKF wiring reviewed statically
(below). Nothing here has been run in Gazebo yet.

## 0. Preconditions
```bash
cd ~/projects/gurdy3
docker ps                                   # must be empty (training finished: `docker ps -a | grep gurdy_train`, log tail shows "wall time")
docker compose run --rm sim bash -c 'cd ~/gurdy2/ros2_ws && colcon build --symlink-install --parallel-workers 2 2>&1 | tail -3'
docker compose run --rm sim bash -c 'cd ~/gurdy2 && python3 -m pytest -q ros2_ws/src/gurdy_navigation/test && (cd gurdy_core && python3 -m pytest -q tests/test_rl_command.py)'
```

## 1. Nav2, 5 goals, policy (default model terrain_dr2_15M; results: phase2_9b_nav_results.md)
Goals (map frame): (1.5, 0.5), (4.0, 0.5) through the doorway, (4.0, −2.2), (1.5, −0.5) back through the doorway, (0, 0).
Map = `maps/flat_lab_tripod_gt` (kept from the first run), GT odometry, headless, ~10–40 min wall.
```bash
docker compose run -d --rm --name gurdy_nav sim bash -c 'cd ~/gurdy2 && scripts/phase3_nav_run.sh nav_policy_min flat_lab_tripod_gt policy ground_truth \
  > docs/results/data/phase3_nav_policy_min.log 2>&1'
timeout 3000 bash -c 'until [ -z "$(docker ps -q -f name=gurdy_nav)" ]; do sleep 10; done'
cat docs/results/data/phase3/nav_policy_min/goals.txt        # per goal: status, sim time, path, final error, recoveries, max stall
head -20 docs/results/data/phase3/nav_policy_min/probe.txt   # velocity chain: cmd_vel_nav -> smoothed -> cmd_vel_min -> cmd_vel -> policy/cmd
```
What to check first: in `probe.txt` the first window must show `cmd_vel_min` ≥ 0.09 m/s while `cmd_vel_nav` is ~0.024 (the old deadlock), and
`moved` > 0. Success = status SUCCEEDED and final error < 0.25 m (the Nav2 tolerance is 0.12 m), no stall > 20 s near goals, no recoveries needed
on goals 1–3. Policy variant with the 2.9b model: append `models/terrain_dr2_15M.onnx` as the 5th argument of the script.
Tripod comparison (same goals): `... phase3_nav_run.sh nav_tripod_min flat_lab_tripod_gt tripod ground_truth`.

## 2. EKF mapping run with a TF + scan bag (tripod, same route as the GT map)
```bash
docker compose run -d --rm --name gurdy_nav sim bash -c 'cd ~/gurdy2 && BAG=1 scripts/phase3_map_run.sh flat_lab_tripod_ekf2 tripod ekf \
  > docs/results/data/phase3_ekf2.log 2>&1'
timeout 3000 bash -c 'until [ -z "$(docker ps -q -f name=gurdy_nav)" ]; do sleep 10; done'
cat docs/results/data/phase3/flat_lab_tripod_ekf2/route_summary.txt          # 18/18? (the previous EKF diagnostic run hung at 133 s: note if it recurs)
# bag -> npz (sim image, no sim running), then dead-reckoned maps per pose source + score (rl image)
docker compose run --rm sim bash -c 'cd ~/gurdy2 && source ros2_ws/install/setup.bash && python3 scripts/bag_export.py \
  docs/results/data/phase3/flat_lab_tripod_ekf2/bag docs/results/data/phase3/flat_lab_tripod_ekf2/run.npz'
docker compose run --rm rl python3 scripts/bag_overlay.py docs/results/data/phase3/flat_lab_tripod_ekf2/run.npz \
  docs/results/data/phase3/flat_lab_tripod_ekf2/ekf2
docker compose run --rm rl python3 scripts/map_quality.py maps/flat_lab_tripod_ekf2.yaml
docker compose run --rm rl python3 scripts/plot_map.py maps/flat_lab_tripod_ekf2.yaml docs/results/phase3_map_tripod_ekf2.png docs/results/data/phase3/flat_lab_tripod_ekf2/route.json
```
Reading `bag_overlay.py` (one line per pose source: dead-reckoned scan map precision + pose error vs ground truth):
| outcome | meaning |
|---|---|
| `ekf_tf` precision ≈ `gt` precision (a few cm), `slam` bad | odometry/TF fine, slam_toolbox with this TF goes wrong (scan matcher / TF timing / message filter): look at `slam` pose error vs gt and the `map_odom` jumps |
| `ekf_tf` bad (pose error vs gt of metres) but `/odometry/filtered` good | the TF the EKF publishes differs from its topic (timestamps / frame): check "TF vs /odometry/filtered" line and `ros2 run tf2_ros tf2_monitor odom base_link` in a rerun |
| `ekf_tf`, `ekf` both fine, map still bad | scans are the issue (lidar timing at low RTF, tilt): compare `scan_hz`, RTF |
| `ekf_tf` bad and `legs` bad | leg odometry fails on the long route (the 125 s check was fine): see where the pose error starts |
Also compare `maps/flat_lab_tripod_ekf2` with `flat_lab_tripod_gt` (`map_quality.py ... --ref maps/flat_lab_tripod_gt.yaml`). Run only AFTER item 1.

## Static review of `odom_source:=ekf` (done, no runs) — covered by `gurdy_navigation/test/test_navigation.py`
| check | finding |
|---|---|
| `use_sim_time` on ekf | true (launch override after ekf.yaml) — tested |
| `use_sim_time` on `leg_odometry_node`, `odom_tf_node` | false **by design**: both copy stamps from the input messages (`/joint_states`, `/imu`, `/odom/ground_truth`), never read the node clock (no `get_clock`/`now()`); rclpy `use_sim_time` costs ~40 % CPU at 1 kHz /clock. Tested (source check) |
| single `odom→base_link` publisher | EKF only (`publish_tf: true`); `odom_tf_node` is not launched with `ekf`; the bridge has no TF topic (Gazebo's OdometryPublisher TF stays inside gz); no other `TransformBroadcaster` in the workspace — tested |
| frames | EKF `odom_frame odom`, `base_link_frame base_link`, `world_frame odom`; `/odom/legs` frame `odom`, child `base_link` (twist read in `base_link`); `/imu/ekf` keeps `imu_link` (fixed child of `base_link` in the URDF); slam `odom odom / base base_link / map map` — tested |
| output stamps | `/odom/legs` = `/joint_states` stamp, `/imu/ekf` = `/imu` header, EKF stamps with its own (sim) clock — all sim time |
| EKF settings | 2D mode, differential IMU yaw + yaw rate, leg vx/vy/wz; consistent with `ekf.yaml` test |
**No wiring defect found**, so nothing was changed in the EKF stack. The first EKF run's distortion is therefore not explained by this review
(odometry itself matched ground truth within 3 cm for 125 s); the bag analysis above is the next step.
Differences GT-mode vs EKF-mode that remain: GT TF carries roll/pitch/z, the 2-D EKF TF is flat; TF stamps come from the EKF's timer rather
than the Gazebo message; EKF + leg odometry add CPU load (RTF 0.57 in that run, host also slower).
