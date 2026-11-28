# Detailed results and RL training commands

Main page: [README](../README.md). Per-experiment write-ups: [results/](results/).
Paths in the commands below are relative to the repo root; links use `results/`.

## Per-phase results (all measured; MuJoCo unless stated)

**Phase 2 — locomotion.** Selected policy: `terrain_dr2` 15M. Terrain: 30 episodes per cell, success = no fall and tracking error < 50 %
([Experiment A](results/exp_A_terrain.md)).

| | tripod gait | `flat_bc_v1` | `dr_energy_ft` 6M | `terrain_v1` 60M | `terrain_dr2` 15M (selected) |
|---|---|---|---|---|---|
| flat tracking error, nominal | 15.9 % | 4.9 % | 9.2 % | 11.2 % | 10.6 % |
| cost of transport (protocol) | 7.35 | 18.3 | 7.34 | 5.76 | 5.77 |
| tracking error, all DR extremes / falls | 13.8 % / 0 % | 35.1 % / 0 % | 10.3 % / 0 % | 14.2 % / 0 % | 11.2 % / 0 % |
| success: rough 4 cm | 100 % | n/a | 100 % | 100 % | 100 % |
| success: slope 12° / 15° | 100 / 100 % | n/a | 43 / 33 % | 100 / 93 % | 93 / 97 % |
| success: steps 3 / 4 / 5 cm | 83 / 57 / 43 % | n/a | 97 / 77 / 43 % | 93 / 80 / 53 % | 100 / 83 / 63 % |
| Gazebo, worst single-axis gap to MuJoCo | 3–11 pts | not tested | stalls on 4 of 7 segments | 24 pts | 14 pts (combined segment: 31) |

The tripod does not fall on slopes up to 15°; the RL gain there is tracking accuracy (12–18 % vs 17–25 %) and, on steps ≥ 3 cm, success rate.

**Experiment B — one leg fails (limp or locked), blind policy.** Fine-tuning with leg failures reduces the tracking error after the failure
(limp 88 → 76 %, locked 61 → 50 %), but **misses the < 40 % criterion and stays worse than the open-loop tripod gait (38 % / 37 %)**; falls ≤ 1 %.
The adapted policy is not deployed (it also regresses on 15° slopes). 20 episodes per leg × type, MuJoCo flat floor:
[results/exp_B_leg_failure.md](results/exp_B_leg_failure.md).

![Experiment B](results/exp_B_leg_failure.png)

**Phase 3 — mapping, navigation, exploration (Gazebo).**

| | tripod gait | `terrain_v1` policy |
|---|---|---|
| mapping, ground-truth odom, 38 m route: mean map error / p95 | **2.4 cm** / 6.4 cm | **3.1 cm** / 12.3 cm |

Autonomous exploration from an empty map (frontier explorer + slam_toolbox + Nav2, policy `terrain_dr2`, then back to the start;
[details](results/phase3_explore.md)):

| | ground-truth odom | EKF odom (legs + IMU) |
|---|---|---|
| lab interior mapped / wall+object recall (10 cm) | 99.1 % / 94.9 % | 98.5 % / 93.1 % |
| sim time to finish exploring (+ return) / distance | 345 s (465 s) / 43.8 m | 468 s (510 s) / 47.4 m |
| failed goals / recoveries / stuck events / ends at start | 0 / 0 / 0 / 0.12 m | 1 / 12 / 0 / 0.06 m |

Nav2 with the policy: 5/5 goals on a saved map ([details](results/phase3_selfhit_and_minspeed.md)).
Time-lapse: [GIF](results/phase3_explore_gt_timelapse.gif) · [MP4](results/phase3_explore_gt_timelapse.mp4).

| tripod gait map | policy map |
|---|---|
| ![map tripod](results/phase3_map_tripod_gt.png) | ![map policy](results/phase3_map_policy_gt.png) |

Details: [flat](results/phase2_flat.md) · [DR](results/phase2_dr.md) · [terrain](results/phase2_terrain.md) ·
[Gazebo transfer](results/phase2_deploy_gazebo.md) · [Phase 3 mapping/Nav2](results/phase3_gazebo_results.md) ·
[3-way video](videos/phase2_3way_tripod_flat_dr.mp4).

## Phase 1 demo commands

```bash
docker compose build sim
docker compose up sim                 # walking robot in Gazebo (first run builds the workspace)
# second terminal:
docker compose exec sim bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.3
docker compose --profile '*' down     # stop everything
```

## Phase 1 results (measured)

| | Result |
|---|---|
| Velocity tracking, Gazebo (vx 0.10 m/s / yaw 0.3 rad/s) | −8.3 % / −11.3 % (limit ±25 %), no fall in 2 min |
| MuJoCo vs Gazebo forward speed | +6.5 % (limit ±30 %) |
| Joint step-response parity Gazebo ↔ MuJoCo | rise time within 4 ms |
| Chatter (60 s standing) | worst joint RMS velocity 4.5e-6 rad/s (limit 1e-2) |

**Phase 2 — earlier policy stages in MuJoCo** (BC-initialised from the tripod gait + PPO). `dr_energy_ft` 6M was the intermediate selection; the final selected walking policy is `terrain_dr2` 15M (table above):

| | tripod gait | flat policy | DR + energy policy (`dr_energy_ft` 6M, superseded) |
|---|---|---|---|
| tracking error, nominal | 15.9 % | 4.9 % | 9.2 % |
| tracking error, all DR extremes | 13.8 % | 35.1 % | 10.3 % |
| tracking error, sensor noise | 16.2 % | 99.2 % | 9.7 % |
| falls (all 9 scenarios) | 0 % | 0 % | 0 % |
| starts from rest (48) | 48/48 | 48/48 | 48/48 |
| cost of transport | 7.35 | 18.3 | 7.34 |
| moves on tiny commands (vx 0.03, vy 0.03, wz 0.1) | yes | mostly | **no (dead zone; fixed at deployment and in later runs)** |

[flat results](results/phase2_flat.md) · [DR results](results/phase2_dr.md) ·
[3-way video](videos/phase2_3way_tripod_flat_dr.mp4) (DR panel: 40M checkpoint)

Details: [parity + chatter](results/phase1_parity.md) ·
[velocity tracking](results/phase1_tracking.md). Design notes and rules: [CLAUDE.md](../CLAUDE.md).

## RL training commands (Phase 2)

Runs live in `rl/runs/<name>/` (`train.log`, `tb/`, `checkpoints/` every 2M steps incl. VecNormalize
stats; `ppo_latest.*` is written on stop). Training runs detached, so it survives closing VS Code.

```bash
# start (example: long flat run, BC-initialised from the tripod gait)
mkdir -p rl/runs/flat_bc_v1
docker compose run -d --rm --name gurdy_train rl bash -c 'exec python3 rl/train.py \
  --config rl/configs/flat_bc_long.yaml --name flat_bc_v1 \
  --init-from rl/runs/c2_bc/checkpoints/ppo_bc.zip > rl/runs/flat_bc_v1/train.log 2>&1'

# progress
docker ps                                            # gurdy_train running?
tail -f rl/runs/flat_bc_v1/train.log                 # live log (Ctrl+C stops tail, not training)
docker compose run --rm rl python3 rl/tools/summarize_run.py flat_bc_v1   # curves as a table
docker stats --no-stream gurdy_train                 # RAM / CPU
# TensorBoard on http://localhost:6006
docker compose run -d --name gurdy_tb --service-ports rl tensorboard --logdir rl/runs --bind_all --port 6006

# stop (saves ppo_latest), then remove the container
docker stop gurdy_train && docker rm gurdy_train

# resume from the newest checkpoint (or --resume rl/runs/<name>/checkpoints/ppo_<N>_steps.zip)
docker compose run -d --rm --name gurdy_train rl bash -c 'exec python3 rl/train.py \
  --config rl/configs/flat_bc_long.yaml --name flat_bc_v1 --resume latest >> rl/runs/flat_bc_v1/train.log 2>&1'
```
Never run training together with Gazebo (RAM rule); training uses ~1.6 GiB and ~7 CPU threads.
