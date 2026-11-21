# Step 2.9b: fine-tune terrain_v1 against the Gazebo gap (prepared 2026-11-16, NOT started)

Why: `phase2_deploy_gazebo.md` (policy gap up to 22 pts vs MuJoCo; latency + stance-load/contact differences).
What changes vs `terrain.yaml` (`rl/configs/terrain_dr2.yaml`, everything else identical, observations unchanged):
latency DR 0–20 ms (was 0–10), per-joint constant offsets on the PD targets (hip_yaw 0.02 / hip_pitch 0.06 / knee 0.08 rad,
hidden from actor and critic), foot+ground contact solref timeconst 0.01–0.04 s (nominal 0.02) and dampratio 0.8–1.5.
15M steps from `terrain_v1` 60M (`ppo_latest`), ≈ 2.0k samples/s → ≈ 2 h 05 min, ≈ 1.9 GiB. Smoke-tested (410k steps ok, then deleted);
tests `rl/tests/test_dr.py` (3 new) pass.

## Start tonight (nothing else may run: `docker ps` empty)
```bash
cd ~/projects/gurdy3 && docker ps
mkdir -p rl/runs/terrain_dr2
docker compose run -d --rm --name gurdy_train rl bash -c 'exec python3 rl/train.py \
  --config rl/configs/terrain_dr2.yaml --name terrain_dr2 \
  --init-from rl/runs/terrain_v1/checkpoints/ppo_latest.zip > rl/runs/terrain_dr2/train.log 2>&1'
tail -f rl/runs/terrain_dr2/train.log            # monitor; stop: docker stop gurdy_train && docker rm gurdy_train
```
Checkpoints every 2M steps in `rl/runs/terrain_dr2/checkpoints/`.

## After training: selection (in this order, all through the same tools as before)
```bash
# 1 start gate + DR scenarios (now incl. latency_20ms, target_offset, soft_contact) + flat regression vs terrain_v1
docker compose run --rm rl python3 rl/eval.py --policies terrain_v1:latest terrain_dr2:latest --scenarios all
# 2 terrain grid (no regression: slopes 100 % to 12°, CoT ≈ 5.8)
docker compose run --rm rl python3 rl/eval.py --policies terrain_v1:latest terrain_dr2:latest --terrain all
# 3 export + Gazebo (the decisive test)
docker compose run --rm rl python3 rl/tools/export_onnx.py --checkpoint rl/runs/terrain_dr2/checkpoints/ppo_latest.zip --out models/terrain_dr2_15M --parity-samples 300
docker compose run --rm sim bash -c 'cd ~/gurdy2 && scripts/policy_gazebo_test.sh docs/results/data/policy_dr2 models/terrain_dr2_15M.onnx'
```
Success: Gazebo tracking within ~10 pts of MuJoCo on every segment, 0 falls (Gazebo + all DR scenarios), flat nominal tracking
≤ 11.2 % (terrain_v1) +1 pt, terrain slopes/CoT no worse, start gate no worse than 40/144.
