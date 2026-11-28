#!/bin/bash
# Phase 4b: plan + render datasets/synth_v2 (Gazebo, sim image), finalize, then train det_v2_s (yolo11s, GPU, vision image) on v1+v2.
# Run from the repo on the host:  nohup scripts/phase4b_pipeline.sh > vision/runs/phase4b_pipeline.log 2>&1 &
# Stops (non-zero exit) if the render fails; the training only starts after a successful finalize. One heavy job at a time.
set -eo pipefail
cd "$(dirname "$0")/.."
source scripts/gpu_lock.sh && gpu_lock_acquire "gurdy3:$(basename "$0")" || exit 1   # shared GPU lock for the whole run (scripts/gpu_lock.sh)
log() { echo "[$(date -u +%H:%M:%S)] $*"; }
[ -z "$(docker ps -q)" ] || { log "another container is running - abort"; exit 1; }
SCENES=${SCENES:-240}
if [ ! -f datasets/synth_v2/worlds/scene_0000.json ]; then
  log "plan $SCENES scenes (profile v2)"
  docker compose run --rm -T --no-deps sim python3 vision/generate_dataset.py plan --out datasets/synth_v2 --scenes $SCENES --views 12 --seed 1 --profile v2
fi
log "render (resumable; log datasets/synth_v2/render.log)"
docker compose run --rm -T --name gurdy3-render sim python3 vision/generate_dataset.py render --out datasets/synth_v2 > datasets/synth_v2/render.log 2>&1
docker compose run --rm -T sim python3 vision/generate_dataset.py finalize --out datasets/synth_v2
log "render done: $(tail -1 datasets/synth_v2/render.log)"
log "train det_v2_s (log vision/runs/train_det_v2_s.log)"
docker compose run --rm -T --name gurdy3-train-det2 vision python3 vision/train.py --data-yaml datasets/synth_v12.yaml --name det_v2_s \
  --model yolo11s.pt --epochs 300 --patience 50 > vision/runs/train_det_v2_s.log 2>&1
log "training finished (exit $?)"
