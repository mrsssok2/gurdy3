#!/usr/bin/env bash
# Resume det_v31_s after the 2026-11-28 power loss (overnight_1127 block d), then export + eval + report (p9b_v31_night.sh stages).
# Ultralytics resume keeps the saved args (time 2.1 h); with `time` set it re-derives the epoch count from the mean epoch time
# (~2.1 h worth in total) and stops there, so the resumed part uses about the remaining budget. Runtime detector NOT switched.
#   nohup scripts/p9b_v31_resume.sh > /tmp/p9b_v31_resume.out 2>&1 &
set -uo pipefail
cd "$(dirname "$0")/.."
docker ps -q | grep -q . && { echo "docker ps not empty: not starting"; exit 1; }
log() { echo "$(date -u +%T) $*"; }
run_vis() { docker compose --profile vision run --rm -T vision "$@"; }
if [ ! -f models/detector/det_v31_s.onnx ]; then
  log "resume det_v31_s from vision/runs/det_v31_s/weights/last.pt"
  run_vis python3 -c "from ultralytics import YOLO; YOLO('vision/runs/det_v31_s/weights/last.pt').train(resume=True)" \
    2>&1 | tee -a vision/runs/train_det_v31_s_resume.log
  log "export"
  run_vis python3 vision/train.py --data-yaml datasets/synth_v1231.yaml --name det_v31_s --model vision/runs/det_v2_s/weights/best.pt \
    --epochs 150 --patience 40 --time-h 2.1 --export-only || { log "export FAILED"; exit 1; }
fi
log "eval"; scripts/p9b_v31_night.sh eval || { log "eval FAILED"; exit 1; }
log "report"; scripts/p9b_v31_night.sh report || { log "report FAILED"; exit 1; }
log "ALL DONE"
