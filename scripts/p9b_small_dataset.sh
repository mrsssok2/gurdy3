#!/usr/bin/env bash
# Phase 9b: synthetic dataset for the 12-class v3 detector + training. NOT run yet (needs Gazebo / GPU, one heavy job at a
# time: check `docker ps` first). Stages (run in order, each re-runnable):
#   scripts/p9b_small_dataset.sh plan        # scene JSON + SDF worlds (no simulator; seconds)
#   scripts/p9b_small_dataset.sh trial       # render 5 scenes of each profile, then LOOK at images + labels
#   scripts/p9b_small_dataset.sh render      # all scenes (~10 s / scene measured for v1/v2 -> ~100 min for 600 scenes)
#   scripts/p9b_small_dataset.sh finalize    # dataset.yaml + stats.json per dataset (12 class names from classes.json)
#   scripts/p9b_small_dataset.sh probe       # 3-epoch training run: MEASURE s/epoch + VRAM before the long run
#   scripts/p9b_small_dataset.sh train       # fine-tune det_v2_s -> det_v3_s (12 classes), export ONNX
#   scripts/p9b_small_dataset.sh eval        # old test sets (regression guard) + new test sets
set -euo pipefail
cd "$(dirname "$0")/.."
BODY_SCENES=${BODY_SCENES:-350}
WRIST_SCENES=${WRIST_SCENES:-250}
EPOCHS=${EPOCHS:-150}
BASE=${BASE:-vision/runs/det_v2_s/weights/best.pt}     # fine-tune from the deployed detector (old classes start good)
run_sim() { docker compose run --rm -T sim "$@"; }
run_vis() { docker compose --profile vision run --rm -T vision "$@"; }
case "${1:-}" in
  plan)
    run_sim python3 vision/generate_dataset.py plan --out datasets/synth_v3_body  --scenes "$BODY_SCENES"  --views 12 --seed 31 --profile v3-body
    run_sim python3 vision/generate_dataset.py plan --out datasets/synth_v3_wrist --scenes "$WRIST_SCENES" --views 12 --seed 32 --profile v3-wrist ;;
  trial)
    for d in synth_v3_body synth_v3_wrist; do
      run_sim python3 vision/generate_dataset.py render --out datasets/$d --start 0 --count 5
      run_sim python3 vision/check_labels.py --data datasets/$d --out docs/results/phase9b_trial/$d --samples 6
    done ;;
  render)
    for d in synth_v3_body synth_v3_wrist; do run_sim python3 vision/generate_dataset.py render --out datasets/$d; done ;;
  finalize)
    for d in synth_v3_body synth_v3_wrist; do run_sim python3 vision/generate_dataset.py finalize --out datasets/$d; done ;;
  probe)
    run_vis python3 vision/train.py --data-yaml datasets/synth_v123.yaml --name det_v3_probe --model "$BASE" --epochs 3 --patience 3 ;;
  train)
    run_vis python3 vision/train.py --data-yaml datasets/synth_v123.yaml --name det_v3_s --model "$BASE" --epochs "$EPOCHS" --patience 40 ;;
  eval)
    for d in synth_v1 synth_v2; do
      run_vis python3 vision/eval_detector.py --model models/detector/det_v3_s.onnx --data datasets/$d --classes v3 --out docs/results/phase9b_eval_${d}.md
      run_vis python3 vision/eval_detector.py --model models/detector/det_v2_s.onnx --data datasets/$d --out docs/results/phase9b_eval_${d}_baseline_v2s.md
    done
    for d in synth_v3_body synth_v3_wrist; do
      run_vis python3 vision/eval_detector.py --model models/detector/det_v3_s.onnx --data datasets/$d --classes v3 --out docs/results/phase9b_eval_${d}.md
    done
    run_vis python3 vision/recall_by_distance.py --model models/detector/det_v3_s.onnx --data datasets/synth_v3_body datasets/synth_v3_wrist \
      --out docs/results/phase9b_recall_by_distance.md
    python3 scripts/p9b_report.py --model det_v3_s:docs/results/phase9b_eval --baseline docs/results/phase9b_eval \
      --out docs/results/phase9b_acceptance.md ;;
  *) sed -n '2,12p' "$0"; exit 1 ;;
esac
