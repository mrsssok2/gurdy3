#!/usr/bin/env bash
# Phase 9b v3.1 (written 2026-11-27, NOT run): the SAME scenes as v3 (same plan seeds), labels with the v3.1 size rule (small
# classes 3 px / 12 px^2, large 6 / 80) + unfiltered boxes and 3D ground truth saved in meta -> train det_v31_s -> eval with the
# same acceptance targets + recall by distance -> v3 vs v3.1 side by side. One heavy job at a time (render, then GPU training).
#   nohup scripts/p9b_v31_night.sh > /tmp/p9b_v31_night.out 2>&1 &          (docker ps must be empty; resumable stage by stage)
#   scripts/p9b_v31_night.sh render|train|eval|report                       one stage (scripts/overnight_1127.sh block d)
#   env: TRAIN_TIME_H = training wall-time budget (vision/train.py --time-h; default 0 = EPOCHS / patience as before)
# Exit: render 2 = aborted (too many failed scenes); train 1 = no det_v31_s.onnx; eval / report 1 = missing model / inputs.
# Expected (measured 2026-11-27 for v3): render ~55 + 43 min, finalize 1 min, training ~85-95 s/epoch x <= 150 = <= 3.8 h,
# eval ~20 min -> about 5.5-6 h in total. The runtime detector is NOT switched.
set -uo pipefail
cd "$(dirname "$0")/.."
STAGE=${1:-all}
case "$STAGE" in all|render|train|eval|report) ;; *) echo "usage: $0 [all|render|train|eval|report]"; exit 2 ;; esac
if [ "$STAGE" != report ] && docker ps -q | grep -q .; then echo "docker ps not empty: not starting"; exit 1; fi
run_sim() { docker compose run --rm -T sim "$@"; }
run_vis() { docker compose --profile vision run --rm -T vision "$@"; }
log() { echo "$(date -u +%T) $*"; }
EPOCHS=${EPOCHS:-150}; TRAIN_TIME_H=${TRAIN_TIME_H:-0}
want() { [ "$STAGE" = all ] || [ "$STAGE" = "$1" ]; }
BASE=${BASE:-vision/runs/det_v2_s/weights/best.pt}
if want render; then
# 1. plan: identical scenes to v3 (same seeds / counts / profiles)
[ -d datasets/synth_v31_body/worlds ] || run_sim python3 vision/generate_dataset.py plan --out datasets/synth_v31_body --scenes 350 --views 12 --seed 31 --profile v3-body
[ -d datasets/synth_v31_wrist/worlds ] || run_sim python3 vision/generate_dataset.py plan --out datasets/synth_v31_wrist --scenes 250 --views 12 --seed 32 --profile v3-wrist
# 2. render with the v3.1 label rule (exit 1 = a few failed scenes, continue; exit 2 = abort)
for d in synth_v31_body synth_v31_wrist; do
  log "render $d"; run_sim python3 vision/generate_dataset.py render --out datasets/$d --labels v3.1; rc=$?
  [ $rc -eq 2 ] && { log "render $d ABORTED"; exit 2; }
  run_sim python3 vision/generate_dataset.py finalize --out datasets/$d
done
cat > datasets/synth_v1231.yaml <<'Y'
# Phase 9b v3.1: v1 + v2 (7 old classes) + v3.1 body / wrist (v3.1 label rule). Test splits evaluated separately.
path: /home/ubuntu/gurdy2/datasets
train: [synth_v1/images/train, synth_v2/images/train, synth_v31_body/images/train, synth_v31_wrist/images/train]
val: [synth_v1/images/val, synth_v2/images/val, synth_v31_body/images/val, synth_v31_wrist/images/val]
names: {0: box, 1: barrel, 2: fire_extinguisher, 3: traffic_cone, 4: toolbox, 5: mannequin, 6: chair, 7: small_box, 8: mini_toolbox, 9: can, 10: mini_cone, 11: mini_extinguisher}
Y
fi
# 3. train
if want train; then
  if [ ! -f models/detector/det_v31_s.onnx ]; then
    log "train det_v31_s (time budget ${TRAIN_TIME_H} h; 0 = epochs ${EPOCHS})"
    run_vis python3 vision/train.py --data-yaml datasets/synth_v1231.yaml --name det_v31_s --model "$BASE" --epochs "$EPOCHS" --patience 40 \
      --time-h "$TRAIN_TIME_H"
  fi
  [ -f models/detector/det_v31_s.onnx ] || { log "train: no det_v31_s.onnx"; exit 1; }
fi
# 4. eval
if want eval; then
[ -f models/detector/det_v31_s.onnx ] || { log "eval: no det_v31_s.onnx"; exit 1; }
# old sets (regression), BOTH new test sets (v3 labels and v3.1 labels) for BOTH models, recall by distance
for d in synth_v1 synth_v2; do
  run_vis python3 vision/eval_detector.py --model models/detector/det_v31_s.onnx --data datasets/$d --classes v3 --out docs/results/phase9b_v31_eval_${d}.md
done
for m in det_v3_s det_v31_s; do
  P=docs/results/phase9b_eval; [ $m = det_v31_s ] && P=docs/results/phase9b_v31_eval
  for d in synth_v3_body synth_v3_wrist synth_v31_body synth_v31_wrist; do
    [ -f ${P}_${d}.md ] || run_vis python3 vision/eval_detector.py --model models/detector/$m.onnx --data datasets/$d --classes v3 --out ${P}_${d}.md
  done
  run_vis python3 vision/recall_by_distance.py --model models/detector/$m.onnx --data datasets/synth_v3_body datasets/synth_v3_wrist \
    datasets/synth_v31_body datasets/synth_v31_wrist --out docs/results/phase9b_recall_by_distance_${m}.md
done
fi
# 5. side by side
if want report; then
[ -f docs/results/phase9b_v31_eval_synth_v1.md ] || { log "report: no v3.1 eval"; exit 1; }
python3 scripts/p9b_report.py --model det_v3_s:docs/results/phase9b_eval --model det_v31_s:docs/results/phase9b_v31_eval \
  --baseline docs/results/phase9b_eval --out docs/results/phase9b_v3_vs_v31.md
fi
log "DONE $STAGE (runtime detector unchanged)"
