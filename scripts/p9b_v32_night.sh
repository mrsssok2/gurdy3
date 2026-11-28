#!/usr/bin/env bash
# Phase 9b v3.2 (written 2026-11-28 for scripts/overnight_1128.sh block a; NOT run yet): coloured mini cones.
# The lathe-cone mesh had no normals and rendered WHITE, so det_v3_s / det_v31_s learnt white mini cones (coloured: 10 % recall,
# docs/results/cone_check.md). v3.2 = copies of the v3.1 datasets in which ONLY the 350 scenes with a mini cone are re-rendered
# with the fixed mesh (same seeds / views / label rule), then det_v31_s is fine-tuned for a FIXED number of epochs (resumable
# after a power cut, unlike a time budget), evaluated, and the runtime is switched only if acceptance passes AND the coloured
# cone recall recovers (scripts/p9b_v32_decide.py). Every stage is resumable / idempotent.
#   scripts/p9b_v32_night.sh make|render|train|eval|report|decide|all
#   env: EPOCHS (40), APPLY=1 (decide: edit detector_node.py when the decision is to switch; default: decision only)
# Exit: render 2 = aborted (too many failed scenes); train 1 = no det_v32_s.onnx; eval / report 1 = missing model / inputs.
# Measured rates (2026-11-27/28): render 11.7 s / scene (350 scenes ~68 min), det_v31_s training 49.5 s / epoch (40 epochs
# ~33 min), eval ~10 min.
set -uo pipefail
cd "$(dirname "$0")/.."
STAGE=${1:-all}
case "$STAGE" in all|make|render|train|eval|report|decide) ;; *) echo "usage: $0 [all|make|render|train|eval|report|decide]"; exit 2 ;; esac
if [ "$STAGE" != report ] && [ "$STAGE" != decide ] && docker ps -q | grep -q .; then echo "docker ps not empty: not starting"; exit 1; fi
run_sim() { docker compose run --rm -T sim "$@"; }
run_vis() { docker compose --profile vision run --rm -T vision "$@"; }
log() { echo "$(date -u +%T) $*"; }
want() { [ "$STAGE" = all ] || [ "$STAGE" = "$1" ]; }
EPOCHS=${EPOCHS:-40}
NAME=det_v32_s; BASE=vision/runs/det_v31_s/weights/best.pt; RUN=vision/runs/$NAME
OUTD=docs/results/data/p9b_v32; mkdir -p "$OUTD"
SETS_V32="datasets/synth_v32_body datasets/synth_v32_wrist"

# 1. copies of v3.1 with the fixed cone mesh; cone scenes marked for re-rendering (vision/cone_check.py retrofit, idempotent)
if want make; then
  for k in body wrist; do
    run_sim python3 vision/cone_check.py retrofit --src datasets/synth_v31_$k --dst datasets/synth_v32_$k || { log "make $k FAILED"; exit 1; }
  done
fi
# 2. render the missing (cone) scenes only (generate_dataset render skips scenes in rendered.txt), finalize, dataset yaml
if want render; then
  for k in body wrist; do
    log "render synth_v32_$k"; run_sim python3 vision/generate_dataset.py render --out datasets/synth_v32_$k --labels v3.1; rc=$?
    [ $rc -eq 2 ] && { log "render synth_v32_$k ABORTED"; exit 2; }
    run_sim python3 vision/generate_dataset.py finalize --out datasets/synth_v32_$k
  done
  cat > datasets/synth_v1232.yaml <<'Y'
# Phase 9b v3.2: v1 + v2 (7 old classes) + v3.2 body / wrist (v3.1 label rule; coloured mini cones). Test splits evaluated separately.
path: /home/ubuntu/gurdy2/datasets
train: [synth_v1/images/train, synth_v2/images/train, synth_v32_body/images/train, synth_v32_wrist/images/train]
val: [synth_v1/images/val, synth_v2/images/val, synth_v32_body/images/val, synth_v32_wrist/images/val]
names: {0: box, 1: barrel, 2: fire_extinguisher, 3: traffic_cone, 4: toolbox, 5: mannequin, 6: chair, 7: small_box, 8: mini_toolbox, 9: can, 10: mini_cone, 11: mini_extinguisher}
Y
fi
# 3. fine-tune det_v31_s for EPOCHS fixed epochs (no time budget: a resumed time-budgeted run stops after 1 epoch)
epochs_done() { if [ -f "$RUN/results.csv" ]; then tr -d '\0' < "$RUN/results.csv" | tail -n +2 | awk 'NF {n++} END {print n + 0}'; else echo 0; fi; }
if want train; then
  if [ ! -f models/detector/$NAME.onnx ]; then
    [ -f datasets/synth_v1232.yaml ] || { log "train: no datasets/synth_v1232.yaml (render first)"; exit 1; }
    if [ -f "$RUN/weights/last.pt" ] && [ "$(epochs_done)" -lt "$EPOCHS" ]; then
      log "train: RESUME $NAME from $(epochs_done) / $EPOCHS epochs"
      run_vis python3 -c "from ultralytics import YOLO; YOLO('$RUN/weights/last.pt').train(resume=True)"
    elif [ ! -f "$RUN/weights/last.pt" ]; then
      log "train $NAME: $EPOCHS epochs from $BASE"
      run_vis python3 vision/train.py --data-yaml datasets/synth_v1232.yaml --name $NAME --model "$BASE" --epochs "$EPOCHS" --patience 40
    fi
    [ -f models/detector/$NAME.onnx ] || run_vis python3 vision/train.py --data-yaml datasets/synth_v1232.yaml --name $NAME \
      --model "$BASE" --epochs "$EPOCHS" --patience 40 --export-only
  fi
  [ -f models/detector/$NAME.onnx ] || { log "train: no $NAME.onnx"; exit 1; }
  tr -d '\0' < "$RUN/results.csv" > "$OUTD/${NAME}_results.csv" 2>/dev/null
fi
# 4. eval: old sets (regression), v3 sets (agreed acceptance, white cones), v3.2 sets (coloured cones) for det_v32_s; det_v31_s
#    on the v3.2 sets for comparison; mini-cone recall: det_v32_s on COLOURED (v3.2 test) vs det_v31_s on WHITE (v3.1 test)
if want eval; then
  [ -f models/detector/$NAME.onnx ] || { log "eval: no $NAME.onnx"; exit 1; }
  for d in synth_v1 synth_v2 synth_v3_body synth_v3_wrist synth_v32_body synth_v32_wrist; do
    f=docs/results/phase9b_v32_eval_$d.md
    [ -f $f ] || run_vis python3 vision/eval_detector.py --model models/detector/$NAME.onnx --data datasets/$d --classes v3 --out $f
  done
  for d in synth_v32_body synth_v32_wrist; do
    f=docs/results/phase9b_v31_eval_$d.md
    [ -f $f ] || run_vis python3 vision/eval_detector.py --model models/detector/det_v31_s.onnx --data datasets/$d --classes v3 --out $f
  done
  [ -f docs/results/phase9b_recall_by_distance_$NAME.md ] || run_vis python3 vision/recall_by_distance.py --model models/detector/$NAME.onnx \
    --data datasets/synth_v3_body datasets/synth_v3_wrist $SETS_V32 --out docs/results/phase9b_recall_by_distance_$NAME.md
  [ -f $OUTD/cone_new.json ] || run_vis python3 vision/recall_by_distance.py --model models/detector/$NAME.onnx --only mini_cone \
    --data $SETS_V32 --json $OUTD/cone_new.json --out $OUTD/cone_recall_${NAME}_coloured.md
  [ -f $OUTD/cone_v31_coloured.json ] || run_vis python3 vision/recall_by_distance.py --model models/detector/det_v31_s.onnx --only mini_cone \
    --data $SETS_V32 --json $OUTD/cone_v31_coloured.json --out $OUTD/cone_recall_det_v31_s_coloured.md
  [ -f $OUTD/cone_ref.json ] || run_vis python3 vision/recall_by_distance.py --model models/detector/det_v31_s.onnx --only mini_cone \
    --data datasets/synth_v31_body datasets/synth_v31_wrist --json $OUTD/cone_ref.json --out $OUTD/cone_recall_det_v31_s_white.md
fi
# 5. side by side + acceptance verdict (json)
if want report; then
  [ -f docs/results/phase9b_v32_eval_synth_v1.md ] || { log "report: no v3.2 eval"; exit 1; }
  python3 scripts/p9b_report.py --model det_v31_s:docs/results/phase9b_v31_eval --model $NAME:docs/results/phase9b_v32_eval \
    --baseline docs/results/phase9b_eval --out docs/results/phase9b_v31_vs_v32.md --json $OUTD/verdict.json
fi
# 6. switch only if acceptance passes AND the coloured-cone recall recovers (APPLY=1 edits detector_node.py)
if want decide; then
  python3 scripts/p9b_v32_decide.py --verdict $OUTD/verdict.json --cone-new $OUTD/cone_new.json --cone-ref $OUTD/cone_ref.json \
    --out $OUTD/decision.json ${APPLY:+--apply}
fi
log "DONE $STAGE"
