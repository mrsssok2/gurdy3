#!/bin/bash
# Gurdy 3.0 held-out evaluation v3: brain/eval/final_suite_lab4_v3.py, 3 runs (seeds 1 2 3), gurdy-sft-400, typed input, fresh stack per mission, one at a time.
#   nohup scripts/heldout_lab4_v3_runs.sh > /tmp/heldout_lab4_v3.out 2>&1 &         resumable: finished missions of a run dir are skipped
# Aggregate afterwards:  R=brain/runs/heldout_lab4_v3; docker compose run --rm -T -e GURDY_FINAL_SUITE=final_suite_lab4_v3 brain python3 -m brain.eval.heldout_report $R/run1 $R/run2 $R/run3
cd "$(dirname "$0")/.."
source scripts/gpu_lock.sh && gpu_lock_acquire "gurdy3:$(basename "$0")" || exit 1   # shared GPU lock for the whole run (scripts/gpu_lock.sh)
for s in ${SEEDS:-1 2 3}; do
  SUITE=final_suite_lab4_v3 SEED=$s RUN_DIR=brain/runs/heldout_lab4_v3/run$s scripts/p5_final_eval.sh
  echo "RUN $s DONE $(date -u +%T)"
done
echo ALL DONE
