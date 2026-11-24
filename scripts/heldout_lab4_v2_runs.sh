#!/bin/bash
# Gurdy 3.0 held-out evaluation v2: brain/eval/final_suite_lab4_v2.py, 3 runs (seeds 1 2 3), gurdy-sft-400, typed input, fresh stack per mission, one at a time.
#   nohup scripts/heldout_lab4_v2_runs.sh > /tmp/heldout_lab4_v2.out 2>&1 &         resumable: finished missions of a run dir are skipped
cd "$(dirname "$0")/.."
for s in ${SEEDS:-1 2 3}; do
  SUITE=final_suite_lab4_v2 SEED=$s RUN_DIR=brain/runs/heldout_lab4_v2/run$s scripts/p5_final_eval.sh
  echo "RUN $s DONE $(date -u +%T)"
done
echo ALL DONE
