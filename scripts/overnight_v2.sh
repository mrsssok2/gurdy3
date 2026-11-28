#!/bin/bash
# Overnight chain (step A): flat_lab 27-mission regression once, then held-out lab4 v2 three times. One heavy workload at a time; each part resumable.
#   nohup scripts/overnight_v2.sh > /tmp/overnight_v2.out 2>&1 &
cd "$(dirname "$0")/.."
source scripts/gpu_lock.sh && gpu_lock_acquire "gurdy3:$(basename "$0")" || exit 1   # shared GPU lock for the whole run (scripts/gpu_lock.sh)
echo "START $(date -u +%T)"
SUITE=final_suite SEED= RUN_DIR=brain/runs/final_eval/v2fixes_flatlab DOC=docs/results/phase5_final_eval_after_v2_fixes.md scripts/p5_final_eval.sh
echo "FLAT_LAB DONE $(date -u +%T)"
scripts/heldout_lab4_v2_runs.sh
echo "OVERNIGHT DONE $(date -u +%T)"
