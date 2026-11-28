#!/bin/bash
# ONE command for the first Gazebo VLN check (planned for 2026-11-29; NOT run on 2026-11-28): the 3 DEV episodes of
# brain/eval/vln_episodes_dev.yaml on the lab4_vln world (lab4 + a chair in the kitchen), keyword planner (no Ollama),
# landmarks from GT, one fresh stack per episode, under the shared GPU lock, graded from ground truth.
#   scripts/vln_dev_run.sh                    # ~25-60 min (3 stacks; builds the worktree's colcon workspace first if needed)
#   MODEL=gurdy-sft-400 scripts/vln_dev_run.sh
# Result: brain/runs/vln_eval/<ts>/report.md (+ summary.json). DEV episodes: never report them as a held-out result.
set -uo pipefail
cd "$(dirname "$0")/.."
exec scripts/vln_eval.sh brain/eval/vln_episodes_dev.yaml "$@"
