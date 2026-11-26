#!/bin/bash
# Phase 8a overnight: doorway_v1 x 3 (seeds 1 2 3) + a lab4 v3 regression subset (navigation-heavy missions, the ones with path-blocked
# failures before the fix) x 3, gurdy-sft-400, typed input, fresh Gazebo stack per mission, one at a time. Ollama is restarted before every
# run (stale Ollama server = the o15 flake of Phase 6d). Results: brain/runs/doorway_v1/run{1,2,3} and brain/runs/doorway_v1/v3_regr{1,2,3}.
#   nohup scripts/doorway_v1_runs.sh > /tmp/doorway_v1.out 2>&1 &          resumable: finished missions of a run dir are skipped
# Reports afterwards:
#   R=brain/runs/doorway_v1; docker compose run --rm -T -e GURDY_FINAL_SUITE=final_suite_doorway_v1 brain python3 -m brain.eval.heldout_report $R/run1 $R/run2 $R/run3
#   docker compose run --rm -T -e GURDY_FINAL_SUITE=final_suite_lab4_v3 brain python3 -m brain.eval.heldout_report $R/v3_regr1 $R/v3_regr2 $R/v3_regr3
# Ollama is stopped at the end (nothing heavy left running).
cd "$(dirname "$0")/.."
V3_SUBSET=${V3_SUBSET:-v3_step_into_charlie,v3_bravo_then_alpha,v3_bravo_delta_home,v3_cone_in_d,v3_toolbox_search,v3_d_back_to_a,v3_room_b,v3_delta_phonetic,v3_c_then_b,v3_leg_walking}

stop_ollama() {   # only our server (port 11435 / ~/ollama); the Windows Ollama on 11434 is not ours
  local pids; pids=$(pgrep -f "[o]llama/bin/ollama serve" || true)
  [ -n "$pids" ] && kill $pids 2>/dev/null
  for _ in $(seq 20); do pgrep -f "[o]llama/bin/ollama serve" >/dev/null || return 0; sleep 1; done
  pkill -9 -f "[o]llama/bin/ollama serve" 2>/dev/null; return 0
}
trap 'docker rm -f gurdy3_final_eval >/dev/null 2>&1; stop_ollama; echo "EXIT $(date -u +%T)"' EXIT

for s in ${SEEDS:-1 2 3}; do
  stop_ollama
  SUITE=final_suite_doorway_v1 SEED=$s RUN_DIR=brain/runs/doorway_v1/run$s scripts/p5_final_eval.sh
  echo "DOORWAY RUN $s DONE $(date -u +%T)"
done
for s in ${SEEDS:-1 2 3}; do
  stop_ollama
  SUITE=final_suite_lab4_v3 SEED=$s ONLY=$V3_SUBSET RUN_DIR=brain/runs/doorway_v1/v3_regr$s scripts/p5_final_eval.sh
  echo "V3 SUBSET RUN $s DONE $(date -u +%T)"
done
echo ALL DONE
