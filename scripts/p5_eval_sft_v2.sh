#!/bin/bash
# Fair comparison with the CURRENT agent code: base qwen3:1.7b (compact prompt), gurdy-sft-400, gurdy-sft-v2 on testset v1 (frozen), v1.1 and ood_v1.
# One run at a time (GPU). Output: docs/results/sft_v2_eval/<agent>_<set>.{txt,json}; finished runs are skipped (resumable).
# Needs the project's Ollama listening on 0.0.0.0:11435 (containers reach it through the WSL address).
cd "$(dirname "$0")/.."
source scripts/gpu_lock.sh && gpu_lock_acquire "gurdy3:$(basename "$0")" || exit 1   # shared GPU lock for the whole run (scripts/gpu_lock.sh)
export OLLAMA_URL=${OLLAMA_URL:-http://$(hostname -I|cut -d' ' -f1):11435}
OUT=docs/results/sft_v2_eval; mkdir -p $OUT
for agent in ${AGENTS:-gurdy-sft-v2 gurdy-sft-400 qwen3-1.7b}; do
  for set in ${SETS:-v1 v1_1 ood_v1}; do
    [ -s $OUT/${agent}_${set}.json ] && continue
    echo "== $agent $set $(date +%T)"
    docker compose run --rm -e OLLAMA_URL brain python3 -m brain.eval.run --agent $agent --set $set --failures --json $OUT/${agent}_${set}.json > $OUT/${agent}_${set}.txt 2>&1
    grep -E "^agent=" $OUT/${agent}_${set}.txt
  done
done
echo ALL_DONE
