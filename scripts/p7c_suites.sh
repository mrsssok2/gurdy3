#!/bin/bash
# Phase 7c: benign_v1 + redteam_v1 + redteam_blind_v1 with gurdy-sft-400. Usage: scripts/p7c_suites.sh TAG   (TAG = base | final)
# Needs the project's Ollama (fresh start recommended). Output docs/results/phase7c/<suite>_<TAG>.{txt,json}
cd "$(dirname "$0")/.."
source scripts/gpu_lock.sh && gpu_lock_acquire "gurdy3:$(basename "$0")" || exit 1   # shared GPU lock for the whole run (scripts/gpu_lock.sh)
source scripts/lib_ollama.sh
ollama_prepare gurdy-sft-400 || exit 1
TAG=${1:?tag}
for s in benign_v1 redteam_v1 redteam_blind_v1; do
  docker compose run --rm -e OLLAMA_URL brain python3 -m brain.eval.redteam --suite brain/eval/$s.yaml --json docs/results/phase7c/${s}_$TAG.json > docs/results/phase7c/${s}_$TAG.txt 2>&1
  echo "== $s"; grep -E "^(TOTAL|FAIL|[a-z_]+ +ok)" docs/results/phase7c/${s}_$TAG.txt
done
