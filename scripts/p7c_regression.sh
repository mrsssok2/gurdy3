#!/bin/bash
# Phase 7c step 4: restart Ollama fresh, then benign_v1 + redteam_v1 + redteam_blind_v1, testset v1 / v1.1 / ood_v1, incident blind_v1 replay (picker off / on).
# Usage: scripts/p7c_regression.sh TAG     Output: docs/results/phase7c/<TAG>_*.{txt,json}. Offline, no Gazebo.
cd "$(dirname "$0")/.."
source scripts/gpu_lock.sh && gpu_lock_acquire "gurdy3:$(basename "$0")" || exit 1   # shared GPU lock for the whole run (scripts/gpu_lock.sh)
TAG=${1:?tag}; OUT=docs/results/phase7c; mkdir -p $OUT
. scripts/lib_ollama.sh
pid=$(ss -ltnp 2>/dev/null | awk -v p=":$OLLAMA_PORT" '$4 ~ p"$" {print $6}' | grep -o 'pid=[0-9]*' | head -1 | cut -d= -f2)
[ -n "$pid" ] && kill "$pid" && sleep 3
ollama_prepare gurdy-sft-400 || exit 1          # exports OLLAMA_URL for the steps below
[ -z "${SKIP_SUITES:-}" ] && scripts/p7c_suites.sh $TAG
for set in v1 v1_1 ood_v1; do
  docker compose run --rm -e OLLAMA_URL brain python3 -m brain.eval.run --agent gurdy-sft-400 --set $set --failures --json $OUT/${TAG}_$set.json > $OUT/${TAG}_$set.txt 2>&1
  echo "== $set"; grep -E "^agent=" $OUT/${TAG}_$set.txt
done
RUN_DIR=brain/runs/incident_eval/20261125_111853
for pk in off on; do
  docker run --rm -v "$PWD:/home/ubuntu/gurdy2" -w /home/ubuntu/gurdy2 -e OLLAMA_URL --add-host host.docker.internal:host-gateway --entrypoint python3 gurdy2/brain:cpu \
    -m brain.eval.incident_replay $RUN_DIR --suite incident_suite_blind_v1 --picker $pk --json $OUT/${TAG}_incident_blind_$pk.json --doc $OUT/${TAG}_incident_blind_$pk.md > $OUT/${TAG}_incident_blind_$pk.txt 2>&1
  echo "== incident blind_v1 picker=$pk"; tail -3 $OUT/${TAG}_incident_blind_$pk.txt
done
echo ALL_DONE
