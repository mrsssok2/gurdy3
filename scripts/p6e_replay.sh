#!/usr/bin/env bash
# Phase 6e: replay suites on the 6c graph, N runs each. Usage: scripts/p6e_replay.sh TAG "SUITES" "PICKERS" [RUNS] [RUN_DIR]
# Restarts the project's Ollama first (stale server = flaky outputs, see phase6d_picker.md). Offline: no Gazebo.
set -u
cd "$(dirname "$0")/.."
TAG=$1; SUITES=$2; PICKERS=$3; RUNS=${4:-3}; RUN_DIR=${5:-brain/runs/incident_eval/20261125_111853}
. scripts/lib_ollama.sh
pid=$(ss -ltnp 2>/dev/null | awk -v p=":$OLLAMA_PORT" '$4 ~ p"$" {print $6}' | grep -o 'pid=[0-9]*' | head -1 | cut -d= -f2)
[ -n "$pid" ] && kill "$pid" && sleep 3
ollama_prepare gurdy-sft-400 || exit 1
OUT=docs/results/phase6e/$TAG; mkdir -p "$OUT"
for suite in $SUITES; do for pk in $PICKERS; do for i in $(seq $RUNS); do
  docker run --rm -v "$PWD:/home/ubuntu/gurdy2" -w /home/ubuntu/gurdy2 -e OLLAMA_URL --add-host host.docker.internal:host-gateway --entrypoint python3 gurdy2/brain:cpu \
    -m brain.eval.incident_replay "$RUN_DIR" --suite "$suite" --picker "$pk" --json "$OUT/${suite}_${pk}_$i.json" --doc "$OUT/${suite}_${pk}_$i.md" \
    | tee -a "$OUT/summary.txt" | sed "s/^/[$suite picker=$pk run=$i] /"
done; done; done
echo DONE >> "$OUT/summary.txt"
