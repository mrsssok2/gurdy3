#!/bin/bash
# Fetch LLM picker DEV eval in ONE command, on the host (no Docker): starts the project's Ollama (:11435) if needed,
# checks that the EXACT model is installed, runs brain.eval.fetch_picker_dev with the host venv, writes JSON + log.
#   scripts/fetch_picker_dev.sh [--model gurdy-sft-400] [--model qwen3:1.7b] [--schema]
#   RESTART=1 scripts/fetch_picker_dev.sh ...   restart Ollama first (CLAUDE.md 6d: a stale server caused the o15 flake)
# Output: docs/results/data/fetch_picker/dev_<timestamp>.{json,log}. GPU: ~1-2 GiB VRAM for gurdy-sft-400 (one GPU job at a time).
set -eo pipefail
cd "$(dirname "$0")/.."
source scripts/lib_ollama.sh
PY=${PY:-$HOME/projects/.venv-gurdy3/bin/python}
[ -x "$PY" ] || { echo "no host venv at $PY: scripts/make_host_venv.sh"; exit 1; }
MODELS=(); ARGS=()
while [ $# -gt 0 ]; do
  case $1 in --model) MODELS+=("$2"); shift 2 ;; *) ARGS+=("$1"); shift ;; esac
done
[ ${#MODELS[@]} -gt 0 ] || MODELS=(gurdy-sft-400)
if [ "${RESTART:-0}" = 1 ] && _ollama_up; then pkill -f "[o]llama serve" || true; sleep 2; fi
_ollama_up || _ollama_start
for m in "${MODELS[@]}"; do
  curl -sf "localhost:$OLLAMA_PORT/api/tags" | "$PY" -c "import json,sys; n={x['name'] for x in json.load(sys.stdin)['models']}; \
sys.exit(0 if '$m' in n or '$m:latest' in n else 1)" || { echo "model '$m' is not installed in this Ollama"; exit 1; }
done
OUT=docs/results/data/fetch_picker; mkdir -p "$OUT"; TS=$(date +%Y%m%d_%H%M%S)
MARGS=(); for m in "${MODELS[@]}"; do MARGS+=(--model "$m"); done
OLLAMA_URL=http://127.0.0.1:$OLLAMA_PORT PYTHONPATH=gurdy_core "$PY" -m brain.eval.fetch_picker_dev "${MARGS[@]}" "${ARGS[@]}" \
  --out "$OUT/dev_$TS.json" 2>&1 | tee "$OUT/dev_$TS.log"
echo "results: $OUT/dev_$TS.json"
