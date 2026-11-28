#!/bin/bash
# GPU eval of the VLM candidates for describe_view (docs/results/vlm_plan.md section 5). NOT run yet (prepared 2026-11-28).
#   scripts/vlm_eval.sh                          # default models: moondream qwen2.5vl:3b; 60 frames
#   scripts/vlm_eval.sh moondream                # one model
#   PULL=1 scripts/vlm_eval.sh ...               # download missing models through the project's Ollama (network, GBs)
#   N=120 DATASETS="dir1 dir2" NO_SCHEMA=1 MOCK=perfect ...   (MOCK = no model, control-flow check)
# Holds the shared GPU lock for the WHOLE batch; refuses when `docker ps` is not empty (the GPU is shared with Gazebo).
# Unloads the planner LLM from VRAM first (keep_alive 0), so the VRAM peak is the VLM's own.
# Frames: Gazebo renders with per-image ground truth (default: datasets/cone_check_normals + datasets/synth_v31_body of the main
# checkout). Result: docs/results/data/vlm_eval/<ts>/{summary.json,report.md,calls_<model>.jsonl}.
set -uo pipefail
cd "$(dirname "$0")/.."
source scripts/gpu_lock.sh && gpu_lock_acquire "gurdy3:$(basename "$0")" || exit 1
MODELS=("$@"); [ ${#MODELS[@]} -eq 0 ] && MODELS=(moondream qwen2.5vl:3b)
DS_ROOT=${DS_ROOT:-$HOME/projects/gurdy3/datasets}
DATASETS=${DATASETS:-"$DS_ROOT/cone_check_normals $DS_ROOT/synth_v31_body"}
N=${N:-60}; PY=${PY:-$HOME/projects/.venv-gurdy3/bin/python}
OUT=${OUT:-docs/results/data/vlm_eval/$(date -u +%Y%m%d_%H%M%S)}
mkdir -p "$OUT"
log() { echo "$(date -u +%H:%M:%S) $*" | tee -a "$OUT/run.log"; }
if [ -n "$(docker ps -q)" ]; then echo "other containers are running (docker ps): stop them first (the GPU is shared)" | tee -a "$OUT/run.log"; exit 1; fi
DS_ARGS=(); for d in $DATASETS; do [ -d "$d" ] && DS_ARGS+=(--dataset "$d") || log "dataset $d missing, skipped"; done
[ ${#DS_ARGS[@]} -gt 0 ] || { log "no dataset found"; exit 1; }
if [ -n "${MOCK:-}" ]; then
  PYTHONPATH=gurdy_core:. "$PY" -m brain.vlm.eval --mock "$MOCK" "${DS_ARGS[@]}" --n "$N" --out "$OUT" 2>&1 | tee -a "$OUT/run.log"
  exit "${PIPESTATUS[0]}"
fi
. scripts/lib_ollama.sh
_ollama_up || _ollama_start || { log "Ollama did not start"; exit 1; }
API="localhost:$OLLAMA_PORT"
for loaded in $(curl -s "$API/api/ps" | "$PY" -c 'import json,sys; print(" ".join(m["name"] for m in json.load(sys.stdin).get("models", [])))' 2>/dev/null); do
  curl -s "$API/api/generate" -d "{\"model\":\"$loaded\",\"keep_alive\":0}" >/dev/null && log "unloaded $loaded from VRAM"
done
have() { curl -s "$API/api/tags" | M="$1" "$PY" -c 'import json,os,sys; m=os.environ["M"]; n=[x["name"] for x in json.load(sys.stdin).get("models",[])]; sys.exit(0 if m in n or m+":latest" in n else 1)'; }
RUN=()
for m in "${MODELS[@]}"; do
  if ! have "$m"; then
    if [ "${PULL:-0}" = 1 ]; then
      log "pulling $m ..."; curl -s "$API/api/pull" -d "{\"model\":\"$m\",\"stream\":false}" | tee -a "$OUT/run.log"; echo
      have "$m" || { log "$m: pull failed, skipped"; continue; }
    else
      log "$m is not installed in this Ollama (PULL=1 downloads it): skipped"; continue
    fi
  fi
  RUN+=(--model "$m")
done
[ ${#RUN[@]} -gt 0 ] || { log "no model to evaluate"; exit 1; }
log "VLM eval: ${RUN[*]} on $N frames -> $OUT"
OLLAMA_URL="http://$API" PYTHONPATH=gurdy_core:. "$PY" -m brain.vlm.eval "${RUN[@]}" "${DS_ARGS[@]}" --n "$N" --out "$OUT" ${NO_SCHEMA:+--no-schema} 2>&1 | tee -a "$OUT/run.log"
rc=${PIPESTATUS[0]}
for m in "${MODELS[@]}"; do curl -s "$API/api/generate" -d "{\"model\":\"$m\",\"keep_alive\":0}" >/dev/null; done      # free the VRAM again
log "done (rc $rc): $OUT/report.md"
exit "$rc"
