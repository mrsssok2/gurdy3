#!/usr/bin/env bash
# Phase 5.4 SFT data pipeline, three stages, resumable. Usage: scripts/p5_datagen.sh OUT_DIR [TRAIN_CLEAN] [VAL_CLEAN] [MAX_CAND]
#   A  clean examples from the mock world            (CPU, brain image, minutes)
#   B  Piper -> Whisper noisy-STT round trip          (GPU Whisper + CPU Piper, vision image, ~2.4 s per unique utterance, hours)
#   C  merge noisy variants, validate, token stats    (CPU, minutes)
# Monitor:  tail -f OUT_DIR/pipeline.log ; cat OUT_DIR/status.txt
set -uo pipefail
cd "$(dirname "$0")/.."
source scripts/gpu_lock.sh && gpu_lock_acquire "gurdy3:$(basename "$0")" || exit 1   # shared GPU lock for the whole run (scripts/gpu_lock.sh)
OUT=${1:?out dir}; TRAIN=${2:-8600}; VAL=${3:-430}; MAXC=${4:-0}
mkdir -p "$OUT"; LOG="$OUT/pipeline.log"; ST="$OUT/status.txt"
say() { echo "[$(date -u +%H:%M:%S)] $*" | tee -a "$LOG"; }
stage() { echo "$1" > "$ST"; say "== $1"; }
# free the GPU: unload any Ollama model (training-style 'one GPU workload at a time')
for m in qwen3:1.7b qwen3:0.6b; do curl -s -m 5 "${OLLAMA_URL:-http://127.0.0.1:11435}/api/generate" -d "{\"model\":\"$m\",\"keep_alive\":0}" >/dev/null 2>&1 || true; done

if [ ! -f "$OUT/stats.json" ]; then
  stage "A: generate clean examples ($TRAIN train / $VAL val)"
  docker compose run --rm brain python3 -m brain.data_gen.generate --out "$OUT" --train "$TRAIN" --val "$VAL" --workers 10 >> "$LOG" 2>&1 \
    || { echo "FAILED at stage A" > "$ST"; exit 1; }
fi
CAND="$OUT/noise_candidates.json"
if [ "$MAXC" != "0" ]; then python3 -c "import json,sys;json.dump(json.load(open('$CAND'))[:$MAXC],open('$OUT/noise_candidates_used.json','w'))"; CAND="$OUT/noise_candidates_used.json"; fi
stage "B: Piper -> Whisper round trip ($(python3 -c "import json;print(len(json.load(open('$CAND'))))") unique utterances)"
for try in 1 2 3; do
  docker run --rm --gpus all -v "$PWD:/w" -v "$HOME/p5_pkgs:/pk" -v "$HOME/p5_tts:/t" -v "$HOME/p5_models:/m" -e PYTHONPATH=/pk:/t -w /w \
    --entrypoint python3 gurdy2/vision:cuda brain/data_gen/noise_synth.py "$CAND" "$OUT/noisy.jsonl" 2>&1 | grep -v "onnxruntime\|W:" >> "$LOG"
  python3 - "$CAND" "$OUT/noisy.jsonl" <<'PY' && break
import json,sys
need=set(json.load(open(sys.argv[1]))); have={json.loads(l)["say"] for l in open(sys.argv[2])} if __import__("os").path.exists(sys.argv[2]) else set()
sys.exit(0 if need<=have else 1)
PY
  say "stage B incomplete (try $try), resuming"
done
stage "C: merge, validate, token stats"
docker compose run --rm brain python3 -m brain.data_gen.merge_noise --dir "$OUT" --noisy "$OUT/noisy.jsonl" >> "$LOG" 2>&1 || { echo "FAILED at stage C (merge)" > "$ST"; exit 1; }
docker compose run --rm brain python3 -m brain.data_gen.validate "$OUT" >> "$LOG" 2>&1 || { echo "FAILED at stage C (validate)" > "$ST"; exit 1; }
docker run --rm -v "$PWD:/w" -v "$HOME/hf_models:/hf" -v "$HOME/p5_pkgs:/pk" -e PYTHONPATH=/pk -w /w --entrypoint python3 gurdy2/vision:cuda \
  brain/data_gen/token_stats.py "$OUT" >> "$LOG" 2>&1
echo "DONE $(date -u +%FT%TZ)" > "$ST"; say "pipeline finished"
