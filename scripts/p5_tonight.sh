#!/usr/bin/env bash
# Phase 5 overnight chain. Safe to re-run: every step checks whether it is already done.
#   1. wait until brain/data/full/status.txt says DONE and `docker ps` is empty
#   2. re-render the system prompt of the SFT data (compact tool block; old JSON prompt kept as system_json.txt)
#   3. base Qwen3-1.7B on the 100-command set with the compact prompt (fair baseline) -> docs/results/data/phase5/eval_base_compact_qwen3-1.7b.json
#   4. start the full SFT run (2 epochs, compact prompt) detached in container gurdy3-sft; sft.py resumes from brain/runs/sft_v1 on its own
# Start:   nohup scripts/p5_tonight.sh > brain/runs/tonight.log 2>&1 &        Watch: tail -f brain/runs/tonight.log
set -uo pipefail
cd "$(dirname "$0")/.."
source scripts/gpu_lock.sh && gpu_lock_acquire "gurdy3:$(basename "$0")" || exit 1   # shared GPU lock for the whole run (scripts/gpu_lock.sh)
mkdir -p brain/runs
D=brain/data/full
EVAL_JSON=docs/results/data/phase5/eval_base_compact_qwen3-1.7b.json
EVAL_TXT=docs/results/phase5_eval_base_compact.txt
OLL_PORT=11435
say() { echo "[$(date -u +%H:%M:%S)] $*"; }
die() { say "FAILED: $*"; exit 1; }

# ---- already training or finished? (re-run safety)
if docker ps --format '{{.Names}}' | grep -qx gurdy3-sft; then say "gurdy3-sft is already running - nothing to do"; exit 0; fi
if grep -q '^DONE' brain/runs/sft_v1/status.txt 2>/dev/null; then say "SFT already finished (brain/runs/sft_v1/status.txt)"; exit 0; fi

# ---- 1. wait
say "waiting for datagen DONE and an empty docker ps"
until grep -q '^DONE' "$D/status.txt" 2>/dev/null && [ -z "$(docker ps -q)" ]; do
  case "$(head -c 6 "$D/status.txt" 2>/dev/null)" in FAILED) die "datagen: $(cat $D/status.txt)";; esac
  sleep 60
done
say "ready: $(cat $D/status.txt)"

# ---- 2. system prompt of the data = compact (idempotent)
docker compose run --rm brain python3 -m brain.train.render_system "$D" || die "render_system"

# ---- 3. base 1.7B, compact prompt, 100 commands (skipped when the result file exists; written atomically)
if [ -s "$EVAL_JSON" ]; then
  say "base eval already done: $EVAL_JSON"
else
  OLL="http://127.0.0.1:$OLL_PORT"; STARTED=""
  if ! curl -s -m 3 "$OLL/api/tags" >/dev/null; then
    say "starting Ollama on :$OLL_PORT"
    OLLAMA_HOST=0.0.0.0:$OLL_PORT OLLAMA_MODELS=$HOME/ollama_models LD_LIBRARY_PATH=/usr/lib/wsl/lib \
      nohup "$HOME/ollama/bin/ollama" serve > brain/runs/ollama.log 2>&1 &
    STARTED=$!
    for _ in $(seq 30); do curl -s -m 2 "$OLL/api/tags" >/dev/null && break; sleep 2; done
    curl -s -m 3 "$OLL/api/tags" >/dev/null || die "Ollama did not start (brain/runs/ollama.log)"
  fi
  export OLLAMA_URL="http://$(hostname -I | cut -d' ' -f1):$OLL_PORT"
  say "base Qwen3-1.7B, compact prompt, via $OLLAMA_URL"
  docker compose run --rm brain python3 -m brain.eval.run --agent qwen3-1.7b --failures --json "$EVAL_JSON.tmp" > "$EVAL_TXT.tmp" 2>&1
  rc=$?
  # free the GPU for training whatever happened
  curl -s -m 5 "$OLL/api/generate" -d '{"model":"qwen3:1.7b","keep_alive":0}' >/dev/null 2>&1
  [ -n "$STARTED" ] && { kill "$STARTED" 2>/dev/null; sleep 3; }
  if [ $rc -ne 0 ] || [ ! -s "$EVAL_JSON.tmp" ]; then rm -f "$EVAL_JSON.tmp"; die "base eval (see $EVAL_TXT.tmp)"; fi
  mv "$EVAL_JSON.tmp" "$EVAL_JSON"; mv "$EVAL_TXT.tmp" "$EVAL_TXT"
  say "base eval done:"; head -14 "$EVAL_TXT"
fi

# ---- 4. SFT (p5_sft.sh re-checks DONE + empty docker ps; --out sft_v1 resumes from its last checkpoint)
[ -z "$(docker ps -q)" ] || die "docker ps not empty before SFT: $(docker ps --format '{{.Names}}' | tr '\n' ' ')"
docker rm -f gurdy3-sft >/dev/null 2>&1
scripts/p5_sft.sh --epochs 2 || die "p5_sft.sh"
say "SFT started. Monitor: cat brain/runs/sft_v1/status.txt ; docker logs --tail 5 gurdy3-sft"
