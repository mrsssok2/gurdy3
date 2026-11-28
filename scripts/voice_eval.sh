#!/bin/bash
# Read-aloud voice evaluation (docs/voice_eval.md).
#   scripts/voice_eval.sh record                          # shows ~40 sentences one by one; Enter, say it, Enter. Saves to brain/runs/voice_eval/<time>/
#   scripts/voice_eval.sh score DIR [--bias none prompt full] [--device auto|cpu|cuda]    # offline: WER, command accuracy, per category, per bias mode
# record needs the mic through WSLg (same as voice_chat.sh); score needs no mic. Whisper runs on the GPU when available (lib_voice.sh), else the CPU. Do not run it while another heavy job holds the RAM.
set -eo pipefail
cd "$(dirname "$0")/.."
source scripts/gpu_lock.sh && gpu_lock_acquire "gurdy3:$(basename "$0")" || exit 1   # shared GPU lock for the whole run (scripts/gpu_lock.sh)
cmd="${1:?usage: voice_eval.sh record | score DIR [--bias ...]}"; shift
source scripts/lib_voice.sh
docker compose run --rm "$(voice_service)" python3 -m brain.voice.eval_read "$cmd" "$@"
