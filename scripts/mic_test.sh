#!/bin/bash
# Microphone check through WSLg PulseAudio, in the `voice` container (CPU only, does not touch the GPU).
#   scripts/mic_test.sh            # list devices, record 5 s, play it back, transcribe with Whisper (CPU)
#   scripts/mic_test.sh list       # only list devices
# Say (when "RECORDING" appears): "go to room B and then find the red box"
set -eo pipefail
cd "$(dirname "$0")/.."
source scripts/gpu_lock.sh && gpu_lock_acquire "gurdy3:$(basename "$0")" || exit 1   # shared GPU lock for the whole run (scripts/gpu_lock.sh)
docker compose run --rm voice bash -c '
  echo "== PulseAudio server"; pactl info | grep -E "Server Name|Default Source|Default Sink" || { echo "no PulseAudio at $PULSE_SERVER (is WSLg running? try: wsl --shutdown, then reopen)"; exit 1; }
  echo "== sources (mic = non-monitor)"; pactl list short sources
  echo "== sinks"; pactl list short sinks
  [ "'"${1:-}"'" = list ] && exit 0
  echo; echo "opening the mic stream (WSLg wakes it in ~1 s; that audio is discarded) ..."
  mkdir -p tmp_audio; python3 scripts/mic_record.py tmp_audio/mic_test.wav 5 1.5      # prints ">>> SPEAK NOW" after the warm-up
  python3 -I scripts/mic_level.py tmp_audio/mic_test.wav
  echo ">>> PLAYBACK"; paplay tmp_audio/mic_test.wav
  echo "== Whisper turbo, CPU int8"
  python3 scripts/mic_transcribe.py tmp_audio/mic_test.wav
'
