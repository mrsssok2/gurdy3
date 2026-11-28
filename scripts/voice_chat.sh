#!/bin/bash
# Talk to Gurdy in the MOCK world (no Gazebo). Needs the mic through WSLg.
#   scripts/voice_chat.sh              # push-to-talk: Enter, speak, Enter (a typed line also works)
#   VAD=true scripts/voice_chat.sh     # always listening (use a headset: the mic hears the robot's own voice)
#   MODEL=gurdy-sft-400 OLLAMA_URL=http://$(hostname -I|cut -d' ' -f1):11435 scripts/voice_chat.sh   # with the LLM instead of the keyword planner
#   VOICE_GPU=false scripts/voice_chat.sh   # force Whisper on the CPU
# Whisper runs on the GPU (service voice_gpu, ~0.3 s per utterance) when the CUDA image + a GPU exist, else on the CPU (6-10 s, service voice, mock agent in the same container).
# GPU mode: the mock agent runs in a `brain` container (it has langgraph) and the GPU voice client connects to it over TCP.
# Say: "go to room A" | "find the red box" | "open room B" (-> "Did you mean: go to room B?" -> say "yes" / "no") | "stop"
set -eo pipefail
cd "$(dirname "$0")/.."
source scripts/gpu_lock.sh && gpu_lock_acquire "gurdy3:$(basename "$0")" || exit 1   # shared GPU lock for the whole run (scripts/gpu_lock.sh)
source scripts/lib_voice.sh; SVC=$(voice_service)
echo "voice service: $SVC ($([ $SVC = voice_gpu ] && echo "Whisper on the GPU" || echo "Whisper on the CPU, slow"))"
if [ "$SVC" = voice_gpu ]; then
  docker rm -f gurdy_mock_agent >/dev/null 2>&1 || true
  docker compose run --rm -d --name gurdy_mock_agent -e OLLAMA_URL -e MODEL brain python3 -m brain.voice.cli --serve-mock 8765 $([ -n "$MODEL" ] && echo "--ollama $MODEL") >/dev/null
  trap "docker rm -f gurdy_mock_agent >/dev/null 2>&1" EXIT
  for _ in $(seq 30); do docker logs gurdy_mock_agent 2>&1 | grep -q "voice server on" && break; sleep 1; done
  docker compose run --rm -e MODEL -e OLLAMA_URL voice_gpu python3 -m brain.voice.cli --mic --remote gurdy_mock_agent:8765 $([ "$VAD" = true ] && echo --vad)
else
  [ -d "$HOME/p5_piper312/piper" ] || { echo "piper missing: docker run --rm --entrypoint pip -v ~/p5_piper312:/pk gurdy2/voice:cpu install --no-deps --target /pk piper-tts"; exit 1; }
  docker compose run --rm -e MODEL -e OLLAMA_URL voice python3 -m brain.voice.cli --mic --mock $([ "$VAD" = true ] && echo --vad)
fi
