#!/bin/bash
# ONE command: Gazebo (headless) + Nav2 + policy + detector + explore + monitors + the fine-tuned brain, with a typed chat in this terminal.
#   scripts/start_agent_chat.sh            # flat_lab, model gurdy-sft-400
#   GUI=true scripts/start_agent_chat.sh   # with the Gazebo window (WSLg)
#   MODEL=qwen3:1.7b scripts/start_agent_chat.sh
#   VOICE=true scripts/start_agent_chat.sh # voice instead of typing: mic + Whisper (GPU if available, else CPU) + Piper in the `voice_gpu` / `voice` container (push-to-talk: Enter; VAD=true = always listening)
#   MEMORY=false scripts/start_agent_chat.sh   # no persistent memory (default: on, file ~/.gurdy/memory.db, rows per map_id)
# Needs: nothing else running (docker ps empty). Starts this project's Ollama on :11435 if it is not up. Takes ~2.5 min until the prompt appears.
# Type e.g.: go to room B | find the red box | what is near you | stop | why did you stop | patrol room A and room B | quit
set -eo pipefail
cd "$(dirname "$0")/.."
source scripts/gpu_lock.sh && gpu_lock_acquire "gurdy3:$(basename "$0")" || exit 1   # shared GPU lock for the whole run (scripts/gpu_lock.sh)
export MODEL=${MODEL:-gurdy-sft-400} GUI=${GUI:-false} MEMORY=${MEMORY:-true} VOICE=${VOICE:-false} VAD=${VAD:-false}; PORT=11435
mkdir -p "$HOME/.gurdy"
if [ -n "$(docker ps -q)" ]; then echo "Other containers are running (docker ps). Stop them first (one heavy workload at a time)."; exit 1; fi
source scripts/lib_ollama.sh
ollama_prepare "$MODEL" || exit 1    # starts Ollama if needed, exact model check, finds an address containers can reach (exports OLLAMA_URL)
# other models still resident in Ollama (it keeps them 5 min) would eat the VRAM that Whisper needs: unload everything except $MODEL
for m in $(curl -s "localhost:$PORT/api/ps" | python3 -c "import sys,json;print(' '.join(x['name'] for x in json.load(sys.stdin).get('models',[])))" 2>/dev/null); do
  case "$m" in "$MODEL"|"$MODEL":*) ;; *) curl -s "localhost:$PORT/api/generate" -d "{\"model\":\"$m\",\"keep_alive\":0}" >/dev/null ;; esac
done
if [ "$VOICE" = true ]; then   # the agent runs headless in the sim container and serves text on gurdy3_agent:8765 (compose network); the voice container talks to it
  docker compose run --rm -T --name gurdy3_agent -e OLLAMA_URL -e MODEL -e GUI -e MEMORY -e DETECTOR_LOG -v "$HOME/.gurdy:/home/ubuntu/.gurdy" sim bash -c '
    cd ~/gurdy2 && source ros2_ws/install/setup.bash && export PYTHONPATH=$HOME/gurdy2:$HOME/gurdy2/gurdy_core:$HOME/gurdy2/sim_mujoco:$PYTHONPATH
    source scripts/lib_nav_start.sh; mkdir -p /tmp/chat; cp maps/flat_lab.places.yaml /tmp/chat/places.yaml
    trap "stop_nav_stack" EXIT
    start_nav_stack /tmp/chat gui:=$GUI world:=flat_lab perception:=true estop_gate:=true target_frame:=map nav:=localization map:=flat_lab_tripod_gt navigation:=true controller:=policy odom_source:=ground_truth > /tmp/chat/start.log 2>&1 < /dev/null \
      || { echo "Nav2 never became ready (see /tmp/chat/launch.log)"; exit 1; }
    ros2 launch gurdy_safety monitors.launch.py odom_topic:=/odom > /tmp/chat/monitors.log 2>&1 < /dev/null &
    ros2 run gurdy_navigation frontier_explorer_node --ros-args -p serve_action:=true -p clock_topic:=/clock_slow -p log_path:=/tmp/chat/explore_log.json -p done_path:=/tmp/chat/explore.done > /tmp/chat/explorer.log 2>&1 < /dev/null &
    sleep 12
    MEMORY_ARGS=""; [ "$MEMORY" = true ] && MEMORY_ARGS="--memory-db /home/ubuntu/.gurdy/memory.db"
    exec python3 -m brain.ros.run_text --ollama "$MODEL" --places /tmp/chat/places.yaml --map-id flat_lab --odom-topic /odom ${MEMORY_ARGS} --serve 8765
  ' > /tmp/gurdy3_agent.log 2>&1 &
  AGENT_PID=$!
  trap "docker rm -f gurdy3_agent >/dev/null 2>&1" EXIT
  echo "starting the simulation (about 2.5 minutes; log /tmp/gurdy3_agent.log) ..."
  for _ in $(seq 400); do
    grep -q "voice server on" /tmp/gurdy3_agent.log 2>/dev/null && break
    grep -q "never became ready" /tmp/gurdy3_agent.log 2>/dev/null && { tail -15 /tmp/gurdy3_agent.log; exit 1; }
    # the agent container died (a crash at start-up): do not wait out the 400 s, show why
    kill -0 $AGENT_PID 2>/dev/null || { echo "the agent container exited before it was ready. Last lines of /tmp/gurdy3_agent.log:"; tail -25 /tmp/gurdy3_agent.log; exit 1; }
    sleep 1
  done
  grep -q "voice server on" /tmp/gurdy3_agent.log || { echo "agent did not start in time"; tail -25 /tmp/gurdy3_agent.log; exit 1; }
  source scripts/lib_voice.sh
  echo "voice service: $(voice_service)"
  docker compose run --rm "$(voice_service)" python3 -m brain.voice.cli --mic --remote gurdy3_agent:8765 $([ "$VAD" = true ] && echo --vad)
  exit 0
fi
docker compose run --rm $([ -t 0 ] || echo -T) --name gurdy3_agent -e OLLAMA_URL -e MODEL -e GUI -e MEMORY -e DETECTOR_LOG -v "$HOME/.gurdy:/home/ubuntu/.gurdy" sim bash -c '
  cd ~/gurdy2 && source ros2_ws/install/setup.bash && export PYTHONPATH=$HOME/gurdy2:$HOME/gurdy2/gurdy_core:$HOME/gurdy2/sim_mujoco:$PYTHONPATH
  source scripts/lib_nav_start.sh; mkdir -p /tmp/chat; cp maps/flat_lab.places.yaml /tmp/chat/places.yaml   # zones the chat adds stay out of git
  trap "stop_nav_stack" EXIT
  echo "starting the simulation (about 2 minutes) ..."
  start_nav_stack /tmp/chat gui:=$GUI world:=flat_lab perception:=true ${DETECTOR_LOG:+detector_log:=$DETECTOR_LOG} estop_gate:=true target_frame:=map nav:=localization map:=flat_lab_tripod_gt navigation:=true controller:=policy odom_source:=ground_truth > /tmp/chat/start.log 2>&1 < /dev/null \
    || { echo "Nav2 never became ready (see /tmp/chat/launch.log)"; exit 1; }
  ros2 launch gurdy_safety monitors.launch.py odom_topic:=/odom > /tmp/chat/monitors.log 2>&1 < /dev/null &   # first: it starts the /clock_slow relay
  ros2 run gurdy_navigation frontier_explorer_node --ros-args -p serve_action:=true -p clock_topic:=/clock_slow -p log_path:=/tmp/chat/explore_log.json -p done_path:=/tmp/chat/explore.done > /tmp/chat/explorer.log 2>&1 < /dev/null &
  sleep 12
  MEMORY_ARGS=""; [ "$MEMORY" = true ] && MEMORY_ARGS="--memory-db /home/ubuntu/.gurdy/memory.db"
  python3 -m brain.ros.run_text --ollama "$MODEL" --places /tmp/chat/places.yaml --map-id flat_lab --odom-topic /odom ${MEMORY_ARGS} 2>&1 | grep --line-buffered -v "^terminate"
'
