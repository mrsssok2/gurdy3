#!/bin/bash
# Mission control on the REAL simulated robot: Gazebo (headless) + Nav2 + policy + detector + monitors + the agent + the web app, ONE heavy stack.
#   scripts/start_web_robot.sh            # this PC only:  http://localhost:8080/#token=...
#   scripts/start_web_robot.sh lan        # also reachable from the phone on the same Wi-Fi (docs/phase7_phone.md: firewall rule)
#   scripts/start_web_robot.sh tailscale voice   # PRIVATE remote access from anywhere via Tailscale Serve (real https, tailnet only; docs/phase7_remote.md)
#   scripts/start_web_robot.sh lan https voice   # + https on :8443 (the phone's microphone needs it) + hold-to-talk mic and spoken replies (docs/phase7_voice.md)
#   MODEL=keyword scripts/start_web_robot.sh   # no LLM (smart keyword planner; no Ollama / GPU needed)
#   MODEL=gurdy-sft-400 (default)         # the fine-tuned brain via this project's Ollama on :11435
#   WORLD=lab4 scripts/start_web_robot.sh # the 4-room lab4 world (default WORLD=flat_lab)
#   GUI=true scripts/start_web_robot.sh   # also the Gazebo window (WSLg)
#   VLN=true WORLD=lab4 scripts/start_web_robot.sh  # VLN instructions (brain/vln, NOT run yet; default VLN=false = as before)
#   FETCH=true scripts/start_web_robot.sh # Phase 9h (first dry run 2026-11-27, docs/results/phase9j_fetch_dry_run.md): world lab4_fetch + arm:=true + object3d:=true + move_group, agent --fetch
#                                         # (brain/ros/fetch_robot.py; default FETCH=false = the stack and agent exactly as before)
#                                         # FETCH=true also writes every detector sighting to /tmp/chat/sightings.jsonl (class, score, position,
#                                         # colour, range; 2026-11-28: needed to tell a missed detection from a failed confirmation / colour);
#                                         # DETECTOR_LOG=false turns it off. Kept on the host with CHAT_LOG_DIR (scripts/lib_fetch_run.sh does).
#   GURDY_WEB_TOKEN=... to choose the access token; otherwise a random one is generated and printed (never written to a file).
# Needs: docker ps empty (one heavy workload at a time). Takes ~2.5 min until the page works. Ctrl+C stops everything.
# RAM / VRAM are sampled every 5 s into /tmp/gurdy_web_ram.log; the peak is printed when you stop it.
set -eo pipefail
cd "$(dirname "$0")/.."
export MODEL=${MODEL:-gurdy-sft-400} GUI=${GUI:-false} MEMORY=${MEMORY:-true}; PORT=11435
export FETCH=${FETCH:-false} DETECTOR_LOG=${DETECTOR_LOG:-true} VLN=${VLN:-false} FETCH_EXTRA=${FETCH_EXTRA:-}   # FETCH_EXTRA: extra run_text fetch flags (scripts/fetch_retest.sh), default none
[ "$FETCH" = true ] && WORLD=${WORLD:-lab4_fetch}
export WORLD=${WORLD:-flat_lab}
case "$WORLD" in
  flat_lab) MAP_NAME=flat_lab_tripod_gt; MAP_ID=flat_lab; PLACES_FILE=maps/flat_lab.places.yaml ;;
  lab4) MAP_NAME=lab4_map; MAP_ID=lab4; PLACES_FILE=maps/lab4.places.yaml ;;
  lab4_fetch) MAP_NAME=lab4_map; MAP_ID=lab4_fetch; PLACES_FILE=maps/lab4_fetch.places.yaml ;;   # same walls; the small objects are below the lidar plane
  *) echo "unknown WORLD: $WORLD (flat_lab | lab4 | lab4_fetch)"; exit 2 ;;
esac
export MAP_NAME MAP_ID PLACES_FILE
PUBLISH=127.0.0.1; HTTPS=false; VOICE=false; TAIL=false
for a in "$@"; do case "$a" in lan) PUBLISH=0.0.0.0 ;; https) HTTPS=true ;; voice) VOICE=true ;; tailscale) TAIL=true ;; *) echo "unknown argument: $a (lan | https | voice | tailscale)"; exit 2 ;; esac; done
export GURDY_WEB_TOKEN="${GURDY_WEB_TOKEN:-$(python3 -c 'import secrets;print(secrets.token_urlsafe(24))')}"
mkdir -p "$HOME/.gurdy"
if [ -n "$(docker ps -q)" ]; then echo "Other containers are running (docker ps). Stop them first (one heavy workload at a time):"; docker ps --format '  {{.Names}}'; exit 1; fi
if [ "$MODEL" != keyword ]; then
  source scripts/lib_ollama.sh
  ollama_prepare "$MODEL" || exit 1      # starts Ollama if needed, checks the exact model, finds an address the containers can reach, exports OLLAMA_URL
fi
if $TAIL; then source scripts/lib_tailscale.sh; ts_check || exit 1; fi      # fail fast, before the 2-minute start-up
PORTS=(-p "$PUBLISH:8080:8080"); EXTRA=()
if $HTTPS; then
  [ -f "$HOME/.gurdy/tls/gurdy.crt" ] || { echo "no certificate yet: run scripts/make_web_cert.sh <the laptop's Wi-Fi / hotspot IP> first (docs/phase7_voice.md)"; exit 1; }
  export GURDY_TLS_PORT=8443 GURDY_TLS_CERT=/tls/gurdy.crt GURDY_TLS_KEY=/tls/gurdy.key GURDY_TLS_CA=/tls/ca.crt
  EXTRA+=(-v "$HOME/.gurdy/tls:/tls:ro" -e GURDY_TLS_PORT -e GURDY_TLS_CERT -e GURDY_TLS_KEY -e GURDY_TLS_CA); PORTS+=(-p "$PUBLISH:8443:8443")
fi
if $VOICE; then source scripts/lib_speech.sh; start_speech_service; EXTRA+=(-e GURDY_SPEECH_URL); fi
LOG=/tmp/gurdy_web.log; RAM=/tmp/gurdy_web_ram.log; : > "$LOG"; : > "$RAM"
cleanup() { $TAIL && ts_stop; [ -n "${SAMPLER:-}" ] && kill "$SAMPLER" 2>/dev/null || true; docker rm -f gurdy3_agent gurdy3_speech >/dev/null 2>&1 || true
  if [ -s "$RAM" ]; then awk '{ if ($2>c) c=$2; if ($3>w) w=$3; if ($4>g) g=$4 } END { printf "peak: container %d MiB, WSL used %d MiB, GPU %d MiB (samples: /tmp/gurdy_web_ram.log)\n", c, w, g }' "$RAM"; fi; }
trap cleanup EXIT INT TERM
[ -n "${CHAT_LOG_DIR:-}" ] && EXTRA+=(-v "$CHAT_LOG_DIR:/tmp/chat")   # optional: keep the stack logs on the host (default off)
SVC=sim; [ "$FETCH" = true ] && SVC=sim_moveit     # fetch needs move_group (MoveIt only in the derived image gurdy3/ros:jazzy-moveit)
docker compose --profile moveit run --rm -T --name gurdy3_agent "${PORTS[@]}" "${EXTRA[@]}" -e OLLAMA_URL -e MODEL -e GUI -e MEMORY -e WORLD -e MAP_NAME -e MAP_ID -e PLACES_FILE -e FETCH -e FETCH_EXTRA -e DETECTOR_LOG -e VLN -e GURDY_WEB_TOKEN -e GURDY_ALLOWED_ORIGINS \
  -v "$HOME/.gurdy:/home/ubuntu/.gurdy" "$SVC" bash -c '
  cd ~/gurdy2 && source ros2_ws/install/setup.bash && export PYTHONPATH=$HOME/gurdy2:$HOME/gurdy2/gurdy_core:$HOME/gurdy2/sim_mujoco:$PYTHONPATH
  source scripts/lib_nav_start.sh; mkdir -p /tmp/chat; cp "$PLACES_FILE" /tmp/chat/places.yaml
  trap "stop_nav_stack" EXIT
  FETCH_LAUNCH=""; FETCH_ARGS=""; [ "$FETCH" = true ] && FETCH_LAUNCH="arm:=true object3d:=true" && FETCH_ARGS="--fetch ${FETCH_EXTRA:-}"
  [ "$FETCH" = true ] && [ "${DETECTOR_LOG:-true}" = true ] && FETCH_LAUNCH="$FETCH_LAUNCH detector_log:=/tmp/chat/sightings.jsonl"
  [ "$VLN" = true ] && FETCH_ARGS="$FETCH_ARGS --vln"
  start_nav_stack /tmp/chat gui:=$GUI world:=$WORLD $FETCH_LAUNCH perception:=true estop_gate:=true target_frame:=map nav:=localization map:=$MAP_NAME navigation:=true controller:=policy odom_source:=ground_truth > /tmp/chat/start.log 2>&1 < /dev/null \
    || { echo "Nav2 never became ready (see /tmp/chat/launch.log)"; exit 1; }
  ros2 launch gurdy_safety monitors.launch.py odom_topic:=/odom > /tmp/chat/monitors.log 2>&1 < /dev/null &
  ros2 run gurdy_navigation frontier_explorer_node --ros-args -p serve_action:=true -p clock_topic:=/clock_slow -p log_path:=/tmp/chat/explore_log.json -p done_path:=/tmp/chat/explore.done > /tmp/chat/explorer.log 2>&1 < /dev/null &
  [ "$FETCH" = true ] && { ros2 launch gurdy_arm_moveit move_group.launch.py arm:=true > /tmp/chat/move_group.log 2>&1 < /dev/null & }
  sleep 12
  MEMORY_ARGS=""; [ "$MEMORY" = true ] && MEMORY_ARGS="--memory-db /home/ubuntu/.gurdy/memory.db"
  LLM_ARGS=""; [ "$MODEL" != keyword ] && LLM_ARGS="--ollama $MODEL"
  exec python3 -m brain.ros.run_text $LLM_ARGS --places /tmp/chat/places.yaml --map-id $MAP_ID --odom-topic /odom ${MEMORY_ARGS} $FETCH_ARGS --web 8080
' > "$LOG" 2>&1 &
AGENT_PID=$!
# RAM sampler: <t> <container MiB> <WSL used MiB> <GPU MiB>
( set +eo pipefail; while true; do
    c=$(docker stats --no-stream --format '{{.MemUsage}}' gurdy3_agent 2>/dev/null | awk '{v=$1; if (v ~ /GiB/) {sub("GiB","",v); v*=1024} else sub("MiB","",v); printf "%d", v}')
    w=$(free -m | awk '/Mem:/ {print $3}'); g=$(/usr/lib/wsl/lib/nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits 2>/dev/null | head -1)
    echo "$(date +%s) ${c:-0} ${w:-0} ${g:-0}" >> "$RAM"; sleep 5; done ) &
SAMPLER=$!
echo "starting the simulation (about 2.5 minutes; log $LOG) ..."
for _ in $(seq 400); do
  grep -q "access token:" "$LOG" 2>/dev/null && break
  grep -q "never became ready" "$LOG" 2>/dev/null && { tail -15 "$LOG"; exit 1; }
  kill -0 $AGENT_PID 2>/dev/null || { echo "the agent container exited before it was ready. Last lines of $LOG:"; tail -25 "$LOG"; exit 1; }
  sleep 1
done
grep -q "access token:" "$LOG" || { echo "the web app did not start in time"; tail -25 "$LOG"; exit 1; }
if [ "$MODEL" != keyword ]; then ollama_verify_in gurdy3_agent "$MODEL" || exit 1; fi      # never say READY with a language model the agent cannot reach
echo
echo "READY. Open in the laptop browser:  http://localhost:8080/#token=$GURDY_WEB_TOKEN"
[ "$PUBLISH" = 0.0.0.0 ] && echo "Phone (same Wi-Fi):                 http://<PC Wi-Fi IPv4>:8080/#token=$GURDY_WEB_TOKEN   (docs/phase7_phone.md)"
$HTTPS && echo "https (phone microphone):           https://<PC IPv4>:8443/#token=$GURDY_WEB_TOKEN   (trust the CA first: docs/phase7_voice.md)"
$VOICE && echo "Voice: hold the mic button (laptop: hold Space) and speak; the speech service is container gurdy3_speech."
if $TAIL; then ts_start 8080 || exit 1; echo "ANYWHERE (private, Tailscale on the phone must be ON):  ${TS_URL}#token=$GURDY_WEB_TOKEN"; fi
echo "Ctrl+C stops everything. Agent / robot log: $LOG"
wait $AGENT_PID
