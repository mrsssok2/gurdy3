#!/bin/bash
# Phase 6 incident-graph evaluation (brain/eval/incident_suite.py): ONE session in ONE fresh lab4 Gazebo + Nav2 stack, real agent, typed questions.
#   nohup scripts/incident_eval.sh > /tmp/incident_eval.out 2>&1 &              about 1 - 2 h wall
#   RUN_DIR=brain/runs/incident_eval/<ts> scripts/incident_eval.sh              re-score an existing run dir only if result.json exists (no new Gazebo run)
# Results: brain/runs/incident_eval/<ts>/{result.json,score.json,report.md,stack/,run.log}; the report is also written to docs/results/phase6_incident_eval.md.
# Preflight refuses to start while any other container runs (one heavy workload at a time), like scripts/p5_final_eval.sh.
set -uo pipefail
cd "$(dirname "$0")/.."
python3 scripts/make_incident_world.py >/dev/null || exit 1           # eval copy of lab4 with the 3 scenario objects dynamic (lab4.sdf itself is never touched)
export WORLD_FILE=ros2_ws/src/gurdy_gazebo/worlds/lab4_incident.sdf MAP_NAME=lab4_map MAP_ID=lab4 PLACES_FILE=maps/lab4.places.yaml MODEL=${MODEL:-gurdy-sft-400}; PORT=11435; WALL=${WALL:-9000}
TS=$(date -u +%Y%m%d_%H%M%S); RUN=${RUN_DIR:-brain/runs/incident_eval/$TS}; mkdir -p "$RUN/stack"
RUN_REL=${RUN#"$PWD"/}; case "$RUN_REL" in /*) echo "RUN_DIR must be inside the repo ($PWD)"; exit 1 ;; esac
log() { echo "$(date -u +%H:%M:%S) $*" | tee -a "$RUN/run.log"; }
DOC=${DOC:-docs/results/phase6_incident_eval.md}

if [ ! -f "$RUN/result.json" ]; then
  [ -z "$(docker ps -q)" ] || { log "other containers are running (docker ps): stop them first"; docker ps --format '{{.Names}}'; exit 1; }
  for d in "$HOME" /mnt/c; do
    free=$(df -BG --output=avail "$d" 2>/dev/null | tail -1 | tr -dc 0-9)
    [ -n "$free" ] && [ "$free" -lt 5 ] && { log "less than 5 GB free on $d ($free GB): refusing to start"; exit 1; }
  done
  if ! curl -sf "localhost:$PORT/api/tags" >/dev/null; then
    log "starting Ollama on :$PORT"
    OLLAMA_HOST=0.0.0.0:$PORT OLLAMA_MODELS=$HOME/ollama_models LD_LIBRARY_PATH=/usr/lib/wsl/lib nohup "$HOME/ollama/bin/ollama" serve > /tmp/ollama_gurdy.log 2>&1 &
    for _ in $(seq 30); do curl -sf "localhost:$PORT/api/tags" >/dev/null && break; sleep 1; done
  fi
  curl -s "localhost:$PORT/api/tags" | grep -q "\"$MODEL" || { log "model $MODEL not in Ollama"; exit 1; }
  export OLLAMA_URL=http://$(hostname -I | cut -d' ' -f1):$PORT
  trap 'docker rm -f gurdy3_incident_eval >/dev/null 2>&1; log "exiting"' EXIT
  # the check that the held-out questions are still held out (no overlap with tests / training data): refuse to run otherwise
  docker compose run --rm -T brain python3 -c "
from brain.eval.incident_suite import check_overlap
h = check_overlap('.'); print('overlap:', h); raise SystemExit(1 if h else 0)" || { log "held-out check failed: a question text occurs elsewhere"; exit 1; }
  curl -s "localhost:$PORT/api/generate" -d "{\"model\":\"$MODEL\",\"keep_alive\":\"30m\"}" >/dev/null
  log "incident eval: model $MODEL, commit $(git rev-parse --short HEAD)$(git status --porcelain | grep -q . && echo ' (dirty)'), run dir $RUN, wall limit ${WALL}s"
  CHAT="$RUN_REL/stack"; export CHAT OUT="$RUN_REL/result.json"
  timeout -k 30 "$WALL" docker compose run --rm -T --name gurdy3_incident_eval -e OLLAMA_URL -e MODEL -e WORLD_FILE -e MAP_NAME -e MAP_ID -e PLACES_FILE -e CHAT -e OUT sim bash -c '
    cd ~/gurdy2 && source ros2_ws/install/setup.bash && export PYTHONPATH=$HOME/gurdy2:$HOME/gurdy2/gurdy_core:$HOME/gurdy2/sim_mujoco:$PYTHONPATH
    source scripts/lib_nav_start.sh; mkdir -p "$CHAT"; cp "$PLACES_FILE" "$CHAT/places.yaml"
    trap "stop_nav_stack" EXIT
    start_nav_stack "$CHAT" gui:=false world:=$HOME/gurdy2/$WORLD_FILE perception:=true estop_gate:=true target_frame:=map nav:=localization map:=$MAP_NAME navigation:=true controller:=policy odom_source:=ground_truth > "$CHAT/start.log" 2>&1 < /dev/null \
      || { echo "Nav2 never became ready"; tail -20 "$CHAT/launch.log"; exit 5; }
    ros2 launch gurdy_safety monitors.launch.py odom_topic:=/odom > "$CHAT/monitors.log" 2>&1 < /dev/null &
    ros2 run gurdy_navigation frontier_explorer_node --ros-args -p serve_action:=true -p clock_topic:=/clock_slow -p log_path:=$CHAT/explore_log.json -p done_path:=$CHAT/explore.done > "$CHAT/explorer.log" 2>&1 < /dev/null &
    sleep 12
    python3 -m brain.ros.incident_eval_runner --out "$OUT" --ollama "$MODEL" --places "$CHAT/places.yaml" --map-id $MAP_ID --odom-topic /odom --db "$CHAT/incidents.db" 2>&1 | grep --line-buffered -v "^terminate"
    exit ${PIPESTATUS[0]}
  ' > "$RUN/runner.log" 2>&1
  rc=$?
  docker rm -f gurdy3_incident_eval >/dev/null 2>&1
  log "runner finished rc=$rc"
fi
[ -f "$RUN/result.json" ] || { log "no result.json (see $RUN/runner.log)"; exit 1; }
docker compose run --rm -T brain python3 -m brain.eval.incident_score "$RUN_REL/result.json" --doc "$DOC" | tee -a "$RUN/run.log"
log "report: $DOC and $RUN/report.md"
