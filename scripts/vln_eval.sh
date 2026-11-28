#!/bin/bash
# Gazebo VLN eval: every episode of a VLN episode file (format: brain/eval/vln_episodes_dev.yaml) through the real agent with VLN on,
# ONE fresh Gazebo + Nav2 stack per episode, one at a time, graded from GROUND TRUTH (brain/vln/gazebo_eval.py).
#   scripts/vln_eval.sh SUITE.yaml                         # e.g. the user's blind suite; scripts/vln_dev_run.sh = the 3 DEV episodes
#   MODEL=gurdy-sft-400 scripts/vln_eval.sh SUITE.yaml     # plan by the LLM (+ keyword fallback); default MODEL=keyword = no Ollama
#   LANDMARKS=map ...                                      # landmarks from the live semantic map only (default gt: GT positions,
#                                                          # confirmation by the live detector)
#   ONLY=id1,id2  ATTEMPTS=3  WALL_MIN=600  WALL_FACTOR=4  GUI=false  RUN_DIR=brain/runs/vln_eval/<ts> (resume: valid episodes skipped)
# Holds the shared GPU lock (scripts/gpu_lock.sh) for the WHOLE batch. Refuses to start when `docker ps` is not empty.
# Builds the colcon workspace in the container first if the world is not installed (a fresh worktree has no ros2_ws/install;
# SKIP_BUILD=1 skips the check). Infrastructure failures (stack never ready, Gazebo hang, wall timeout) are retried ATTEMPTS
# times and then recorded, outside the rates.
# Results: $RUN/{meta.json,run.log,status.txt,episodes/<id>.json,logs/,scores.json,summary.json,report.md}.
set -uo pipefail
cd "$(dirname "$0")/.."
SUITE_FILE=${1:?usage: scripts/vln_eval.sh SUITE.yaml}
PY=${PY:-python3}
source scripts/gpu_lock.sh && gpu_lock_acquire "gurdy3:$(basename "$0")" || exit 1
MODEL=${MODEL:-keyword}; LANDMARKS=${LANDMARKS:-gt}; ATTEMPTS=${ATTEMPTS:-3}; WALL_MIN=${WALL_MIN:-600}; WALL_FACTOR=${WALL_FACTOR:-4}; GUI=${GUI:-false}
INSTALL_DIR=${INSTALL_DIR:-ros2_ws/install}
TS=$(date -u +%Y%m%d_%H%M%S); RUN=${RUN_DIR:-brain/runs/vln_eval/$TS}
RUN_REL=${RUN#"$PWD"/}
case "$RUN_REL" in /*) echo "RUN_DIR must be inside the repo ($PWD)"; exit 1 ;; esac
mkdir -p "$RUN/episodes" "$RUN/logs"
log() { echo "$(date -u +%H:%M:%S) $*" | tee -a "$RUN/run.log"; }

read -r WORLD MAP_NAME PLACES_FILE MAP_ID <<<"$($PY -c "
import sys, yaml; sys.path.insert(0, '.')
from brain.vln.gazebo_eval import load_suite
s = load_suite(sys.argv[1]); print(s['world'], s['map_name'], s['places'], yaml.safe_load(open(s['places']))['map_id'])" "$SUITE_FILE")" \
  || { echo "cannot read the suite $SUITE_FILE"; exit 1; }
[ -n "$WORLD" ] || { echo "cannot read the suite $SUITE_FILE"; exit 1; }
IDS=$($PY -c "
import os, sys; sys.path.insert(0, '.')
from brain.vln.gazebo_eval import load_suite
only = [x for x in os.environ.get('ONLY', '').split(',') if x]
print(' '.join(e['id'] for e in load_suite(sys.argv[1])['episodes'] if not only or e['id'] in only))" "$SUITE_FILE")
[ -n "$IDS" ] || { echo "no episodes selected"; exit 1; }
N=$(echo $IDS | wc -w)
limit_of() { $PY -c "
import sys; sys.path.insert(0, '.')
from brain.vln.gazebo_eval import load_suite
print(int(next(e for e in load_suite(sys.argv[1])['episodes'] if e['id'] == sys.argv[2])['limit_s']))" "$SUITE_FILE" "$1"; }
status_of() { $PY -c "import json,sys;print(json.load(open(sys.argv[1])).get('status','?'))" "$1" 2>/dev/null || echo unreadable; }

# ---- preflight
if [ -n "$(docker ps -q)" ]; then echo "other containers are running (docker ps): stop them first (one heavy workload at a time)" | tee -a "$RUN/run.log"; exit 1; fi
for d in "$HOME" /mnt/c; do
  free=$(df -BG --output=avail "$d" 2>/dev/null | tail -1 | tr -dc 0-9)
  [ -n "$free" ] && [ "$free" -lt 5 ] && { log "less than 5 GB free on $d: refusing to start"; exit 1; }
done
if [ "${SKIP_BUILD:-0}" != 1 ] && [ ! -e "$INSTALL_DIR/gurdy_gazebo/share/gurdy_gazebo/worlds/$WORLD.sdf" ]; then
  log "world $WORLD not installed in $INSTALL_DIR: building the workspace in the container (once)"
  docker compose run --rm -T --name gurdy3_vln_build sim bash -c 'cd ~/gurdy2/ros2_ws && colcon build --symlink-install' > "$RUN/logs/colcon_build.log" 2>&1
  [ -e "$INSTALL_DIR/gurdy_gazebo/share/gurdy_gazebo/worlds/$WORLD.sdf" ] || { log "build did not install $WORLD.sdf (see $RUN/logs/colcon_build.log)"; exit 1; }
fi
OLLAMA_ARG=""
if [ "$MODEL" != keyword ]; then
  . scripts/lib_ollama.sh
  ollama_prepare "$MODEL" > "$RUN/ollama_prepare.log" 2>&1 || { cat "$RUN/ollama_prepare.log"; log "no usable Ollama for $MODEL"; exit 1; }
  OLLAMA_ARG="--ollama $MODEL"
fi
cat > "$RUN/meta.json" <<JSON
{"started": "$(date -u +%FT%TZ)", "commit": "$(git rev-parse --short HEAD 2>/dev/null)", "dirty": $(git status --porcelain 2>/dev/null | grep -q . && echo true || echo false), "suite": "$SUITE_FILE", "world": "$WORLD", "model": "$MODEL", "landmarks": "$LANDMARKS", "episodes": $N, "attempts_max": $ATTEMPTS}
JSON
trap 'docker rm -f gurdy3_vln_eval >/dev/null 2>&1; log "exiting"' EXIT
log "VLN eval: $N episodes of $SUITE_FILE, world $WORLD, model $MODEL, landmarks $LANDMARKS, run dir $RUN"

run_one() {
  local id=$1 out="$RUN/episodes/$1.json" limit wall attempt rc st chat walled=0 t_att
  limit=$(limit_of "$id"); wall=$(( WALL_FACTOR * limit )); [ "$wall" -lt "$WALL_MIN" ] && wall=$WALL_MIN
  if [ -f "$out" ] && [[ "$(status_of "$out")" =~ ^(ok|timeout)$ ]]; then log "$id: valid result exists, skipped"; return 0; fi
  for attempt in $(seq "$ATTEMPTS"); do
    rm -f "$out"; docker rm -f gurdy3_vln_eval >/dev/null 2>&1
    chat="$RUN_REL/logs/$id.attempt$attempt.stack"; mkdir -p "$chat"
    log "$id: attempt $attempt/$ATTEMPTS (limit ${limit}s sim, wall ${wall}s)"
    t_att=$SECONDS
    timeout -k 30 "$wall" docker compose run --rm -T --name gurdy3_vln_eval -e OLLAMA_URL -e EP="$id" -e OUT="$out" -e CHAT="$chat" \
      -e SUITE_FILE="$SUITE_FILE" -e WORLD="$WORLD" -e MAP_NAME="$MAP_NAME" -e MAP_ID="$MAP_ID" -e PLACES_FILE="$PLACES_FILE" \
      -e LANDMARKS="$LANDMARKS" -e OLLAMA_ARG="$OLLAMA_ARG" -e GUI="$GUI" sim bash -c '
      cd ~/gurdy2 && source ros2_ws/install/setup.bash && export PYTHONPATH=$HOME/gurdy2:$HOME/gurdy2/gurdy_core:$HOME/gurdy2/sim_mujoco:$PYTHONPATH
      source scripts/lib_nav_start.sh; mkdir -p "$CHAT"; cp "$PLACES_FILE" "$CHAT/places.yaml"
      trap "stop_nav_stack" EXIT
      start_nav_stack "$CHAT" gui:=$GUI world:=$WORLD perception:=true estop_gate:=true target_frame:=map nav:=localization map:=$MAP_NAME navigation:=true controller:=policy odom_source:=ground_truth > "$CHAT/start.log" 2>&1 < /dev/null \
        || { echo "Nav2 never became ready"; tail -20 "$CHAT/launch.log"; exit 5; }
      ros2 launch gurdy_safety monitors.launch.py odom_topic:=/odom > "$CHAT/monitors.log" 2>&1 < /dev/null &
      sleep 12
      python3 -m brain.ros.vln_eval_runner --suite "$SUITE_FILE" --episode "$EP" --out "$OUT" --places "$CHAT/places.yaml" --map-id "$MAP_ID" \
        --odom-topic /odom --gt-topic /odom/ground_truth --landmarks "$LANDMARKS" $OLLAMA_ARG 2>&1 | grep --line-buffered -v "^terminate"
      exit ${PIPESTATUS[0]}
    ' > "$RUN/logs/$id.attempt$attempt.log" 2>&1
    rc=$?
    docker rm -f gurdy3_vln_eval >/dev/null 2>&1
    if [ "$rc" = 124 ] || { [ "$((SECONDS - t_att))" -ge "$((wall - 2))" ] && [ ! -f "$out" ]; }; then walled=$((walled + 1)); log "$id: attempt $attempt hit the wall limit"; fi
    st=none; [ -f "$out" ] && st=$(status_of "$out")
    log "$id: attempt $attempt finished rc=$rc result=$st"
    [[ "$st" =~ ^(ok|timeout)$ ]] && return 0
    sleep "${RETRY_SLEEP:-5}"
  done
  $PY - "$out" "$id" "$RUN/logs/$id.attempt$ATTEMPTS.log" "$walled" "$ATTEMPTS" <<'PY'
import json, sys
out, eid, lg, walled, attempts = sys.argv[1:]
try:
    tail = open(lg, errors="replace").read()[-600:]
except OSError:
    tail = ""
st = "wall_timeout" if int(walled) == int(attempts) else "infra_failed"
json.dump({"episode": eid, "status": st, "error": f"no valid run in {attempts} attempts ({walled} wall timeouts); last log: " + tail}, open(out, "w"))
PY
  return 1
}

i=0
for id in $IDS; do
  i=$((i + 1)); echo "$i/$N running $id ($(date -u +%H:%M:%S))" > "$RUN/status.txt"
  run_one "$id" && log "$id: done" || log "$id: INFRASTRUCTURE FAILURE after $ATTEMPTS attempts"
  $PY -m brain.vln.gazebo_eval "$RUN" "$SUITE_FILE" > /dev/null 2>> "$RUN/run.log"
done
$PY -m brain.vln.gazebo_eval "$RUN" "$SUITE_FILE" 2>&1 | tee -a "$RUN/run.log"
echo "DONE $N/$N ($(date -u +%H:%M:%S))" > "$RUN/status.txt"
log "finished: $RUN/report.md"
