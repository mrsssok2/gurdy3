#!/bin/bash
# Phase 5.11 final Gazebo eval: the scripted mission suite (brain/eval/final_suite.py) through the real agent, typed input, model gurdy-sft-400.
# ONE fresh Gazebo + Nav2 stack per mission (clean world, battery, faults, zones, memory), one stack at a time, per-mission time limit,
# automatic restart + retry (max ATTEMPTS) when the stack fails to start, Gazebo hangs (no /clock progress 90 s) or the container dies.
# WALL-CLOCK TIMEOUT per attempt = max(WALL_MIN 300 s, WALL_FACTOR 4 x the mission's sim limit), covering start-up + mission: on expiry the container is
# killed, the attempt is retried, and if every attempt hit the wall limit the mission is recorded as FAILED with status "wall_timeout" (reason in the report).
# A stack's logs (start.log, nav_ready.txt with phase timestamps, launch.log, explorer / monitors logs, memory.db) are written straight into
# $RUN/logs/<id>.attemptN.stack/ so they survive the container (before, they died with it).
#   nohup scripts/p5_final_eval.sh > /tmp/p5_final_eval.out 2>&1 &            # whole suite (about 3-4 h)
#   ONLY=nav_room_b,stop_mid_walk scripts/p5_final_eval.sh                    # a subset
#   RUN_DIR=brain/runs/final_eval/<ts> scripts/p5_final_eval.sh               # resume: missions with a valid result are skipped
# Results: brain/runs/final_eval/<timestamp>/{meta.json,status.txt,run.log,missions/<id>.json,logs/,report.md,summary.json}; at the end docs/results/phase5_final_eval.md.
set -uo pipefail
cd "$(dirname "$0")/.."
# EXTRA_LAUNCH_ARGS="shield:=true" adds sim.launch.py arguments to every stack (default empty = unchanged; Phase 8b live check).
# RECORD_COSTMAP=true saves the global costmap + pose at every planner failure into <stack dir>/costmaps (default false).
# SUITE=final_suite_lab4 (Gurdy 3.0 held-out, world lab4) selects the suite module, world, saved map, places file and map id; SEED=N seeds the start offset (final_eval_runner --seed).
export GURDY_FINAL_SUITE=${SUITE:-final_suite}; SEED=${SEED:-}
read -r WORLD MAP_NAME MAP_ID PLACES_FILE <<<"$(python3 -c "
from brain.eval.suites import suite_module as m
s = m(); print(getattr(s, 'WORLD', 'flat_lab'), getattr(s, 'MAP_NAME', 'flat_lab_tripod_gt'), getattr(s, 'MAP_ID', 'flat_lab'), getattr(s, 'PLACES_FILE', 'maps/flat_lab.places.yaml'))")"
export WORLD MAP_NAME MAP_ID PLACES_FILE
export MODEL=${MODEL:-gurdy-sft-400}; PORT=11435; ATTEMPTS=${ATTEMPTS:-3}; WALL_FACTOR=${WALL_FACTOR:-4}; WALL_MIN=${WALL_MIN:-300}; DOC=${DOC:-$([ "${SUITE:-final_suite}" = final_suite ] && echo docs/results/phase5_final_eval.md || echo "")}
TS=$(date -u +%Y%m%d_%H%M%S); RUN=${RUN_DIR:-brain/runs/final_eval/$TS}
[ -n "$DOC" ] || DOC="$RUN/report_full.md"
mkdir -p "$RUN/missions" "$RUN/logs" "$RUN/attempts"
RUN_REL=${RUN#"$PWD"/}      # the repo is mounted in the container at ~/gurdy2 (same relative layout), so logs can be written there directly
case "$RUN_REL" in /*) echo "RUN_DIR must be inside the repo ($PWD)"; exit 1 ;; esac
log() { echo "$(date -u +%H:%M:%S) $*" | tee -a "$RUN/run.log"; }
IDS=$(python3 -c "
import os
from brain.eval.suites import suite_module
SUITE = suite_module().SUITE
only=[x for x in os.environ.get('ONLY','').split(',') if x]
print(' '.join(m['id'] for m in SUITE if not only or m['id'] in only))")
[ -n "$IDS" ] || { echo "no missions selected"; exit 1; }
N=$(echo $IDS | wc -w)

# ---- preflight
if [ -n "$(docker ps -q)" ]; then log "other containers are running (docker ps): stop them first (one heavy workload at a time)"; docker ps --format '{{.Names}}' | tee -a "$RUN/run.log"; exit 1; fi
for d in "$HOME" /mnt/c; do
  free=$(df -BG --output=avail "$d" 2>/dev/null | tail -1 | tr -dc 0-9)
  [ -n "$free" ] && [ "$free" -lt 5 ] && { log "less than 5 GB free on $d ($free GB): refusing to start (a full disk crashed WSL before)"; exit 1; }
done
# start Ollama if none answers, check the EXACT model, and pick an address a container can really reach (probed from a throwaway container;
# `hostname -I` alone was refused from containers on 2026-11-19, see scripts/lib_ollama.sh). Exports OLLAMA_URL.
. scripts/lib_ollama.sh
ollama_prepare "$MODEL" > "$RUN/ollama_prepare.log" 2>&1 || { cat "$RUN/ollama_prepare.log"; log "no usable Ollama for $MODEL"; exit 1; }   # no pipe: it must export OLLAMA_URL into THIS shell
cat "$RUN/ollama_prepare.log" >> "$RUN/run.log"
[ -n "${OLLAMA_URL:-}" ] || { log "OLLAMA_URL not set"; exit 1; }
cat > "$RUN/meta.json" <<JSON
{"started": "$(date -u +%FT%TZ)", "commit": "$(git rev-parse --short HEAD)", "dirty": $(git status --porcelain | grep -q . && echo true || echo false), "model": "$MODEL", "missions": $N, "attempts_max": $ATTEMPTS, "host": "$(hostname)"}
JSON
trap 'docker rm -f gurdy3_final_eval >/dev/null 2>&1; log "interrupted / exiting"' EXIT
log "final eval: $N missions, model $MODEL, run dir $RUN"

result_status() { python3 -c "import json,sys;print(json.load(open(sys.argv[1])).get('status','?'))" "$1" 2>/dev/null || echo "unreadable"; }
limit_of() { python3 -c "from brain.eval.suites import suite_module;print(suite_module().BY_ID['$1']['limit_s'])"; }

run_one() {   # $1 = mission id; returns 0 when a valid result file exists
  local id=$1 limit attempt rc st t_att wall walled=0 out="$RUN/missions/$1.json" chat
  limit=$(limit_of "$id")
  wall=$(( WALL_FACTOR * limit )); [ "$wall" -lt "$WALL_MIN" ] && wall=$WALL_MIN
  if [ -f "$out" ] && [[ "$(result_status "$out")" =~ ^(ok|timeout)$ ]]; then log "$id: valid result exists, skipped"; return 0; fi
  echo '{"attempts": 0, "log": []}' > "$RUN/attempts/$id.json"
  for attempt in $(seq "$ATTEMPTS"); do
    rm -f "$out"
    docker rm -f gurdy3_final_eval >/dev/null 2>&1
    curl -s "localhost:$PORT/api/generate" -d "{\"model\":\"$MODEL\",\"keep_alive\":\"30m\"}" >/dev/null   # model resident before the sim takes RAM
    chat="$RUN_REL/logs/$id.attempt$attempt.stack"; mkdir -p "$chat"
    log "$id: attempt $attempt/$ATTEMPTS (limit ${limit}s sim, wall limit ${wall}s)"
    t_att=$SECONDS
    timeout -k 30 "$wall" docker compose run --rm -T --name gurdy3_final_eval -e OLLAMA_URL -e MODEL -e GURDY_DEBUG_NAV -e GURDY_FINAL_SUITE -e WORLD -e MAP_NAME -e MAP_ID -e PLACES_FILE -e SEED -e EXTRA_LAUNCH_ARGS -e RECORD_COSTMAP -e ID="$id" -e OUT="$out" -e CHAT="$chat" sim bash -c '
      cd ~/gurdy2 && source ros2_ws/install/setup.bash && export PYTHONPATH=$HOME/gurdy2:$HOME/gurdy2/gurdy_core:$HOME/gurdy2/sim_mujoco:$PYTHONPATH
      source scripts/lib_nav_start.sh; mkdir -p "$CHAT"; cp "$PLACES_FILE" "$CHAT/places.yaml"
      trap "stop_nav_stack" EXIT
      start_nav_stack "$CHAT" gui:=false world:=$WORLD perception:=true estop_gate:=true target_frame:=map nav:=localization map:=$MAP_NAME navigation:=true controller:=policy odom_source:=ground_truth $EXTRA_LAUNCH_ARGS $([ "${RECORD_COSTMAP:-false}" = true ] && echo costmap_record_dir:=$HOME/gurdy2/$CHAT/costmaps) > "$CHAT/start.log" 2>&1 < /dev/null \
        || { echo "Nav2 never became ready"; tail -20 "$CHAT/launch.log"; exit 5; }
      ros2 launch gurdy_safety monitors.launch.py odom_topic:=/odom > "$CHAT/monitors.log" 2>&1 < /dev/null &
      ros2 run gurdy_navigation frontier_explorer_node --ros-args -p serve_action:=true -p clock_topic:=/clock_slow -p log_path:=$CHAT/explore_log.json -p done_path:=$CHAT/explore.done > "$CHAT/explorer.log" 2>&1 < /dev/null &
      sleep 12
      python3 -m brain.ros.final_eval_runner --mission "$ID" --out "$OUT" --ollama "$MODEL" --places "$CHAT/places.yaml" --map-id $MAP_ID --odom-topic /odom ${SEED:+--seed $SEED} --memory-db "$CHAT/memory.db" 2>&1 | grep --line-buffered -v "^terminate"
      exit ${PIPESTATUS[0]}
    ' > "$RUN/logs/$id.attempt$attempt.log" 2>&1
    rc=$?
    docker rm -f gurdy3_final_eval >/dev/null 2>&1                  # also kills the stack when `timeout` only stopped the docker client
    if [ "$rc" = 124 ] || { [ "$((SECONDS - t_att))" -ge "$((wall - 2))" ] && [ ! -f "$out" ]; }; then walled=$((walled + 1)); log "$id: attempt $attempt hit the wall limit (${wall}s): container removed"; fi
    st=none; [ -f "$out" ] && st=$(result_status "$out")
    python3 - "$RUN/attempts/$id.json" "$attempt" "$rc" "$st" <<'PY'
import json, sys
p, a, rc, st = sys.argv[1:]
d = json.load(open(p)); d["attempts"] = int(a); d["log"].append({"attempt": int(a), "rc": int(rc), "status": st}); json.dump(d, open(p, "w"))
PY
    log "$id: attempt $attempt finished rc=$rc result=$st"
    if [[ "$st" =~ ^(ok|timeout)$ ]]; then return 0; fi
    sleep 5
  done
  python3 - "$out" "$id" "$RUN/logs/$id.attempt$ATTEMPTS.log" "$walled" "$ATTEMPTS" "$wall" <<'PY'
import json, sys
out, mid, lg, walled, attempts, wall = sys.argv[1:]
try:
    tail = open(lg, errors="replace").read()[-600:]
except OSError:
    tail = ""
if int(walled) == int(attempts):          # every attempt ran into the wall-clock limit: a FAILED mission with that reason (not an infrastructure failure)
    rec = {"mission": mid, "status": "wall_timeout", "error": f"wall timeout: {attempts} of {attempts} attempts exceeded {wall} s wall; last log: " + tail}
else:
    rec = {"mission": mid, "status": "infra_failed", "error": "no valid run in all attempts" + (f" ({walled} wall timeouts)" if int(walled) else "") + "; last log: " + tail}
rec.update({"poses": [], "messages": [], "turns": []})
json.dump(rec, open(out, "w"))
PY
  return 1
}

i=0
for id in $IDS; do
  i=$((i + 1))
  echo "$i/$N running $id ($(date -u +%H:%M:%S))" > "$RUN/status.txt"
  run_one "$id" && log "$id: done" || log "$id: INFRASTRUCTURE FAILURE (no valid result after $ATTEMPTS attempts)"
  python3 -m brain.eval.final_report "$RUN" --note "PARTIAL: $i of $N missions finished" 2>&1 | tail -1 | tee -a "$RUN/run.log"
done
python3 -m brain.eval.final_report "$RUN" --doc "$DOC" 2>&1 | tee -a "$RUN/run.log"
echo "DONE $N/$N ($(date -u +%H:%M:%S))" > "$RUN/status.txt"
log "finished; report: $DOC and $RUN/report.md"
