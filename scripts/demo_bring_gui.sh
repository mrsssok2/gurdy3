#!/usr/bin/env bash
# One watched "bring me X" take in Gazebo WITH the GUI (GUI=false: headless), demo rehearsal 2026-11-28, graded from Gazebo GT.
#   scripts/demo_bring_gui.sh [COMMAND] [OUT_DIR]       default "bring me the blue can", docs/results/data/demo_bring_<date>
# Holds the shared GPU lock for the whole take. Stack: GUI=true, MODEL=keyword, FETCH=true with --fetch-bring-me (comes back
# to where the request came from, hands over standing still, "Here's your X" only when confirmed) + GT log + attempt log.
# The token is random, memory only; every saved log is redacted. Keeps the stack up KEEP_S (60) s after the reply, then stops.
set -uo pipefail
cd "$(dirname "$0")/.."
source scripts/gpu_lock.sh && gpu_lock_acquire "gurdy3:$(basename "$0")" || exit 1   # shared GPU lock for the whole run (scripts/gpu_lock.sh)
CMD=${1:-bring me the blue can}
OUT=${2:-docs/results/data/demo_bring_$(date -u +%Y%m%d_%H%M)}
MODEL_NAME=${MODEL_NAME:-f_blue_can}
mkdir -p "$OUT"; RAW=$PWD/.overnight_raw/demo_$(date -u +%s); mkdir -p "$RAW/chat"
LOG=$OUT/run.log; EVENTS=$OUT/events.jsonl
TOKEN=$(python3 -c 'import secrets;print(secrets.token_urlsafe(24))'); export GURDY_WEB_TOKEN=$TOKEN
source scripts/lib_fetch_run.sh
docker ps -q | grep -q . && { log "docker ps not empty: not starting"; exit 1; }
cleanup() { [ -n "${PID:-}" ] && kill -INT "$PID" 2>/dev/null
  for _ in $(seq 30); do [ -z "$(docker ps -q)" ] && break; sleep 2; done; docker rm -f gurdy3_agent >/dev/null 2>&1
  [ -f /tmp/gurdy_web.log ] && python3 scripts/redact_logs.py copy /tmp/gurdy_web.log "$OUT/agent.log" >/dev/null 2>&1
  save_logs "$RAW" "$OUT/raw"; rmdir "$(dirname "$RAW")" 2>/dev/null; true; }
trap cleanup EXIT
log "demo take: '$CMD' (GUI=${GUI:-true})"
CHAT_LOG_DIR="$RAW/chat" GUI=${GUI:-true} MODEL=keyword MEMORY=false FETCH=true \
  FETCH_EXTRA="--fetch-bring-me --fetch-gt log --fetch-log /tmp/chat/attempts.jsonl" scripts/start_web_robot.sh > "$RAW/stack.out" 2>&1 &
PID=$!
for _ in $(seq 72); do grep -q READY "$RAW/stack.out" 2>/dev/null && break; kill -0 $PID 2>/dev/null || break; sleep 10; done
grep -q READY "$RAW/stack.out" || { log "stack did NOT start: $(tail -2 "$RAW/stack.out" | tr '\n' ' ')"; exit 1; }
log "stack ready (GUI); waiting 20 s for the detector"; sleep 20
before=$(gt_pose "$MODEL_NAME"); robot0=$(gt_pose gurdy); t0=$(date +%s)
log "GT before: $MODEL_NAME $before, robot $robot0"
reply=$(say "$CMD" 1800)
wall=$(( $(date +%s) - t0 ))
after=$(gt_pose "$MODEL_NAME"); robot1=$(gt_pose gurdy); sleep 5; after5=$(gt_pose "$MODEL_NAME")
log "reply after ${wall}s: $reply"
log "GT after: $MODEL_NAME $after (5 s later $after5), robot $robot1"
python3 - "$OUT/result.json" "$CMD" "$reply" "$wall" "$before" "$robot0" "$after" "$after5" "$robot1" <<'EOF'
import json, math, sys
f, cmd, reply, wall, b, r0, a, a5, r1 = sys.argv[1:]
P = lambda s: [float(v) for v in s.split()] if s.strip() else None          # noqa: E731
b, r0, a, a5, r1 = map(P, (b, r0, a, a5, r1))
res = {"command": cmd, "reply": reply, "wall_s": int(wall), "object_before": b, "robot_before": r0, "object_after": a,
       "object_5s": a5, "robot_after": r1}
if b and a and a5 and r0:
    moved = math.hypot(a5[0] - b[0], a5[1] - b[1])
    on_floor = a5[2] < 0.05
    near_start = math.hypot(a5[0] - r0[0], a5[1] - r0[1])
    res.update(moved_m=round(moved, 3), on_floor=on_floor, dist_to_requester_start_m=round(near_start, 3),
               delivered_gt=bool(moved > 0.3 and on_floor and near_start < 1.0),
               still_held_gt=bool(a5[2] > 0.15))
json.dump(res, open(f, "w"), indent=1)
print(json.dumps(res))
EOF
log "keeping the stack up ${KEEP_S:-60} s (GUI)"; sleep "${KEEP_S:-60}"
