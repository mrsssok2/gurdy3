#!/usr/bin/env bash
# Fetch retest (written 2026-11-27 after the first fetch dry run, docs/results/phase9j_fetch_dry_run.md; NOT run yet).
# Three variants to separate the causes. Target: the blue can (lab4_fetch GT (-1.30, 0.30)), 1.33 m from the spawn point
# (inside the ~2 m where det_v3_s finds small objects reliably). Command: "pick up the blue can". No LLM (fetch router).
#   A  robot TELEPORTED to the grasp_test pose (object in the class zone behind it) + detector / servo pose   -> grasp pipeline only
#   B  Nav2 approach + GROUND-TRUTH object pose for the grasp plan                                              -> approach only
#   C  full fetch: search (fresh stack, can not in the map) + Nav2 approach + detector / servo pose             -> everything
# Every attempt writes a diagnostics record (detector / servo estimate vs GT, robot->object distance, plan, centring,
# fingers, failure reason): brain/ros/fetch_robot.py cfg.attempt_log. The can is put back at its GT pose before each try.
#
#   scripts/fetch_retest.sh [A] [B] [C]          (default: A B C)   env: TRIES (3, A and B), C_TRIES (2: one fresh stack each),
#                                                                         TRY_TIMEOUT_S (900 wall s per command, then e-stop)
# Output: docs/results/data/fetch_retest/<UTC stamp>/<variant>/{tries.jsonl,attempts.jsonl,chat/...}, then
#         scripts/fetch_retest_report.py writes docs/results/phase9j_fetch_retest.md. Needs docker ps empty (Gazebo + GPU).
# Expected wall time (RTF ~0.3, start-up ~3-5 min per stack, try 2 of the dry run took 4 min): A ~20 min, B ~20, C ~25 -> ~65-80 min.
set -uo pipefail
cd "$(dirname "$0")/.."
VARIANTS=("$@"); [ ${#VARIANTS[@]} -eq 0 ] && VARIANTS=(A B C)
TRIES=${TRIES:-3}; C_TRIES=${C_TRIES:-2}; TRY_TIMEOUT_S=${TRY_TIMEOUT_S:-900}
TARGET_MODEL=f_blue_can; TX=-1.30; TY=0.30; TYAW=0.0; CMD="pick up the blue can"
if [ -n "$(docker ps -q)" ]; then echo "Other containers are running (docker ps). Stop them first:"; docker ps --format '  {{.Names}}'; exit 1; fi
OUT=docs/results/data/fetch_retest/$(date -u +%Y%m%d_%H%M%S); mkdir -p "$OUT"
TOKEN=$(python3 -c 'import secrets;print(secrets.token_urlsafe(24))')        # never written to a file
STACK_PID=""
log() { echo "$(date -u +%T) $*" | tee -a "$OUT/retest.log"; }
api() { curl -s -m "${3:-10}" -H "Authorization: Bearer $TOKEN" -H "Content-Type: application/json" ${2:+-d "$2"} "localhost:8080$1"; }
gzx() { docker exec gurdy3_agent /entrypoint.sh "$@"; }
gt_pose() {   # model -> "x y" (gz dynamic pose; empty if unknown)
  gzx timeout 8 gz topic -e -n 1 -t /world/lab4_fetch/pose/info 2>/dev/null | python3 -c '
import re,sys
for blk in re.split(r"\npose \{", sys.stdin.read()):
    m = re.search(r"name: \"([^\"]+)\"", blk)
    if m and m.group(1) == sys.argv[1]:
        p = re.search(r"position \{([^}]*)\}", blk); g = lambda k: float((re.search(k + r": ([-0-9.e]+)", p.group(1)) or [0, 0])[1]) if p else 0.0
        print("%.4f %.4f" % (g("x"), g("y")))' "$1"; }
set_pose() {  # model x y z yaw
  gzx timeout 10 gz service -s /world/lab4_fetch/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
    --req "name: \"$1\" position {x: $2 y: $3 z: $4} orientation {x: 0 y: 0 z: $(python3 -c "import math;print(math.sin($5/2))") w: $(python3 -c "import math;print(math.cos($5/2))")}" | grep -q "data: true"; }
stop_stack() {
  [ -n "$STACK_PID" ] && kill -INT "$STACK_PID" 2>/dev/null; for _ in $(seq 30); do [ -z "$(docker ps -q)" ] && break; sleep 2; done
  docker rm -f gurdy3_agent >/dev/null 2>&1; STACK_PID=""; }
trap 'stop_stack; log "stopped"' EXIT INT TERM
start_stack() {   # dir extra-flags
  mkdir -p "$1/chat"
  CHAT_LOG_DIR="$PWD/$1/chat" GURDY_WEB_TOKEN="$TOKEN" FETCH=true MODEL=keyword MEMORY=false GUI=false FETCH_EXTRA="$2" \
    scripts/start_web_robot.sh > "$1/stack.out" 2>&1 &
  STACK_PID=$!
  for _ in $(seq 60); do grep -q "READY" "$1/stack.out" && { log "stack ready ($2)"; return 0; }; kill -0 "$STACK_PID" 2>/dev/null || break; sleep 10; done
  log "stack did NOT start: $(tail -2 "$1/stack.out" | tr '\n' ' ')"; stop_stack; return 1; }
prime_map() {     # turn the robot (in place, gz set_pose; GT odometry keeps the map pose right) to face the can, let the detector confirm it
  set_pose gurdy 0.0 0.0 0.115 3.14159; sleep 25
  api /api/state | python3 -c 'import json,sys; o=[x for x in json.load(sys.stdin).get("objects",[]) if x.get("class")=="can" and x.get("color")=="blue"]; print("blue can in the map:", [(x["x"],x["y"]) for x in o])' | tee -a "$OUT/retest.log"; }
one_try() {       # dir variant k
  set_pose "$TARGET_MODEL" "$TX" "$TY" 0.0005 "$TYAW" || log "WARNING: can reset failed"; sleep 3
  local before robot0 t0 reply after robot1
  before=$(gt_pose "$TARGET_MODEL"); robot0=$(gt_pose gurdy); t0=$(date +%s)
  reply=$(api /api/say "{\"text\": \"$CMD\"}" "$TRY_TIMEOUT_S")
  if [ -z "$reply" ]; then api /api/estop '{}' >/dev/null; sleep 10; reply="TIMEOUT after ${TRY_TIMEOUT_S}s (e-stop)"; api /api/estop/release '{}' >/dev/null; fi
  after=$(gt_pose "$TARGET_MODEL"); robot1=$(gt_pose gurdy)
  python3 -c 'import json,sys; v=sys.argv; print(json.dumps({"variant": v[1], "try": int(v[2]), "wall_s": int(v[3]), "reply": v[4],
      "can_before": v[5], "can_after": v[6], "robot_before": v[7], "robot_after": v[8]}))' "$2" "$3" "$(( $(date +%s) - t0 ))" "$reply" "$before" "$after" "$robot0" "$robot1" >> "$1/tries.jsonl"
  log "$2 try $3: $(tail -1 "$1/tries.jsonl" | cut -c1-240)"
  if echo "$reply" | grep -q "I picked up"; then api /api/say '{"text": "put it down"}' 600 >/dev/null; fi   # free the gripper for the next try
}
for V in "${VARIANTS[@]}"; do
  D="$OUT/$V"; mkdir -p "$D"
  case "$V" in
    A) FL="--fetch-approach teleport --fetch-gt log --fetch-log /tmp/chat/attempts.jsonl" ;;
    B) FL="--fetch-gt use --fetch-log /tmp/chat/attempts.jsonl" ;;
    C) FL="--fetch-gt log --fetch-log /tmp/chat/attempts.jsonl" ;;
    *) log "unknown variant $V (A | B | C)"; continue ;;
  esac
  if [ "$V" = C ]; then
    for k in $(seq "$C_TRIES"); do               # fresh stack per try: the can must NOT be in the map yet (search first)
      start_stack "$D/run$k" "$FL" || continue
      one_try "$D/run$k" C "$k"; stop_stack
      cat "$D/run$k/tries.jsonl" >> "$D/tries.jsonl" 2>/dev/null; cat "$D/run$k/chat/attempts.jsonl" >> "$D/attempts.jsonl" 2>/dev/null
    done
  else
    start_stack "$D" "$FL" || continue
    prime_map
    for k in $(seq "$TRIES"); do one_try "$D" "$V" "$k"; done
    stop_stack
    cp "$D/chat/attempts.jsonl" "$D/attempts.jsonl" 2>/dev/null
  fi
done
trap - EXIT; stop_stack
python3 scripts/fetch_retest_report.py "$OUT" --out docs/results/phase9j_fetch_retest.md | tee -a "$OUT/retest.log"
log "done: $OUT (docker ps: $(docker ps -q | wc -l) containers)"
