# Shared helpers for unattended fetch runs in Gazebo (scripts/overnight_1127.sh, scripts/fetch_retest.sh). Source it; needs:
#   TOKEN (dashboard token, in memory only), LOG (run log file). Raw logs go to a git-ignored dir first and reach the repo ONLY
#   through scripts/redact_logs.py (save_logs): no token / secret is ever written into docs/.
# DRY_RUN=1: scripts/lib_fetch_dryrun.sh replaces every Docker / Gazebo / HTTP call with a stub (CPU test of the control flow).
export REDACT_VALUES="${TOKEN:-}"          # exact value for the redactor (environment only, never the command line)
STACK_PID=""

log() { echo "$(date -u +%T) $*" | python3 scripts/redact_logs.py filter 2>/dev/null | tee -a "$LOG"; }
event() {   # event TYPE KEY=VALUE ...  -> one JSON line in $EVENTS (block start / end / skip, stack failures)
  python3 - "$EVENTS" "$@" <<'EOF'
import json, sys, time
f, typ, kv = sys.argv[1], sys.argv[2], sys.argv[3:]
d = {"t": time.strftime("%H:%M:%S", time.gmtime()), "type": typ, **dict(x.split("=", 1) for x in kv)}
open(f, "a").write(json.dumps(d) + "\n")
EOF
}
api() { curl -s -m "${3:-10}" -H "Authorization: Bearer $TOKEN" -H "Content-Type: application/json" ${2:+-d "$2"} "localhost:8080$1"; }
gzx() { docker exec gurdy3_agent /entrypoint.sh "$@"; }
say() {     # say TEXT TIMEOUT_S -> the reply text ("" on timeout / no answer)
  api /api/say "$(python3 -c 'import json,sys; print(json.dumps({"text": sys.argv[1]}))' "$1")" "$2" |
    python3 -c 'import json,sys; t=sys.stdin.read(); print(json.loads(t).get("reply", t) if t.strip() else "")' 2>/dev/null; }
gt_pose() {   # model -> "x y z yaw" (gz pose; a small object's origin = its base, z ~0 standing; empty if unknown)
  gzx timeout 8 gz topic -e -n 1 -t /world/lab4_fetch/pose/info 2>/dev/null | python3 -c '
import math, re, sys
for blk in re.split(r"\npose \{", sys.stdin.read()):
    m = re.search(r"name: \"([^\"]+)\"", blk)
    if not (m and m.group(1) == sys.argv[1]):
        continue
    def sec(name):
        s = re.search(name + r" \{([^}]*)\}", blk)
        return (lambda k: float((re.search(k + r": ([-0-9.e]+)", s.group(1)) or [0, 0])[1])) if s else None
    p, q = sec("position"), sec("orientation")
    x, y, z = (p("x"), p("y"), p("z")) if p else (0.0, 0.0, 0.0)
    qx, qy, qz, qw = (q("x"), q("y"), q("z"), q("w")) if q else (0.0, 0.0, 0.0, 1.0)
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    print("%.4f %.4f %.4f %.4f" % (x, y, z, yaw))
    break' "$1"; }
set_pose() {  # model x y z yaw
  gzx timeout 10 gz service -s /world/lab4_fetch/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
    --req "name: \"$1\" position {x: $2 y: $3 z: $4} orientation {x: 0 y: 0 z: $(python3 -c "import math;print(math.sin($5/2))") w: $(python3 -c "import math;print(math.cos($5/2))")}" | grep -q "data: true"; }
save_logs() { # save_logs RAW_DIR REPO_DIR: redacted copy (every file), then the raw copy is deleted
  [ -e "$1" ] || return 0
  local n; n=$(python3 scripts/redact_logs.py copy "$1" "$2" 2>&1) && rm -rf "$1"; echo "$(date -u +%T) saved $2 ($n)" >> "$LOG"; }
stop_stack() {  # stop_stack [REPO_DIR for the redacted agent log]
  [ -n "$STACK_PID" ] && kill -INT "$STACK_PID" 2>/dev/null
  for _ in $(seq 30); do [ -z "$(docker ps -q)" ] && break; sleep 2; done
  docker rm -f gurdy3_agent >/dev/null 2>&1; STACK_PID=""
  if [ -f /tmp/gurdy_web.log ]; then           # start_web_robot.sh's own log (prints the token): redacted copy + redact in place
    [ -n "${1:-}" ] && python3 scripts/redact_logs.py copy /tmp/gurdy_web.log "$1/agent.log" >/dev/null 2>&1
    python3 scripts/redact_logs.py inplace /tmp/gurdy_web.log >/dev/null 2>&1
  fi; }
start_stack() {   # start_stack RAW_DIR EXTRA_FETCH_FLAGS -> 0 when READY (stack.out + chat/ in RAW_DIR)
  mkdir -p "$1/chat"
  CHAT_LOG_DIR="$1/chat" GURDY_WEB_TOKEN="$TOKEN" FETCH=true MODEL=keyword MEMORY=false GUI=false FETCH_EXTRA="$2" \
    scripts/start_web_robot.sh > "$1/stack.out" 2>&1 &
  STACK_PID=$!
  for _ in $(seq 60); do
    grep -q "READY" "$1/stack.out" 2>/dev/null && { log "stack ready ($2)"; return 0; }
    kill -0 "$STACK_PID" 2>/dev/null || break; sleep 10
  done
  log "stack did NOT start: $(tail -2 "$1/stack.out" | tr '\n' ' ')"; stop_stack; return 1; }
prime_map() {     # turn the robot in place (gz set_pose; GT odometry keeps the map pose right) to face XY, let the detector confirm
  local yaw; yaw=$(python3 -c "import math;print(math.atan2($2, $1))")
  set_pose gurdy 0.0 0.0 0.115 "$yaw"; sleep 25; }
if [ "${DRY_RUN:-0}" = 1 ]; then source scripts/lib_fetch_dryrun.sh; fi
