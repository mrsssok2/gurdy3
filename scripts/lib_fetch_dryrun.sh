# DRY_RUN=1 stubs for scripts/lib_fetch_run.sh users (scripts/overnight_1127.sh): every Docker / Gazebo / HTTP call is
# replaced, so the control flow (block order, skip rules, deadline, redaction, report) is testable on the CPU.
# Knobs: DRY_START_FAIL="3,4,5" (these stack starts fail, counted from 1), DRY_FETCH_FAIL="can_2,..." (these fetch ids are
# 'grasp failed'), DRY_CLAIM_ONLY="mini_cone_1" (robot claims success but the object stays on the floor), DRY_WALK_FAIL="2,3,4"
# (walk runs, counted from 1, without walk.json). The fake logs contain the TOKEN on purpose: the saved copies must not.
DRY_STATE=$(mktemp -d)
DRY_N_START=0; DRY_N_WALK=0; DRY_RAW=""; DRY_ID=""          # held object: file $DRY_STATE/held (say runs in a subshell)
_in_list() { [[ ",$2," == *",$1,"* ]]; }
docker() { return 0; }                                  # never touches Docker
sleep() { :; }
docker_busy() { false; }
start_stack() {
  DRY_N_START=$((DRY_N_START + 1)); DRY_RAW=$1; mkdir -p "$1/chat"
  printf 'starting the simulation ...\naccess token: %s\n' "$TOKEN" > "$1/stack.out"
  printf '[detector_node] detector det_v3_s.onnx\nAuthorization: Bearer %s\n' "$TOKEN" > "$1/chat/launch.log"
  if _in_list "$DRY_N_START" "${DRY_START_FAIL:-}"; then
    echo "Nav2 never became ready (see /tmp/chat/launch.log)" >> "$1/stack.out"; log "stack did NOT start (dry)"; return 1
  fi
  echo "READY. Open in the laptop browser:  http://localhost:8080/#token=$TOKEN" >> "$1/stack.out"; log "stack ready ($2)"; return 0; }
stop_stack() {
  [ -n "${1:-}" ] && { mkdir -p "$1"; printf 'access token: %s\nGurdy: done\n' "$TOKEN" > "$DRY_STATE/agent.log"
                       python3 scripts/redact_logs.py copy "$DRY_STATE/agent.log" "$1/agent.log" >/dev/null 2>&1; }
  rm -f "$DRY_STATE/held"; true; }
set_pose() { [ "$1" = gurdy ] && return 0; echo "$2 $3 $5" > "$DRY_STATE/pose_$1"; DRY_ID=$1; }
gt_pose() {
  if [ "$1" = gurdy ]; then echo "0.0000 0.0000 0.1070 0.0000"; return; fi
  if [ "$(cat "$DRY_STATE/held" 2>/dev/null)" = "$1" ]; then echo "0.1500 0.0000 0.1700 0.0000"; return; fi
  read -r x y yaw < "$DRY_STATE/pose_$1" 2>/dev/null || { x=-1.3; y=0.3; yaw=0; }
  echo "$x $y 0.0000 $yaw"; }
api() { echo '{"ok": true}'; }
_cur_id() {   # the plan id of this fetch try = the raw dir name (fetch/<id>); a/b tries use the variant name
  basename "$DRY_RAW"; }
say() {
  case "$1" in
    "put it down") rm -f "$DRY_STATE/held"; echo "I put the blue can down."; return ;;
  esac
  local id; id=$(_cur_id)
  local model=$DRY_ID
  if _in_list "$id" "${DRY_FETCH_FAIL:-}"; then
    for k in 1 2 3; do echo "{\"attempt\": $k, \"outcome\": \"missed\", \"reason\": \"fingers fully closed: nothing between them\", \"fingers_lifted\": [0.0, 0.0], \"robot\": [0, 0, 0]}" >> "$DRY_RAW/chat/attempts.jsonl"; done
    echo "I couldn't get a grip on it after 3 attempts. It is still where it was."; return
  fi
  echo "{\"attempt\": 1, \"outcome\": \"success\", \"reason\": \"\", \"fingers_lifted\": [0.03, 0.03], \"robot\": [0, 0, 0]}" >> "$DRY_RAW/chat/attempts.jsonl"
  _in_list "$id" "${DRY_CLAIM_ONLY:-}" || echo "$model" > "$DRY_STATE/held"
  echo "I picked up the thing in room A and I'm holding it with the arm folded."; }
run_walk_step() {
  DRY_N_WALK=$((DRY_N_WALK + 1)); local d="$RAW/walk/$1"; mkdir -p "$d"
  echo "console GURDY_WEB_TOKEN=$TOKEN" > "$d/console.log"
  _in_list "$DRY_N_WALK" "${DRY_WALK_FAIL:-}" && return 0
  python3 -c "import json,random,sys; r=random.Random(int(sys.argv[2])); json.dump({'pass': True, 'forward': {'cmd': 0.1, 'err_pct': r.uniform(5, 15)}, 'turn': {'cmd': 0.3, 'err_pct': r.uniform(1, 4)}, **({} if sys.argv[3] == '05_walk_no_arm' else {'arm_max_dev_rad': 0.04})}, open(sys.argv[1], 'w'))" "$d/walk.json" "$DRY_N_WALK" "$1"; }
