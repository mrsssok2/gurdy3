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
# block d (YOLO v3.1): fake datasets / runs / docs under $DRY_STATE; DRY_YOLO_FAIL=render|train|eval makes that stage fail
YOLO_DOCS=$DRY_STATE/docs; YOLO_DATA=$DRY_STATE/datasets; YOLO_RUNS=$DRY_STATE/runs
run_yolo_stage() {
  mkdir -p "$YOLO_DOCS" "$YOLO_DATA" "$YOLO_RUNS"; echo "$1 TRAIN_TIME_H=${TRAIN_TIME_H:-0}" >> "$DRY_STATE/yolo_calls"
  [ "${DRY_YOLO_FAIL:-}" = "$1" ] && { [ "$1" = render ] && return 2; return 1; }
  case $1 in
    render) for d in synth_v31_body synth_v31_wrist; do mkdir -p "$YOLO_DATA/$d"; echo "names: {}" > "$YOLO_DATA/$d/dataset.yaml"; done ;;
    train)  mkdir -p "$YOLO_RUNS/det_v31_s"; { echo "epoch,time,a,b,c,d,e,metrics/mAP50(B),metrics/mAP50-95(B)"
              for e in $(seq 1 90); do echo "$e,$((e * 73)),0,0,0,0,0,0.96$((e % 10)),0.90"; done; } > "$YOLO_RUNS/det_v31_s/results.csv" ;;
    eval)   for m in det_v3_s det_v31_s; do echo "recall table $m (Bearer $TOKEN)" > "$YOLO_DOCS/phase9b_recall_by_distance_$m.md"; done ;;
    report) printf '| check | target | det_v2_s | det_v3_s | det_v31_s |\n|---|---|---|---|---|\n| **acceptance (old + v3 sets)** | | | **PASS** | **PASS** |\n' > "$YOLO_DOCS/phase9b_v3_vs_v31.md" ;;
  esac; }
# fake clock for the deadline rule: DRY_RENDER_ADD_S seconds pass during the (stub) render stage
elapsed() { echo $(( $(date +%s) - START_S + $(cat "$DRY_STATE/clock" 2>/dev/null || echo 0) )); }
_real_run_yolo_stage=$(declare -f run_yolo_stage)
eval "${_real_run_yolo_stage/run_yolo_stage/_dry_yolo_stage}"
run_yolo_stage() { [ "$1" = render ] && echo "${DRY_RENDER_ADD_S:-0}" > "$DRY_STATE/clock"; _dry_yolo_stage "$@"; }
# overnight_1128 block a (cone retrain, scripts/p9b_v32_night.sh stages): fake outputs under $DRY_STATE/p9b_v32 (the real
# detector_node.py is NEVER edited in DRY_RUN). DRY_CONE_FAIL=make|render|train|eval|report|decide makes that stage fail;
# DRY_CONE_DECISION=switch|keep (default keep); DRY_CONE_CALLS counts stage calls (resume tests).
CONE_OUTD=${CONE_OUTD:-$DRY_STATE/p9b_v32}            # tests may keep it across two runs (resume)
run_cone_stage() {
  mkdir -p "$CONE_OUTD"; echo "$1" >> "$DRY_STATE/cone_calls"; echo "fake $1 log Bearer $TOKEN" > "$RAW/cone/$1.log"
  [ "${DRY_CONE_FAIL:-}" = "$1" ] && { [ "$1" = render ] && return 2; return 1; }
  case $1 in
    train)  { echo "epoch,time"; for e in $(seq 1 40); do echo "$e,$((e * 50))"; done; } > "$CONE_OUTD/det_v32_s_results.csv" ;;
    eval)   echo '{"recall_all": 0.83, "n": 300}' > "$CONE_OUTD/cone_new.json"; echo '{"recall_all": 0.86, "n": 300}' > "$CONE_OUTD/cone_ref.json"
            echo '{"recall_all": 0.10, "n": 300}' > "$CONE_OUTD/cone_v31_coloured.json" ;;
    report) echo '{"det_v31_s": true, "det_v32_s": true}' > "$CONE_OUTD/verdict.json" ;;
    decide) if [ "${DRY_CONE_DECISION:-keep}" = switch ]; then m=det_v32_s; s=true; w='[]'; else m=det_v31_s; s=false; w='["coloured-cone recall 0.5 < 0.77"]'; fi
            echo "{\"switch\": $s, \"model\": \"$m\", \"acceptance\": true, \"cone_recall_coloured\": 0.83, \"cone_recall_white_ref\": 0.86, \"cone_needed\": 0.774, \"why_not\": $w}" > "$CONE_OUTD/decision.json" ;;
  esac; return 0; }
