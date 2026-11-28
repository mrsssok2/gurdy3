#!/usr/bin/env bash
# Overnight run 2026-11-28 (written that day; NOT run yet).
#
#   nohup scripts/overnight_1128.sh > /tmp/overnight_1128.out 2>&1 &         full run, commit + push at the end
#   same command again after a power cut / Ctrl+C: RESUMES (same --out): finished fetch tries are skipped, every cone stage
#   resumes or skips itself (render: rendered.txt, training: last.pt with a FIXED epoch count, eval: existing files, decision file)
#   DRY_RUN=1 scripts/overnight_1128.sh --out DIR                            CPU test of the control flow (stubs, no Docker)
#
# Blocks (one heavy job at a time):
#   a) CONE RETRAIN (scripts/p9b_v32_night.sh make -> render -> train -> eval -> report -> decide): v3.1 datasets copied to v3.2,
#      only the 350 scenes with a mini cone re-rendered with the fixed (coloured) cone mesh, det_v31_s fine-tuned 40 FIXED epochs,
#      evaluated (agreed acceptance + recall by distance + mini-cone recall on coloured vs white cones); the runtime detector is
#      switched to det_v32_s ONLY if acceptance passes AND the coloured-cone recall recovers (>= 0.9 x det_v31_s on white
#      cones); else det_v31_s stays (rollback line kept in detector_node.py).
#   b) FULL FETCH x 15 (5 classes x 3, placements docs/results/data/overnight_1127/plan.json, fresh stack per try, GT-graded),
#      with whatever detector block a left as the default; sighting log (FETCH=true default) + wrist-view diagnostics on.
#   c) summary docs/results/overnight_1128.md (scripts/overnight_1128_report.py) + commit + push.
# Rules: a block is skipped after 3 failed stack starts IN A ROW; a unit is not started when it could overrun the deadline
# (DEADLINE_H, default 10 h from THIS start); every saved log passes scripts/redact_logs.py; the token is random, memory only.
# Env: TRY_TIMEOUT_S (1800: the map-sized search limit is ~470 s), DEADLINE_H (10), EPOCHS (40), SKIP_CONE=1, SKIP_FETCH=1.
set -uo pipefail
cd "$(dirname "$0")/.."
source scripts/gpu_lock.sh && gpu_lock_acquire "gurdy3:$(basename "$0")" || exit 1   # shared GPU lock for the whole run (scripts/gpu_lock.sh)
OUT=""
while [ $# -gt 0 ]; do case "$1" in --out) OUT=$2; shift ;; *) echo "unknown argument $1 (--out DIR)"; exit 2 ;; esac; shift; done
OUT=${OUT:-docs/results/data/overnight_1128}
SUMMARY_MD=docs/results/overnight_1128.md; [ "${DRY_RUN:-0}" = 1 ] && SUMMARY_MD=$OUT/summary.md
PLAN=docs/results/data/overnight_1127/plan.json
TRY_TIMEOUT_S=${TRY_TIMEOUT_S:-1800}; DEADLINE_S=$(python3 -c "print(int(float('${DEADLINE_H:-10}') * 3600))")
FETCH_UNIT_S=$(( TRY_TIMEOUT_S + 600 ))                       # worst case per try: starts (~2 min each) + the try + stop
# a) worst cases (2x the measured rates): render 2 x 68 min, 40 epochs 2 x 33 min + 10 min load / export, eval 2 x 10 min
A_MAKE_S=600; A_RENDER_S=8200; A_TRAIN_S=4600; A_EVAL_S=1800; A_END_S=600
START_S=$(date +%s); STAMP=$(date -u +%Y%m%d_%H%M%S)
RAW_REL=.overnight_raw/$STAMP; RAW=$PWD/$RAW_REL
mkdir -p "$OUT" "$RAW"
LOG=$OUT/run.log; EVENTS=$OUT/events.jsonl
TOKEN=$(python3 -c 'import secrets;print(secrets.token_urlsafe(24))')
[ "${DRY_RUN:-0}" = 1 ] && [ -n "${DRY_TOKEN:-}" ] && TOKEN=$DRY_TOKEN
export GURDY_WEB_TOKEN=$TOKEN
source scripts/lib_fetch_run.sh
: "${CONE_OUTD:=docs/results/data/p9b_v32}"               # where p9b_v32_night.sh writes its json / tables (DRY: stub dir)

declare -F docker_busy >/dev/null || docker_busy() { [ -n "$(docker ps -q)" ]; }
declare -F elapsed >/dev/null || elapsed() { echo $(( $(date +%s) - START_S )); }
fits() { [ $(( $(elapsed) + $1 )) -le "$DEADLINE_S" ]; }
detector_default() { grep -oE '^DEFAULT_MODEL = "[^"]+"' ros2_ws/src/gurdy_perception/gurdy_perception/detector_node.py | cut -d'"' -f2; }
REPORTED=0
finish() {
  [ $REPORTED = 1 ] && return; REPORTED=1
  stop_stack
  python3 scripts/overnight_1128_report.py "$OUT" --out "$SUMMARY_MD" > /dev/null 2>>"$LOG" && log "summary: $SUMMARY_MD"
  log "end after $(( $(elapsed) / 60 )) min; docker ps: $(docker ps -q | wc -l) containers"
  if [ "${DRY_RUN:-0}" != 1 ]; then
    local extra=(); for f in docs/results/phase9b_v31_vs_v32.md docs/results/phase9b_recall_by_distance_det_v32_s.md \
        docs/results/phase9b_v32_eval_*.md docs/results/phase9b_v31_eval_synth_v32_*.md models/detector/det_v32_s.json "$CONE_OUTD" \
        ros2_ws/src/gurdy_perception/gurdy_perception/detector_node.py; do
      [ -e "$f" ] && { python3 scripts/redact_logs.py inplace "$f" >/dev/null 2>&1; extra+=("$f"); }; done
    git add -- "$OUT" "$SUMMARY_MD" "${extra[@]}" && git commit -q -m "Overnight 2026-11-28: mini-cone retrain (v3.2, coloured cones; runtime detector: $(detector_default)) + full fetch x15 GT-graded (results + summary)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" -- "$OUT" "$SUMMARY_MD" "${extra[@]}" && git pull --no-rebase -q && git push -q \
      && log "committed + pushed" || log "WARNING: commit / push failed (results are on disk)"
  fi
  find "$RAW" -type d -empty -delete 2>/dev/null; rmdir "$(dirname "$RAW")" 2>/dev/null; true; }
trap 'log "interrupted"; finish; exit 130' INT TERM
trap finish EXIT

record() {    # record FILE KEY=VALUE ...  (values redacted) -> one JSON line
  python3 - "$@" <<'EOF'
import json, os, sys
sys.path.insert(0, "scripts")
from redact_logs import redact_text
f, kv = sys.argv[1], sys.argv[2:]
d = {}
for x in kv:
    k, v = x.split("=", 1)
    d[k] = int(v) if k in ("try", "wall_s") and v.lstrip("-").isdigit() else (json.loads(v) if k == "attempts" else redact_text(v)[0])
os.makedirs(os.path.dirname(f) or ".", exist_ok=True)
open(f, "a").write(json.dumps(d) + "\n")
EOF
}

fetch_try() { # fetch_try RAWDIR REPODIR MODEL X Y YAW COMMAND -> sets TRY_FIELDS (array)   [stack must be up]
  local raw=$1 repo=$2 model=$3 x=$4 y=$5 yaw=$6 cmd=$7 before robot0 t0 reply at robot1 at5 robot5 a0 a1
  set_pose "$model" "$x" "$y" 0.0005 "$yaw" || log "WARNING: set_pose $model failed"; sleep 5
  before=$(gt_pose "$model"); robot0=$(gt_pose gurdy); t0=$(date +%s)
  a0=$(cat "$raw/chat/attempts.jsonl" 2>/dev/null | wc -l)
  reply=$(say "$cmd" "$TRY_TIMEOUT_S")
  if [ -z "$reply" ]; then api /api/estop '{}' >/dev/null; sleep 10; reply="TIMEOUT after ${TRY_TIMEOUT_S}s (e-stop)"; api /api/estop/release '{}' >/dev/null; fi
  local wall=$(( $(date +%s) - t0 ))
  at=$(gt_pose "$model"); robot1=$(gt_pose gurdy); sleep 5; at5=$(gt_pose "$model"); robot5=$(gt_pose gurdy)
  a1=$(cat "$raw/chat/attempts.jsonl" 2>/dev/null | wc -l)
  TRY_FIELDS=("wall_s=$wall" "reply=$reply" "can_before=$before" "robot_before=$robot0" "can_after=$at" "robot_after=$robot1"
              "can_5s=$at5" "robot_5s=$robot5" "attempts=[$a0, $a1]")
  log "  -> ${wall}s: $reply"; }

# ---------------------------------------------------------------- preflight
if docker_busy; then echo "docker ps not empty: not starting (one heavy workload at a time)"; docker ps --format '  {{.Names}}'; trap - EXIT; exit 1; fi
[ -f "$PLAN" ] || { echo "no $PLAN"; trap - EXIT; exit 1; }
RESUMED=0; [ -s "$EVENTS" ] && RESUMED=1
log "overnight_1128 start (dry=${DRY_RUN:-0}, resumed=$RESUMED, deadline $(( DEADLINE_S / 3600 )) h, out $OUT, detector $(detector_default))"
event run_start resumed="$RESUMED"
log "estimate a) cone retrain: make ~5 + render ~68 + train 40 epochs ~33 (+10 load / export) + eval ~10 min = ~2 h, worst $(( (A_MAKE_S + A_RENDER_S + A_TRAIN_S + A_EVAL_S) / 60 )) min"
log "estimate b) fetch: 15 tries x ~10 min = ~150 min, worst $(( 15 * FETCH_UNIT_S / 60 )) min; total ~4.5 h, hard limit ${DEADLINE_H:-10} h"

# ---------------------------------------------------------------- a) cone retrain (v3.2)
declare -F run_cone_stage >/dev/null || run_cone_stage() {   # run_cone_stage STAGE -> exit code of scripts/p9b_v32_night.sh STAGE
  EPOCHS=${EPOCHS:-40} APPLY=1 scripts/p9b_v32_night.sh "$1" > "$RAW/cone/$1.log" 2>&1; }
cone_stage() {   # cone_stage STAGE -> 0 ok; timing in $OUT/cone/stages.jsonl, log saved (redacted)
  local t0 rc; t0=$(date +%s); mkdir -p "$RAW/cone"; log "a) $1"
  run_cone_stage "$1"; rc=$?
  record "$OUT/cone/stages.jsonl" stage="$1" rc="$rc" wall_s="$(( $(date +%s) - t0 ))"
  [ -f "$RAW/cone/$1.log" ] && save_logs "$RAW/cone/$1.log" "$OUT/cone/logs/$1.log"
  log "a) $1 rc=$rc after $(( ($(date +%s) - t0) / 60 )) min"; return $rc; }
if [ "${SKIP_CONE:-0}" != 1 ]; then
  event block_start block=cone
  DEFAULT_BEFORE=$(detector_default)
  if [ -f "$CONE_OUTD/decision.json" ]; then
    log "a) already decided (resume): $(cat "$CONE_OUTD/decision.json")"
  elif ! fits $(( A_MAKE_S + A_RENDER_S + A_TRAIN_S + A_EVAL_S + A_END_S )); then
    event block_skip block=cone reason="deadline" at=make; log "a) skipped: does not fit before the deadline"
  else
    ok=1
    for st in make render train eval report decide; do
      case $st in render) need=$(( A_RENDER_S + A_TRAIN_S + A_EVAL_S )) ;; train) need=$(( A_TRAIN_S + A_EVAL_S )) ;; *) need=$A_EVAL_S ;; esac
      if ! fits $(( need + A_END_S )); then event block_skip block=cone reason="deadline" at=$st; ok=0; break; fi
      cone_stage $st || { event block_skip block=cone reason="stage $st failed" at=$st; ok=0; break; }
    done
  fi
  mkdir -p "$OUT/cone"
  for f in decision.json verdict.json cone_new.json cone_ref.json cone_v31_coloured.json det_v32_s_results.csv; do
    [ -f "$CONE_OUTD/$f" ] && cp "$CONE_OUTD/$f" "$OUT/cone/$f"; done
  [ -f docs/results/phase9b_v31_vs_v32.md ] && [ "${DRY_RUN:-0}" != 1 ] && cp docs/results/phase9b_v31_vs_v32.md "$OUT/cone/"
  record "$OUT/cone/runtime_detector.jsonl" default_before="$DEFAULT_BEFORE" default_model="$(detector_default)"
  log "a) runtime detector: $DEFAULT_BEFORE -> $(detector_default)"
  event block_end block=cone
fi

# ---------------------------------------------------------------- b) full fetch x 15 (resume: tries already run are skipped)
if [ "${SKIP_FETCH:-0}" != 1 ]; then
  event block_start block=fetch
  FAILS=0
  mapfile -t IDS < <(python3 -c "import json;print('\n'.join(p['id'] for p in json.load(open('$PLAN'))['placements']))")
  for id in "${IDS[@]}"; do
    if grep -qs "\"id\": \"$id\", .*\"status\": \"ran\"" "$OUT/fetch/tries.jsonl"; then log "b) $id: done before (resume), skipped"; continue; fi
    if [ $FAILS -ge 3 ]; then event block_skip block=fetch reason="3 failed stack starts in a row" at="$id"; log "b) skipped from $id: 3 failed starts in a row"; break; fi
    if ! fits $(( FETCH_UNIT_S + A_END_S )); then event block_skip block=fetch reason="deadline" at="$id"; log "b) skipped from $id: deadline"; break; fi
    read -r cls model x y yaw cmd dist < <(python3 -c "
import json; p=next(p for p in json.load(open('$PLAN'))['placements'] if p['id']=='$id')
print(p['class'], p['model'], p['x'], p['y'], p['yaw'], p['command'].replace(' ', '_'), p['dist_m'])")
    cmd=${cmd//_/ }
    log "b) $id: $cmd at ($x, $y), $dist m"
    raw=$RAW/fetch/$id; repo=$OUT/fetch/$id
    rm -rf "$repo"                                                # a try cut off by a power loss is redone from scratch
    if ! start_stack "$raw" "--fetch-gt log --fetch-log /tmp/chat/attempts.jsonl"; then
      FAILS=$((FAILS + 1)); event stack_fail block=fetch at="$id" n="$FAILS"
      stop_stack "$repo"; save_logs "$raw" "$repo"
      record "$OUT/fetch/tries.jsonl" id="$id" class="$cls" model="$model" command="$cmd" dist_m="$dist" status=start_failed; continue
    fi
    FAILS=0
    fetch_try "$raw" "$repo" "$model" "$x" "$y" "$yaw" "$cmd"
    stop_stack "$repo"; save_logs "$raw" "$repo"
    record "$OUT/fetch/tries.jsonl" id="$id" class="$cls" model="$model" command="$cmd" dist_m="$dist" \
      detector="$(detector_default)" status=ran "${TRY_FIELDS[@]}"
  done
  event block_end block=fetch
fi
log "all blocks done"
