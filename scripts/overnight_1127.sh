#!/usr/bin/env bash
# Overnight reliability run 2026-11-27 ("Plan A"; written while the GPU belonged to another job: NOT run yet).
#
#   nohup scripts/overnight_1127.sh > /tmp/overnight_1127.out 2>&1 &          full run (blocks a, b, c), commit + push at the end
#   scripts/overnight_1127.sh --smoke                                         1 fetch try only (~10 min), no commit
#   DRY_RUN=1 scripts/overnight_1127.sh [--smoke] --out DIR                   CPU test of the control flow (stubs, no Docker)
#
# Blocks, in this order (one heavy job at a time; each fetch try and each walk = its own fresh stack / container):
#   a) FULL FETCH (fetch-retest variant C path: search + Nav2 approach + detector / servo pose), 5 small classes x 3 tries,
#      targets placed 1-2.5 m from the spawn point (docs/results/data/overnight_1127/plan.json, scripts/overnight_1127_plan.py:
#      same free-space test as the fetch approach, off doorways), FRESH stack per try (so the search runs: nothing in the map).
#      Graded from Gazebo ground truth, not from the reply: lifted (can z), between the fingers (sim finger joints), still held
#      5 s after the reply (z + position at the folded grasp centre).
#   b) Fetch-retest variants A (teleport to the grasp_test pose + detector pose) and B (Nav2 + ground-truth pose), 3 tries each
#      on the blue can at its GT spot (diagnostics: only matters if a) fails).
#   c) Walking variance with the arm: 90 s at 0.10 m/s + 90 s at 0.3 rad/s, 10 runs per config (no arm / arm folded / + 80 g),
#      interleaved (as scripts/overnight_grasp_walk.sh part B); report scripts/walk_variance_report.py.
# Rules: a block is skipped (rest of it) after 3 failed stack starts / walk runs IN A ROW; a unit is not started when it could
# overrun the deadline (DEADLINE_H, default 10 h from the start); every saved log passes scripts/redact_logs.py (raw logs live in
# the git-ignored .overnight_raw/ and are deleted after the redacted copy); the token is random, in memory only.
# Output: docs/results/data/overnight_1127/ (run.log, events.jsonl, fetch/, ab/, walk/) + morning summary docs/results/overnight_1127.md
# (scripts/overnight_1127_report.py). Env: TRY_TIMEOUT_S (900), DEADLINE_H (10), RUNS (10), WALK_S (90), SMOKE_ID (can_1).
set -uo pipefail
cd "$(dirname "$0")/.."
SMOKE=0; OUT=""
while [ $# -gt 0 ]; do case "$1" in --smoke) SMOKE=1 ;; --out) OUT=$2; shift ;; *) echo "unknown argument $1 (--smoke | --out DIR)"; exit 2 ;; esac; shift; done
[ -z "$OUT" ] && { [ $SMOKE = 1 ] && OUT=docs/results/data/overnight_1127_smoke || OUT=docs/results/data/overnight_1127; }
SUMMARY_MD=docs/results/overnight_1127.md; [ $SMOKE = 1 ] && SUMMARY_MD=docs/results/overnight_1127_smoke.md
[ "${DRY_RUN:-0}" = 1 ] && SUMMARY_MD=$OUT/summary.md
PLAN=docs/results/data/overnight_1127/plan.json
TRY_TIMEOUT_S=${TRY_TIMEOUT_S:-900}; DEADLINE_S=$(python3 -c "print(int(float('${DEADLINE_H:-10}') * 3600))"); RUNS=${RUNS:-10}
FETCH_UNIT_S=1500; AB_UNIT_S=1500; WALK_UNIT_S=660        # worst case per unit (start 3 x ~2 min + try timeout / walk 600 s cap)
START_S=$(date +%s); STAMP=$(date -u +%Y%m%d_%H%M%S)
RAW_REL=.overnight_raw/$STAMP; RAW=$PWD/$RAW_REL           # git-ignored; inside the repo because the walk container mounts it
mkdir -p "$OUT" "$RAW"
LOG=$OUT/run.log; EVENTS=$OUT/events.jsonl
TOKEN=$(python3 -c 'import secrets;print(secrets.token_urlsafe(24))')
[ "${DRY_RUN:-0}" = 1 ] && [ -n "${DRY_TOKEN:-}" ] && TOKEN=$DRY_TOKEN       # tests: a known value to search the saved files for
export GURDY_WEB_TOKEN=$TOKEN
source scripts/lib_fetch_run.sh

declare -F docker_busy >/dev/null || docker_busy() { [ -n "$(docker ps -q)" ]; }     # (DRY_RUN stubs define their own)
elapsed() { echo $(( $(date +%s) - START_S )); }
fits() { [ $(( $(elapsed) + $1 )) -le "$DEADLINE_S" ]; }   # fits UNIT_S: would this unit end before the deadline?
REPORTED=0
finish() {
  [ $REPORTED = 1 ] && return; REPORTED=1
  stop_stack; docker rm -f gurdy3_day0927 >/dev/null 2>&1
  [ -d "$RAW/walk" ] && save_logs "$RAW/walk" "$OUT/walk/raw_rest"
  python3 scripts/overnight_1127_report.py "$OUT" --out "$SUMMARY_MD" > /dev/null 2>>"$LOG" && log "summary: $SUMMARY_MD"
  log "end after $(( $(elapsed) / 60 )) min; docker ps: $(docker ps -q | wc -l) containers"
  if [ $SMOKE = 0 ] && [ "${DRY_RUN:-0}" != 1 ]; then
    git add -- "$OUT" "$SUMMARY_MD" && git commit -q -m "Overnight 2026-11-27 reliability run: full fetch x15 (GT graded), variants A/B, walking variance with the arm (results + morning summary)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" -- "$OUT" "$SUMMARY_MD" && git pull --no-rebase -q && git push -q \
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

# ---------------------------------------------------------------- preflight + estimates
if docker_busy; then echo "docker ps not empty: not starting (one heavy workload at a time)"; docker ps --format '  {{.Names}}'; trap - EXIT; exit 1; fi
[ -f "$PLAN" ] || { echo "no $PLAN (python3 scripts/overnight_1127_plan.py)"; trap - EXIT; exit 1; }
N_FETCH=$(python3 -c "import json;print(len(json.load(open('$PLAN'))['placements']))"); [ $SMOKE = 1 ] && N_FETCH=1
log "overnight_1127 start (smoke=$SMOKE, dry=${DRY_RUN:-0}, deadline $(( DEADLINE_S / 3600 )) h, out $OUT)"
log "estimate a) fetch: $N_FETCH tries x ~6 min (C1 retest: start ~1 min + 253 s try + stop) = ~$(( N_FETCH * 6 )) min, worst $(( N_FETCH * FETCH_UNIT_S / 60 )) min"
[ $SMOKE = 0 ] && log "estimate b) variants A + B: 2 stacks x ~3 min + 6 tries x ~6 min = ~45 min, worst $(( 2 * AB_UNIT_S * 3 / 60 )) min"
[ $SMOKE = 0 ] && log "estimate c) walking: $(( RUNS * 3 )) runs x ~5 min (start ~1.5 min + 2 x 90 s sim at RTF ~0.8) = ~$(( RUNS * 3 * 5 )) min, worst $(( RUNS * 3 * WALK_UNIT_S / 60 )) min"
[ $SMOKE = 0 ] && log "estimate total ~$(( N_FETCH * 6 + 45 + RUNS * 15 )) min; hard limit ${DEADLINE_H:-10} h (units that could overrun are skipped)"

# ---------------------------------------------------------------- a) full fetch x 15
event block_start block=fetch
FAILS=0
mapfile -t IDS < <(python3 -c "import json;print('\n'.join(p['id'] for p in json.load(open('$PLAN'))['placements']))")
[ $SMOKE = 1 ] && IDS=("${SMOKE_ID:-can_1}")
for id in "${IDS[@]}"; do
  if [ $FAILS -ge 3 ]; then event block_skip block=fetch reason="3 failed stack starts in a row" at="$id"; log "a) skipped from $id: 3 failed starts in a row"; break; fi
  if ! fits $FETCH_UNIT_S; then event block_skip block=fetch reason="deadline" at="$id"; log "a) skipped from $id: deadline"; break; fi
  read -r cls model x y yaw cmd dist < <(python3 -c "
import json; p=next(p for p in json.load(open('$PLAN'))['placements'] if p['id']=='$id')
print(p['class'], p['model'], p['x'], p['y'], p['yaw'], p['command'].replace(' ', '_'), p['dist_m'])")
  cmd=${cmd//_/ }
  log "a) $id: $cmd at ($x, $y), $dist m"
  raw=$RAW/fetch/$id; repo=$OUT/fetch/$id
  if ! start_stack "$raw" "--fetch-gt log --fetch-log /tmp/chat/attempts.jsonl"; then
    FAILS=$((FAILS + 1)); event stack_fail block=fetch at="$id" n="$FAILS"
    stop_stack "$repo"; save_logs "$raw" "$repo"
    record "$OUT/fetch/tries.jsonl" id="$id" class="$cls" model="$model" command="$cmd" dist_m="$dist" status=start_failed; continue
  fi
  FAILS=0
  fetch_try "$raw" "$repo" "$model" "$x" "$y" "$yaw" "$cmd"
  stop_stack "$repo"; save_logs "$raw" "$repo"
  record "$OUT/fetch/tries.jsonl" id="$id" class="$cls" model="$model" command="$cmd" dist_m="$dist" status=ran "${TRY_FIELDS[@]}"
done
event block_end block=fetch

# ---------------------------------------------------------------- b) variants A and B x 3
if [ $SMOKE = 0 ]; then
  event block_start block=ab
  FAILS=0
  for V in A B; do
    if [ $FAILS -ge 3 ]; then event block_skip block=ab reason="3 failed stack starts in a row" at="$V"; break; fi
    if ! fits $(( 3 * AB_UNIT_S )); then event block_skip block=ab reason="deadline" at="$V"; log "b) skipped from $V: deadline"; break; fi
    case $V in A) FL="--fetch-approach teleport --fetch-gt log --fetch-log /tmp/chat/attempts.jsonl" ;;
               B) FL="--fetch-gt use --fetch-log /tmp/chat/attempts.jsonl" ;; esac
    raw=$RAW/ab/$V; repo=$OUT/ab/$V
    started=0
    for k in 1 2 3; do start_stack "$raw" "$FL" && { started=1; break; }; FAILS=$((FAILS + 1)); event stack_fail block=ab at="$V" n="$FAILS"
                       [ $FAILS -ge 3 ] && break; done
    if [ $started = 0 ]; then stop_stack "$repo"; save_logs "$raw" "$repo"; continue; fi
    FAILS=0
    prime_map -1.30 0.30
    for k in 1 2 3; do
      log "b) $V try $k"
      fetch_try "$raw" "$repo" f_blue_can -1.30 0.30 0.0 "pick up the blue can"
      record "$OUT/ab/$V/tries.jsonl" variant="$V" try="$k" class=can status=ran "${TRY_FIELDS[@]}"
      case "${TRY_FIELDS[1]}" in *"I picked up"*) say "put it down" 600 >/dev/null ;; esac     # free the gripper
    done
    stop_stack "$repo"; save_logs "$raw" "$repo"
  done
  event block_end block=ab
fi

# ---------------------------------------------------------------- c) walking variance with the arm, 10 runs per config
declare -F run_walk_step >/dev/null || run_walk_step() { DAY0927_OUT="$RAW_REL/walk" WALK_S="${WALK_S:-90}" scripts/day_0927_checks.sh "$1" 2>&1 | cut -c1-200 | tee -a "$LOG" >/dev/null; }
if [ $SMOKE = 0 ]; then
  event block_start block=walk
  FAILS=0; stop_walk=0
  for r in $(seq 1 "$RUNS"); do
    for s in 05_walk_no_arm 06_walk_arm_folded 07_walk_arm_object; do
      if [ $FAILS -ge 3 ]; then event block_skip block=walk reason="3 failed walk runs in a row" at="${s}_r$r"; stop_walk=1; break; fi
      if ! fits $WALK_UNIT_S; then event block_skip block=walk reason="deadline" at="${s}_r$r"; log "c) skipped from ${s}_r$r: deadline"; stop_walk=1; break; fi
      log "c) $s run $r"
      run_walk_step "$s"
      if [ -f "$RAW/walk/$s/walk.json" ]; then FAILS=0; else FAILS=$((FAILS + 1)); event walk_fail at="${s}_r$r" n="$FAILS"; fi
      save_logs "$RAW/walk/$s" "$OUT/walk/${s}_r$r"
    done
    [ $stop_walk = 1 ] && break
  done
  python3 scripts/walk_variance_report.py "$OUT/walk" --out "$OUT/walk/report.md" >/dev/null 2>>"$LOG" || log "walk report failed"
  event block_end block=walk
fi
log "all blocks done"
