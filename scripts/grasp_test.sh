#!/usr/bin/env bash
# Phase 9 first grasp test in Gazebo lab4_fetch (NOT RUN YET). Style of scripts/day_0927_checks.sh.
#
#   scripts/grasp_test.sh [RUN_NAME] [-- runner args]       e.g.  scripts/grasp_test.sh first -- --classes can small_box --tries 1
#   MOVEIT=off scripts/grasp_test.sh ...                    no move_group (direct joint goals only; default: start move_group)
#   scripts/grasp_test.sh stop                               remove the container
#
# Host side: checks `docker ps` is empty (rule 8), runs ONE container (gurdy3_grasp) with a hard wall timeout
# (GRASP_TIMEOUT, default 3600 s), headless sim: world lab4_fetch, arm:=true, controller:=tripod (stands still; the
# servo's body steps are teleports by default), then move_group (unless MOVEIT=off), then scripts/grasp_test.py.
# Output docs/results/data/grasp_test/<run>/: console.log, launch.log, move_group.log, controllers.txt, tries.jsonl,
# summary.json; one line per class 'PASS grasp_<class> ...' / 'FAIL grasp_<class> ...' (+ setup lines) appended to
# docs/results/data/grasp_test/summary.txt. Prerequisite: day_0927 steps 01_build, 04_arm_spawn, 09_wrist_camera PASS.
set -uo pipefail
cd "$(dirname "$0")/.."
NAME=gurdy3_grasp
[ "${1:-}" = stop ] && { docker rm -f "$NAME"; exit 0; }
RUN=${1:-$(date +%Y%m%d_%H%M%S)}; shift || true
[ "${1:-}" = "--" ] && shift
RARGS="$*"
BASE=docs/results/data/grasp_test
OUT=$BASE/$RUN
SUMMARY=$BASE/summary.txt
MOVEIT=${MOVEIT:-auto}
mkdir -p "$OUT"

result() { echo "$1 $(date '+%F %T') [$RUN] $2" | tee -a "$SUMMARY"; }
cleanup() { docker rm -f "$NAME" >/dev/null 2>&1 || true; }
trap 'echo "interrupted: removing $NAME"; cleanup; exit 130' INT TERM

if docker ps --format '{{.Names}}' | grep -q .; then
  echo "other containers are running:"; docker ps --format '  {{.Names}}'
  result FAIL "grasp_setup not started: docker ps not empty"; exit 1
fi
git rev-parse HEAD > "$OUT/commit.txt" 2>/dev/null || true

SERVICE=sim; [ "$MOVEIT" != off ] && SERVICE=sim_moveit     # MoveIt lives only in gurdy3/ros:jazzy-moveit
timeout --signal=INT "${GRASP_TIMEOUT:-3600}" docker compose --profile moveit run --rm -T --name "$NAME" $SERVICE bash -c "
  set -o pipefail; cd ~/gurdy2; set +u; source ros2_ws/install/setup.bash
  export PYTHONPATH=\$HOME/gurdy2/gurdy_core:\$HOME/gurdy2:\$HOME/gurdy2/scripts:\${PYTHONPATH:-}
  D=$OUT
  (ros2 launch gurdy_bringup sim.launch.py gui:=false world:=lab4_fetch arm:=true controller:=tripod > \$D/launch.log 2>&1 &)
  for i in \$(seq 1 90); do timeout 3 ros2 topic echo --once /joint_states >/dev/null 2>&1 && break; sleep 2; done
  sleep 10
  ros2 control list_controllers > \$D/controllers.txt 2>&1
  if ! grep -q 'arm_controller.*active' \$D/controllers.txt || ! grep -q 'gripper_controller.*active' \$D/controllers.txt; then
    echo 'FAIL grasp_setup arm_controller / gripper_controller not active (controllers.txt)'; exit 1
  fi
  if [ '$MOVEIT' != off ]; then
    (ros2 launch gurdy_arm_moveit move_group.launch.py arm:=true > \$D/move_group.log 2>&1 &)
    sleep 25
  fi
  (python3 scripts/measure_rtf.py > \$D/rtf.txt 2>&1 &)
  python3 scripts/grasp_test.py \$D --moveit $([ "$MOVEIT" = off ] && echo off || echo auto) $RARGS
  ps -eo rss,pcpu,cmd --sort=-rss | head -12 > \$D/ps_top.txt
" > "$OUT/console.log" 2>&1
rc=$?
cleanup
[ $rc -eq 124 ] && echo "TIMEOUT after ${GRASP_TIMEOUT:-3600}s" >> "$OUT/console.log"

lines=$(grep -E "^(PASS|FAIL) grasp_" "$OUT/console.log" || true)
if [ -z "$lines" ]; then
  result FAIL "grasp_setup no result lines (rc $rc; see $OUT/console.log, launch.log)"; exit 1
fi
while IFS= read -r l; do result "${l%% *}" "${l#* }"; done <<< "$lines"
grep -q "^FAIL" <<< "$lines" && exit 1
exit 0
