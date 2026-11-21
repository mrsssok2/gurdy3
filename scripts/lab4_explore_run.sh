#!/bin/bash
# Phase 3 autonomous exploration (headless): NO map, slam_toolbox + Nav2 + frontier explorer in lab4 (Gurdy 3.0, world:=lab4).
# Usage (sim container, nothing else running): scripts/phase3_explore_run.sh NAME [controller=policy] [odom=ground_truth] [model]
# Output: docs/results/data/phase3/NAME/{explore_log.json,timeseries.csv,monitor.json,snaps/,rtf.txt,...}, maps/NAME.*
set -eo pipefail
NAME=${1:?}; CTL=${2:-policy}; ODOM=${3:-ground_truth}; MODEL=${4:-}
MAXWALL=${MAXWALL:-3600}
cd ~/gurdy2; set +u; source ros2_ws/install/setup.bash
export PYTHONPATH=$HOME/gurdy2/gurdy_core:$HOME/gurdy2/sim_mujoco:${PYTHONPATH:-}
export WORLD_BOUNDS=-2,6,-2,6
OUT=docs/results/data/lab4/$NAME; rm -rf "$OUT"; mkdir -p "$OUT"
DONE="$OUT/explore.done"
source scripts/lib_nav_start.sh
start_nav_stack "$OUT" gui:=false world:=lab4 nav:=slam navigation:=true controller:=$CTL odom_source:=$ODOM ${MODEL:+model:=$MODEL} \
  || { echo "Nav2 never became ready" | tee "$OUT/FAILED.txt"; stop_nav_stack; exit 1; }
(python3 scripts/measure_rtf.py > "$OUT/rtf.txt" 2>&1 &)
python3 scripts/explore_monitor.py "$OUT" "$DONE" > "$OUT/monitor.log" 2>&1 &
MON=$!
ros2 run gurdy_navigation frontier_explorer_node --ros-args -p log_path:="$OUT/explore_log.json" -p done_path:="$DONE" \
  > "$OUT/explorer.log" 2>&1 &
EXP=$!
for _ in $(seq $((MAXWALL / 5))); do [ -f "$DONE" ] && break; sleep 5; done
[ -f "$DONE" ] || echo "WALL TIMEOUT ${MAXWALL}s" | tee "$OUT/wall_timeout.txt"
sleep 8
scripts/map_tools.sh save "$NAME" > "$OUT/map_save.log" 2>&1 || true
scripts/map_tools.sh serialize "$NAME" > "$OUT/serialize.log" 2>&1 || true
ps -eo rss,cmd --sort=-rss | head -8 > "$OUT/ps_top.txt"
grep -iE "warn|error" "$OUT/launch.log" | grep -v "gz_ros_control\|controller_manager\|Utils.cc" | cut -c1-220 | sort | uniq -c | sort -rn | head -20 > "$OUT/warnings.txt" || true
kill -TERM $EXP $MON 2>/dev/null || true
stop_nav_stack
ls -la maps/$NAME.* 2>/dev/null || true
