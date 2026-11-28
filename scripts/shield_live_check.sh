#!/bin/bash
# Phase 8b live check of the LiDAR safety shield in Gazebo (lab4, localization on lab4_map, policy, GT odom, headless). One run each.
# Usage (host): scripts/gpu_lock.sh run gurdy3 -- docker compose run --rm -T --name gurdy3_shield sim bash -c 'cd ~/gurdy2 && scripts/shield_live_check.sh off|on'
#   off: Nav2 goals home -> room D centre (2 doorways) -> room C (1 doorway), no shield (baseline pass times)
#   on : the same goals with shield:=true, then (direct /cmd_vel, the teleop path) diagonal into room C's NE corner + turning in place there,
#        Nav2 to (0, 0.5) facing +x, full speed at the A-B divider, then the stale-scan test (scan producer killed 4 s).
# Output: docs/results/data/phase8b_live/<off|on>/
set -eo pipefail
MODE=${1:?off|on}
cd ~/gurdy2; set +u; source ros2_ws/install/setup.bash
export PYTHONPATH=$HOME/gurdy2/gurdy_core:$HOME/gurdy2/sim_mujoco:$HOME/gurdy2/scripts:${PYTHONPATH:-}
OUT=docs/results/data/phase8b_live/$MODE; rm -rf "$OUT"; mkdir -p "$OUT"
SH=false; [ "$MODE" = on ] && SH=true
source scripts/lib_nav_start.sh
start_nav_stack "$OUT" gui:=false world:=lab4 nav:=localization map:=lab4_map controller:=policy odom_source:=ground_truth shield:=$SH \
  || { echo "Nav2 never became ready" | tee "$OUT/FAILED.txt"; stop_nav_stack; exit 1; }
sleep 5
(python3 scripts/measure_rtf.py > "$OUT/rtf.txt" 2>&1 &)
(timeout 3000 ros2 topic echo --csv /shield/status > "$OUT/shield_status.csv" 2>/dev/null &) || true
python3 scripts/nav_eval.py "$OUT/goals.json" "4.0,4.0,1.57" "-0.1,4.5,0.0" | tee "$OUT/goals.txt"
if [ "$MODE" = on ]; then
  python3 scripts/shield_probe.py "$OUT/corner_turn.json" corner_turn | tee "$OUT/corner_turn.txt"
  python3 scripts/nav_eval.py "$OUT/goals2.json" "0.0,0.5,0.0" | tee "$OUT/goals2.txt"
  python3 scripts/shield_probe.py "$OUT/wall.json" wall | tee "$OUT/wall.txt"
  python3 scripts/shield_probe.py "$OUT/stale.json" stale | tee "$OUT/stale.txt"
fi
ps -eo rss,pcpu,cmd --sort=-rss | head -12 > "$OUT/ps_top.txt"
grep -iE "shield" "$OUT/launch.log" | cut -c1-220 | sort | uniq -c | sort -rn | head -20 > "$OUT/shield_log.txt" || true
stop_nav_stack
