#!/bin/bash
# Gurdy 3.0: does Nav2 + the policy pass EVERY lab4 doorway, in both directions? (saved map lab4_map, localization, headless, nothing else running)
# Usage: docker compose run --rm -T --name gurdy3_doors sim bash -c 'cd ~/gurdy2 && scripts/lab4_door_check.sh [MAP]'
# Goals (x,y,yaw): doorway centres and room centres around the ring A-B-D-C-A forward, then backwards. Output: docs/results/data/lab4/door_check/
set -eo pipefail
MAP=${1:-lab4_map}
cd ~/gurdy2; set +u; source ros2_ws/install/setup.bash
export PYTHONPATH=$HOME/gurdy2/gurdy_core:$HOME/gurdy2/sim_mujoco:${PYTHONPATH:-}
OUT=docs/results/data/lab4/door_check; rm -rf "$OUT"; mkdir -p "$OUT"
source scripts/lib_nav_start.sh
start_nav_stack "$OUT" gui:=false world:=lab4 nav:=localization map:=$MAP controller:=policy odom_source:=ground_truth \
  || { echo "Nav2 never became ready" | tee "$OUT/FAILED.txt"; stop_nav_stack; exit 1; }
sleep 5
(python3 scripts/measure_rtf.py > "$OUT/rtf.txt" 2>&1 &)
python3 scripts/nav_eval.py "$OUT/goals.json" \
  "2.0,-0.7,0.0" "4.0,0.0,1.57" "4.0,2.0,1.57" "4.0,4.0,3.14" "2.0,4.2,3.14" "0.0,4.0,-1.57" "-0.7,2.0,-1.57" "0.0,0.0,0.0" \
  "-0.7,2.0,1.57" "0.0,4.0,0.0" "2.0,4.2,0.0" "4.0,4.0,-1.57" "4.0,2.0,-1.57" "4.0,0.0,3.14" "2.0,-0.7,3.14" "0.0,0.0,0.0" | tee "$OUT/goals.txt"
ps -eo rss,cmd --sort=-rss | head -8 > "$OUT/ps_top.txt"
grep -iE "warn|error" "$OUT/launch.log" | grep -v "gz_ros_control\|controller_manager\|Utils.cc" | cut -c1-220 | sort | uniq -c | sort -rn | head -20 > "$OUT/warnings.txt" || true
stop_nav_stack
