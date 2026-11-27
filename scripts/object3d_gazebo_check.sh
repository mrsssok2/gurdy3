#!/bin/bash
# Phase 8c accuracy check of the 3D object positions in Gazebo semantic_lab (NOT RUN YET; runbook: docs/results/phase8c_plan.md §8).
# Usage (host, nothing else running: check `docker ps` first, one heavy workload at a time):
#   docker compose run --rm -T --name gurdy3_o3d sim bash -c 'cd ~/gurdy2 && scripts/object3d_gazebo_check.sh NAME [OBJECT ...]'
#   NAME     run name -> docs/results/data/phase8c/gazebo/NAME/
#   OBJECT   optional GT object names (e.g. red_box toolbox) for a short first run; default all 11 (132 stops)
# Stack: semantic_lab headless, tripod gait standing (no /cmd_vel), ground-truth odom TF (odom_tf_node), detector_node with
# position_method object3d, target_frame odom (no SLAM: perception error only), sightings log. The robot is TELEPORTED to each
# stop (scripts/object3d_gazebo_check.py), stands 3 s + 2 s, and the scorer (vision/accuracy3d.py) compares every sighting
# (object3d AND the Phase 4 pixel estimate of the same box) and the final semantic map with the 3D ground truth.
# Env: DET (detector ONNX, default det_v2_s.onnx), SETTLE / HOLD (sim s, default 3 / 2), FLOOR_FIT (true), SHAPE_FIT (true).
set -eo pipefail
NAME=${1:?run name}; shift || true
DET=${DET:-det_v2_s.onnx}; SETTLE=${SETTLE:-3}; HOLD=${HOLD:-2}
cd ~/gurdy2; set +u; source ros2_ws/install/setup.bash
export PYTHONPATH=$HOME/gurdy2/gurdy_core:$HOME/gurdy2/scripts:$HOME/gurdy2/vision:${PYTHONPATH:-}
OUT=docs/results/data/phase8c/gazebo/$NAME; rm -rf "$OUT"; mkdir -p "$OUT"
source scripts/lib_nav_start.sh            # _kill_matching / stop_nav_stack (no Nav2 is started here)
NAV_KILL_PATTERN="gz sim|robot_state_publisher|parameter_bridge|gait_node|detector_node|odom_tf_node|ros2 launch|measure_rtf"

python3 vision/accuracy3d.py plan --out "$OUT/stops_plan.json"
git rev-parse HEAD > "$OUT/commit.txt" 2>/dev/null || true
ros2 launch gurdy_bringup sim.launch.py gui:=false world:=semantic_lab controller:=tripod perception:=true \
  detector_model:=$DET target_frame:=odom object3d:=true detector_log:=$PWD/$OUT/sightings.jsonl > "$OUT/launch.log" 2>&1 &
LAUNCH=$!
ros2 run gurdy_navigation odom_tf_node > "$OUT/odom_tf.log" 2>&1 &
ready=false
for _ in $(seq 60); do
  sleep 3
  kill -0 "$LAUNCH" 2>/dev/null || { echo "launch exited"; tail -20 "$OUT/launch.log"; break; }
  if timeout 5 ros2 run tf2_ros tf2_echo odom camera_optical_frame 2>/dev/null | grep -q Translation \
     && grep -q "position method object3d" "$OUT/launch.log"; then ready=true; break; fi
done
if [ "$ready" != true ]; then echo "stack not ready" | tee "$OUT/FAILED.txt"; stop_nav_stack; exit 1; fi
sleep 5
(python3 scripts/measure_rtf.py > "$OUT/rtf.txt" 2>&1 &)
ONLY=(); [ $# -gt 0 ] && ONLY=(--only "$@")
python3 scripts/object3d_gazebo_check.py "$OUT" --settle "$SETTLE" --hold "$HOLD" "${ONLY[@]}" 2>&1 | tee "$OUT/runner.log" || true
sleep 2
timeout 30 python3 vision/save_semantic_map.py "$OUT/semantic.json" > "$OUT/semantic_save.log" 2>&1 || true
ps -eo rss,pcpu,cmd --sort=-rss | head -10 > "$OUT/ps_top.txt"
grep -iE "warn|error|object3d|floor" "$OUT/launch.log" | grep -v "gz_ros_control\|controller_manager\|Utils.cc" | cut -c1-220 \
  | sort | uniq -c | sort -rn | head -30 > "$OUT/warnings.txt" || true
stop_nav_stack
python3 vision/accuracy3d.py score "$OUT" --md "$OUT/report.md" | tee "$OUT/score.txt"
echo "DONE: $OUT/report.md"
