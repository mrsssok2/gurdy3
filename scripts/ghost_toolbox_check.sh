#!/bin/bash
# Phase 6b: is the "ghost toolbox" a detector false positive or a still-rendered removed model? Run in the sim container, nothing else running.
#   scripts/gpu_lock.sh run gurdy3 -- docker compose run --rm sim bash -c 'cd ~/gurdy2 && scripts/ghost_toolbox_check.sh OUT_DIR'
# Stands the robot 1.4 m from the toolbox facing it (set_pose), records detections, REMOVES the toolbox, keeps looking at the empty spot, then
# turns away. Every detector sighting goes to OUT/sightings.jsonl, camera frames to OUT/frames/ (before / after removal).
set -uo pipefail
OUT=${1:?}; mkdir -p "$OUT/frames"; cd ~/gurdy2; set +u; source ros2_ws/install/setup.bash
export PYTHONPATH=$HOME/gurdy2:$HOME/gurdy2/gurdy_core:$HOME/gurdy2/sim_mujoco:${PYTHONPATH:-}
ros2 launch gurdy_bringup sim.launch.py gui:=false world:=$HOME/gurdy2/ros2_ws/src/gurdy_gazebo/worlds/lab4_incident.sdf perception:=true target_frame:=base_link \
  detector_log:=$OUT/sightings.jsonl controller:=policy > "$OUT/launch.log" 2>&1 &
L=$!
trap 'kill -TERM $L 2>/dev/null; sleep 3; pkill -KILL -f "gz sim" 2>/dev/null' EXIT
for _ in $(seq 60); do [ "$(timeout 10 ros2 control list_controllers 2>/dev/null | grep -c " active")" = 2 ] && break; sleep 2; done
sleep 8
W=lab4
pose() { gz service -s /world/$W/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 --req "name: \"gurdy\", position: {x: $1, y: $2, z: 0.30}, orientation: {z: $(python3 -c "import math;print(math.sin($3/2))"), w: $(python3 -c "import math;print(math.cos($3/2))")}" >/dev/null; }
snap() { python3 scripts/ghost_snap.py "$OUT/frames/$1.png" 2>&1 | tail -1; }
mark() { echo "$(date +%s.%N) $1" >> "$OUT/marks.txt"; }
mark start
pose 3.2 3.9 1.5708; sleep 25; mark facing_toolbox_present; snap 1_present
gz service -s /world/$W/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 5000 --req 'name: "l4_toolbox", type: MODEL' > "$OUT/remove.txt" 2>&1
mark removed; sleep 3
snap 2_after_removal
sleep 30; snap 3_after_removal_30s; mark looked_at_empty_spot
pose 0.6 3.9 3.1416; sleep 40; mark facing_away
snap 4_away
gz topic -e -n 1 -t /world/$W/pose/info 2>/dev/null | grep -c 'name: "l4_toolbox"' > "$OUT/toolbox_in_world_after.txt"
ros2 topic echo --once /semantic_map/objects 2>/dev/null | head -c 3000 > "$OUT/semantic_map_end.txt" || true
mark end
