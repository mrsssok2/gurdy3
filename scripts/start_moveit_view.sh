#!/usr/bin/env bash
# Arm viewing session: headless Gazebo (lab4_fetch, arm:=true, policy standing, lidar + map + AMCL, detector) + move_group +
# RViz2 WINDOW (WSLg) with the MotionPlanning panel + floor in the planning scene + wrist camera triggered at 2 Hz.
# Image gurdy3/ros:jazzy-moveit (compose service sim_moveit). Logs: brain/runs/moveit_view/ (gitignored).
#   scripts/start_moveit_view.sh          start (returns when RViz is up)
#   scripts/start_moveit_view.sh stop     stop everything (container removed)
set -uo pipefail
cd "$(dirname "$0")/.."
NAME=gurdy3_view
if [ "${1:-}" = stop ]; then docker rm -f $NAME >/dev/null 2>&1; echo "stopped ($NAME removed)"; exit 0; fi
if docker ps -q | grep -q .; then echo "other containers are running:"; docker ps --format '  {{.Names}}'; exit 1; fi
L=brain/runs/moveit_view; rm -rf $L; mkdir -p $L
docker compose --profile moveit run -d --name $NAME sim_moveit bash -c "
  cd ~/gurdy2 && source /opt/ros/jazzy/setup.bash
  (cd ros2_ws && colcon build --symlink-install --packages-select gurdy_arm_moveit > ~/gurdy2/$L/build.log 2>&1)
  source ros2_ws/install/setup.bash
  export PYTHONPATH=\$HOME/gurdy2/gurdy_core:\$HOME/gurdy2:\${PYTHONPATH:-}
  ros2 launch gurdy_bringup sim.launch.py gui:=false controller:=policy arm:=true world:=lab4_fetch nav:=localization map:=lab4_map navigation:=false perception:=true > $L/sim.log 2>&1 &
  for i in \$(seq 1 90); do timeout 3 ros2 topic echo --once /joint_states >/dev/null 2>&1 && break; sleep 2; done
  sleep 5
  ros2 launch gurdy_arm_moveit move_group.launch.py arm:=true rviz:=true > $L/moveit.log 2>&1 &
  python3 scripts/moveit_view_extras.py > $L/extras.log 2>&1 &
  wait" > /dev/null
echo "starting ... (logs in $L)"
for i in $(seq 1 90); do grep -q "floor added" $L/extras.log 2>/dev/null && break; sleep 2; done
cat $L/extras.log 2>/dev/null | tail -1
grep -m1 -E "You can start planning now" $L/moveit.log || echo "move_group not ready yet (see $L/moveit.log)"
