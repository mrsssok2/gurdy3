#!/usr/bin/env bash
# Move the arm of the running GUI sim (container gurdy3_gui): scripts/arm_pose.sh folded|servo_view|servo_view_far|pre_grasp [seconds]
exec docker exec gurdy3_gui /entrypoint.sh bash -c "cd ~/gurdy2 && source ros2_ws/install/setup.bash && PYTHONPATH=\$HOME/gurdy2/gurdy_core:\$PYTHONPATH python3 scripts/arm_goto.py $*"
