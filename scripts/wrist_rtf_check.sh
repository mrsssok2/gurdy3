#!/usr/bin/env bash
# Phase 9 wrist-camera cost: RTF of the standing robot (arm:=true, policy, headless) with the wrist RGB-D camera in each mode,
# + arm:=false. One fresh container per variant, one at a time. Results: docs/results/data/day_0927/wrist_rtf/<variant>.log
#   scripts/wrist_rtf_check.sh [variant ...]     variants: noarm off untriggered10 untriggered2 triggered10
set -uo pipefail
cd "$(dirname "$0")/.."
D=docs/results/data/day_0927/wrist_rtf
for v in ${@:-noarm off untriggered10 untriggered2 triggered10}; do
  if docker ps -q | grep -q .; then echo "docker ps not empty: stop"; exit 1; fi
  if [ "$v" = noarm ]; then ARGS="arm:=false"; else ARGS="arm:=true arm_yaml:=/home/ubuntu/gurdy2/$D/arm_$v.yaml"; fi
  timeout 400 docker compose run --rm -T --name gurdy3_wristrtf sim bash -c "
    cd ~/gurdy2; source ros2_ws/install/setup.bash; export PYTHONPATH=\$HOME/gurdy2:\$HOME/gurdy2/gurdy_core:\${PYTHONPATH:-}
    (ros2 launch gurdy_bringup sim.launch.py gui:=false controller:=policy $ARGS > /tmp/launch.log 2>&1 &)
    for i in \$(seq 1 90); do timeout 3 ros2 topic echo --once /joint_states >/dev/null 2>&1 && break; sleep 2; done
    sleep 10
    echo \"variant $v ($ARGS)\"
    python3 scripts/measure_rtf.py --warmup 5 --windows 3 --window 15
    echo -n 'wrist depth rate without trigger: '; timeout 12 ros2 topic hz /wrist_camera/depth_image 2>&1 | grep average | tail -1 || true; echo
    if [ $v = triggered10 ]; then
      (timeout 14 bash -c 'while true; do ros2 topic pub --once /wrist_camera/trigger std_msgs/msg/Bool \"{data: true}\" >/dev/null 2>&1; done' &)
      echo -n 'wrist depth rate while triggering: '; timeout 12 ros2 topic hz /wrist_camera/depth_image 2>&1 | grep average | tail -1 || true; echo
    fi
    grep -iE 'triggered|trigger_topic' /tmp/launch.log | head -3
    top -bn1 | grep -E 'gz sim|ruby|gz-sim' | head -3
  " > $D/$v.log 2>&1
  docker rm -f gurdy3_wristrtf >/dev/null 2>&1
  grep -E "variant|RTF|rtf|rate|Warning" $D/$v.log | cut -c1-200
done
