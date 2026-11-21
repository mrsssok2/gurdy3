#!/bin/bash
# 5.6 polish live run (one container, headless flat_lab): e-stop gate test, idle CPU of the nodes, 9 typed missions, detector sighting log.
#   scripts/p56_polish_run.sh OUT_DIR       (OUT_DIR on the host, inside the repo)
set -eo pipefail
cd "$(dirname "$0")/.."
OUT=${1:?out dir}; mkdir -p "$OUT"; export MODEL=${MODEL:-gurdy-sft-400}; PORT=11435
[ -z "$(docker ps -q)" ] || { echo "containers running"; exit 1; }
if ! curl -sf "localhost:$PORT/api/tags" >/dev/null; then
  OLLAMA_HOST=0.0.0.0:$PORT OLLAMA_MODELS=$HOME/ollama_models LD_LIBRARY_PATH=/usr/lib/wsl/lib nohup "$HOME/ollama/bin/ollama" serve > /tmp/ollama_gurdy.log 2>&1 &
  for _ in $(seq 30); do curl -sf "localhost:$PORT/api/tags" >/dev/null && break; sleep 1; done
fi
curl -s "localhost:$PORT/api/generate" -d "{\"model\":\"$MODEL\",\"keep_alive\":\"30m\"}" >/dev/null
export OLLAMA_URL=http://$(hostname -I | cut -d' ' -f1):$PORT
docker compose run --rm -T --name gurdy3_agent -e OLLAMA_URL -e MODEL -e OUT=$OUT -e ONLY sim bash -c '
  cd ~/gurdy2 && (cd ros2_ws && colcon build --symlink-install --packages-select gurdy_safety gurdy_bringup gurdy_navigation >/dev/null 2>&1); source ros2_ws/install/setup.bash
  export PYTHONPATH=$HOME/gurdy2:$HOME/gurdy2/gurdy_core:$HOME/gurdy2/sim_mujoco:$PYTHONPATH
  source scripts/lib_nav_start.sh; O=/tmp/o; mkdir -p $O; cp maps/flat_lab.places.yaml $O/places.yaml
  trap "stop_nav_stack" EXIT
  start_nav_stack $O gui:=false world:=flat_lab perception:=true detector_log:=$O/sightings.jsonl estop_gate:=true target_frame:=map nav:=localization map:=flat_lab_tripod_gt navigation:=true controller:=policy odom_source:=ground_truth > $O/start.log 2>&1 < /dev/null
  ros2 launch gurdy_safety monitors.launch.py odom_topic:=/odom > $O/monitors.log 2>&1 < /dev/null &
  ros2 run gurdy_navigation frontier_explorer_node --ros-args -p serve_action:=true -p clock_topic:=/clock_slow -p log_path:=$O/explore_log.json -p done_path:=$O/explore.done > $O/explorer.log 2>&1 < /dev/null &
  sleep 15
  echo "== idle CPU (robot standing, 30 s, % of one core)"; 
  top -b -n 4 -d 10 -w 200 | awk "/frontier_expl|alert_monitor|battery_sim|sim_clock|detector_node|leg_health|estop_gate|policy_node|gz sim/ {print}" | sort -k12 | tail -40 > $O/idle_cpu_top.txt; cat $O/idle_cpu_top.txt
  echo "== e-stop gate"; python3 scripts/estop_gate_test.py $O/estop_gate.json 2>&1 | grep -v "^terminate"
  echo "== missions"; python3 -m brain.ros.run_missions --ollama "$MODEL" --places $O/places.yaml --map-id flat_lab --odom-topic /odom --out $O/missions.json ${ONLY:+--only $ONLY} 2>&1 | grep --line-buffered -v "^terminate"
  cp -r $O /home/ubuntu/gurdy2/$OUT/ 2>/dev/null || true
'
