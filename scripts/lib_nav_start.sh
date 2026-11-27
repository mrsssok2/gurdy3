#!/bin/bash
# Shared start-up helper for the headless Nav2 runs (source it; needs ROS + workspace sourced, cwd = repo root).
#
#   start_nav_stack OUT_DIR  <sim.launch.py arguments...>      # sets LAUNCH (pid of the launch), returns 0 when Nav2 is ready
#   stop_nav_stack                                              # kills the launch and every leftover sim/nav process
#
# "Ready" = scripts/wait_nav_ready.py: navigate_to_pose, compute_path_to_pose and follow_path action servers + map->base_link TF.
# Known start-up flake (seen 2026-11-16): Nav2's lifecycle_manager sometimes never finishes bring-up because a DDS service
# response from controller_server/change_state is lost ("failed to send response ... timeout"); the manager then waits forever and
# re-sending RESET/STARTUP hangs too. Waiting longer does not help, so after NAV_READY_TIMEOUT s the whole stack is restarted
# (up to NAV_START_ATTEMPTS times, failed launch logs kept as launch_failed_N.log). The wait for the two controllers is bounded in WALL
# time too (NAV_CTRL_TIMEOUT, default 120 s; before, up to 180 polls x 10 s with no wall limit: one 2026-11-18 mission spent 12 min here).
# Timestamps of every phase go to $OUT_DIR/nav_ready.txt.
# Processes that belong to a sim / nav stack (override NAV_KILL_PATTERN in tests).
NAV_KILL_PATTERN_DEFAULT="gz sim|robot_state_publisher|parameter_bridge|policy_node|gait_node|nav2|slam_toolbox|amcl|ros2 launch|measure_rtf|component_container|frontier_explorer|explore_monitor|lifecycle_manager|controller_server|planner_server|bt_navigator"

# Kill every process matching the pattern EXCEPT this shell and its ancestors. A plain `pkill -f` also matches the caller: the script text
# (it contains words like "ros2 launch") is part of the command line of `bash -c '...'` / the entrypoint, so the old version killed its own
# container (rc 137) and the restart path below could never run (found 2026-11-18).
_kill_matching() {
  local pat=$1 skip=" $$ $BASHPID " pid p
  p=$$
  while [ "${p:-0}" -gt 1 ]; do p=$(awk '{print $4}' "/proc/$p/stat" 2>/dev/null); skip="$skip$p "; done      # ancestors
  for pid in $(pgrep -f "$pat" 2>/dev/null); do
    case "$skip" in *" $pid "*) continue ;; esac
    kill -KILL "$pid" 2>/dev/null || true
  done
}

stop_nav_stack() {
  kill -TERM "$LAUNCH" 2>/dev/null || true
  sleep "${NAV_STOP_GRACE:-4}"
  _kill_matching "${NAV_KILL_PATTERN:-$NAV_KILL_PATTERN_DEFAULT}"
  sleep 2
}

start_nav_stack() {
  local out=$1; shift
  local attempts=${NAV_START_ATTEMPTS:-3} timeout=${NAV_READY_TIMEOUT:-90} ctrl_to=${NAV_CTRL_TIMEOUT:-120} t0 ctrl_ok
  : > "$out/nav_ready.txt"
  for attempt in $(seq "$attempts"); do
    ros2 launch gurdy_bringup sim.launch.py "$@" > "$out/launch.log" 2>&1 &
    LAUNCH=$!
    sleep 3; kill -0 "$LAUNCH" 2>/dev/null || { echo "ros2 launch exited at once:"; tail -5 "$out/launch.log"; return 1; }   # bad launch argument etc.: fail fast
    t0=$SECONDS; ctrl_ok=0
    echo "$(date -u +%T) attempt $attempt/$attempts: launch started" >> "$out/nav_ready.txt"
    while [ $((SECONDS - t0)) -lt "$ctrl_to" ]; do
      kill -0 "$LAUNCH" 2>/dev/null || break   # launch died (e.g. controller spawner timeout): go to the restart path
      # >= 2: joint_state_broadcaster + the leg controller; arm:=true adds arm + gripper controllers (exactly-2 raced: fetch retest C run 2)
      [ "$(timeout 10 ros2 control list_controllers 2>/dev/null | grep -c " active")" -ge 2 ] && { ctrl_ok=1; break; }
      sleep 1
    done
    echo "$(date -u +%T) attempt $attempt/$attempts: controllers active=$ctrl_ok after $((SECONDS - t0)) s wall" >> "$out/nav_ready.txt"
    if [ "$ctrl_ok" = 1 ] && kill -0 "$LAUNCH" 2>/dev/null && python3 scripts/wait_nav_ready.py --timeout "$timeout" >> "$out/nav_ready.txt" 2>&1; then
      echo "attempt $attempt/$attempts: ready" >> "$out/nav_ready.txt"
      cat "$out/nav_ready.txt"
      return 0
    fi
    echo "attempt $attempt/$attempts: NOT ready after ${timeout}s -> restarting the stack" | tee -a "$out/nav_ready.txt"
    cp "$out/launch.log" "$out/launch_failed_$attempt.log"
    stop_nav_stack
  done
  return 1
}
