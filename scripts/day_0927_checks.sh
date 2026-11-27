#!/usr/bin/env bash
# Day 2026-11-27 (0927) Gazebo / ROS checks for branch integration-0927, in order. Written 2026-11-26, NOT run yet.
#
#   scripts/day_0927_checks.sh list            # show the steps
#   scripts/day_0927_checks.sh all             # every step in order (stops at the first FAIL of a step marked [gate])
#   scripts/day_0927_checks.sh 04_arm_spawn    # one step (names below; several may be given)
#   scripts/day_0927_checks.sh stop            # kill the running check container
#
# Every step: headless (gui:=false), ONE container at a time (name gurdy3_day0927), a hard wall-clock timeout, its
# own log directory docs/results/data/day_0927/<step>/ (console.log + the step's files), and exactly one line
# 'PASS <step> ...', 'FAIL <step> ...' or 'SKIP <step> ...' appended to docs/results/data/day_0927/summary.txt.
# Ctrl+C stops the current step (the trap removes the container). Check `docker ps` first: nothing else may run.
set -uo pipefail
cd "$(dirname "$0")/.."
OUT=docs/results/data/day_0927
SUMMARY=$OUT/summary.txt
NAME=gurdy3_day0927
mkdir -p "$OUT"

STEPS=(01_build 02_unit_in_image 03_8c_noise_and_wiring 04_arm_spawn 05_walk_no_arm 06_walk_arm_folded 07_walk_arm_object
       08_moveit_plan 09_wrist_camera 10_scan_gaps 11_small_object_render)
GATES=" 01_build 04_arm_spawn "          # a FAIL here stops `all`

cleanup() { docker rm -f "$NAME" >/dev/null 2>&1 || true; }
trap 'echo "interrupted: removing $NAME"; cleanup; exit 130' INT TERM

result() {   # result STEP STATUS MESSAGE
  local line="$2 $1 $(date '+%F %T') $3"
  echo "$line" | tee -a "$SUMMARY"
}

# run_in STEP TIMEOUT_S 'bash commands inside the sim container (repo at ~/gurdy2, workspace sourced)'
run_in() {
  local step=$1 tmo=$2 cmd=$3 dir=$OUT/$1
  rm -rf "$dir"; mkdir -p "$dir"
  if docker ps --format '{{.Names}}' | grep -q .; then
    echo "other containers are running:"; docker ps --format '  {{.Names}}'; result "$step" FAIL "not started: docker ps not empty"; return 1
  fi
  timeout --signal=INT "$tmo" docker compose run --rm -T --name "$NAME" sim bash -c "
    set -o pipefail; cd ~/gurdy2; set +u; source ros2_ws/install/setup.bash 2>/dev/null || true
    export PYTHONPATH=\$HOME/gurdy2/gurdy_core:\$HOME/gurdy2:\$HOME/gurdy2/scripts:\${PYTHONPATH:-}
    export STEP_DIR=$dir
    export LAUNCH_ARGS='${LAUNCH_ARGS:-}'
    echo \"launch args: \$LAUNCH_ARGS\"
    $cmd" > "$dir/console.log" 2>&1
  local rc=$?
  cleanup
  [ $rc -eq 124 ] && echo "TIMEOUT after ${tmo}s" >> "$dir/console.log"
  return $rc
}

# launch the sim in the background inside the container and wait until /joint_states flows
SIM_UP='(ros2 launch gurdy_bringup sim.launch.py gui:=false controller:=policy $LAUNCH_ARGS > $STEP_DIR/launch.log 2>&1 &)
  for i in $(seq 1 90); do timeout 3 ros2 topic echo --once /joint_states >/dev/null 2>&1 && break; sleep 2; done
  sleep 10'

step_01_build() {      # [gate] the workspace with the new packages builds; the arm URDF is valid
  run_in 01_build 2400 '
    cd ros2_ws && colcon build --symlink-install --event-handlers console_cohesion+ 2>&1 | tail -40 && cd .. &&
    source ros2_ws/install/setup.bash &&
    xacro ros2_ws/src/gurdy_description/urdf/gurdy_with_arm.urdf.xacro arm:=true sim_gazebo:=true > $STEP_DIR/arm.urdf &&
    check_urdf $STEP_DIR/arm.urdf | tail -5 &&
    xacro ros2_ws/src/gurdy_description/urdf/gurdy_with_arm.urdf.xacro > $STEP_DIR/noarm.urdf &&
    xacro ros2_ws/src/gurdy_description/urdf/gurdy.urdf.xacro > $STEP_DIR/base.urdf &&
    python3 -c "import re,sys; a,b=(re.sub(r\"<!--.*?-->\",\"\",open(f).read(),flags=re.S).split() for f in sys.argv[1:]); sys.exit(a!=b)" $STEP_DIR/noarm.urdf $STEP_DIR/base.urdf &&
    ros2 pkg prefix gurdy_arm_moveit && (ros2 pkg prefix pick_ik || echo "PICK_IK_MISSING")'
  if [ $? -eq 0 ]; then
    if grep -q PICK_IK_MISSING "$OUT/01_build/console.log"; then result 01_build PASS "built, URDF valid, arm:=false == default; pick_ik NOT installed (step 08 will fail)"
    else result 01_build PASS "built, URDF valid, arm:=false == default, pick_ik present"; fi
  else result 01_build FAIL "see $OUT/01_build/console.log"; return 1; fi
}

step_02_unit_in_image() {   # the ROS packages' tests + the pure-Python suites inside the image (numpy < 2 there)
  # pytest per package: `colcon test` falls back to unittest discovery for these ament_python packages ("Ran 0 tests", exit 5)
  run_in 02_unit_in_image 1800 '
    for p in gurdy_safety gurdy_navigation gurdy_description gurdy_arm_moveit; do
      echo "== $p"; (cd ros2_ws/src/$p && python3 -m pytest -q test 2>&1 | tail -4); done
    echo "== gurdy_core"; (cd gurdy_core && python3 -m pytest -q 2>&1 | tail -4)'
  local s; s=$(grep -E "^[0-9]+ (passed|failed)|^[0-9]+ failed" "$OUT/02_unit_in_image/console.log" | tr '\n' ';')
  if grep -qE "[0-9]+ failed|[0-9]+ errors?" "$OUT/02_unit_in_image/console.log"; then result 02_unit_in_image FAIL "$s $(grep ^FAILED "$OUT/02_unit_in_image/console.log" | tr '\n' ' ')"
  else result 02_unit_in_image PASS "$s"; fi
}

step_03_8c_noise_and_wiring() {   # 8c: conventions were measured on 2026-11-26 (K "half", z-depth, floor map z -0.11); left: noise + wiring
  if ! grep -q "object3d" ros2_ws/src/gurdy_perception/gurdy_perception/detector_node.py; then
    result 03_8c_noise_and_wiring SKIP "detector_node is not wired to gurdy_core.vision.object3d yet (code task first; plan: docs/results/phase8c_plan.md section 3, use the fitted floor plane)"
    return 0
  fi
  LAUNCH_ARGS="world:=semantic_lab perception:=true target_frame:=odom" run_in 03_8c_noise_and_wiring 1500 "$SIM_UP
    python3 vision/save_semantic_map.py \$STEP_DIR/map.json 2>&1 | tail -3
    python3 vision/compare_semantic_map.py \$STEP_DIR/map.json --gt docs/results/data/phase8c/semantic_lab_gt3d.json | tee \$STEP_DIR/compare.txt"
  if [ $? -eq 0 ]; then result 03_8c_noise_and_wiring PASS "see $OUT/03_8c_noise_and_wiring/compare.txt (targets: xy median <= 2 cm / p90 <= 4 cm static)"
  else result 03_8c_noise_and_wiring FAIL "see console.log"; fi
}

step_04_arm_spawn() {   # [gate] arm:=true spawns, both control blocks load, the arm holds `folded`, the robot stands
  LAUNCH_ARGS="arm:=true" run_in 04_arm_spawn 600 "$SIM_UP
    ros2 control list_controllers 2>&1 | tee \$STEP_DIR/controllers.txt
    grep -q 'arm_controller.*active' \$STEP_DIR/controllers.txt || { echo 'FAIL arm_controller not active (two ros2_control blocks?)'; exit 1; }
    python3 scripts/day0927_probe.py arm_state \$STEP_DIR/arm_state.json"
  local l; l=$(grep -E "^(PASS|FAIL)" "$OUT/04_arm_spawn/console.log" | tail -1)
  if [ -z "$l" ]; then result 04_arm_spawn FAIL "no probe result (see console.log / launch.log)"; return 1; fi
  case $l in PASS*) result 04_arm_spawn PASS "${l#PASS }";; *) result 04_arm_spawn FAIL "${l#FAIL }"; return 1;; esac
}

walk_step() {   # walk_step STEP LAUNCH_ARGS
  LAUNCH_ARGS="$2" run_in "$1" 600 "$SIM_UP
    python3 scripts/day0927_probe.py walk \$STEP_DIR/walk.json 30"
  local l; l=$(grep -E "^(PASS|FAIL)" "$OUT/$1/console.log" | tail -1)
  case $l in PASS*) result "$1" PASS "${l#PASS }";; FAIL*) result "$1" FAIL "${l#FAIL }";; *) result "$1" FAIL "no probe result";; esac
}
step_05_walk_no_arm()      { walk_step 05_walk_no_arm ""; }
step_06_walk_arm_folded()  { walk_step 06_walk_arm_folded "arm:=true"; }
step_07_walk_arm_object()  {   # folded + 0.08 kg in the palm (design payload limit) via an arm.yaml variant
  python3 - <<'EOF'
import yaml, pathlib
p = pathlib.Path("ros2_ws/src/gurdy_description/config/arm.yaml"); d = yaml.safe_load(p.read_text())
d["links"]["palm"]["mass"] = round(d["links"]["palm"]["mass"] + d["payload"]["max_kg"], 4)
out = pathlib.Path("docs/results/data/day_0927/arm_payload.yaml"); out.parent.mkdir(parents=True, exist_ok=True)
out.write_text(yaml.safe_dump(d))
EOF
  walk_step 07_walk_arm_object "arm:=true arm_yaml:=/home/ubuntu/gurdy2/$OUT/arm_payload.yaml"
}

step_08_moveit_plan() {   # move_group with the generated SRDF; plan only: folded -> servo_view, -> analytic pre-grasp
  LAUNCH_ARGS="arm:=true" run_in 08_moveit_plan 900 "$SIM_UP
    (ros2 launch gurdy_arm_moveit move_group.launch.py arm:=true > \$STEP_DIR/move_group.log 2>&1 &)
    sleep 25
    python3 scripts/day0927_probe.py moveit \$STEP_DIR/moveit.json"
  local l; l=$(grep -E "^(PASS|FAIL)" "$OUT/08_moveit_plan/console.log" | tail -1)
  case $l in PASS*) result 08_moveit_plan PASS "${l#PASS }";; FAIL*) result 08_moveit_plan FAIL "${l#FAIL }";; *) result 08_moveit_plan FAIL "no probe result (move_group.log)";; esac
}

step_09_wrist_camera() {   # arm to servo_view, trigger, frames + K + fitted floor height
  LAUNCH_ARGS="arm:=true" run_in 09_wrist_camera 600 "$SIM_UP
    python3 scripts/day0927_probe.py wrist \$STEP_DIR/wrist.json"
  local l; l=$(grep -E "^(PASS|FAIL)" "$OUT/09_wrist_camera/console.log" | tail -1)
  case $l in PASS*) result 09_wrist_camera PASS "${l#PASS }";; FAIL*) result 09_wrist_camera FAIL "${l#FAIL }";; *) result 09_wrist_camera FAIL "no probe result";; esac
}

step_10_scan_gaps() {   # 10 min of /scan under detector + Nav2 load -> recommended shield scan_timeout
  run_in 10_scan_gaps 2700 '
    source scripts/lib_nav_start.sh
    start_nav_stack "$STEP_DIR" gui:=false world:=lab4 nav:=localization map:=lab4_map controller:=policy odom_source:=ground_truth perception:=true \
      || { echo "FAIL Nav2 never became ready"; exit 1; }
    (timeout 620 ros2 bag record -o $STEP_DIR/bag -s sqlite3 /scan > $STEP_DIR/bag.log 2>&1 &)
    for i in 1 2 3; do python3 scripts/nav_eval.py $STEP_DIR/goals_$i.json "4.0,4.0,1.57" "-0.1,4.5,0.0" "0.0,0.0,0.0" > $STEP_DIR/goals_$i.txt 2>&1 || true; done
    sleep 30; stop_nav_stack
    python3 scripts/scan_gaps.py $STEP_DIR/bag | tee $STEP_DIR/scan_gaps.txt'
  local l; l=$(grep "recommended scan_timeout" "$OUT/10_scan_gaps/console.log" | tail -1)
  if [ -n "$l" ]; then result 10_scan_gaps PASS "$l (then regenerate the 9f patch on top of 9c with this value)"
  else result 10_scan_gaps FAIL "no gap statistics (bag.log / console.log)"; fi
}

step_11_small_object_render() {   # 5 + 5 trial scenes of the v3 profiles; small-class labels (ids 7-11) must exist
  run_in 11_small_object_render 1800 '
    D=datasets/day0927_trial
    rm -rf ${D}_body ${D}_wrist
    python3 vision/generate_dataset.py plan --out ${D}_body --scenes 5 --views 12 --seed 31 --profile v3-body &&
    python3 vision/generate_dataset.py plan --out ${D}_wrist --scenes 5 --views 12 --seed 32 --profile v3-wrist &&
    python3 vision/generate_dataset.py render --out ${D}_body && python3 vision/generate_dataset.py render --out ${D}_wrist &&
    for d in ${D}_body ${D}_wrist; do python3 vision/check_labels.py --data $d --out $STEP_DIR/$(basename $d) --samples 6; done &&
    n=$(cat ${D}_body/labels/*/*.txt ${D}_wrist/labels/*/*.txt 2>/dev/null | awk "\$1>=7 && \$1<=11" | wc -l) &&
    echo "small-class labels: $n" && [ "$n" -gt 20 ]'
  local n; n=$(grep "small-class labels" "$OUT/11_small_object_render/console.log" | tail -1)
  if [ $? -eq 0 ] && [ -n "$n" ] && ! grep -q "TIMEOUT" "$OUT/11_small_object_render/console.log"; then
    result 11_small_object_render PASS "$n; LOOK at $OUT/11_small_object_render/*/ before the full render"
  else result 11_small_object_render FAIL "${n:-render failed} (console.log)"; fi
}

case "${1:-list}" in
  list) printf '%s\n' "${STEPS[@]}" ;;
  stop) cleanup; echo "removed $NAME (if it was running)" ;;
  all)
    echo "=== day_0927 all $(date '+%F %T') ===" >> "$SUMMARY"
    for s in "${STEPS[@]}"; do
      echo ">>> $s"
      if ! "step_$s" && [[ $GATES == *" $s "* ]]; then echo "gate $s failed: stopping"; break; fi
    done ;;
  *) for s in "$@"; do "step_$s"; done ;;
esac
