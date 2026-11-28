#!/bin/bash
# Run the same checks as .github/workflows/ci.yml on this machine, in one command. Host only:
# NO Docker, NO GPU, NO Ollama, NO Gazebo (CUDA hidden, OLLAMA_URL pointed at a closed port).
#
#   scripts/ci_local.sh [STEP ...]        steps: yaml urdf lint core vision brain web ros ops safety results (default: all)
#   PY=/path/to/python scripts/ci_local.sh   (default: ~/projects/.venv-gurdy3/bin/python, see scripts/make_host_venv.sh)
#   LINT_STRICT=1 scripts/ci_local.sh lint   critical lint findings (syntax / undefined names) fail the run
#
# Exit code 0 = every step passed. Lint style findings never fail the run (warn-only for now).
# Logs: $CI_LOG_DIR (default /tmp/gurdy3_ci_<time>), one file per step.
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"
PY=${PY:-$HOME/projects/.venv-gurdy3/bin/python}
[ -x "$PY" ] || PY=$(command -v python3)
LOG=${CI_LOG_DIR:-/tmp/gurdy3_ci_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$LOG"

export CUDA_VISIBLE_DEVICES="" OLLAMA_URL="http://127.0.0.1:9" GURDY_CI=1 PYTHONDONTWRITEBYTECODE=1
export MPLBACKEND=Agg
PYTEST=("$PY" -m pytest -q -p no:cacheprovider -rs --tb=short)

ALL=(yaml urdf lint core vision brain web ros ops safety results)
STEPS=("$@"); [ ${#STEPS[@]} -eq 0 ] && STEPS=("${ALL[@]}")

step_yaml()   { "$PY" scripts/ci/check_yaml.py; }
step_urdf()   { "$PY" scripts/ci/check_urdf.py --out "$LOG/urdf"; }
step_lint() {
  "$PY" -m ruff --version || { echo "ruff missing: $PY -m pip install ruff (requirements/host-tests.txt)"; return 1; }
  echo "--- critical (E9 F63 F7 F82: syntax errors, undefined names)"
  "$PY" -m ruff check --no-cache --select E9,F63,F7,F82 --output-format concise .
  local crit=$?
  echo "--- style (warn-only)"
  "$PY" -m ruff check --no-cache --statistics --exit-zero .
  if [ $crit -ne 0 ]; then
    if [ "${LINT_STRICT:-0}" = 1 ]; then return 1; fi
    echo "WARNING: critical lint findings (not blocking: LINT_STRICT=0)"
  fi
  return 0
}
step_core()   { (cd gurdy_core && PYTHONPATH=../ros2_ws/src/gurdy_navigation "${PYTEST[@]}" tests); }
step_vision() { PYTHONPATH=gurdy_core "${PYTEST[@]}" vision/tests; }
step_brain()  { PYTHONPATH=gurdy_core "${PYTEST[@]}" brain/tests; }
step_web()    { PYTHONPATH=gurdy_core "$PY" -m unittest discover -s web/tests -t .; }
step_ops()    { "${PYTEST[@]}" ops/tests; }
step_ros() {   # pure-Python parts of the ROS packages; node tests skip without rclpy (run in the ROS image)
  local rc=0 p
  for p in gurdy_description gurdy_navigation gurdy_perception gurdy_arm_moveit gurdy_gait gurdy_policy gurdy_safety; do
    echo "=== $p"
    (cd "ros2_ws/src/$p" && PYTHONPATH=../../../gurdy_core:. "${PYTEST[@]}" test)
    case $? in
      0) ;;
      5) echo "$p: no test ran here (whole module skipped: needs rclpy / ROS, runs in the ROS image)" ;;
      *) rc=1 ;;
    esac
  done
  return $rc
}
step_safety() { "$PY" scripts/ci/safety_gate.py --json "$LOG/safety.json"; }
step_results() { "$PY" scripts/results_summary.py --check; }     # docs/RESULTS.md regenerated from the committed result files == the committed one

avail=$(free -m 2>/dev/null | awk '/^Mem:/{print $7}')
echo "gurdy3 CI (local)  python: $("$PY" --version 2>&1)  available RAM: ${avail:-?} MB  logs: $LOG"
[ -n "${avail:-}" ] && [ "$avail" -lt 1500 ] && echo "WARNING: < 1500 MB available RAM"

declare -A RES DUR
t_all=$(date +%s)
for s in "${STEPS[@]}"; do
  if ! declare -F "step_$s" >/dev/null; then echo "unknown step: $s (steps: ${ALL[*]})"; exit 2; fi
  t0=$(date +%s)
  printf '%-7s ... ' "$s"
  "step_$s" >"$LOG/$s.log" 2>&1
  rc=$?
  DUR[$s]=$(( $(date +%s) - t0 ))
  if [ $rc -eq 0 ]; then RES[$s]=PASS; else RES[$s]=FAIL; fi
  last=$(grep -E "passed|failed|^Ran |^OK|FAILED|: [0-9]+/[0-9]+ |gate:|All checks|Found [0-9]+ error" "$LOG/$s.log" | tail -1)
  echo "${RES[$s]} (${DUR[$s]} s)  $last"
  [ $rc -ne 0 ] && { echo "----- last 30 lines of $LOG/$s.log"; tail -30 "$LOG/$s.log"; echo "-----"; }
done

fail=0
for s in "${STEPS[@]}"; do [ "${RES[$s]}" = PASS ] || fail=1; done
{ echo; echo "step     result  seconds"; for s in "${STEPS[@]}"; do printf '%-8s %-7s %s\n' "$s" "${RES[$s]}" "${DUR[$s]}"; done
  echo "total $(( $(date +%s) - t_all )) s"; } | tee "$LOG/summary.txt"
[ $fail -eq 0 ] && echo "CI LOCAL: PASS" || echo "CI LOCAL: FAIL"
exit $fail
