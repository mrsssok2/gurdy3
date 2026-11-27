#!/usr/bin/env bash
# Overnight (written 2026-11-27, NOT started): (A) grasp walk-to-object variant, (B) walking variance with the arm.
#   nohup scripts/overnight_grasp_walk.sh > /tmp/overnight_grasp_walk.out 2>&1 &        (check `docker ps` is empty first)
# A: scripts/grasp_test.sh: all 5 small classes x 3 tries, the robot starts 2.0 / 1.5 / 1.0 m in front of the grasp placement
#    and WALKS there (closed loop on GT odom, <= 0.08 m/s), then wrist servo with walking body steps, centring, grasp, lift, hold.
#    -> docs/results/data/grasp_test/overnight_walk/ + summary.txt lines.
# B: walking 90 s at 0.10 m/s + 90 s at 0.3 rad/s (policy), 10 runs each: no arm / arm folded / arm folded + 80 g, interleaved
#    (r1: 05 06 07, r2: 05 06 07, ...) so slow drifts of the host hit all configs alike.
#    -> docs/results/data/walk_variance/<config>_r<k>/ + report.md (scripts/walk_variance_report.py).
# Expected duration (measured parts: grasp try ~1.5-2 min, walk step ~1.5 min at 30 s windows): A ~50-60 min, B ~30 x 4.5 min
# = ~2 h 15 min; total ~3-3.5 h. One container at a time; Ollama is not used. Ends with docker ps empty.
set -uo pipefail
cd "$(dirname "$0")/.."
if docker ps -q | grep -q .; then echo "docker ps not empty: not starting"; docker ps --format '  {{.Names}}'; exit 1; fi
trap 'docker rm -f gurdy3_grasp gurdy3_day0927 >/dev/null 2>&1; echo "EXIT $(date -u +%T)"' EXIT
echo "A: grasp walk-to-object $(date -u +%T)"
GRASP_TIMEOUT=5400 scripts/grasp_test.sh overnight_walk -- --tries 3 --yaw wrist --approach walk --step walk
echo "B: walking variance $(date -u +%T)"
OUT=docs/results/data/walk_variance
mkdir -p $OUT
for r in $(seq 1 ${RUNS:-10}); do
  for s in 05_walk_no_arm 06_walk_arm_folded 07_walk_arm_object; do
    [ -f $OUT/${s}_r$r/walk.json ] && continue                       # resumable
    WALK_S=${WALK_S:-90} scripts/day_0927_checks.sh $s | cut -c1-160
    rm -rf $OUT/${s}_r$r; cp -r docs/results/data/day_0927/$s $OUT/${s}_r$r
  done
done
python3 scripts/walk_variance_report.py $OUT --out $OUT/report.md
echo "ALL DONE $(date -u +%T)"
