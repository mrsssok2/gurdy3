#!/bin/bash
# Quick environment check inside the sim container: GUI sockets, OpenGL renderer, tool versions.
# Usage: scripts/gpu_lock.sh run gurdy3 -- docker compose run --rm sim scripts/check_display.sh
set -u
ok()   { echo "  [ok]   $*"; }
bad()  { echo "  [FAIL] $*"; FAIL=1; }
FAIL=0

echo "== Display sockets"
[ -S /tmp/.X11-unix/X0 ] && ok "X11 socket /tmp/.X11-unix/X0" || bad "no X11 socket"
[ -S "${XDG_RUNTIME_DIR:-/x}/${WAYLAND_DISPLAY:-x}" ] && ok "Wayland socket" || bad "no Wayland socket"
[ -e /dev/dxg ] && ok "/dev/dxg (WSL GPU)" || echo "  [warn] no /dev/dxg -> software rendering only"

echo "== OpenGL"
if out=$(glxinfo -B 2>&1); then
  echo "$out" | grep -E "OpenGL renderer|OpenGL core profile version|OpenGL version" | sed 's/^/  /'
else
  bad "glxinfo failed: $(echo "$out" | head -2)"
fi

echo "== Versions"
echo "  ROS_DISTRO=${ROS_DISTRO:-unset}"
echo "  $(gz sim --versions 2>/dev/null | head -1 | sed 's/^/gz sim /')"
python3 -c "import mujoco; print('  mujoco', mujoco.__version__)" || bad "mujoco import"
ros2 pkg prefix gz_ros2_control >/dev/null 2>&1 && ok "gz_ros2_control" || bad "gz_ros2_control missing"
ros2 pkg prefix position_controllers >/dev/null 2>&1 && ok "ros2_controllers" || bad "ros2_controllers missing"
command -v check_urdf >/dev/null && ok "check_urdf" || bad "check_urdf missing"

echo "== MuJoCo headless step"
python3 - <<'EOF' || FAIL=1
import mujoco, time
m = mujoco.MjModel.from_xml_string("<mujoco><worldbody><body pos='0 0 1'><freejoint/><geom size='.1'/></body></worldbody></mujoco>")
d = mujoco.MjData(m); t = time.perf_counter()
for _ in range(10000): mujoco.mj_step(m, d)
dt = time.perf_counter()-t
print(f"  10k steps ({10000*m.opt.timestep:.0f} s sim) in {dt:.3f} s wall, ball z={d.qpos[2]:.1f} (free fall, expected < 0)")
EOF

[ $FAIL -eq 0 ] && echo "ALL CHECKS PASSED" || { echo "SOME CHECKS FAILED"; exit 1; }
