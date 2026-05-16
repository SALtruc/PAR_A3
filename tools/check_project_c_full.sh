#!/usr/bin/env bash
# Check that Project C is built from this repo and that the full-fusion robot
# topics are available before running the autonomous trial.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DISTRO="${ROS_DISTRO:-jazzy}"
EXPECTED_PREFIX="${ROOT}/install/rosbot_obstacle_avoidance"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

required_topics=(
  "/scan_filtered"
  "/oak/points"
  "/range/fl"
  "/range/fr"
  "/range/rl"
  "/range/rr"
  "/cmd_vel"
  "/rosbot_base_controller/odom"
  "/imu_broadcaster/imu"
)

if [ ! -f "/opt/ros/${DISTRO}/setup.bash" ]; then
  echo "[error] /opt/ros/${DISTRO}/setup.bash not found."
  exit 1
fi

if [ ! -f "${ROOT}/install/setup.bash" ]; then
  echo "[error] ${ROOT}/install/setup.bash not found."
  echo "        Build first: bash tools/build_project_c.sh"
  exit 2
fi

cd "$ROOT"

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
unset LD_LIBRARY_PATH PYTHONPATH

set +u
# shellcheck source=/dev/null
source "/opt/ros/${DISTRO}/setup.bash"
# shellcheck source=/dev/null
source "${ROOT}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION

actual_prefix="$(ros2 pkg prefix rosbot_obstacle_avoidance 2>/dev/null || true)"
if [ "$actual_prefix" != "$EXPECTED_PREFIX" ]; then
  echo "[error] rosbot_obstacle_avoidance resolved to the wrong workspace:"
  echo "        actual  : ${actual_prefix:-<not found>}"
  echo "        expected: ${EXPECTED_PREFIX}"
  echo
  echo "        Rebuild first: bash tools/build_project_c.sh"
  exit 3
fi

echo "[ok] Package: $actual_prefix"
echo "[ok] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[error] ros2 command not found after sourcing ROS."
  exit 4
fi

if command -v timeout >/dev/null 2>&1; then
  topics="$(timeout 8 ros2 topic list 2>/tmp/project_c_topic_list.err || true)"
else
  topics="$(ros2 topic list 2>/tmp/project_c_topic_list.err || true)"
fi

if [ -z "$topics" ]; then
  echo "[error] ros2 topic list returned no topics."
  echo "        stderr:"
  sed 's/^/        /' /tmp/project_c_topic_list.err || true
  exit 5
fi

missing=0
for topic in "${required_topics[@]}"; do
  if printf '%s\n' "$topics" | grep -Fx "$topic" >/dev/null; then
    echo "[ok] topic present: $topic"
  else
    echo "[missing] topic missing: $topic"
    missing=1
  fi
done

echo
echo "[info] OAK topics:"
printf '%s\n' "$topics" | grep -E '^/oak|^/camera|depth' || true

if [ "$missing" -ne 0 ]; then
  echo
  echo "[error] Full-fusion prerequisites are not ready."
  echo "        Try:"
  echo "        sudo snap restart husarion-depthai"
  echo "        sudo snap restart husarion-rplidar"
  echo "        sudo snap restart rosbot"
  exit 6
fi

echo
echo "[ok] Full-fusion topics are ready:"
echo "     S2 LIDAR=/scan_filtered, OAK-D depth pointcloud=/oak/points, ToF=/range/*"
echo "[next] Run: bash tools/run_project_c_safety.sh"
