#!/usr/bin/env bash
# Build only the Project C obstacle avoidance package from this repository.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DISTRO="${ROS_DISTRO:-jazzy}"

if [ ! -f "/opt/ros/${DISTRO}/setup.bash" ]; then
  echo "[error] /opt/ros/${DISTRO}/setup.bash not found."
  echo "        Set ROS_DISTRO first, for example: ROS_DISTRO=jazzy bash $0"
  exit 1
fi

cd "$ROOT"

rm -rf \
  build/rosbot_obstacle_avoidance \
  install/rosbot_obstacle_avoidance \
  log

# Avoid accidentally building against another workspace sourced by .bashrc.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH

# shellcheck source=/dev/null
source "/opt/ros/${DISTRO}/setup.bash"

colcon build --symlink-install \
  --base-paths src \
  --packages-select rosbot_obstacle_avoidance \
  --event-handlers console_direct+

echo
echo "[ok] Built Project C in: ${ROOT}/install/rosbot_obstacle_avoidance"
echo "[next] Run: bash tools/run_project_c_safety.sh"
