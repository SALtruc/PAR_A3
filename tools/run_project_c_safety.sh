#!/usr/bin/env bash
# Launch Project C from this repository's install space, even if another ROS 2
# workspace is sourced in the user's shell.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DISTRO="${ROS_DISTRO:-jazzy}"
EXPECTED_PREFIX="${ROOT}/install/rosbot_obstacle_avoidance"

if [ ! -f "/opt/ros/${DISTRO}/setup.bash" ]; then
  echo "[error] /opt/ros/${DISTRO}/setup.bash not found."
  echo "        Set ROS_DISTRO first, for example: ROS_DISTRO=jazzy bash $0"
  exit 1
fi

if [ ! -f "${ROOT}/install/setup.bash" ]; then
  echo "[error] ${ROOT}/install/setup.bash not found."
  echo "        Build first: bash tools/build_project_c.sh"
  exit 2
fi

cd "$ROOT"

# Reset overlay variables so ~/ros2_ws cannot shadow this repo.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH

# Drop stale ROS library/python paths from previously sourced workspaces. The
# ROS setup files below will repopulate them in the right order.
unset LD_LIBRARY_PATH PYTHONPATH

# shellcheck source=/dev/null
source "/opt/ros/${DISTRO}/setup.bash"
# shellcheck source=/dev/null
source "${ROOT}/install/setup.bash"

actual_prefix="$(ros2 pkg prefix rosbot_obstacle_avoidance 2>/dev/null || true)"
if [ "$actual_prefix" != "$EXPECTED_PREFIX" ]; then
  echo "[error] rosbot_obstacle_avoidance resolved to the wrong workspace:"
  echo "        actual  : ${actual_prefix:-<not found>}"
  echo "        expected: ${EXPECTED_PREFIX}"
  echo
  echo "        Rebuild first: bash tools/build_project_c.sh"
  exit 3
fi

echo "[ok] Using package: $actual_prefix"

exec ros2 launch rosbot_obstacle_avoidance project_c_safety.launch.py \
  scan_topic:="${SCAN_TOPIC:-/scan_filtered}" \
  depth_topic:="${DEPTH_TOPIC:-/camera/depth/image_rect_raw}" \
  pointcloud_topic:="${POINTCLOUD_TOPIC:-/oak/points}" \
  cmd_vel_topic:="${CMD_VEL_TOPIC:-/cmd_vel}" \
  use_nav2_collision_monitor:="${USE_NAV2_COLLISION_MONITOR:-false}" \
  max_speed:="${MAX_SPEED:-0.10}" \
  backup_speed:="${BACKUP_SPEED:-0.04}" \
  require_battery_ok:="${REQUIRE_BATTERY_OK:-false}" \
  "$@"
