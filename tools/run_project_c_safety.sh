#!/usr/bin/env bash
# Launch Project C full-fusion mode from this repository's install space.
#
# Full fusion for the ROSbot 3 PRO means:
#   - S2 LIDAR: /scan_filtered
#   - OAK-D depth stream as PointCloud2: /oak/points
#   - VL53L0X ToF: /range/fl,/range/fr,/range/rl,/range/rr

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DISTRO="${ROS_DISTRO:-jazzy}"
EXPECTED_PREFIX="${ROOT}/install/rosbot_obstacle_avoidance"
# FastRTPS segfaults on the lab ROSbot image. Force CycloneDDS by default so
# running this script does not depend on the user's current shell exports.
RMW_IMPLEMENTATION="${PROJECT_C_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
# Keep this off by default because ROSbot firmware/sensor participants may be
# exposed through robot-local network namespaces instead of localhost.
PROJECT_C_LOCAL_ONLY="${PROJECT_C_LOCAL_ONLY:-false}"

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

set +u
# shellcheck source=/dev/null
source "/opt/ros/${DISTRO}/setup.bash"
# shellcheck source=/dev/null
source "${ROOT}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION

case "${PROJECT_C_LOCAL_ONLY,,}" in
  1|true|yes|on)
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
    ;;
  *)
    unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
    ;;
esac

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
echo "[ok] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "[ok] PROJECT_C_LOCAL_ONLY=$PROJECT_C_LOCAL_ONLY"
if [ "${PROJECT_C_LOCAL_ONLY,,}" = "true" ] || [ "${PROJECT_C_LOCAL_ONLY}" = "1" ]; then
  echo "[ok] ROS discovery is restricted to localhost"
fi

exec ros2 launch rosbot_obstacle_avoidance project_c_safety.launch.py \
  scan_topic:="${SCAN_TOPIC:-/scan_filtered}" \
  depth_topic:="${DEPTH_TOPIC:-/camera/depth/image_rect_raw}" \
  pointcloud_topic:="${POINTCLOUD_TOPIC:-/oak/points}" \
  tof_topics:="${TOF_TOPICS:-/range/fl,/range/fr,/range/rl,/range/rr}" \
  front_tof_topics:="${FRONT_TOF_TOPICS:-/range/fl,/range/fr}" \
  cmd_vel_topic:="${CMD_VEL_TOPIC:-/cmd_vel}" \
  odom_topic:="${ODOM_TOPIC:-/rosbot_base_controller/odom}" \
  imu_topic:="${IMU_TOPIC:-/imu_broadcaster/imu}" \
  use_depth:="${USE_DEPTH:-false}" \
  use_pointcloud:="${USE_POINTCLOUD:-true}" \
  use_tof:="${USE_TOF:-true}" \
  use_nav2_collision_monitor:="${USE_NAV2_COLLISION_MONITOR:-false}" \
  local_only:="${PROJECT_C_LOCAL_ONLY}" \
  max_speed:="${MAX_SPEED:-0.30}" \
  backup_speed:="${BACKUP_SPEED:-0.06}" \
  backup_sec:="${BACKUP_SEC:-1.60}" \
  require_battery_ok:="${REQUIRE_BATTERY_OK:-false}" \
  "$@"
