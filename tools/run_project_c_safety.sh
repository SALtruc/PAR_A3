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
# Reset the CLI daemon by default so an old daemon started from a shell using
# FastRTPS cannot keep poisoning ros2 graph commands.
PROJECT_C_RESET_ROS2_DAEMON="${PROJECT_C_RESET_ROS2_DAEMON:-true}"

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

case "${PROJECT_C_RESET_ROS2_DAEMON,,}" in
  1|true|yes|on)
    ros2 daemon stop >/dev/null 2>&1 || true
    ;;
esac

case "${PROJECT_C_LOCAL_ONLY,,}" in
  1|true|yes|on)
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
    ;;
  *)
    unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
    ;;
esac

# husarion-depthai snap uses FastRTPS with UDPv4 on 127.0.0.1 only and
# useBuiltinTransports=false (no multicast, no SHM). CycloneDDS by default
# skips loopback and uses multicast on physical interfaces, so it never
# reaches depthai. Fix: force CycloneDDS onto loopback with multicast
# disabled so discovery uses unicast peer-to-peer (matching FastRTPS config).
# All other snaps also run on the same machine so loopback is sufficient.
if [ -z "${CYCLONEDDS_URI:-}" ]; then
  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>'
  echo "[ok] CYCLONEDDS_URI=loopback/unicast (matching depthai FastRTPS udp-lo profile)"
else
  echo "[ok] CYCLONEDDS_URI (user-defined): ${CYCLONEDDS_URI}"
fi

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
  depth_topic:="${DEPTH_TOPIC:-/oak/stereo/image_raw}" \
  pointcloud_topic:="${POINTCLOUD_TOPIC:-/oak/points}" \
  pointcloud_qos:="${POINTCLOUD_QOS:-best_effort}" \
  tof_topics:="${TOF_TOPICS:-/range/fl,/range/fr,/range/rl,/range/rr}" \
  tof_msg_type:="${TOF_MSG_TYPE:-scan}" \
  front_tof_topics:="${FRONT_TOF_TOPICS:-/range/fl,/range/fr}" \
  rear_tof_topics:="${REAR_TOF_TOPICS:-/range/rl,/range/rr}" \
  cmd_vel_topic:="${CMD_VEL_TOPIC:-/cmd_vel}" \
  odom_topic:="${ODOM_TOPIC:-/rosbot_base_controller/odom}" \
  imu_topic:="${IMU_TOPIC:-/imu_broadcaster/imu}" \
  use_depth:="${USE_DEPTH:-true}" \
  use_pointcloud:="${USE_POINTCLOUD:-true}" \
  use_tof:="${USE_TOF:-true}" \
  use_nav2_collision_monitor:="${USE_NAV2_COLLISION_MONITOR:-false}" \
  local_only:="${PROJECT_C_LOCAL_ONLY}" \
  max_speed:="${MAX_SPEED:-0.215}" \
  backup_speed:="${BACKUP_SPEED:-0.06}" \
  backup_sec:="${BACKUP_SEC:-0.70}" \
  side_escape_release_distance:="${SIDE_ESCAPE_RELEASE_DISTANCE:-0.08}" \
  side_escape_forward_speed:="${SIDE_ESCAPE_FORWARD_SPEED:-0.025}" \
  side_escape_counter_scale:="${SIDE_ESCAPE_COUNTER_SCALE:-0.60}" \
  side_escape_sec:="${SIDE_ESCAPE_SEC:-0.75}" \
  side_escape_max_attempts:="${SIDE_ESCAPE_MAX_ATTEMPTS:-4}" \
  require_battery_ok:="${REQUIRE_BATTERY_OK:-false}" \
  "$@"
