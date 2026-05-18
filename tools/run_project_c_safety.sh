#!/usr/bin/env bash
# Launch Project C safety mode from this repository's install space.
#
# Full fusion for the ROSbot 3 PRO means:
#   - S2 LIDAR: /scan_filtered
#   - OAK-D depth image and/or PointCloud2
#   - VL53L0X ToF: /range/fl,/range/fr,/range/rl,/range/rr
# By default this runner auto-enables OAK depth/pointcloud only when the topic
# is visible and publishing messages.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DISTRO="${ROS_DISTRO:-jazzy}"
EXPECTED_PREFIX="${ROOT}/install/rosbot_obstacle_avoidance"
# rmw_fastrtps_cpp segfaults on this ROSbot image; keep CycloneDDS.
# CycloneDDS interoperates with the depthai snap's FastRTPS when QoS matches.
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

# Always clear stale CYCLONEDDS_URI left from diagnostic sessions so the
# script's RMW behaviour is predictable regardless of shell exports.
if [ -n "${CYCLONEDDS_URI:-}" ]; then
  echo "[warn] Clearing shell-exported CYCLONEDDS_URI"
  unset CYCLONEDDS_URI
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
echo "[ok] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
if [ "${PROJECT_C_LOCAL_ONLY,,}" = "true" ] || [ "${PROJECT_C_LOCAL_ONLY}" = "1" ]; then
  echo "[ok] ROS discovery is restricted to localhost"
fi

DEPTH_TOPIC_ARG="${DEPTH_TOPIC:-/oak/stereo/image_raw}"
POINTCLOUD_TOPIC_ARG="${POINTCLOUD_TOPIC:-/oak/points}"
USE_DEPTH_ARG="${USE_DEPTH:-auto}"
USE_POINTCLOUD_ARG="${USE_POINTCLOUD:-auto}"

list_topics_with_types() {
  if command -v timeout >/dev/null 2>&1; then
    timeout 5 ros2 topic list -t 2>/dev/null || true
  else
    ros2 topic list -t 2>/dev/null || true
  fi
}

topic_is_pointcloud() {
  local topic="$1"
  local topics_with_types="$2"
  printf '%s\n' "$topics_with_types" | awk -v topic="$topic" '
    $1 == topic && index($0, "[sensor_msgs/msg/PointCloud2]") { found = 1 }
    END { exit found ? 0 : 1 }
  '
}

topic_is_depth_image() {
  local topic="$1"
  local topics_with_types="$2"
  printf '%s\n' "$topics_with_types" | awk -v topic="$topic" '
    $1 == topic && index($0, "[sensor_msgs/msg/Image]") { found = 1 }
    END { exit found ? 0 : 1 }
  '
}

first_depth_topic() {
  local topics_with_types="$1"
  printf '%s\n' "$topics_with_types" \
    | awk 'index($0, "[sensor_msgs/msg/Image]") { print $1 }' \
    | grep -Eiv 'rgb|color|compressed|theora|raw/compressed' \
    | grep -Ei 'oak|camera|depth|stereo' \
    | head -n 1
}

depth_has_sample() {
  local topic="$1"
  timeout 3 ros2 topic echo --once --qos-profile sensor_data \
    "$topic" >/dev/null 2>&1 \
    || timeout 3 ros2 topic echo --once --qos-reliability best_effort \
    "$topic" sensor_msgs/msg/Image >/dev/null 2>&1 \
    || timeout 3 ros2 topic echo --once --qos-reliability reliable \
    "$topic" sensor_msgs/msg/Image >/dev/null 2>&1
}

first_pointcloud_topic() {
  local topics_with_types="$1"
  printf '%s\n' "$topics_with_types" \
    | awk 'index($0, "[sensor_msgs/msg/PointCloud2]") { print $1 }' \
    | grep -Ei 'oak|camera|depth|stereo|point|cloud' \
    | head -n 1
}

pointcloud_has_sample() {
  local topic="$1"
  timeout 3 ros2 topic echo --once --qos-profile sensor_data \
    "$topic" >/dev/null 2>&1 \
    || timeout 3 ros2 topic echo --once --qos-reliability best_effort \
    "$topic" sensor_msgs/msg/PointCloud2 >/dev/null 2>&1 \
    || timeout 3 ros2 topic echo --once --qos-reliability reliable \
    "$topic" sensor_msgs/msg/PointCloud2 >/dev/null 2>&1
}

topics_with_types=''
case "${USE_DEPTH_ARG,,}" in
  auto)
    for attempt in 1 2 3; do
      topics_with_types="$(list_topics_with_types)"
      [ -n "$topics_with_types" ] && break
      echo "[wait] ROS graph not visible for depth preflight, retry ${attempt}/3..."
      sleep 1
    done
    if [ -z "$topics_with_types" ]; then
      echo "[warn] Could not inspect ROS graph for depth topics; leaving OAK depth enabled."
      USE_DEPTH_ARG=true
    elif topic_is_depth_image "$DEPTH_TOPIC_ARG" "$topics_with_types" && depth_has_sample "$DEPTH_TOPIC_ARG"; then
      USE_DEPTH_ARG=true
      echo "[ok] depth image topic publishing data: $DEPTH_TOPIC_ARG"
    else
      candidate="$(first_depth_topic "$topics_with_types" || true)"
      if [ -n "$candidate" ] && depth_has_sample "$candidate"; then
        echo "[warn] $DEPTH_TOPIC_ARG is not active; using active depth image topic $candidate"
        DEPTH_TOPIC_ARG="$candidate"
        USE_DEPTH_ARG=true
      else
        echo "[warn] No active OAK depth Image data is visible; disabling OAK depth image."
        USE_DEPTH_ARG=false
      fi
    fi
    ;;
  1|true|yes|on)
    USE_DEPTH_ARG=true
    for attempt in 1 2 3 4 5; do
      topics_with_types="$(list_topics_with_types)"
      if [ -z "$topics_with_types" ] || topic_is_depth_image "$DEPTH_TOPIC_ARG" "$topics_with_types"; then
        break
      fi
      echo "[wait] Requested depth topic not visible yet: $DEPTH_TOPIC_ARG (${attempt}/5)"
      sleep 1
    done
    if [ -n "$topics_with_types" ] && ! topic_is_depth_image "$DEPTH_TOPIC_ARG" "$topics_with_types"; then
      candidate="$(first_depth_topic "$topics_with_types" || true)"
      if [ -z "${DEPTH_TOPIC+x}" ] && [ -n "$candidate" ] && depth_has_sample "$candidate"; then
        echo "[warn] $DEPTH_TOPIC_ARG is not visible; using active depth image topic $candidate"
        DEPTH_TOPIC_ARG="$candidate"
      else
        echo "[warn] Requested depth topic is not visible: $DEPTH_TOPIC_ARG"
        echo "[info] Visible depth-like Image topics:"
        printf '%s\n' "$topics_with_types" \
          | awk 'index($0, "[sensor_msgs/msg/Image]") { print $0 }' \
          | grep -Eiv 'rgb|color|compressed|theora|raw/compressed' \
          | grep -Ei 'oak|camera|depth|stereo' \
          | sed 's/^/       /' || true
      fi
    elif [ -n "$topics_with_types" ] && ! depth_has_sample "$DEPTH_TOPIC_ARG"; then
      echo "[warn] $DEPTH_TOPIC_ARG exists but no depth Image samples arrived."
      echo "[warn] Project C will start, but OAK depth image will be unavailable until data flows."
    fi
    ;;
  0|false|no|off)
    USE_DEPTH_ARG=false
    ;;
  *)
    echo "[warn] Unknown USE_DEPTH=$USE_DEPTH_ARG; disabling OAK depth image."
    USE_DEPTH_ARG=false
    ;;
esac

case "${USE_POINTCLOUD_ARG,,}" in
  auto)
    for attempt in 1 2 3; do
      topics_with_types="$(list_topics_with_types)"
      [ -n "$topics_with_types" ] && break
      echo "[wait] ROS graph not visible for pointcloud preflight, retry ${attempt}/3..."
      sleep 1
    done
    if [ -z "$topics_with_types" ]; then
      echo "[warn] Could not inspect ROS graph for PointCloud2 topics; leaving OAK pointcloud enabled."
      USE_POINTCLOUD_ARG=true
    elif topic_is_pointcloud "$POINTCLOUD_TOPIC_ARG" "$topics_with_types"; then
      if pointcloud_has_sample "$POINTCLOUD_TOPIC_ARG"; then
        USE_POINTCLOUD_ARG=true
        echo "[ok] pointcloud topic publishing data: $POINTCLOUD_TOPIC_ARG"
      else
        USE_POINTCLOUD_ARG=false
        echo "[warn] $POINTCLOUD_TOPIC_ARG exists but no PointCloud2 samples arrived; disabling OAK pointcloud."
      fi
    else
      candidate="$(first_pointcloud_topic "$topics_with_types" || true)"
      if [ -n "$candidate" ] && pointcloud_has_sample "$candidate"; then
        echo "[warn] $POINTCLOUD_TOPIC_ARG is not visible; using active PointCloud2 topic $candidate"
        POINTCLOUD_TOPIC_ARG="$candidate"
        USE_POINTCLOUD_ARG=true
      else
        echo "[warn] No active PointCloud2 data is visible; disabling OAK pointcloud for this safety run."
        echo "       For full fusion, run: PROJECT_C_STOP_DEPTHAI_SNAP=true bash tools/start_oak_pointcloud.sh"
        USE_POINTCLOUD_ARG=false
      fi
    fi
    ;;
  1|true|yes|on)
    USE_POINTCLOUD_ARG=true
    for attempt in 1 2 3 4 5; do
      topics_with_types="$(list_topics_with_types)"
      if [ -z "$topics_with_types" ] || topic_is_pointcloud "$POINTCLOUD_TOPIC_ARG" "$topics_with_types"; then
        break
      fi
      echo "[wait] Requested pointcloud topic not visible yet: $POINTCLOUD_TOPIC_ARG (${attempt}/5)"
      sleep 1
    done
    if [ -n "$topics_with_types" ] && ! topic_is_pointcloud "$POINTCLOUD_TOPIC_ARG" "$topics_with_types"; then
      candidate="$(first_pointcloud_topic "$topics_with_types" || true)"
      if [ -z "${POINTCLOUD_TOPIC+x}" ] && [ -n "$candidate" ] && pointcloud_has_sample "$candidate"; then
        echo "[warn] $POINTCLOUD_TOPIC_ARG is not visible; using active PointCloud2 topic $candidate"
        POINTCLOUD_TOPIC_ARG="$candidate"
      else
        echo "[warn] Requested pointcloud topic is not visible: $POINTCLOUD_TOPIC_ARG"
        echo "[info] Visible PointCloud2 topics:"
        printf '%s\n' "$topics_with_types" \
          | awk 'index($0, "[sensor_msgs/msg/PointCloud2]") { print "       " $0 }'
      fi
    elif [ -n "$topics_with_types" ] && ! pointcloud_has_sample "$POINTCLOUD_TOPIC_ARG"; then
      echo "[warn] $POINTCLOUD_TOPIC_ARG exists but no PointCloud2 samples arrived."
      echo "[warn] Project C will start, but OAK pointcloud fields will stay pc=0/pts=0 until data flows."
    fi
    ;;
  0|false|no|off)
    USE_POINTCLOUD_ARG=false
    ;;
  *)
    echo "[warn] Unknown USE_POINTCLOUD=$USE_POINTCLOUD_ARG; disabling OAK pointcloud."
    USE_POINTCLOUD_ARG=false
    ;;
esac

echo "[ok] POINTCLOUD_TOPIC=$POINTCLOUD_TOPIC_ARG"
echo "[ok] DEPTH_TOPIC=$DEPTH_TOPIC_ARG"
echo "[ok] USE_DEPTH=$USE_DEPTH_ARG"
echo "[ok] USE_POINTCLOUD=$USE_POINTCLOUD_ARG"

exec ros2 launch rosbot_obstacle_avoidance project_c_safety.launch.py \
  scan_topic:="${SCAN_TOPIC:-/scan_filtered}" \
  depth_topic:="${DEPTH_TOPIC_ARG}" \
  pointcloud_topic:="${POINTCLOUD_TOPIC_ARG}" \
  pointcloud_qos:="${POINTCLOUD_QOS:-auto}" \
  tof_topics:="${TOF_TOPICS:-/range/fl,/range/fr,/range/rl,/range/rr}" \
  tof_msg_type:="${TOF_MSG_TYPE:-scan}" \
  front_tof_topics:="${FRONT_TOF_TOPICS:-/range/fl,/range/fr}" \
  rear_tof_topics:="${REAR_TOF_TOPICS:-/range/rl,/range/rr}" \
  cmd_vel_topic:="${CMD_VEL_TOPIC:-/cmd_vel}" \
  odom_topic:="${ODOM_TOPIC:-/rosbot_base_controller/odom}" \
  imu_topic:="${IMU_TOPIC:-/imu_broadcaster/imu}" \
  use_depth:="${USE_DEPTH_ARG}" \
  use_pointcloud:="${USE_POINTCLOUD_ARG}" \
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
