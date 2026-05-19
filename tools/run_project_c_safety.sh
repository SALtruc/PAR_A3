#!/usr/bin/env bash
# Project C safety runner.
#
# Design goal:
#   - project_c_safety.launch.py is the source of truth for behaviour defaults.
#   - This script only fixes workspace resolution, checks OAK topics, and passes
#     sensor topics/modes to the launch file.
#   - Behaviour thresholds are only overridden when explicitly provided as env vars.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DISTRO="${ROS_DISTRO:-jazzy}"
EXPECTED_PREFIX="${ROOT}/install/rosbot_obstacle_avoidance"
RMW_IMPLEMENTATION="${PROJECT_C_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
PROJECT_C_LOCAL_ONLY="${PROJECT_C_LOCAL_ONLY:-false}"
PROJECT_C_RESET_ROS2_DAEMON="${PROJECT_C_RESET_ROS2_DAEMON:-true}"

if [ ! -f "/opt/ros/${DISTRO}/setup.bash" ]; then
  echo "[error] /opt/ros/${DISTRO}/setup.bash not found."
  exit 1
fi

if [ ! -f "${ROOT}/install/setup.bash" ]; then
  echo "[error] ${ROOT}/install/setup.bash not found."
  echo "        Build first: cd ${ROOT} && bash tools/build_project_c.sh"
  exit 2
fi

cd "$ROOT"

# Prevent ~/ros2_ws or another overlay from shadowing this repo.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
unset LD_LIBRARY_PATH PYTHONPATH

set +u
# shellcheck source=/dev/null
source "/opt/ros/${DISTRO}/setup.bash"
# shellcheck source=/dev/null
source "${ROOT}/install/local_setup.bash"
set -u

export RMW_IMPLEMENTATION
export ROS_DOMAIN_ID="${PROJECT_C_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-0}}"

case "${PROJECT_C_RESET_ROS2_DAEMON,,}" in
  1|true|yes|on) ros2 daemon stop >/dev/null 2>&1 || true ;;
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

# Avoid stale CycloneDDS debug config from previous sessions.
if [ -n "${CYCLONEDDS_URI:-}" ]; then
  echo "[warn] Clearing shell-exported CYCLONEDDS_URI"
  unset CYCLONEDDS_URI
fi

actual_prefix="$(ros2 pkg prefix rosbot_obstacle_avoidance 2>/dev/null || true)"
echo "[check] Package prefix: ${actual_prefix:-<not found>}"
if [ "$actual_prefix" != "$EXPECTED_PREFIX" ]; then
  echo "[error] Wrong package prefix."
  echo "        actual  : ${actual_prefix:-<not found>}"
  echo "        expected: ${EXPECTED_PREFIX}"
  echo "        Rebuild:  cd ${ROOT} && bash tools/build_project_c.sh"
  exit 3
fi

# Defaults for sensor topics only. Behaviour defaults live in launch file.
SCAN_TOPIC_ARG="${SCAN_TOPIC:-/scan_filtered}"
DEPTH_TOPIC_ARG="${DEPTH_TOPIC:-/camera/camera/depth/image_rect_raw}"
POINTCLOUD_TOPIC_ARG="${POINTCLOUD_TOPIC:-/camera/camera/depth/color/points}"
USE_DEPTH_ARG="${USE_DEPTH:-auto}"
USE_POINTCLOUD_ARG="${USE_POINTCLOUD:-auto}"
USE_TOF_ARG="${USE_TOF:-true}"
POINTCLOUD_QOS_ARG="${POINTCLOUD_QOS:-auto}"
DEPTH_QOS_ARG="${DEPTH_QOS:-auto}"

list_topics_with_types() {
  timeout 5 ros2 topic list -t 2>/dev/null || true
}

topic_has_type() {
  local topic="$1" expected="$2" topics_with_types="$3"
  printf '%s\n' "$topics_with_types" | awk -v topic="$topic" -v expected="[$expected]" '
    $1 == topic && index($0, expected) { found = 1 }
    END { exit found ? 0 : 1 }
  '
}

first_topic_by_type() {
  local expected="$1" topics_with_types="$2" pattern="${3:-}"
  printf '%s\n' "$topics_with_types" \
    | awk -v expected="[$expected]" 'index($0, expected) { print $1 }' \
    | grep -Ei "${pattern:-.*}" \
    | head -n 1
}

topic_has_sample() {
  local topic="$1"
  timeout 5 ros2 topic hz "$topic" --qos-profile sensor_data 2>/dev/null | grep -q 'average rate:' \
    || timeout 5 ros2 topic hz "$topic" --qos-reliability reliable --qos-durability transient_local 2>/dev/null | grep -q 'average rate:' \
    || timeout 5 ros2 topic hz "$topic" --qos-reliability reliable 2>/dev/null | grep -q 'average rate:' \
    || timeout 5 ros2 topic hz "$topic" --qos-reliability best_effort 2>/dev/null | grep -q 'average rate:' \
    || timeout 5 ros2 topic hz "$topic" 2>/dev/null | grep -q 'average rate:'
}

topics_with_types="$(list_topics_with_types)"

# Auto mode means: use the sensor only if a topic exists and publishes samples.
case "${USE_DEPTH_ARG,,}" in
  auto)
    if [ -n "$topics_with_types" ] && topic_has_type "$DEPTH_TOPIC_ARG" sensor_msgs/msg/Image "$topics_with_types" && topic_has_sample "$DEPTH_TOPIC_ARG"; then
      USE_DEPTH_ARG=true
    else
      candidate="$(first_topic_by_type sensor_msgs/msg/Image "$topics_with_types" 'camera|oak|depth|stereo' || true)"
      if [ -n "$candidate" ] && topic_has_sample "$candidate"; then
        echo "[warn] Requested depth topic not active; using $candidate"
        DEPTH_TOPIC_ARG="$candidate"
        USE_DEPTH_ARG=true
      else
        echo "[warn] No active OAK depth image data found; disabling depth support."
        USE_DEPTH_ARG=false
      fi
    fi
    ;;
  1|true|yes|on) USE_DEPTH_ARG=true ;;
  *) USE_DEPTH_ARG=false ;;
esac

case "${USE_POINTCLOUD_ARG,,}" in
  auto)
    if [ -n "$topics_with_types" ] && topic_has_type "$POINTCLOUD_TOPIC_ARG" sensor_msgs/msg/PointCloud2 "$topics_with_types" && topic_has_sample "$POINTCLOUD_TOPIC_ARG"; then
      USE_POINTCLOUD_ARG=true
    else
      candidate="$(first_topic_by_type sensor_msgs/msg/PointCloud2 "$topics_with_types" 'camera|oak|point|cloud|depth' || true)"
      if [ -n "$candidate" ] && topic_has_sample "$candidate"; then
        echo "[warn] Requested pointcloud topic not active; using $candidate"
        POINTCLOUD_TOPIC_ARG="$candidate"
        USE_POINTCLOUD_ARG=true
      else
        echo "[warn] No active OAK PointCloud2 data found; disabling pointcloud support."
        USE_POINTCLOUD_ARG=false
      fi
    fi
    ;;
  1|true|yes|on) USE_POINTCLOUD_ARG=true ;;
  *) USE_POINTCLOUD_ARG=false ;;
esac

# Build launch args. Only include behaviour overrides if the env var exists.
launch_args=(
  "scan_topic:=${SCAN_TOPIC_ARG}"
  "depth_topic:=${DEPTH_TOPIC_ARG}"
  "pointcloud_topic:=${POINTCLOUD_TOPIC_ARG}"
  "depth_qos:=${DEPTH_QOS_ARG}"
  "pointcloud_qos:=${POINTCLOUD_QOS_ARG}"
  "use_depth:=${USE_DEPTH_ARG}"
  "use_pointcloud:=${USE_POINTCLOUD_ARG}"
  "use_tof:=${USE_TOF_ARG}"
  "local_only:=${PROJECT_C_LOCAL_ONLY}"
  "tof_topics:=${TOF_TOPICS:-/range/fl,/range/fr,/range/rl,/range/rr}"
  "tof_msg_type:=${TOF_MSG_TYPE:-scan}"
  "front_tof_topics:=${FRONT_TOF_TOPICS:-/range/fl,/range/fr}"
  "rear_tof_topics:=${REAR_TOF_TOPICS:-/range/rl,/range/rr}"
  "cmd_vel_topic:=${CMD_VEL_TOPIC:-/cmd_vel}"
  "odom_topic:=${ODOM_TOPIC:-/rosbot_base_controller/odom}"
  "imu_topic:=${IMU_TOPIC:-/imu_broadcaster/imu}"
)

add_optional_arg() {
  local env_name="$1" launch_name="$2"
  if [ -n "${!env_name+x}" ]; then
    launch_args+=("${launch_name}:=${!env_name}")
  fi
}

# Optional behaviour overrides. Launch defaults remain source of truth unless set here.
add_optional_arg MAX_SPEED max_speed
add_optional_arg CLEAR_DISTANCE clear_distance
add_optional_arg STOP_DISTANCE stop_distance
add_optional_arg HARD_BACKUP_DISTANCE hard_backup_distance
add_optional_arg LOW_OBSTACLE_DISTANCE low_obstacle_distance
add_optional_arg LOW_OBSTACLE_BACKUP_DISTANCE low_obstacle_backup_distance
add_optional_arg LOW_OBSTACLE_MIN_POINTS low_obstacle_min_points
add_optional_arg LOW_OBSTACLE_HOLD_SEC low_obstacle_hold_sec
add_optional_arg PRE_DODGE_BACKUP_ENABLED pre_dodge_backup_enabled
add_optional_arg FRONT_TOF_OBSTACLE_DISTANCE front_tof_obstacle_distance
add_optional_arg FRONT_TOF_HARD_DISTANCE front_tof_hard_distance
add_optional_arg DODGE_CLEARANCE dodge_clearance
add_optional_arg SIDE_GUARD_DISTANCE side_guard_distance
add_optional_arg EDGE_ESCAPE_FRONT_DISTANCE edge_escape_front_distance
add_optional_arg EDGE_ESCAPE_CLEARANCE edge_escape_clearance
add_optional_arg DODGE_FORWARD_SPEED dodge_forward_speed
add_optional_arg DODGE_ANGULAR_SPEED dodge_angular_speed
add_optional_arg ROTATION_ANGULAR_SPEED rotation_angular_speed
add_optional_arg BACKUP_SPEED backup_speed
add_optional_arg BACKUP_SEC backup_sec
add_optional_arg OBSERVE_FRAMES observe_frames
add_optional_arg CLEAR_OBSERVE_FRAMES clear_observe_frames
add_optional_arg DYNAMIC_OBSERVE_DISTANCE dynamic_observe_distance
add_optional_arg DYNAMIC_TIMEOUT_SEC dynamic_timeout_sec
add_optional_arg DEPTH_MOTION_ENABLED depth_motion_enabled
add_optional_arg DEPTH_MOTION_DELTA_M depth_motion_delta_m
add_optional_arg DEPTH_MOTION_NEAR_M depth_motion_near_m
add_optional_arg DEPTH_MOTION_MIN_RATIO depth_motion_min_ratio
add_optional_arg DEPTH_MOTION_CONFIRM_FRAMES depth_motion_confirm_frames
add_optional_arg DEPTH_MOTION_EGO_SUPPRESS_LIN depth_motion_ego_suppress_lin
add_optional_arg DEPTH_MOTION_EGO_SUPPRESS_ANG depth_motion_ego_suppress_ang
add_optional_arg DEPTH_MOTION_ODOM_STALE_SEC depth_motion_odom_stale_sec
add_optional_arg FRONT_CENTER_ANGLE_DEG front_center_angle_deg
add_optional_arg CREEP_SPEED creep_speed
add_optional_arg REQUIRE_BATTERY_OK require_battery_ok
add_optional_arg DEBUG_PERIOD_SEC debug_period_sec

cat <<STATUS
[ok] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION
[ok] ROS_DOMAIN_ID=$ROS_DOMAIN_ID
[ok] USE_DEPTH=$USE_DEPTH_ARG  DEPTH_TOPIC=$DEPTH_TOPIC_ARG
[ok] USE_POINTCLOUD=$USE_POINTCLOUD_ARG  POINTCLOUD_TOPIC=$POINTCLOUD_TOPIC_ARG
[ok] USE_TOF=$USE_TOF_ARG
[ok] Behaviour defaults are from project_c_safety.launch.py unless env overrides are provided.
STATUS

exec ros2 launch rosbot_obstacle_avoidance project_c_safety.launch.py "${launch_args[@]}" "$@"
