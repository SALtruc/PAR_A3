#!/usr/bin/env bash
# Start the official DepthAI RGB-D pointcloud driver and wait for /oak/points.
#
# Keep this script running in its own terminal, then launch Project C from a
# second terminal once the PointCloud2 topic is visible.

set -euo pipefail

DISTRO="${ROS_DISTRO:-jazzy}"
RMW_IMPLEMENTATION="${PROJECT_C_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
CAMERA_MODEL="${CAMERA_MODEL:-OAK-D-PRO}"
POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/oak/points}"
WAIT_SEC="${WAIT_SEC:-20}"
PROJECT_C_STOP_DEPTHAI_SNAP="${PROJECT_C_STOP_DEPTHAI_SNAP:-false}"
PROJECT_C_LOCAL_ONLY="${PROJECT_C_LOCAL_ONLY:-false}"

if [ ! -f "/opt/ros/${DISTRO}/setup.bash" ]; then
  echo "[error] /opt/ros/${DISTRO}/setup.bash not found."
  echo "        Set ROS_DISTRO first, for example: ROS_DISTRO=jazzy bash $0"
  exit 1
fi

set +u
# shellcheck source=/dev/null
source "/opt/ros/${DISTRO}/setup.bash"
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

if [ -n "${CYCLONEDDS_URI:-}" ]; then
  echo "[warn] Clearing shell-exported CYCLONEDDS_URI"
  unset CYCLONEDDS_URI
fi

if ! ros2 pkg prefix depthai_ros_driver >/dev/null 2>&1; then
  echo "[error] depthai_ros_driver is not visible in this ROS environment."
  echo "        Install/source the DepthAI ROS 2 driver before running this helper."
  exit 2
fi

case "${PROJECT_C_STOP_DEPTHAI_SNAP,,}" in
  1|true|yes|on)
    if command -v snap >/dev/null 2>&1 && snap list husarion-depthai >/dev/null 2>&1; then
      echo "[snap] stopping husarion-depthai so depthai_ros_driver can own the OAK camera..."
      sudo snap stop husarion-depthai || true
    fi
    ;;
esac

ros2 daemon stop >/dev/null 2>&1 || true

echo "[ok] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "[ok] PROJECT_C_LOCAL_ONLY=$PROJECT_C_LOCAL_ONLY"
echo "[ok] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
echo "[oak] launching depthai_ros_driver rgbd_pcl.launch.py camera_model:=$CAMERA_MODEL"
ros2 launch depthai_ros_driver rgbd_pcl.launch.py \
  camera_model:="${CAMERA_MODEL}" \
  "$@" &
driver_pid=$!

cleanup() {
  if kill -0 "$driver_pid" >/dev/null 2>&1; then
    kill "$driver_pid" >/dev/null 2>&1 || true
    wait "$driver_pid" >/dev/null 2>&1 || true
  fi
}
trap cleanup INT TERM EXIT

deadline=$((SECONDS + WAIT_SEC))
seen_topic=false
seen_message=false
while [ "$SECONDS" -lt "$deadline" ]; do
  if ! kill -0 "$driver_pid" >/dev/null 2>&1; then
    wait "$driver_pid" || true
    echo "[error] depthai_ros_driver exited before $POINTCLOUD_TOPIC appeared."
    exit 3
  fi

  topics_with_types="$(ros2 topic list -t 2>/dev/null || true)"
  if printf '%s\n' "$topics_with_types" \
      | awk -v topic="$POINTCLOUD_TOPIC" '
          $1 == topic && index($0, "[sensor_msgs/msg/PointCloud2]") { found = 1 }
          END { exit found ? 0 : 1 }
        '; then
    seen_topic=true
    if timeout 5 ros2 topic echo --once --qos-reliability best_effort \
        "$POINTCLOUD_TOPIC" sensor_msgs/msg/PointCloud2 >/dev/null 2>&1 \
        || timeout 5 ros2 topic echo --once --qos-reliability reliable \
        "$POINTCLOUD_TOPIC" sensor_msgs/msg/PointCloud2 >/dev/null 2>&1; then
      seen_message=true
      break
    fi
  fi

  sleep 1
done

if [ "$seen_message" = true ]; then
  echo "[ok] PointCloud2 topic is publishing messages: $POINTCLOUD_TOPIC"
  echo "[next] In another terminal: POINTCLOUD_TOPIC=$POINTCLOUD_TOPIC USE_POINTCLOUD=true bash tools/run_project_c_safety.sh"
elif [ "$seen_topic" = true ]; then
  echo "[warn] $POINTCLOUD_TOPIC is visible but no PointCloud2 messages arrived within ${WAIT_SEC}s."
  echo "[hint] If another service owns the camera, retry with:"
  echo "       PROJECT_C_STOP_DEPTHAI_SNAP=true bash tools/start_oak_pointcloud.sh"
else
  echo "[warn] $POINTCLOUD_TOPIC did not appear within ${WAIT_SEC}s."
  echo "[info] Visible PointCloud2 topics:"
  printf '%s\n' "${topics_with_types:-}" \
    | awk 'index($0, "[sensor_msgs/msg/PointCloud2]") { print "       " $0 }'
  echo "[hint] If another service owns the camera, retry with:"
  echo "       PROJECT_C_STOP_DEPTHAI_SNAP=true bash tools/start_oak_pointcloud.sh"
fi

wait "$driver_pid"
