#!/usr/bin/env bash
# Start the official DepthAI pointcloud driver and wait for PointCloud2 data.
#
# Keep this script running in its own terminal, then launch Project C from a
# second terminal once the PointCloud2 topic is publishing messages.

set -euo pipefail

DISTRO="${ROS_DISTRO:-jazzy}"
RMW_IMPLEMENTATION="${PROJECT_C_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
CAMERA_MODEL="${CAMERA_MODEL:-OAK-D-PRO}"
DEPTHAI_PACKAGE="${DEPTHAI_PACKAGE:-auto}"
DEPTHAI_LAUNCH="${DEPTHAI_LAUNCH:-auto}"
POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-auto}"
WAIT_SEC="${WAIT_SEC:-20}"
PROJECT_C_STOP_DEPTHAI_SNAP="${PROJECT_C_STOP_DEPTHAI_SNAP:-false}"
PROJECT_C_LOCAL_ONLY="${PROJECT_C_LOCAL_ONLY:-false}"
DEPTHAI_RS_COMPAT="${DEPTHAI_RS_COMPAT:-true}"
DEPTHAI_ENABLE_POINTCLOUD="${DEPTHAI_ENABLE_POINTCLOUD:-true}"
DEPTHAI_PIPELINE_TYPE="${DEPTHAI_PIPELINE_TYPE:-RGBD}"
DEPTHAI_STEREO_PUBLISH="${DEPTHAI_STEREO_PUBLISH:-true}"

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

if [ "$DEPTHAI_PACKAGE" = "auto" ]; then
  for candidate in depthai_ros_driver_v3 depthai_ros_driver; do
    if ros2 pkg prefix "$candidate" >/dev/null 2>&1; then
      DEPTHAI_PACKAGE="$candidate"
      break
    fi
  done
elif ! ros2 pkg prefix "$DEPTHAI_PACKAGE" >/dev/null 2>&1; then
  echo "[warn] $DEPTHAI_PACKAGE is not visible in this ROS environment."
  for candidate in depthai_ros_driver_v3 depthai_ros_driver; do
    if ros2 pkg prefix "$candidate" >/dev/null 2>&1; then
      echo "[warn] Falling back to available package: $candidate"
      DEPTHAI_PACKAGE="$candidate"
      break
    fi
  done
fi

if [ "$DEPTHAI_PACKAGE" = "auto" ]; then
  echo "[error] No DepthAI ROS driver package is visible."
  echo "        Install/source depthai_ros_driver_v3 or depthai_ros_driver first."
  exit 2
fi

DEPTHAI_SHARE="$(ros2 pkg prefix "$DEPTHAI_PACKAGE")/share/$DEPTHAI_PACKAGE"

launch_exists() {
  [ -f "${DEPTHAI_SHARE}/launch/$1" ]
}

if [ "$DEPTHAI_LAUNCH" = "auto" ]; then
  for candidate in camera.launch.py driver.launch.py pointcloud.launch.py rgbd_pcl.launch.py; do
    if launch_exists "$candidate"; then
      DEPTHAI_LAUNCH="$candidate"
      break
    fi
  done
elif ! launch_exists "$DEPTHAI_LAUNCH"; then
  echo "[warn] $DEPTHAI_LAUNCH was not found in ${DEPTHAI_SHARE}/launch"
  for candidate in camera.launch.py driver.launch.py pointcloud.launch.py rgbd_pcl.launch.py; do
    if launch_exists "$candidate"; then
      echo "[warn] Falling back to available launch file: $candidate"
      DEPTHAI_LAUNCH="$candidate"
      break
    fi
  done
fi

if [ "$DEPTHAI_LAUNCH" = "auto" ]; then
  echo "[error] No supported DepthAI launch file found in ${DEPTHAI_SHARE}/launch"
  echo "[info] Available launch files:"
  find "${DEPTHAI_SHARE}/launch" -maxdepth 1 -type f -name '*.launch.py' -printf '       %f\n' 2>/dev/null || true
  exit 3
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
echo "[ok] DEPTHAI_PACKAGE=$DEPTHAI_PACKAGE"
echo "[oak] launching $DEPTHAI_PACKAGE $DEPTHAI_LAUNCH camera_model:=$CAMERA_MODEL"
if { [ "$DEPTHAI_LAUNCH" = "driver.launch.py" ] || [ "$DEPTHAI_LAUNCH" = "camera.launch.py" ]; } \
    && [ "$DEPTHAI_PACKAGE" = "depthai_ros_driver_v3" ]; then
  ros2 launch "$DEPTHAI_PACKAGE" "$DEPTHAI_LAUNCH" \
    camera_model:="${CAMERA_MODEL}" \
    rs_compat:="${DEPTHAI_RS_COMPAT}" \
    pointcloud.enable:="${DEPTHAI_ENABLE_POINTCLOUD}" \
    pipeline_gen.i_pipeline_type:="${DEPTHAI_PIPELINE_TYPE}" \
    stereo.i_publish_topic:="${DEPTHAI_STEREO_PUBLISH}" \
    "$@" &
elif [ "$DEPTHAI_LAUNCH" = "driver.launch.py" ] || [ "$DEPTHAI_LAUNCH" = "camera.launch.py" ]; then
  ros2 launch "$DEPTHAI_PACKAGE" "$DEPTHAI_LAUNCH" \
    camera_model:="${CAMERA_MODEL}" \
    rs_compat:="${DEPTHAI_RS_COMPAT}" \
    pointcloud.enable:="${DEPTHAI_ENABLE_POINTCLOUD}" \
    "$@" &
else
  ros2 launch "$DEPTHAI_PACKAGE" "$DEPTHAI_LAUNCH" \
    camera_model:="${CAMERA_MODEL}" \
    "$@" &
fi
driver_pid=$!

cleanup() {
  if kill -0 "$driver_pid" >/dev/null 2>&1; then
    kill "$driver_pid" >/dev/null 2>&1 || true
    wait "$driver_pid" >/dev/null 2>&1 || true
  fi
}
trap cleanup INT TERM EXIT

pointcloud_topic_visible() {
  local topics_with_types="$1"
  if [ "$POINTCLOUD_TOPIC" = "auto" ]; then
    printf '%s\n' "$topics_with_types" \
      | awk 'index($0, "[sensor_msgs/msg/PointCloud2]") { found = 1 }
          END { exit found ? 0 : 1 }'
    return
  fi
  printf '%s\n' "$topics_with_types" \
    | awk -v topic="$POINTCLOUD_TOPIC" '
        $1 == topic && index($0, "[sensor_msgs/msg/PointCloud2]") { found = 1 }
        END { exit found ? 0 : 1 }
      '
}

pointcloud_topics() {
  local topics_with_types="$1"
  printf '%s\n' "$topics_with_types" \
    | awk 'index($0, "[sensor_msgs/msg/PointCloud2]") { print $1 }'
}

pointcloud_message_ready() {
  local topic="$1"
  timeout 6 ros2 topic hz "$topic" --qos-reliability reliable --qos-durability transient_local 2>/dev/null \
    | grep -q 'average rate:' \
    || timeout 6 ros2 topic hz "$topic" --qos-reliability reliable 2>/dev/null \
    | grep -q 'average rate:' \
    || timeout 6 ros2 topic hz "$topic" --qos-profile sensor_data 2>/dev/null \
    | grep -q 'average rate:' \
    || timeout 6 ros2 topic hz "$topic" --qos-reliability best_effort 2>/dev/null \
    | grep -q 'average rate:' \
    || timeout 6 ros2 topic hz "$topic" 2>/dev/null \
    | grep -q 'average rate:'
}

print_oak_diagnostics() {
  local topics_with_types="${1:-}"

  echo "[diag] /oak and camera topics visible:"
  printf '%s\n' "$topics_with_types" \
    | grep -E '^/oak|^/camera|depth|rgb|color|stereo' \
    | sed 's/^/       /' || true

  echo "[diag] PointCloud2 topics visible:"
  printf '%s\n' "$topics_with_types" \
    | awk 'index($0, "[sensor_msgs/msg/PointCloud2]") { print "       " $0 }'

  for topic in $(pointcloud_topics "$topics_with_types"); do
    echo "[diag] $topic endpoint info:"
    timeout 8 ros2 topic info "$topic" --verbose 2>/dev/null \
      | sed 's/^/       /' || true
  done
}

deadline=$((SECONDS + WAIT_SEC))
seen_topic=false
seen_message=false
ready_topic=''
last_status=0
while [ "$SECONDS" -lt "$deadline" ]; do
  if ! kill -0 "$driver_pid" >/dev/null 2>&1; then
    wait "$driver_pid" || true
    echo "[error] depthai_ros_driver exited before $POINTCLOUD_TOPIC appeared."
    exit 3
  fi

  topics_with_types="$(ros2 topic list -t 2>/dev/null || true)"
  if pointcloud_topic_visible "$topics_with_types"; then
    if [ "$seen_topic" = false ]; then
      echo "[ok] PointCloud2 topic visible"
      printf '%s\n' "$topics_with_types" \
        | awk 'index($0, "[sensor_msgs/msg/PointCloud2]") { print "     " $0 }'
      echo "[wait] Waiting for the first PointCloud2 message..."
    fi
    seen_topic=true
    for topic in $(pointcloud_topics "$topics_with_types"); do
      if [ "$POINTCLOUD_TOPIC" != "auto" ] && [ "$topic" != "$POINTCLOUD_TOPIC" ]; then
        continue
      fi
      if pointcloud_message_ready "$topic"; then
        ready_topic="$topic"
        seen_message=true
        break 2
      fi
    done
  fi

  if [ $((SECONDS - last_status)) -ge 5 ]; then
    last_status=$SECONDS
    echo "[wait] Still waiting for PointCloud2 data... $((deadline - SECONDS))s left"
  fi
  sleep 1
done

if [ "$seen_message" = true ]; then
  echo "[ok] PointCloud2 topic is publishing messages: $ready_topic"
  echo "[next] In another terminal: POINTCLOUD_TOPIC=$ready_topic USE_POINTCLOUD=true bash tools/run_project_c_safety.sh"
elif [ "$seen_topic" = true ]; then
  echo "[warn] PointCloud2 topic is visible but no PointCloud2 messages arrived within ${WAIT_SEC}s."
  print_oak_diagnostics "${topics_with_types:-}"
  echo "[hint] If another service owns the camera, retry with:"
  echo "       PROJECT_C_STOP_DEPTHAI_SNAP=true bash tools/start_oak_pointcloud.sh"
  echo "[hint] To compare the example launch files, retry with one of:"
  echo "       DEPTHAI_LAUNCH=camera.launch.py PROJECT_C_STOP_DEPTHAI_SNAP=true bash tools/start_oak_pointcloud.sh"
  echo "       DEPTHAI_LAUNCH=pointcloud.launch.py PROJECT_C_STOP_DEPTHAI_SNAP=true bash tools/start_oak_pointcloud.sh"
  echo "       DEPTHAI_LAUNCH=rgbd_pcl.launch.py PROJECT_C_STOP_DEPTHAI_SNAP=true bash tools/start_oak_pointcloud.sh"
  echo "[hint] If topics exist but no data flows, run:"
  echo "       ros2 topic list -t | grep PointCloud2"
else
  echo "[warn] No PointCloud2 topic appeared within ${WAIT_SEC}s."
  print_oak_diagnostics "${topics_with_types:-}"
  echo "[hint] If another service owns the camera, retry with:"
  echo "       PROJECT_C_STOP_DEPTHAI_SNAP=true bash tools/start_oak_pointcloud.sh"
fi

wait "$driver_pid"
