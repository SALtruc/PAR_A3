#!/usr/bin/env bash
# Diagnose OAK depth/pointcloud availability and print the right Project C command.
#
# This does not modify snap configs. It only reports whether an OAK depth image
# or PointCloud2 stream is visible and publishing real messages.

set -euo pipefail

DISTRO="${ROS_DISTRO:-jazzy}"
RMW_IMPLEMENTATION="${PROJECT_C_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
WAIT_SEC="${WAIT_SEC:-12}"

if [ ! -f "/opt/ros/${DISTRO}/setup.bash" ]; then
  echo "[error] /opt/ros/${DISTRO}/setup.bash not found."
  exit 1
fi

set +u
# shellcheck source=/dev/null
source "/opt/ros/${DISTRO}/setup.bash"
set -u
export RMW_IMPLEMENTATION
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS CYCLONEDDS_URI
ros2 daemon stop >/dev/null 2>&1 || true

echo "[ok] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "[ok] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"

echo
echo "[snap] husarion-depthai status:"
if command -v snap >/dev/null 2>&1 && snap list husarion-depthai >/dev/null 2>&1; then
  snap services husarion-depthai 2>/dev/null || true
else
  echo "       husarion-depthai snap is not installed/visible"
fi

params_file="/var/snap/husarion-depthai/common/camera-params-default.yaml"
echo
echo "[snap] depthai params:"
if [ -r "$params_file" ]; then
  echo "       $params_file"
  grep -Ein 'pipeline|rgbd|depth|stereo|point|cloud|disparity' "$params_file" \
    | sed 's/^/       /' || echo "       no depth/stereo/pointcloud keys found"
elif [ -e "$params_file" ]; then
  echo "       $params_file exists but is not readable by this user"
  echo "       run: sudo grep -Ein 'pipeline|rgbd|depth|stereo|point|cloud|disparity' $params_file"
else
  echo "       $params_file not found"
fi

echo
echo "[udev] Luxonis USB rule:"
if grep -R '03e7' /etc/udev/rules.d /lib/udev/rules.d >/dev/null 2>&1; then
  grep -R '03e7' /etc/udev/rules.d /lib/udev/rules.d 2>/dev/null \
    | sed 's/^/       /' || true
else
  echo "       no 03e7 rule found"
  echo "       if v3 reports X_LINK_UNBOOTED permission errors, run:"
  echo "       echo 'SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"03e7\", MODE=\"0666\"' | sudo tee /etc/udev/rules.d/80-depthai.rules"
  echo "       sudo udevadm control --reload-rules && sudo udevadm trigger && sudo udevadm settle"
fi

list_topics() {
  timeout 8 ros2 topic list -t 2>/dev/null || true
}

topic_rate_ready() {
  local topic="$1"
  shift
  local output
  output="$(timeout "$WAIT_SEC" ros2 topic hz "$topic" "$@" 2>&1 || true)"
  if printf '%s\n' "$output" | grep -q 'average rate:'; then
    local rate_line
    rate_line="$(printf '%s\n' "$output" | grep 'average rate:' | tail -n 1)"
    printf '[ok] %s %s\n' "$topic" "$rate_line"
    return 0
  fi
  if printf '%s\n' "$output" | grep -Eiq 'segmentation fault|core dumped'; then
    echo "[warn] $topic hz command crashed with these QoS args: $*"
  fi
  return 1
}

depth_ready() {
  local topic="$1"
  topic_rate_ready "$topic" --qos-profile sensor_data \
    || topic_rate_ready "$topic" --qos-reliability best_effort \
    || topic_rate_ready "$topic" --qos-reliability reliable \
    || topic_rate_ready "$topic"
}

pointcloud_ready() {
  local topic="$1"
  topic_rate_ready "$topic" --qos-reliability reliable --qos-durability transient_local \
    || topic_rate_ready "$topic" --qos-reliability reliable \
    || topic_rate_ready "$topic" --qos-profile sensor_data \
    || topic_rate_ready "$topic" --qos-reliability best_effort \
    || topic_rate_ready "$topic"
}

topics_with_types="$(list_topics)"

echo
echo "[ros] OAK/camera/depth topics:"
printf '%s\n' "$topics_with_types" \
  | grep -Ei 'oak|camera|depth|stereo|point|cloud|rgb|image' \
  | sed 's/^/       /' || echo "       none"

mapfile -t depth_topics < <(
  printf '%s\n' "$topics_with_types" \
    | awk 'index($0, "[sensor_msgs/msg/Image]") { print $1 }' \
    | grep -Eiv 'rgb|color|compressed|theora|ffmpeg|zstd|raw/compressed' \
    | grep -Ei 'oak|camera|depth|stereo' || true
)
mapfile -t pointcloud_topics < <(
  printf '%s\n' "$topics_with_types" \
    | awk 'index($0, "[sensor_msgs/msg/PointCloud2]") { print $1 }' \
    | grep -Ei 'oak|camera|depth|stereo|point|cloud' || true
)

ready_depth=""
ready_pointcloud=""

echo
echo "[check] depth Image streams:"
if [ "${#depth_topics[@]}" -eq 0 ]; then
  echo "       no depth-like sensor_msgs/msg/Image topics"
else
  for topic in "${depth_topics[@]}"; do
    echo "[wait] checking depth data on $topic ..."
    if depth_ready "$topic"; then
      ready_depth="$topic"
      break
    else
      echo "[warn] $topic visible but no depth Image rate detected"
    fi
  done
fi

echo
echo "[check] PointCloud2 streams:"
if [ "${#pointcloud_topics[@]}" -eq 0 ]; then
  echo "       no sensor_msgs/msg/PointCloud2 topics"
else
  for topic in "${pointcloud_topics[@]}"; do
    echo "[wait] checking pointcloud data on $topic ..."
    if pointcloud_ready "$topic"; then
      ready_pointcloud="$topic"
      break
    else
      echo "[warn] $topic visible but no PointCloud2 rate detected"
      timeout 6 ros2 topic info "$topic" --verbose 2>/dev/null \
        | sed 's/^/       /' || true
    fi
  done
fi

echo
echo "[result]"
if [ -n "$ready_depth" ]; then
  echo "[ok] Usable OAK depth image: $ready_depth"
  echo "[next] DEPTH_TOPIC=$ready_depth USE_DEPTH=true USE_POINTCLOUD=false bash tools/run_project_c_safety.sh"
elif [ -n "$ready_pointcloud" ]; then
  echo "[ok] Usable OAK pointcloud: $ready_pointcloud"
  echo "[next] POINTCLOUD_TOPIC=$ready_pointcloud USE_POINTCLOUD=true USE_DEPTH=false POINTCLOUD_QOS=auto bash tools/run_project_c_safety.sh"
else
  echo "[warn] No usable OAK depth/pointcloud stream is publishing real data."
  echo "[hint] If snap only shows RGB, start v3 in another terminal:"
  echo "       sudo snap stop husarion-depthai"
  echo "       source /opt/ros/${DISTRO}/setup.bash"
  echo "       export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
  echo "       ros2 launch depthai_ros_driver_v3 driver.launch.py rs_compat:=true pointcloud.enable:=true pipeline_gen.i_pipeline_type:=RGBD stereo.i_publish_topic:=true"
fi
