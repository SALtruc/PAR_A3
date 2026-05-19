#!/usr/bin/env bash
set -e

cd ~/PAR_A3

unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset PYTHONPATH

source /opt/ros/jazzy/setup.bash
source ~/PAR_A3/install/local_setup.bash

echo "[check] Package prefix:"
PREFIX="$(ros2 pkg prefix rosbot_obstacle_avoidance)"
echo "$PREFIX"

if [[ "$PREFIX" != "/home/husarion/PAR_A3/install/rosbot_obstacle_avoidance" ]]; then
  echo "[ERROR] Wrong package prefix. Expected PAR_A3 install, got: $PREFIX"
  exit 1
fi

DEPTH_TOPIC=${DEPTH_TOPIC:-/camera/camera/depth/image_rect_raw}
POINTCLOUD_TOPIC=${POINTCLOUD_TOPIC:-/camera/camera/depth/color/points}

USE_DEPTH=${USE_DEPTH:-true}
USE_POINTCLOUD=${USE_POINTCLOUD:-true}
USE_TOF=${USE_TOF:-true}
POINTCLOUD_QOS=${POINTCLOUD_QOS:-auto}

ros2 launch rosbot_obstacle_avoidance project_c_safety.launch.py \
  depth_topic:="$DEPTH_TOPIC" \
  pointcloud_topic:="$POINTCLOUD_TOPIC" \
  use_depth:="$USE_DEPTH" \
  use_pointcloud:="$USE_POINTCLOUD" \
  use_tof:="$USE_TOF" \
  pointcloud_qos:="$POINTCLOUD_QOS"