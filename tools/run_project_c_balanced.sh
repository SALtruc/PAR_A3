#!/usr/bin/env bash
set -e

cd ~/PAR_A3

source /opt/ros/jazzy/setup.bash
source ~/PAR_A3/install/setup.bash

echo "[check] Package prefix:"
ros2 pkg prefix rosbot_obstacle_avoidance

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