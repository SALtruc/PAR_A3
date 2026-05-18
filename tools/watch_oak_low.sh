#!/usr/bin/env bash
# Print the OAK low-view fields from /obstacle_representation.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DISTRO="${ROS_DISTRO:-jazzy}"
EXPECTED_PREFIX="${ROOT}/install/rosbot_obstacle_avoidance"
RMW_IMPLEMENTATION="${PROJECT_C_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
OBSTACLE_TOPIC="${OBSTACLE_TOPIC:-/obstacle_representation}"

if [ ! -f "/opt/ros/${DISTRO}/setup.bash" ]; then
  echo "[error] /opt/ros/${DISTRO}/setup.bash not found."
  exit 1
fi

if [ ! -f "${ROOT}/install/setup.bash" ]; then
  echo "[error] ${ROOT}/install/setup.bash not found."
  echo "        Build first: bash tools/build_project_c.sh"
  exit 2
fi

cd "$ROOT"

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
unset LD_LIBRARY_PATH PYTHONPATH

set +u
# shellcheck source=/dev/null
source "/opt/ros/${DISTRO}/setup.bash"
# shellcheck source=/dev/null
source "${ROOT}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION
ros2 daemon stop >/dev/null 2>&1 || true

actual_prefix="$(ros2 pkg prefix rosbot_obstacle_avoidance 2>/dev/null || true)"
if [ "$actual_prefix" != "$EXPECTED_PREFIX" ]; then
  echo "[error] rosbot_obstacle_avoidance resolved to the wrong workspace:"
  echo "        actual  : ${actual_prefix:-<not found>}"
  echo "        expected: ${EXPECTED_PREFIX}"
  exit 3
fi

echo "[ok] Watching $OBSTACLE_TOPIC with RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "[hint] Move a foot/low object in front of OAK-D. Ctrl-C to stop."

python3 - <<'PY'
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def cm(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return 'inf'
    return f'{value * 100:.0f}cm' if math.isfinite(value) else 'inf'


def finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return math.inf
    return value if math.isfinite(value) else math.inf


class OakLowWatch(Node):
    def __init__(self):
        super().__init__('oak_low_watch')
        topic = os.environ.get('OBSTACLE_TOPIC', '/obstacle_representation')
        self.create_subscription(String, topic, self.on_msg, 10)
        self.last_print = 0.0

    def on_msg(self, msg):
        now = time.monotonic()
        if now - self.last_print < 0.25:
            return
        self.last_print = now
        try:
            rep = json.loads(msg.data)
        except json.JSONDecodeError:
            print('[oak_low] invalid obstacle JSON', flush=True)
            return

        depth = rep.get('depth', {})
        lidar = rep.get('lidar', {})
        fused = rep.get('fused', {})
        tof = rep.get('tof', {})
        image_low = finite(depth.get('image_low_front_min'))
        image_front = finite(depth.get('image_front_min'))
        image_ok = bool(depth.get('image_available', False))
        oak_low = depth.get('pointcloud_low_front_min')
        oak_low_f = finite(oak_low)
        pts = int(depth.get('pointcloud_low_front_count') or 0)
        pc = int(depth.get('pointcloud_sample_count') or 0)
        fallback = int(depth.get('pointcloud_low_fallback_count') or 0)
        pc_status = 'LOW_OBSTACLE' if pts > 0 and math.isfinite(oak_low_f) else 'clear'
        if not image_ok:
            image_status = 'no_depth'
        elif math.isfinite(image_low) and image_low <= 0.35:
            image_status = 'LOW_OBSTACLE'
        elif math.isfinite(image_low):
            image_status = 'clear'
        else:
            image_status = 'no_low_data'
        lidar_front = finite(lidar.get('front_control'))
        lidar_miss = (
            (pc_status == 'LOW_OBSTACLE' or image_status == 'LOW_OBSTACLE')
            and (not math.isfinite(lidar_front) or lidar_front > 0.35)
        )
        print(
            '[oak_low] '
            f'depth_img={"ok" if image_ok else "no"} '
            f'img_front={cm(image_front)} '
            f'img_low={image_status} img_low_dist={cm(image_low)} '
            f'pc_low={pc_status} pc_low_dist={cm(oak_low_f)} pts={pts} '
            f'pc={pc} fallback={fallback} '
            f'lidar_front={cm(lidar_front)} '
            f'fused_front={cm(fused.get("front_distance"))} '
            f'tof={cm(tof.get("range"))} '
            f'lidar_miss={"yes" if lidar_miss else "no"}',
            flush=True,
        )


rclpy.init()
node = OakLowWatch()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
