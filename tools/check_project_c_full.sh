#!/usr/bin/env bash
# Check that Project C is built from this repo and that the full-fusion robot
# topics are available before running the autonomous trial.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DISTRO="${ROS_DISTRO:-jazzy}"
EXPECTED_PREFIX="${ROOT}/install/rosbot_obstacle_avoidance"
# FastRTPS segfaults on the lab ROSbot image. Force CycloneDDS by default so
# this check is stable even if the user's shell exported RMW_IMPLEMENTATION.
RMW_IMPLEMENTATION="${PROJECT_C_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
# Match the run script. Local-only can hide ROSbot firmware/sensor topics on
# the lab image, so it is opt-in instead of default.
PROJECT_C_LOCAL_ONLY="${PROJECT_C_LOCAL_ONLY:-false}"

core_topics=(
  "/scan_filtered"
  "/oak/points"
)

optional_topics=(
  "/range/fl"
  "/range/fr"
  "/range/rl"
  "/range/rr"
  "/rosbot_base_controller/odom"
  "/imu_broadcaster/imu"
)

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

echo "[ok] Package: $actual_prefix"
echo "[ok] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "[ok] PROJECT_C_LOCAL_ONLY=$PROJECT_C_LOCAL_ONLY"
if [ "${PROJECT_C_LOCAL_ONLY,,}" = "true" ] || [ "${PROJECT_C_LOCAL_ONLY}" = "1" ]; then
  echo "[ok] ROS discovery is restricted to localhost"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[error] ros2 command not found after sourcing ROS."
  exit 4
fi

topics=''
for attempt in 1 2 3 4 5; do
  if command -v timeout >/dev/null 2>&1; then
    topics="$(timeout 8 ros2 topic list 2>/tmp/project_c_topic_list.err || true)"
  else
    topics="$(ros2 topic list 2>/tmp/project_c_topic_list.err || true)"
  fi
  if [ -n "$topics" ]; then
    break
  fi
  echo "[wait] ROS graph not visible yet, retry ${attempt}/5..."
  sleep 1
done

if [ -z "$topics" ]; then
  echo "[error] ros2 topic list returned no topics."
  echo "        stderr:"
  sed 's/^/        /' /tmp/project_c_topic_list.err || true
  exit 5
fi

core_present=0
for topic in "${core_topics[@]}"; do
  if printf '%s\n' "$topics" | grep -Fx "$topic" >/dev/null; then
    echo "[ok] topic present: $topic"
    core_present=1
  else
    echo "[missing] topic missing: $topic"
  fi
done

optional_missing=0
for topic in "${optional_topics[@]}"; do
  if printf '%s\n' "$topics" | grep -Fx "$topic" >/dev/null; then
    echo "[ok] topic present: $topic"
  else
    echo "[warn] optional topic missing before launch: $topic"
    optional_missing=1
  fi
done

echo
echo "[info] OAK topics:"
printf '%s\n' "$topics" | grep -E '^/oak|^/camera|depth' || true

if [ "$core_present" -eq 0 ]; then
  echo
  echo "[error] No core obstacle sensor topic is visible."
  echo "        Try:"
  echo "        sudo snap restart husarion-depthai"
  echo "        sudo snap restart husarion-rplidar"
  echo "        sudo snap restart rosbot"
  exit 6
fi

if [ "$optional_missing" -ne 0 ]; then
  echo
  echo "[warn] Some optional topics are not visible before launch."
  echo "       The run can still start if perception receives /scan_filtered or /oak/points."
fi

echo
echo "[ok] Robot-local obstacle sensing is ready."
echo "[next] Run: bash tools/run_project_c_safety.sh"
