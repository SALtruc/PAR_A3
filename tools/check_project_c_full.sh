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
# Project C assessment mode requires all robot sensors. Set this false only for
# degraded demos where LIDAR/OAK are enough and ToF is intentionally disabled.
PROJECT_C_REQUIRE_FULL_FUSION="${PROJECT_C_REQUIRE_FULL_FUSION:-true}"

required_topics=(
  "/scan_filtered"
  "/oak/points"
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
echo "[ok] PROJECT_C_REQUIRE_FULL_FUSION=$PROJECT_C_REQUIRE_FULL_FUSION"
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

missing=0
for topic in "${required_topics[@]}"; do
  if printf '%s\n' "$topics" | grep -Fx "$topic" >/dev/null; then
    echo "[ok] topic present: $topic"
  else
    echo "[missing] topic missing: $topic"
    missing=1
  fi
done

echo
echo "[info] OAK topics:"
printf '%s\n' "$topics" | grep -E '^/oak|^/camera|depth' || true
echo
echo "[info] range/ToF topics:"
printf '%s\n' "$topics" | grep -E 'range|tof|vl53|distance' || true

if [ "$missing" -ne 0 ] && {
    [ "${PROJECT_C_REQUIRE_FULL_FUSION,,}" = "true" ] ||
    [ "${PROJECT_C_REQUIRE_FULL_FUSION}" = "1" ]; }; then
  echo
  echo "[error] Full-fusion prerequisites are not ready."
  echo "        Try:"
  echo "        sudo snap restart husarion-depthai"
  echo "        sudo snap restart husarion-rplidar"
  echo "        sudo snap restart rosbot"
  echo
  echo "        Diagnose missing ToF with:"
  echo "        ros2 topic list | sort | grep -E 'range|tof|vl53|distance'"
  echo "        snap logs rosbot -n 100"
  exit 6
fi

if [ "$missing" -ne 0 ]; then
  echo
  echo "[warn] Missing topics ignored because PROJECT_C_REQUIRE_FULL_FUSION=false."
fi

echo
echo "[ok] Full-fusion robot sensing is ready."
echo "[next] Run: bash tools/run_project_c_safety.sh"
