#!/usr/bin/env bash
# Build the Project C obstacle avoidance package from this repository.
# Safe to re-run; colcon only recompiles changed files (--symlink-install).

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DISTRO="${ROS_DISTRO:-jazzy}"

if [ ! -f "/opt/ros/${DISTRO}/setup.bash" ]; then
  echo "[error] /opt/ros/${DISTRO}/setup.bash not found."
  echo "        Set ROS_DISTRO first, e.g.:  ROS_DISTRO=jazzy bash $0"
  exit 1
fi

cd "$ROOT"

# Clean previous build artifacts so stale .pyc files don't survive.
rm -rf \
  build/rosbot_obstacle_avoidance \
  install/rosbot_obstacle_avoidance \
  log

# Avoid accidentally building against another workspace sourced by .bashrc.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
unset LD_LIBRARY_PATH PYTHONPATH

set +u
# shellcheck source=/dev/null
source "/opt/ros/${DISTRO}/setup.bash"
set -u

t0=$(date +%s)

colcon build --symlink-install \
  --base-paths src \
  --packages-select rosbot_obstacle_avoidance \
  --event-handlers console_direct+

elapsed=$(( $(date +%s) - t0 ))
echo
echo "[ok] Built Project C in ${elapsed}s → ${ROOT}/install/rosbot_obstacle_avoidance"
echo "[next] Run: bash tools/run_project_c_safety.sh   (or run_project_c_full.sh)"
