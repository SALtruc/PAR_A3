#!/usr/bin/env bash
# One-command Project C full-fusion run.
#
# This script:
#   1. restarts robot sensor/firmware snaps,
#   2. builds this repository's package,
#   3. waits until LIDAR + OAK + ToF + odom + IMU topics are visible,
#   4. launches the safety controller.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESTART_SNAPS="${PROJECT_C_RESTART_SNAPS:-true}"
CHECK_ATTEMPTS="${PROJECT_C_CHECK_ATTEMPTS:-8}"
CHECK_SLEEP_SEC="${PROJECT_C_CHECK_SLEEP_SEC:-4}"

cd "$ROOT"

restart_snaps() {
  case "${RESTART_SNAPS,,}" in
    1|true|yes|on)
      echo "[step] Restarting ROSbot sensor/firmware snaps..."
      sudo -v
      sudo snap restart husarion-rplidar
      sudo snap restart rosbot
      sudo snap restart husarion-depthai
      echo "[step] Waiting for snap nodes to republish topics..."
      sleep 5
      ;;
    *)
      echo "[step] Snap restart skipped: PROJECT_C_RESTART_SNAPS=$RESTART_SNAPS"
      ;;
  esac
}

wait_for_full_fusion() {
  local attempt
  local check_log

  for attempt in $(seq 1 "$CHECK_ATTEMPTS"); do
    echo "[step] Full-fusion check attempt ${attempt}/${CHECK_ATTEMPTS}..."
    check_log="$(mktemp)"
    if PROJECT_C_REQUIRE_FULL_FUSION=true bash tools/check_project_c_full.sh 2>&1 | tee "$check_log"; then
      rm -f "$check_log"
      return 0
    fi

    if [ "$attempt" -lt "$CHECK_ATTEMPTS" ]; then
      echo "[wait] Full sensor set not ready yet; retrying in ${CHECK_SLEEP_SEC}s..."
      sleep "$CHECK_SLEEP_SEC"
    else
      echo
      echo "[error] Full-fusion topics did not become ready."
      echo "        Last check output was saved at: $check_log"
      echo
      echo "[diag] Snap service status:"
      snap services 2>/dev/null | grep -E 'rosbot|rplidar|depthai|micro|agent' || true
      echo
      echo "[diag] Current range/ToF-like topics:"
      ros2 topic list 2>/dev/null | sort | grep -E 'range|tof|vl53|distance' || true
      echo
      echo "[diag] Current robot topics:"
      ros2 topic list 2>/dev/null | sort | grep -E 'scan|range|tof|vl53|odom|imu|battery|cmd_vel|motor|button|led' || true
      echo
      echo "[diag] Recent rosbot snap logs:"
      sudo snap logs rosbot -n 120 || true
      echo
      echo "[hint] If no /range/* topics appear above, the ROSbot firmware/snap is not publishing ToF."
      echo "       Project C cannot force full-fusion mode until those topics exist."
      return 1
    fi
  done
}

restart_snaps

echo "[step] Building Project C..."
bash tools/build_project_c.sh

wait_for_full_fusion

echo "[step] Launching Project C full-fusion safety run..."
exec bash tools/run_project_c_safety.sh "$@"
