#!/usr/bin/env bash
# Start/restart the ROSbot snap services, then verify the key sensor topics
# are alive before launching any project.
#
# Usage:
#   bash tools/rosbot_snap_bringup.sh          # restart all snaps, then check
#   WAIT_SEC=15 bash tools/rosbot_snap_bringup.sh

set -u

WAIT_SEC="${WAIT_SEC:-12}"

# ── snap helpers ───────────────────────────────────────────────────────────────

if ! command -v snap >/dev/null 2>&1; then
  echo "[error] snap is not installed on this machine."
  exit 1
fi

run_snap_command() {
  local action="$1" snap_name="$2"
  if ! snap list "$snap_name" >/dev/null 2>&1; then
    echo "[skip] snap '$snap_name' is not installed"
    return 0
  fi
  echo "[snap] sudo snap $action $snap_name"
  if sudo snap "$action" "$snap_name"; then
    echo "[ok]   $snap_name ${action}d"
  else
    echo "[warn] sudo snap $action $snap_name failed; continuing"
  fi
}

sudo -v  # cache credentials

# rosbot first — it resets STM32 over GPIO so micro-ROS can reconnect.
run_snap_command restart rosbot
echo "[wait] 10 s for STM32 firmware to boot and micro-ROS to connect..."
sleep 10

run_snap_command restart husarion-rplidar
run_snap_command restart husarion-depthai

echo "[wait] ${WAIT_SEC}s for sensor topics to stabilise..."
sleep "$WAIT_SEC"

# ── snap service status ────────────────────────────────────────────────────────

echo
echo "[snap] service status:"
for sn in rosbot husarion-rplidar husarion-depthai; do
  if snap list "$sn" >/dev/null 2>&1; then
    snap services "$sn" 2>/dev/null || true
  fi
done

# ── ROS topic check ────────────────────────────────────────────────────────────

if ! command -v ros2 >/dev/null 2>&1; then
  for distro in "${ROS_DISTRO:-}" jazzy humble; do
    if [ -n "$distro" ] && [ -f "/opt/ros/$distro/setup.bash" ]; then
      # shellcheck source=/dev/null
      source "/opt/ros/$distro/setup.bash"
      break
    fi
  done
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[warn] ros2 not found after sourcing ROS — skipping topic checks"
  exit 0
fi

topics="$(timeout 8 ros2 topic list 2>/dev/null || true)"

check_topic() {
  local topic="$1"
  if printf '%s\n' "$topics" | grep -Fx "$topic" >/dev/null; then
    echo "[ok]      $topic"
  else
    echo "[missing] $topic"
  fi
}

echo
echo "[ros] core sensor topics:"
check_topic /scan_filtered
check_topic /oak/points
check_topic /range/fl
check_topic /range/fr
check_topic /range/rl
check_topic /range/rr
check_topic /rosbot_base_controller/odom
check_topic /imu_broadcaster/imu

echo
echo "[ros] all range/ToF topics found:"
printf '%s\n' "$topics" | grep -E 'range|tof|vl53|distance' || echo "  (none — run: sudo rosbot.flash)"

echo
echo "[hint] If /range/* are missing:  sudo rosbot.flash && sudo snap restart rosbot"
echo "[hint] To build and run:          bash tools/run_project_c_full.sh"
