#!/usr/bin/env bash
# Start/restart the ROSbot snap services used by the QR navigation demo, then
# print the key ROS topics so the real robot can be checked before launch.

set -u

WAIT_SEC="${WAIT_SEC:-2}"

if ! command -v snap >/dev/null 2>&1; then
  echo "[error] snap is not installed on this machine."
  exit 1
fi

run_snap_command() {
  local action="$1"
  local snap_name="$2"

  if ! snap list "$snap_name" >/dev/null 2>&1; then
    echo "[skip] snap '$snap_name' is not installed."
    return 0
  fi

  echo "[snap] sudo snap $action $snap_name"
  if sudo snap "$action" "$snap_name"; then
    echo "[ok] $snap_name $action complete"
  else
    echo "[warn] sudo snap $action $snap_name failed; continuing."
  fi
}

run_snap_command start rosbot
run_snap_command restart husarion-depthai
run_snap_command restart husarion-webui

echo
echo "[snap] service status"
for snap_name in rosbot husarion-depthai husarion-webui; do
  if snap list "$snap_name" >/dev/null 2>&1; then
    snap services "$snap_name" 2>/dev/null || true
  fi
done

echo
echo "[ros] waiting ${WAIT_SEC}s for services to republish topics..."
sleep "$WAIT_SEC"

if ! command -v ros2 >/dev/null 2>&1; then
  for distro in "${ROS_DISTRO:-}" jazzy humble iron foxy; do
    if [ -n "$distro" ] && [ -f "/opt/ros/$distro/setup.bash" ]; then
      # shellcheck source=/dev/null
      source "/opt/ros/$distro/setup.bash"
      break
    fi
  done
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[warn] ros2 command not found; skipped topic checks."
  exit 0
fi

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "$HOME/ros2_ws/install/setup.bash"
fi

echo
echo "[ros] package resolution"
if ros2 pkg prefix rosbot_qr_navigation >/tmp/rosbot_qr_navigation_prefix.txt 2>/dev/null; then
  echo "[ok] rosbot_qr_navigation prefix: $(cat /tmp/rosbot_qr_navigation_prefix.txt)"
else
  echo "[warn] rosbot_qr_navigation not found in the sourced ROS environment."
fi

topic_list() {
  if command -v timeout >/dev/null 2>&1; then
    timeout 5 ros2 topic list 2>/dev/null
  else
    ros2 topic list 2>/dev/null
  fi
}

topics="$(topic_list || true)"

check_topic() {
  local topic="$1"
  if printf '%s\n' "$topics" | grep -Fx "$topic" >/dev/null; then
    echo "[ok] topic present: $topic"
  else
    echo "[warn] topic missing: $topic"
  fi
}

check_topic /cmd_vel
check_topic /scan
check_topic /oak/rgb/image_raw

echo
echo "[ros] /cmd_vel info"
ros2 topic info /cmd_vel 2>/dev/null || echo "[warn] /cmd_vel info unavailable"
