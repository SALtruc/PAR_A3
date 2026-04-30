# Project A Demo Checklist

Use this checklist before the live demonstration. Save the generated CSV logs
from `~/rosbot_qr_logs` for the report results table.

## Core QR Commands

Run each command at least three times.

| Scenario | Expected result | Evidence |
|---|---|---|
| `GO` from `STOPPED` | Robot enters `DRIVING` and publishes forward velocity | `/fsm_state`, CSV `STATE,DRIVING` |
| `STOP` while driving | Robot stops immediately and stays stopped | CSV `COMMAND,STOP`, `STATE,STOPPED` |
| `TURN_LEFT` | Robot rotates left, then returns to `STOPPED` | CSV `STATE,TURNING`, `STATE,STOPPED` |
| `TURN_RIGHT` | Robot rotates right, then returns to `STOPPED` | CSV `STATE,TURNING`, `STATE,STOPPED` |
| `U_TURN` | Robot rotates 180 degrees, then returns to `STOPPED` | CSV `COMMAND,U_TURN` |
| `SPEED_UP` | Cruise speed increases but stays below `max_speed` | log message `Speed ->` |
| `SPEED_DOWN` | Cruise speed decreases but stays above `min_speed` | log message `Speed ->` |

## Edge Cases

| Scenario | Expected result | Evidence |
|---|---|---|
| Simultaneous QR codes | Higher-priority immediate command wins | command interpreter warning log |
| Invalid/false QR text | Command is ignored; robot state does not change | no `COMMAND` row for invalid text |
| Degraded/tilted QR | Detector recovers with deskew/contrast fallback when readable | `DETECTION,<COMMAND>` row |
| `GO` seen during a turn | Turn finishes first, then robot resumes | `QUEUE,GO`, then `STATE,DRIVING` |
| Front obstacle while driving | FSM enters `AVOIDING`, goes around, then returns to `DRIVING` | `AVOID,*`, `STATE,DRIVING` |
| Obstacle still blocked after avoidance | Robot retries, then stops safely after retry limit | `AVOID,retry_*`, `STATE,STOPPED` |
| ToF emergency stop | Robot hard-stops below `tof_emergency_dist` | `STATE,TOF_EMERGENCY` |

## Suggested Live Run

```bash
bash tools/rosbot_snap_bringup.sh

cd ~/ros2_ws
source install/setup.bash
ros2 launch rosbot_qr_navigation project_a.launch.py \
  image_topic:=/oak/rgb/image_raw \
  cmd_vel_topic:=/cmd_vel \
  scan_topic:=/scan \
  start_state:=STOPPED
```

Keep one terminal open for:

```bash
ros2 topic echo /fsm_state
ros2 topic echo /qr_command
ros2 topic info /cmd_vel
```
