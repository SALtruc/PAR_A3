# Project C Demo Checklist

Run each scenario for 2-4 trials. For the final autonomous roaming trial, run at
least five minutes without human joystick control.

## Required Scenarios

| Scenario | Expected result | Evidence |
|---|---|---|
| Static obstacle | Robot slows and steers toward the clearest gap | `AVOID` state, no collision event |
| Dynamic obstacle | Person crossing front field triggers a confirmed stop before re-planning | `DYNAMIC_SEEN` then `DYNAMIC_AVOID`, CSV latency |
| Dead end | Robot backs up, turns to clearer side, returns to `DRIVE` | `DEAD_END`, `BACKUP`, `TURN_OUT`, recovery success |
| Narrow passage | Robot slows and centers between walls | lower speed, left/right distances converge |
| Emergency range | Robot hard-stops when ToF/front fused range is too close | `EMERGENCY` state |
| LIDAR-only ablation | Run same scenario with depth disabled | CSV labelled by launch command |
| LIDAR + depth | Run same scenario with depth enabled | Compare dynamic response/collision rate |

## Run

```bash
ros2 launch rosbot_obstacle_avoidance project_c.launch.py \
  scan_topic:=/scan \
  depth_topic:=/camera/depth/image_rect_raw \
  tof_topic:=/range \
  cmd_vel_topic:=/cmd_vel
```

Watch:

```bash
ros2 topic echo /obstacle_representation
ros2 topic echo /obstacle_avoidance_state
ros2 topic echo /obstacle_trial_summary
ros2 topic echo /cmd_vel
```

Mark physical contacts:

```bash
ros2 topic pub --once /collision_event std_msgs/msg/String "{data: collision}"
```

## Ablation Commands

```bash
# LIDAR + ToF only
ros2 launch rosbot_obstacle_avoidance project_c.launch.py use_depth:=false

# LIDAR + depth + ToF
ros2 launch rosbot_obstacle_avoidance project_c.launch.py use_depth:=true
```

## Tuning Order

1. Start with wheels lifted or a clear test area.
2. Tune `emergency_distance` first so the robot never touches obstacles.
3. Tune `obstacle_distance`, `depth_obstacle_distance`, and `avoid_turn_only_distance` for earlier/later avoidance.
4. Tune `corridor_kp` in narrow passages.
5. Tune `dynamic_closing_speed` with a person crossing in front of the robot.
6. Tune `backup_sec` and `turn_out_sec` for dead ends.
