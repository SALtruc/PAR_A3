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
| Emergency range | Robot backs up when front ToF/front fused range is too close | `BACKUP` state |
| LIDAR-only ablation | Run same scenario with depth disabled | CSV labelled by launch command |
| LIDAR + depth | Run same scenario with depth enabled | Compare dynamic response/collision rate |

## Run

Use the repo wrapper on the robot for the live demo. It selects CycloneDDS and
resets the ROS 2 CLI daemon inside the script, so you do not need to export
`RMW_IMPLEMENTATION` manually in your shell:

```bash
cd ~/PAR_A3
bash tools/run_project_c_full.sh
```

Keep the laptop as an SSH client only during the run. Do not run local laptop
`ros2 topic list`, `ros2 topic echo`, RViz, or Foxglove directly on the same
ROS domain during the timed demo; those make the laptop a DDS participant and
can stall discovery or high-bandwidth OAK traffic on the lab network. If you
need a monitor, open a second SSH terminal and run the watch commands on the
robot.

If you must use a manual launch instead of the wrapper, run it on the robot:

```bash
ros2 launch rosbot_obstacle_avoidance project_c_safety.launch.py \
  scan_topic:=/scan_filtered \
  depth_topic:=/camera/depth/image_rect_raw \
  tof_topic:=/range \
  pointcloud_topic:=/oak/points \
  cmd_vel_topic:=/cmd_vel \
  use_nav2_collision_monitor:=true \
  max_speed:=0.10 \
  backup_speed:=0.04 \
  require_battery_ok:=true
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
ros2 launch rosbot_obstacle_avoidance project_c_safety.launch.py \
  use_depth:=false \
  use_pointcloud:=false

# Full fusion: LIDAR + depth image + pointcloud + ToF
ros2 launch rosbot_obstacle_avoidance project_c_safety.launch.py \
  use_depth:=true \
  use_pointcloud:=true \
  use_tof:=true
```

## Tuning Order

1. Start with wheels lifted or a clear test area.
2. Tune `emergency_distance` first so the robot never touches obstacles.
3. Tune `obstacle_distance`, `depth_obstacle_distance`, and `avoid_turn_only_distance` for earlier/later avoidance.
4. Tune `corridor_kp` in narrow passages.
5. Tune `dynamic_closing_speed` with a person crossing in front of the robot.
6. Tune `backup_sec` and `turn_out_sec` for dead ends.
