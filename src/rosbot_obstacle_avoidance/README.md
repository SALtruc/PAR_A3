# Project C - Reactive Autonomous Navigation

Reactive obstacle avoidance package for the Husarion ROSbot 3 PRO. It uses no
map and no goal: every command comes from the current fused sensor snapshot.

## Nodes

| Node | Role |
|---|---|
| `obstacle_perception` | Fuses S2 LIDAR sectors, OAK-D depth ROIs, and VL53L0X ToF into `/obstacle_representation` JSON |
| `obstacle_avoidance` | Reactive decision policy publishing `/cmd_vel` |
| `obstacle_trial_logger` | CSV logger for collision rate, recovery success, dynamic latency, coverage, and ablation trials |

## Behaviour Policy

| Scenario | Controller behaviour |
|---|---|
| Free roaming | Drive toward the clearest local gap with a small random-walk bias in open space |
| Static obstacle | Slow and steer toward the best fused gap |
| Dynamic obstacle | Confirm fast closing front range, enter `DYNAMIC_AVOID`, and hold a stop before re-planning |
| Dead end | Reverse, turn toward the clearer side, and retry |
| Narrow passage | Reduce speed and use left/right corridor centering |
| Emergency range | Hard stop when ToF/depth or robust LIDAR front control is below `emergency_distance` |

## Run

```bash
ros2 launch rosbot_obstacle_avoidance project_c.launch.py \
  scan_topic:=/scan \
  depth_topic:=/camera/depth/image_rect_raw \
  tof_topic:=/range \
  cmd_vel_topic:=/cmd_vel
```

Useful topics:

```bash
ros2 topic echo /obstacle_representation
ros2 topic echo /obstacle_avoidance_state
ros2 topic echo /obstacle_trial_summary
ros2 topic echo /cmd_vel
```

Logs are written to `~/rosbot_obstacle_logs/project_c_trial_<timestamp>.csv`.

## Sensor Ablation

LIDAR-only trial:

```bash
ros2 launch rosbot_obstacle_avoidance project_c.launch.py use_depth:=false
```

LIDAR + depth trial:

```bash
ros2 launch rosbot_obstacle_avoidance project_c.launch.py use_depth:=true
```

Compare the CSV files for collision rate, dynamic response latency, recovery
success rate, and coverage.

## Collision Events

If a physical contact happens during a trial, mark it manually so the CSV can
compute collision rate:

```bash
ros2 topic pub --once /collision_event std_msgs/msg/String "{data: collision}"
```

## Tuning

| Parameter | Default | Notes |
|---|---|---|
| `max_speed` | `0.10` | Roaming speed in clear space |
| `obstacle_distance` | `0.55` | Distance that starts avoidance |
| `clear_distance` | `0.80` | Distance treated as comfortably clear |
| `emergency_distance` | `0.25` | Hard stop threshold |
| `depth_obstacle_distance` | `0.80` | Depth-camera front obstacle threshold |
| `dynamic_closing_speed` | `0.80` | m/s closing rate that marks dynamic obstacle |
| `dynamic_hold_sec` | `0.80` | Stop hold after dynamic obstacle confirmation |
| `obstacle_hold_sec` | `0.35` | Keeps obstacle detection latched across brief noisy clear frames |
| `clear_confirm_sec` | `0.20` | Requires a stable clear front sector before leaving obstacle mode |
| `avoid_turn_only_distance` | `0.45` | Rotate in place instead of creeping forward below this range |
| `avoid_forward_distance` | `0.80` | Front range required before slow forward motion in `AVOID` |
| `gap_angle_limit_deg` | `110.0` | LIDAR arc searched for navigable gaps |
| `corridor_kp` | `0.18` | Narrow-passage centering strength |
| `backup_sec` | `0.45` | Reverse duration for dead-end recovery |
| `turn_out_sec` | `1.2` | Turn duration after backing out |
| `wander_interval_sec` | `4.0` | How often open-space random walk changes steering bias |
