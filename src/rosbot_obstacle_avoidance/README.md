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
| Dynamic obstacle | Detect fast closing front range and enter `DYNAMIC_AVOID` by stopping/turning toward the best gap |
| Dead end | Reverse, turn toward the clearer side, and retry |
| Narrow passage | Reduce speed and use left/right corridor centering |
| Emergency range | Hard stop when fused front range or ToF is below `emergency_distance` |

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
| `max_speed` | `0.18` | Roaming speed in clear space |
| `obstacle_distance` | `0.55` | Distance that starts avoidance |
| `clear_distance` | `0.75` | Distance treated as comfortably clear |
| `emergency_distance` | `0.18` | Hard stop threshold |
| `depth_obstacle_distance` | `0.65` | Depth-camera front obstacle threshold |
| `dynamic_closing_speed` | `0.25` | m/s closing rate that marks dynamic obstacle |
| `gap_angle_limit_deg` | `110.0` | LIDAR arc searched for navigable gaps |
| `corridor_kp` | `0.45` | Narrow-passage centering strength |
| `backup_sec` | `0.9` | Reverse duration for dead-end recovery |
| `turn_out_sec` | `1.8` | Turn duration after backing out |
| `wander_interval_sec` | `4.0` | How often open-space random walk changes steering bias |
