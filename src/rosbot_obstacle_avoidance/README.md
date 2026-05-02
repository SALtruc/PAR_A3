# Project C - Reactive Autonomous Navigation

Reactive obstacle avoidance package for the Husarion ROSbot 3 PRO. It uses no
map, no goal, and no external motion commands: each velocity comes from the
current fused sensor snapshot.

## Nodes

| Node | Role |
|---|---|
| `obstacle_perception` | Fuses S2 LIDAR sectors, OAK-D depth ROIs, and VL53L0X ToF into `/obstacle_representation` JSON |
| `obstacle_avoidance` | Reactive decision policy publishing `/cmd_vel` |
| `obstacle_trial_logger` | CSV logger for collision rate, recovery success, dynamic latency, coverage, and ablation trials |

## Behaviour Policy

| Scenario | Controller behaviour |
|---|---|
| Free roaming | Drive straight while the live sensor snapshot is clear |
| Static obstacle | S2 LIDAR detects spacing range, then the robot stops forward motion and turns toward a clear gap |
| Dynamic obstacle | If depth/dynamic evidence appears while LIDAR is clear, stop first and confirm it over `dynamic_check_frames` |
| Dead end | Stop, choose one turn direction, and keep turning that direction until current sensors show a path |
| Narrow passage | Reduce speed and use left/right centering only when sides are close |
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
| `obstacle_distance` | `0.30` | Body spacing that starts static obstacle avoidance |
| `clear_distance` | `0.40` | Body spacing treated as clear again |
| `slow_distance` | `0.55` | Body spacing where straight driving slows down before avoidance |
| `front_body_offset_m` | `0.10` | Approximate distance from front bumper to the range sensor, subtracted in logs/control |
| `emergency_distance` | `0.12` | Hard stop threshold at the sensor |
| `front_close_min_rays` | `3` | Minimum LIDAR rays needed for a small obstacle cluster |
| `front_close_min_ratio` | `0.01` | Minimum fraction of front rays needed for that cluster |
| `depth_obstacle_distance` | `0.80` | Depth-camera front obstacle threshold |
| `dynamic_closing_speed` | `0.80` | m/s closing rate that marks dynamic obstacle |
| `dynamic_check_frames` | `4` | Stop and confirm a depth/dynamic obstacle for this many control frames |
| `dynamic_clear_frames` | `2` | Resume straight driving after this many clear dynamic-check frames |
| `obstacle_hold_sec` | `0.35` | Keeps obstacle detection latched across brief noisy clear frames |
| `clear_confirm_sec` | `0.20` | Requires a stable clear front sector before leaving obstacle mode |
| `gap_angle_limit_deg` | `110.0` | LIDAR arc searched for navigable gaps |
| `corridor_kp` | `0.14` | Narrow-passage centering strength |
| `turn_out_sec` | `0.90` | Dead-end turn check interval; if still blocked, it keeps turning |
| `side_protect_distance` | `0.30` | Stops forward motion and turns away from a close side wall |
| `turn_direction_hold_sec` | `0.80` | Prevents left/right avoid direction from flipping every scan |
