# Project C - Reactive Autonomous Navigation

Reactive obstacle avoidance package for the Husarion ROSbot 3 PRO. It uses no
map, no goal, and no external motion commands: each velocity comes from the
current fused sensor snapshot.

## Nodes

| Node | Role |
|---|---|
| `obstacle_perception` | Fuses S2 LIDAR sectors, OAK-D point cloud/depth, and VL53L0X ToF into `/obstacle_representation` JSON |
| `obstacle_avoidance` | Reactive decision policy publishing `/cmd_vel` or `/cmd_vel_raw` |
| `obstacle_trial_logger` | CSV logger for collision rate, recovery success, dynamic latency, coverage, and ablation trials |
| `collision_monitor` | Optional Nav2 safety shield that filters `/cmd_vel_raw` into `/cmd_vel` |

## Behaviour Policy

| Scenario | Controller behaviour |
|---|---|
| Free roaming | Drive straight while the live sensor snapshot is clear |
| Static obstacle | S2 LIDAR detects spacing range, then the robot stops forward motion and turns toward a clear gap |
| Face-close wall | If static spacing is below `face_wall_distance`, reverse for `backup_sec`, then turn out |
| Dynamic obstacle | If depth/dynamic evidence appears while LIDAR is clear, stop first and confirm it over `dynamic_check_frames` |
| Dead end | Stop, choose one turn direction, and keep turning that direction until current sensors show a path |
| Narrow passage | Reduce speed and use left/right centering only when sides are close |
| Emergency range | Hard stop when ToF/depth or robust LIDAR front control is below `emergency_distance` |

## Run

```bash
ros2 launch rosbot_obstacle_avoidance project_c.launch.py \
  scan_topic:=/scan_filtered \
  pointcloud_topic:=/oak/points \
  cmd_vel_topic:=/cmd_vel
```

Safer run with Nav2 Collision Monitor:

```bash
ros2 launch rosbot_obstacle_avoidance project_c_safety.launch.py \
  scan_topic:=/scan_filtered \
  pointcloud_topic:=/oak/points \
  cmd_vel_topic:=/cmd_vel
```

The safety launch routes the custom controller through `/cmd_vel_raw` first.
Collision Monitor then checks front stop/slow zones using `/scan_filtered` and
`/oak/points`, and publishes the final command to `/cmd_vel`. The safety zones
are velocity-dependent, so a wall in front blocks forward motion but still lets
the controller back away during a face-wall recovery.

Useful topics:

```bash
ros2 topic echo /obstacle_representation
ros2 topic echo /obstacle_avoidance_state
ros2 topic echo /obstacle_trial_summary
ros2 topic echo /cmd_vel
ros2 topic echo /collision_monitor_state
```

Logs are written to `~/rosbot_obstacle_logs/project_c_trial_<timestamp>.csv`.

## Sensor Ablation

LIDAR-only trial:

```bash
ros2 launch rosbot_obstacle_avoidance project_c.launch.py use_depth:=false
```

LIDAR + depth trial:

```bash
ros2 launch rosbot_obstacle_avoidance project_c.launch.py use_pointcloud:=true
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
| `obstacle_distance` | `0.35` | Body spacing from bumper that starts static obstacle avoidance |
| `clear_distance` | `0.50` | Body spacing treated as clear again |
| `slow_distance` | `0.65` | Body spacing where straight driving slows down before avoidance |
| `front_body_offset_m` | `0.11` | Approximate distance from front bumper to the range sensor, subtracted in logs/control |
| `face_wall_distance` | `0.20` | If a static obstacle is this close to the bumper, back up before turning |
| `backup_speed` | `0.08` | Reverse speed during face-wall recovery |
| `backup_sec` | `0.90` | Reverse duration before turning out |
| `backup_rear_stop_distance` | `0.25` | Cancels reverse if the rear is too close to an obstacle |
| `emergency_distance` | `0.18` | Hard stop threshold at the sensor |
| `perception_obstacle_distance` | `0.45` | Raw sensor distance that latches an obstacle in perception |
| `perception_clear_distance` | `0.60` | Raw sensor distance that releases the perception obstacle latch |
| `front_center_angle_deg` | `0.0` | Adjust if the LIDAR front sector is rotated relative to `base_link` |
| `front_close_min_rays` | `3` | Minimum LIDAR rays needed for a small obstacle cluster |
| `front_close_min_ratio` | `0.01` | Minimum fraction of front rays needed for that cluster |
| `depth_obstacle_distance` | `0.55` | OAK point cloud/depth front obstacle threshold |
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
