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
| Free roaming | Drive straight. No gap-following and no corridor-centering by default |
| Suspicious front obstacle | Stop/slow to observe for a few frames before deciding |
| LIDAR near, OAK clear | Observe briefly, then continue straight instead of rotating immediately |
| Confirmed obstacle | Dodge gently toward the clearer side for a limited step, then drive straight in the new heading |
| Too close | If front is under `stop_distance`, back up before checking direction |
| Dead end | Front blocked and both body sides lack clearance: back up, then rotate to search |
| Side scrape risk | Only when side body clearance is very small, rotate away briefly |
| ToF emergency | Front virtual bumper interrupts the policy and starts backup recovery |
| IMU tilt risk | Pause briefly, back up, then rotate/search for a safer heading |

## Run

```bash
ros2 launch rosbot_obstacle_avoidance project_c_safety.launch.py \
  scan_topic:=/scan_filtered \
  depth_topic:=/camera/depth/image_rect_raw \
  pointcloud_topic:=/oak/points \
  cmd_vel_topic:=/cmd_vel \
  use_nav2_collision_monitor:=true \
  max_speed:=0.10 \
  backup_speed:=0.04 \
  require_battery_ok:=true
```

Use this as the real-robot safety run after confirming `/battery`,
`/scan_filtered`, `/camera/depth/image_rect_raw`, `/oak/points`, and the front
ToF topics are publishing. The controller publishes to `/cmd_vel_raw`; Nav2
Collision Monitor checks stop/slow zones and publishes the final command to
`/cmd_vel`.

If Nav2 Collision Monitor is not installed or does not start on the robot, use
the same safety launch without the Nav2 layer:

```bash
ros2 launch rosbot_obstacle_avoidance project_c_safety.launch.py \
  scan_topic:=/scan_filtered \
  depth_topic:=/camera/depth/image_rect_raw \
  pointcloud_topic:=/oak/points \
  cmd_vel_topic:=/cmd_vel \
  max_speed:=0.06 \
  backup_speed:=0.04 \
  use_nav2_collision_monitor:=false
```

This package keeps only `project_c_safety.launch.py` as the Project C launch
entrypoint.

The default configuration matches the report methodology: S2 LIDAR, OAK depth
image/point cloud, and front ToF are fused into `/obstacle_representation`.
Front ToF acts as a virtual bumper: a hard front reading triggers backup
recovery. If the IMU reports a severe tilt, the robot pauses briefly, backs up,
then rotates to recover.

Useful topics:

```bash
ros2 topic echo /obstacle_representation
ros2 topic echo /obstacle_avoidance_state
ros2 topic echo /obstacle_trial_summary
ros2 topic echo /cmd_vel
ros2 topic echo /collision_monitor_state
```

Logs are written to `~/rosbot_obstacle_logs/project_c_trial_<timestamp>.csv`.
The `[NAV]` console line also reports the OAK depth-image low ROI as
`depth_img=ok img_front=... low=... low_dist=...`. This is separate from
`oak_low=...`, which is pointcloud-only, so low obstacles such as feet can
still be documented when `/oak/points` is visible but not publishing samples.
The CSV includes the same depth evidence in `depth_image_front_min`,
`depth_image_low_front_min`, `depth_image_available`, and pointcloud diagnostic
columns.

## Pre-Run Safety Check

Do this before a full 5 minute trial on the real robot:

```bash
ros2 topic echo --once /battery
ros2 topic hz /scan_filtered
ros2 topic hz /camera/depth/image_rect_raw
ros2 topic hz /oak/points
ros2 topic echo --once /range/fl
ros2 topic echo --once /range/fr
ros2 topic echo --once /obstacle_representation
```

If the OAK pointcloud topic is named differently on the robot, find it with
`ros2 topic list -t | grep PointCloud2` and run with
`POINTCLOUD_TOPIC=/actual/points`. The safety runner disables OAK pointcloud
for that run when no `PointCloud2` topic is visible; `run_project_c_full.sh`
keeps treating it as a required full-fusion input.

OAK depth image is checked separately from pointcloud. `run_project_c_safety.sh`
uses `USE_DEPTH=auto` by default and will enable an active depth `Image` topic
when one is publishing, even if `/oak/points` is not usable. Force a known depth
topic with `DEPTH_TOPIC=/actual/depth USE_DEPTH=true`.

For a clean OAK-only diagnosis, run:

```bash
bash tools/oak_depth_doctor.sh
```

The doctor checks the Husarion depthai snap config, looks for Luxonis USB udev
rules, lists OAK depth/pointcloud topics, and verifies real message flow with
`ros2 topic hz` instead of `ros2 topic echo` so large Image/PointCloud messages
do not trip the ROS CLI. It prints the exact `DEPTH_TOPIC=...` or
`POINTCLOUD_TOPIC=...` command to use for Project C.

If the built-in depthai snap is not publishing `/oak/points`, start the
official driver in a separate terminal:

```bash
bash tools/start_oak_pointcloud.sh
```

The helper auto-detects the available DepthAI launch file. On older driver
packages this is usually `camera.launch.py`; on newer packages it can be
`driver.launch.py`. For those launch files it passes `rs_compat:=true` and
`pointcloud.enable:=true`. For `depthai_ros_driver_v3`, it also requests an
`RGBD` pipeline and publishes the stereo stream, then waits for a real
`PointCloud2` rate before printing the Project C command. To compare DepthAI
example launches, set
`DEPTHAI_LAUNCH=pointcloud.launch.py` or `DEPTHAI_LAUNCH=rgbd_pcl.launch.py`.
It also prefers `depthai_ros_driver_v3` when installed, falling back to the
older `depthai_ros_driver` package.

If the camera is already owned by the Husarion depthai snap, stop that snap for
the session:

```bash
PROJECT_C_STOP_DEPTHAI_SNAP=true bash tools/start_oak_pointcloud.sh
```

Do not run the motors if `/battery` is physically unsafe for the pack. The
controller can hold `EMERGENCY` and publish zero velocity when
`require_battery_ok:=true` and the battery topic is missing, stale, or below
`min_battery_voltage`.

## Sensor Ablation

LIDAR-only trial:

```bash
ros2 launch rosbot_obstacle_avoidance project_c_safety.launch.py \
  use_depth:=false \
  use_pointcloud:=false
```

Full fusion trial:

```bash
ros2 launch rosbot_obstacle_avoidance project_c_safety.launch.py \
  use_depth:=true \
  use_pointcloud:=true \
  use_tof:=true
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
| `max_speed` | `0.20` | Roaming speed in clear space |
| `clear_distance` | `0.25` | Front path distance that starts observe for static obstacles |
| `stop_distance` | `0.25` | Confirmed static obstacle distance that starts backup/dodge |
| `low_obstacle_distance` | `0.30` | OAK low-view distance that starts avoidance for low objects without long-range false positives |
| `low_obstacle_backup_distance` | `0.20` | OAK low-view distance that backs up before the chassis rides over a low object |
| `low_obstacle_min_points` | `8` | Minimum low-region pointcloud points before low-object avoidance is trusted |
| `low_obstacle_hold_sec` | `0.70` | Keeps a recent low-object hit alive across brief pointcloud dropouts |
| `dodge_clearance` | `0.45` | Required side clearance for dodge/static recovery |
| `observe_frames` | `8` | Frames to observe before dodging |
| `clear_observe_frames` | `3` | Frames to verify LIDAR/depth disagreement before driving straight |
| `dodge_step_deg` | `60.0` | Dodge arc limit after obstacle confirmation |
| `robot_half_width_m` | `0.13` | Half-width of the ROSbot body plus a small margin for side clearance |
| `front_path_half_width_m` | `0.14` | LIDAR path corridor checked in front of the full robot width |
| `side_guard_forward_m` | `0.35` | Forward extent of side-edge collision checking |
| `side_guard_rear_m` | `0.20` | Rear extent of side-edge collision checking |
| `side_percentile` | `10.0` | Robust percentile used for side-edge clearance |
| `backup_speed` | `0.06` | Reverse speed during too-close/dead-end recovery |
| `backup_sec` | `1.00` | Reverse duration before rotate/search |
| `rear_stop_distance` | `0.20` | Cancels reverse if the rear is too close |
| `emergency_distance` | `0.14` | Perception emergency threshold |
| `perception_obstacle_distance` | `0.25` | Raw LIDAR distance that marks an obstacle |
| `perception_clear_distance` | `0.25` | Raw distance that releases perception block |
| `front_center_angle_deg` | `0.0` | Adjust if the LIDAR front sector is rotated relative to `base_link` |
| `front_close_min_rays` | `3` | Minimum LIDAR rays needed for a small obstacle cluster |
| `front_close_min_ratio` | `0.01` | Minimum fraction of front rays needed for that cluster |
| `depth_obstacle_distance` | `0.45` | OAK point cloud/depth front obstacle threshold |
| `dynamic_closing_speed` | `0.80` | m/s closing rate that marks dynamic obstacle |
| `dynamic_observe_distance` | `1.00` | Dynamic front evidence inside this range triggers observe |
| `obstacle_hold_sec` | `0.30` | Keeps obstacle detection latched across brief noisy clear frames |
| `clear_confirm_sec` | `0.20` | Requires a stable clear front sector before leaving obstacle mode |
| `gap_angle_limit_deg` | `110.0` | LIDAR arc searched for navigable gaps |
| `side_guard_distance` | `0.08` | Side clearance that triggers side escape before scraping a doorway |
| `side_escape_distance` | `0.08` | Side clearance needed to keep side escape active |
| `contact_stall_sec` | `5.0` | Forward command plus near-zero odom for this long triggers backup then rotate |
| `require_battery_ok` | `false` | When true, requires a fresh battery reading before motion |
| `min_battery_voltage` | `8.5` | Holds zero velocity below this pack voltage when `require_battery_ok` is true |
| `warn_battery_voltage` | `9.0` | Logs a low-battery warning but still allows motion |
