# Project A – QR Code Command Navigation
**Programming Autonomous Robots | RMIT University**

Autonomous navigation system for the Husarion ROSbot 3 PRO that reads QR codes
mounted in the environment and executes the encoded navigation commands in real time.

---

## Package Layout

```
src/
├── rosbot_qr_navigation/          # Project A + obstacle avoidance add-on
│   ├── rosbot_qr_navigation/
│   │   ├── qr_detector_node.py
│   │   ├── command_interpreter_node.py
│   │   ├── navigation_fsm_node.py
│   │   └── event_logger_node.py
│   ├── launch/project_a.launch.py
│   └── config/params.yaml
├── rosbot_traffic_light/          # Project B traffic light behaviour
│   ├── rosbot_traffic_light/
│   │   └── traffic_light_detector_node.py
│   ├── launch/project_b.launch.py
│   └── config/params.yaml
└── rosbot_obstacle_avoidance/     # Project C obstacle avoidance
    ├── rosbot_obstacle_avoidance/
    │   ├── obstacle_perception_node.py
    │   ├── obstacle_avoidance_node.py
    │   └── obstacle_trial_logger_node.py
    ├── launch/project_c_safety.launch.py
    └── config/params.yaml
tools/
├── generate_qr_codes.py           # prints QR PNGs for all commands
└── rosbot_snap_bringup.sh         # starts ROSbot snaps and checks topics
```

## Supported Commands

| QR Code Content | Robot Behaviour |
|---|---|
| `TURN_LEFT`  | 90° left turn, then stop |
| `TURN_RIGHT` | 90° right turn, then stop |
| `STOP`       | Halt and wait for GO |
| `GO`         | Resume driving |
| `SPEED_UP`   | +0.05 m/s cruising speed |
| `SPEED_DOWN` | −0.05 m/s cruising speed |
| `U_TURN`     | 180° turn, then stop |

Any command can be prefixed with `AND_` to queue it instead of interrupting the
current action. Example: `AND_TURN_LEFT`, `AND_U_TURN`, `AND_SPEED_UP`.
Immediate commands still interrupt whatever is running; queued commands run FIFO
after the current turn/avoidance action finishes.

Turn commands always finish in `STOPPED`. If `GO` is received during a turn, it
waits until the turn finishes; the FSM publishes a stop command first, then
resumes driving on the next control tick.

The FSM also includes a Project C-style obstacle safety add-on. By default,
front obstacles detected by LIDAR or the depth camera stop the robot instead of
running a blind timed side-step. Timed side-step avoidance can still be enabled
with `obstacle_stop_only:=false` for controlled tests. For QR-only demos, disable
the obstacle layer with `obstacle_safety_enabled:=false`.

Supported queued QR contents are:

| QR Code Content | Robot Behaviour |
|---|---|
| `AND_TURN_LEFT` | Queue a 90° left turn |
| `AND_TURN_RIGHT` | Queue a 90° right turn |
| `AND_STOP` | Queue a stop |
| `AND_GO` | Queue resume driving, with obstacle check |
| `AND_SPEED_UP` | Queue speed increase |
| `AND_SPEED_DOWN` | Queue speed decrease |
| `AND_U_TURN` | Queue a 180° turn |

## Quick Start (ROSbot 3 PRO)

```bash
# 1. Clone into your ROS 2 workspace
cd ~/ros2_ws/src
git clone <this-repo>

# 2. Install Python deps
pip install opencv-python zxingcpp

# 3. Build
cd ~/ros2_ws
colcon build --packages-select rosbot_qr_navigation
source install/setup.bash

# 4. Run (single command)
ros2 launch rosbot_qr_navigation project_a.launch.py
```

## Deploy to the Husarion ROSbot

Use this when you want to run the QR navigation package on the real ROSbot
instead of the local simulator.

For the live-demo rubric scenarios, use:

- `docs/project_a_demo_checklist.md`
- `docs/project_b_demo_checklist.md`
- `docs/project_c_demo_checklist.md`

### 1. Connect your laptop and robot

Put your laptop and the ROSbot on the same network. The usual options are:

- Connect both devices to the same Wi-Fi router.
- Connect your laptop to the ROSbot hotspot, if your ROSbot image exposes one.
- Use Ethernet directly or through a router.

Find the robot IP address from the robot screen/router page, or from a terminal
on the robot:

```bash
hostname -I
```

From your laptop, confirm the robot is reachable:

```bash
ping <robot-ip>
```

Then SSH into the robot. Replace the username if your ROSbot image uses a
different account.

```bash
ssh husarion@<robot-ip>
```

### 2. Prepare a ROS 2 workspace on the robot

On the robot through SSH:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <repo-url> rosbot_qr_navigation_project
```

If you are copying from this folder instead of GitHub, use `scp` from your
laptop:

```bash
scp -r /path/to/PAR_A3 husarion@<robot-ip>:~/ros2_ws/src/rosbot_qr_navigation_project
```

The package folder must end up at:

```text
~/ros2_ws/src/rosbot_qr_navigation_project/src/rosbot_qr_navigation
```

### 3. Install dependencies

On the robot:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
sudo apt update
sudo apt install -y python3-pip ros-$ROS_DISTRO-cv-bridge
pip3 install opencv-python zxingcpp "qrcode[pil]"
```

If `opencv-python` conflicts with the robot image, remove it and use the Ubuntu
OpenCV package instead:

```bash
sudo apt install -y python3-opencv
```

### 4. Build and source

Because this repository has a workspace-like layout, build from `~/ros2_ws` and
point colcon at the nested package:

```bash
cd ~/ros2_ws
colcon build --symlink-install --base-paths src/rosbot_qr_navigation_project/src \
  --packages-select rosbot_qr_navigation
source install/setup.bash
```

Optional: add sourcing to `.bashrc` so new terminals know about the package:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 5. Start robot snap services

On the robot, restart the ROSbot services before launching:

```bash
cd ~/ros2_ws/src/rosbot_qr_navigation_project
bash tools/rosbot_snap_bringup.sh
```

The script runs:

```bash
sudo snap start rosbot
sudo snap restart husarion-depthai
sudo snap restart husarion-webui
```

It also prints `/cmd_vel`, `/scan`, and `/oak/rgb/image_raw` status.

### 6. Check robot topics

Before running autonomous motion, check the real topic names:

```bash
ros2 topic list
```

Common expected topics:

| Purpose | Expected topic | How to check |
|---|---|---|
| Camera image | `/oak/rgb/image_raw` | `ros2 topic echo --once /oak/rgb/image_raw/header` |
| Velocity command | `/cmd_vel` | `ros2 topic info /cmd_vel` |
| Laser scan | `/scan` | `ros2 topic echo --once /scan/header` |

By default this package publishes `/cmd_vel` as `geometry_msgs/msg/TwistStamped`,
which matches the current ROSbot snap stack. If your robot reports
`geometry_msgs/msg/Twist` instead, launch with `cmd_vel_stamped:=false`.

If your robot uses different names, pass them into the launch file:

```bash
ros2 launch rosbot_qr_navigation project_a.launch.py \
  image_topic:=<camera-topic> \
  cmd_vel_topic:=<cmd-vel-topic> \
  scan_topic:=<scan-topic>
```

### 7. Run a safe dry test first

Use a dummy velocity topic so the robot will not move yet:

```bash
ros2 launch rosbot_qr_navigation project_a.launch.py \
  cmd_vel_topic:=/dummy_cmd_vel \
  start_state:=STOPPED
```

In another SSH terminal, verify outputs:

```bash
ros2 topic echo /qr_detected
ros2 topic echo /qr_command
ros2 topic echo /fsm_state
ros2 topic echo /dummy_cmd_vel
```

Show QR codes to the camera. If detections appear and `/dummy_cmd_vel` changes,
the perception-to-action pipeline is working.

### 8. Run on the real robot

Clear the area, lift the robot wheels or keep an emergency stop ready for the
first run, then launch with the real velocity topic:

```bash
ros2 launch rosbot_qr_navigation project_a.launch.py \
  image_topic:=/oak/rgb/image_raw \
  cmd_vel_topic:=/cmd_vel \
  cmd_vel_stamped:=true \
  scan_topic:=/scan \
  start_state:=STOPPED
```

Use a `GO` QR code to start. Immediate commands such as `STOP` interrupt the
current action. Queued commands such as `AND_TURN_LEFT` are added to the wait
list and run after the current turn or avoidance action completes.

### 9. Collect logs for the report

CSV logs are written on the robot:

```bash
ls ~/rosbot_qr_logs
```

Copy logs back to your laptop:

```bash
scp husarion@<robot-ip>:~/rosbot_qr_logs/*.csv ./rosbot_qr_logs/
```

Use these logs for detection accuracy, command execution accuracy, and failure
analysis.

### 10. Common issues

If SSH fails, check that the laptop and robot are on the same network and that
`ping <robot-ip>` works.

If the package is not found, run:

```bash
source ~/ros2_ws/install/setup.bash
ros2 pkg list | grep rosbot_qr_navigation
```

If no QR detections appear, confirm the camera topic and try:

```bash
ros2 launch rosbot_qr_navigation project_a.launch.py show_debug:=true
```

If the robot does not move, confirm the motor controller listens to the same
velocity topic and message type:

```bash
ros2 topic info /cmd_vel
ros2 topic echo /cmd_vel
```

If `/cmd_vel` reports `geometry_msgs/msg/Twist`, add `cmd_vel_stamped:=false` to
the launch command. If it reports `geometry_msgs/msg/TwistStamped`, keep the
default `cmd_vel_stamped:=true`.

If obstacle safety never triggers, confirm `/scan` is publishing and tune
`obstacle_distance` in `config/params.yaml`. If it reacts too early, reduce
`obstacle_distance`, increase `obstacle_confirm_sec` or `obstacle_min_points`,
or reduce `depth_obstacle_dist`; if it gets too close, increase
them slightly.

### Test without a robot (webcam)

```bash
# Terminal 1 – stream webcam as ROS image
ros2 run image_tools cam2image --ros-args -p device_id:=0

# Terminal 2 – launch with webcam topic
ros2 launch rosbot_qr_navigation project_a.launch.py \
    image_topic:=/image \
    cmd_vel_topic:=/dummy_cmd_vel \
    show_debug:=true \
    start_state:=STOPPED
```

### Test FSM without camera or robot

This launch file sends fake QR detections through the same command pipeline and
prints the resulting FSM states, events, and `/cmd_vel` values:

```bash
ros2 launch rosbot_qr_navigation sim_test.launch.py
```

Custom script format is `<seconds>:<COMMAND>` separated by commas:

```bash
ros2 launch rosbot_qr_navigation sim_test.launch.py \
    script:="1.0:GO,2.0:TURN_LEFT,2.5:AND_TURN_RIGHT,3.0:AND_U_TURN" \
    stop_after_sec:=12.0
```

To test obstacle avoidance, the sim launch publishes a fake `/scan`. By default
an obstacle exists from 0.5s to 2.5s, so the first `GO` should enter `AVOIDING`
before returning to `DRIVING`.

To test the continuous-while-driving branch, move the fake obstacle later:

```bash
ros2 launch rosbot_qr_navigation sim_test.launch.py \
    script:="1.0:GO" \
    obstacle_start_sec:=4.0 \
    obstacle_end_sec:=7.0 \
    stop_after_sec:=14.0
```

### Generate printable QR codes

```bash
pip install "qrcode[pil]"
python tools/generate_qr_codes.py --out qr_codes/
# → prints qr_codes/TURN_LEFT.png, STOP.png, etc.
```

## Configuration

All tunable parameters are in `config/params.yaml`.
Key values to calibrate on the real robot:

| Parameter | Default | Notes |
|---|---|---|
| `cruise_speed` | 0.20 m/s | Forward driving speed |
| `turn_90_sec`  | 3.14 s   | **Calibrate** by timing a 90° turn |
| `turn_180_sec` | 6.28 s   | **Calibrate** by timing a 180° turn |
| `stop_after_turn` | true | Compatibility option; QR turn commands always stop after rotating |
| `debounce_sec` | 2.0 s    | Suppresses duplicate QR re-triggers |
| `recovery_sec` | 10.0 s   | Seconds without QR before RECOVERING |
| `obstacle_distance` | 0.30 m | Driving/GO reacts if several front `/scan` rays stay closer than this |
| `obstacle_confirm_sec` | 0.35 s | Close readings must persist before obstacle safety reacts |
| `obstacle_min_points` | 5 | Minimum close LIDAR rays before obstacle safety reacts |
| `obstacle_safety_enabled` | true | Enable LIDAR/depth obstacle safety layer |
| `obstacle_stop_only` | true | Stop safely instead of running timed side-step avoidance |
| `avoid_forward_sec` | 1.5 s | Timed side-step outward when `obstacle_stop_only=false` |
| `avoid_pass_sec` | 1.2 s | Timed forward motion after re-aligning past the obstacle |
| `avoid_return_sec` | 1.5 s | Timed side-step back toward the original path |
| `avoid_return_to_path` | true | Merge back after the side-step when `obstacle_stop_only=false` |
| `continuous_obstacle_avoidance` | true | React to obstacles automatically while driving |
| `avoid_side_sector_deg` | 70.0° | LIDAR side sector used to choose the clearer avoidance side |
| `avoid_retry_limit` | 3 | Stop safely after repeated failed avoidance attempts |
| `sensor_stale_sec` | 1.0 s | Ignore old scan/depth readings and keep ToF emergency active until a fresh clear reading arrives |
| `depth_obstacle_dist` | 0.50 m | Depth-camera front obstacle threshold |
| `cmd_vel_stamped` | true | Publish `geometry_msgs/msg/TwistStamped` instead of `Twist` |

## Topics

| Topic | Type | Description |
|---|---|---|
| `/qr_detected` | `std_msgs/String` | Raw decoded command from detector |
| `/qr_command`  | `std_msgs/String` | Priority-resolved command to FSM |
| `/qr_event`    | `std_msgs/String` | Timestamped events for logger |
| `/fsm_state`   | `std_msgs/String` | Current FSM state |
| `/cmd_vel`     | `geometry_msgs/msg/TwistStamped` | Velocity commands to ROSbot |
| `/scan`        | `sensor_msgs/LaserScan` | Obstacle input used by `GO` avoidance |

## Event Logs

CSV files are written to `~/rosbot_qr_logs/qr_events_<TIMESTAMP>.csv` with columns:

```
wall_clock_iso, ros_time, event_type, value
```

Rows use `DETECTION`, `COMMAND`, `STATE`, `QUEUE`, `DEQUEUE`, and `AVOID` event
types, so the same log can support detection accuracy and command execution
metrics required by the rubric.

## Notes on Turn Calibration

The default `turn_90_sec = π / 0.5 rad/s ≈ 3.14 s` assumes the angular velocity
in `params.yaml` matches the actual robot rotation. On the real ROSbot, run:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
    "{header: {frame_id: base_link}, twist: {linear: {x: 0.0}, angular: {z: 0.5}}}" \
    --rate 10
```

Time how long it takes to complete exactly 90°, then set `turn_90_sec` accordingly.
For a more robust solution, replace the timer with odometry/IMU yaw feedback by
subscribing to `/odom` and comparing `current_yaw` vs `target_yaw`.
