# Project C Simulation

This folder runs Husarion's official ROSbot Gazebo simulation and launches the
Project C obstacle avoidance stack against it.

The upstream simulation source was downloaded for reference:

```bash
git clone --depth 1 --branch jazzy https://github.com/husarion/rosbot_ros.git external/rosbot_ros_jazzy
```

The relevant simulated topics are:

| Real robot | Gazebo simulation |
|---|---|
| `/scan_filtered` | `/scan_filtered` |
| `/oak/points` | `/oak/stereo/depth/points` |
| `/range/fl`, `/range/fr`, `/range/rl`, `/range/rr` as `Range` | simulated as `LaserScan`, so Project C disables ToF in sim |
| `/cmd_vel` | `/cmd_vel` as `TwistStamped` |

## Run

Start Docker Desktop first. On Linux, allow Docker containers to access the X
server:

```bash
xhost +local:docker
```

Then run:

```bash
docker compose -f sim/compose.project_c_sim.yaml up
```

The first run pulls `husarion/rosbot-gazebo:jazzy`, installs Project C runtime
dependencies in the Project C container, builds the package, and launches:

```bash
ros2 launch rosbot_obstacle_avoidance project_c_safety.launch.py \
  scan_topic:=/scan_filtered \
  pointcloud_topic:=/oak/stereo/depth/points \
  use_tof:=false \
  cmd_vel_topic:=/cmd_vel
```

## Useful Debug Commands

In another terminal:

```bash
docker compose -f sim/compose.project_c_sim.yaml exec project_c bash
source /project_ws/install/setup.bash
ros2 topic list | grep -E "scan|oak|cmd_vel|collision|obstacle"
ros2 topic echo --once /obstacle_representation
ros2 topic echo /collision_monitor_state
```

Stop everything:

```bash
docker compose -f sim/compose.project_c_sim.yaml down
```

If Docker reports that the Linux engine is missing, start Docker Desktop and
wait until it says the engine is running.
