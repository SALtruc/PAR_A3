# Project C - Obstacle Avoidance

Reactive obstacle avoidance package for the Husarion ROSbot 3 PRO.

| Scenario | Controller behaviour |
|---|---|
| Static obstacle | Follow the clearest LIDAR gap |
| Dynamic obstacle | Stop/avoid when the front sector becomes blocked |
| Dead end | Reverse, turn toward the clearer side, and retry |
| Narrow passage | Centre between left/right LIDAR sectors |
| Emergency range | Stop when front LIDAR or ToF is below `emergency_distance` |

Run on ROSbot:

```bash
ros2 launch rosbot_obstacle_avoidance project_c.launch.py \
  scan_topic:=/scan \
  cmd_vel_topic:=/cmd_vel
```

Useful topics:

```bash
ros2 topic echo /obstacle_avoidance_state
ros2 topic echo /cmd_vel
```

Tune these first on the real robot:

| Parameter | Default |
|---|---|
| `max_speed` | `0.18` |
| `obstacle_distance` | `0.55` |
| `emergency_distance` | `0.18` |
| `gap_angle_limit_deg` | `110.0` |
| `corridor_kp` | `0.45` |
