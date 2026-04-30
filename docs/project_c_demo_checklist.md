# Project C Demo Checklist

Run for at least five minutes without collisions for an Excellent demo.

## Required Scenarios

| Scenario | Expected result |
|---|---|
| Static obstacle | Robot turns toward the clearest gap and passes without contact |
| Dynamic obstacle | Robot stops/turns when a person or object enters the front sector |
| Dead end | Robot backs up, turns toward the clearer side, and retries |
| Narrow passage | Robot centres between left and right walls and continues |
| Emergency range | Robot stops when front LIDAR or ToF is inside `emergency_distance` |

## Run

```bash
ros2 launch rosbot_obstacle_avoidance project_c.launch.py \
  scan_topic:=/scan \
  cmd_vel_topic:=/cmd_vel
```

Watch:

```bash
ros2 topic echo /obstacle_avoidance_state
ros2 topic echo /cmd_vel
```

## Tuning Order

1. Start with wheels lifted or a clear test area.
2. Tune `emergency_distance` first so the robot never touches obstacles.
3. Tune `obstacle_distance` for earlier/later avoidance.
4. Tune `corridor_kp` in narrow passages.
5. Tune `backup_sec` and `turn_out_sec` for dead ends.
