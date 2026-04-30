# Project B - Traffic Light Behaviour

Detects red, yellow, and green traffic lights from the OAK RGB stream and maps
them to robot motion.

| Node | Purpose |
|---|---|
| `traffic_light_detector` | HSV + morphology + circularity + stability filtering, publishes `/traffic_light_state` |
| `traffic_light_controller` | Converts traffic light state to `/cmd_vel` commands |

Behaviour:

| State | Robot action |
|---|---|
| `RED` | Stop |
| `YELLOW` | Slow crawl |
| `GREEN` | Drive forward |
| `UNKNOWN` | Stop safely |

Run on ROSbot:

```bash
ros2 launch rosbot_traffic_light project_b.launch.py \
  image_topic:=/oak/rgb/image_raw \
  cmd_vel_topic:=/cmd_vel
```

Dry test without camera:

```bash
ros2 launch rosbot_traffic_light sim_test.launch.py
```

Demo checklist:

- Test red, yellow, and green under at least two lighting conditions.
- Present a non-traffic-light coloured object; expected state is `UNKNOWN`.
- Keep `/cmd_vel` echo visible to show red/unknown stop, yellow slow, green go.
