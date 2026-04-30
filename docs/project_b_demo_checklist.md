# Project B Demo Checklist

Keep `/traffic_light_state` and `/cmd_vel` visible during the demo.

## Required Scenarios

| Scenario | Expected result |
|---|---|
| Red light | `/traffic_light_state` becomes `RED`; robot stops |
| Yellow light | `/traffic_light_state` becomes `YELLOW`; robot crawls slowly |
| Green light | `/traffic_light_state` becomes `GREEN`; robot drives forward |
| False positive object | Detector publishes `UNKNOWN` or keeps previous safe state; robot does not accelerate |
| Lighting condition 1 | Red/yellow/green all detected consistently |
| Lighting condition 2 | Red/yellow/green all detected consistently after threshold tuning |

## Run

```bash
ros2 launch rosbot_traffic_light project_b.launch.py \
  image_topic:=/oak/rgb/image_raw \
  cmd_vel_topic:=/cmd_vel
```

Dry controller test:

```bash
ros2 launch rosbot_traffic_light sim_test.launch.py
```

## Tuning Order

1. Keep `show_debug:=true` and adjust `min_blob_area` for distance.
2. If coloured posters trigger false positives, increase `min_circularity` and `stable_frames`.
3. If dim lights are missed, lower `min_confidence` slightly before loosening HSV thresholds.
