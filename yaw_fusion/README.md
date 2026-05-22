# yaw_fusion

`yaw_fusion` publishes a low-latency yaw estimate on `/Nav_yaw`.

The current implementation does forward prediction from odometry yaw using the latest
gimbal/chassis yaw-rate topic. It does not perform the old TF offset calibration workflow.

## Formula

```text
Nav_yaw = odom_yaw + v_yaw * clamp(now - odom_stamp, 0, max_predict_dt)
```

If yaw-rate input is disabled, stale, not finite, or outside `yaw_rate.max_abs`, the node
publishes odometry yaw without the prediction term.

## Topics

| Topic | Type | Direction | Notes |
| --- | --- | --- | --- |
| `odometry` | `nav_msgs/msg/Odometry` | subscribe | Base yaw and stamp |
| `/serial/v_yaw` | `std_msgs/msg/Float64` | subscribe | Default yaw-rate source |
| `/Nav_yaw` | `std_msgs/msg/Float64` | publish | Absolute global topic consumed by `fake_vel_transform` |
| `yaw_fusion/debug` | `std_msgs/msg/Float64` | publish | Same yaw value, namespaced |
| `yaw_fusion/status` | `std_msgs/msg/String` | publish | `v_yaw_valid` or `v_yaw_invalid_or_stale` |
| `yaw_fusion/odom_latency` | `std_msgs/msg/Float64` | publish | Prediction dt in ms |
| `yaw_fusion/arrow` | `visualization_msgs/msg/Marker` | publish | RViz marker in `odom` |

## Parameters

```yaml
yaw_fusion:
  ros__parameters:
    odom_topic: odometry
    max_predict_dt: 0.15
    yaw_rate:
      enable: true
      topic: /serial/v_yaw
      sign: 1.0
      max_abs: 12.0
      timeout: 0.05
```

Legacy flat names `v_yaw_topic`, `v_yaw_sign`, `v_yaw_max_abs`, and `v_yaw_timeout`
are still accepted when the nested `yaw_rate.*` override is not provided.

## Launch Integration

Reality launch defaults to `use_yaw_fusion:=True`; simulation launch defaults to
`use_yaw_fusion:=False`. In the full navigation chain, `fake_vel_transform.use_nav_yaw`
is enabled in reality so the final velocity transform uses `/Nav_yaw` instead of raw
odometry yaw.

## Build

```bash
colcon build --packages-select yaw_fusion
```
