# dynamic_obstacle_tracker

`dynamic_obstacle_tracker` converts the dynamic point cloud produced by
`dynamic_point_detector` into tracked obstacle predictions for the MPC controller,
smoother, and BT trajectory replan checks when dynamic-obstacle mode is enabled.

## Current Chain

```text
registered_scan + odometry
  -> dynamic_point_detector
  -> dynamic_points
  -> dynamic_obstacle_tracker
  -> dynamic_obstacles
  -> f_mpc_controller / safe_geometric_smoother / ReplanCondition
```

The tracker no longer performs raw point-cloud ground segmentation itself. Ground/static
separation is owned by `dynamic_point_detector`; this package clusters `dynamic_points`,
associates detections across frames, and publishes predicted obstacle positions.
The old standalone DBSCAN/EKF/ground-filter header-only implementation was removed because
it was not built or launched by the current navigation stack.

## Node

`obstacle_tracker_node`

## Topics

| Topic | Type | Direction | Notes |
| --- | --- | --- | --- |
| `dynamic_points` | `sensor_msgs/msg/PointCloud2` | subscribe | Dynamic-only points from M-detector |
| `dynamic_obstacles` | `sentry_nav_interfaces/msg/TrackedObstacleArray` | publish | Tracked obstacle state and prediction |
| `vis/tracked_obstacles` | `visualization_msgs/msg/MarkerArray` | publish | Debug markers |

## Algorithm

1. Convert incoming `PointCloud2` to PCL XYZ points.
2. Extract Euclidean clusters with `cluster_tolerance`, `cluster_min_size`, and
   `cluster_max_size`.
3. Use Hungarian assignment with `match_dist_max` to associate clusters to existing tracks.
4. Track each obstacle with a constant-velocity Kalman filter state `[x, y, vx, vy]`.
5. Publish confirmed tracks after `confirm_frames`, remove stale tracks after
   `max_missed_frames`.
6. Fill `predicted_positions` using `prediction_steps` and `prediction_dt`.

## Parameters

```yaml
dynamic_obstacle_tracker:
  ros__parameters:
    input_topic: dynamic_points
    output_topic: dynamic_obstacles
    viz_topic: vis/tracked_obstacles
    cluster_tolerance: 0.4
    cluster_min_size: 3
    cluster_max_size: 500
    match_dist_max: 2.0
    vel_alpha: 0.4
    max_missed_frames: 5
    confirm_frames: 2
    prediction_steps: 20
    prediction_dt: 0.1
    max_output_obstacles: 7
```

`vel_alpha` is retained in the node parameters for compatibility, but the current Kalman
update path does not use it directly.

## Build

```bash
colcon build --packages-select dynamic_obstacle_tracker
```
