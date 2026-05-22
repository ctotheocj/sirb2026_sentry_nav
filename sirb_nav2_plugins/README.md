# sirb_nav2_plugins

`sirb_nav2_plugins` contains the Navigation2 extensions used by this workspace:

- behavior plugin: `pb_nav2_behaviors/BackUpFreeSpace`
- costmap layers: `pb_nav2_costmap_2d::IntensityVoxelLayer` and
  `pb_nav2_costmap_2d::OccupancyGridObstacleLayer`
- behavior-tree nodes exported through `sirb_nav2_plugins_nodes`

The current `sirb2026_nav_bringup` configuration uses `BackUpFreeSpace`,
`OccupancyGridObstacleLayer`, and the BT node library. `IntensityVoxelLayer` is still built
and exported, but it is not part of the default navigation YAML.

## BackUpFreeSpace

`BackUpFreeSpace` replaces Nav2's default backup behavior with an omnidirectional escape
behavior. It queries a costmap service, samples directions around the robot, selects the
widest free angular sector, and publishes a backup velocity toward that sector.

Runtime parameters are read from `behavior_server.ros__parameters`, except the inherited
Nav2 behavior plugin id under `backup.plugin`.

| Parameter | Default | Notes |
| --- | --- | --- |
| `service_name` | `local_costmap/get_costmap` | Costmap service used to search free space |
| `global_frame` | `map` | Pose query frame |
| `robot_radius` | `0.25` | Inner search radius |
| `max_radius` | `1.0` | Direction search radius |
| `visualize` | `false` | Publish direction markers |
| `backup.allow_escape_from_collision` | `true` | Permit short escape from an already occupied footprint |
| `backup.escape_collision_max_distance` | `0.45` | Max distance for occupied-footprint escape |

Default bringup sets `service_name: global_costmap/get_costmap`,
`robot_radius: 0.3`, and `max_radius: 2.0`.

## OccupancyGridObstacleLayer

`OccupancyGridObstacleLayer` stamps occupied cells from the latest full
`nav_msgs/msg/OccupancyGrid` snapshot into a Nav2 costmap. In this workspace it consumes
the online `occupancy_grid` published by `plan_env/grid_map_node`, so local and global
costmaps can react to obstacle points from `lidar_preprocessor`. The layer treats each
incoming grid as a full state snapshot: cells absent from the newest occupied set are
cleared by reporting both previous and current occupied bounds to Nav2's update cycle.

The layer also exposes `<layer_name>/set_semantic_layer_mode`
(`sentry_nav_interfaces/srv/SetSemanticLayerMode`).

The semantic mode service is used by hole-pass logic to suppress this layer while passing
through configured holes. In the default bringup only the global costmap service is called, so
global planning ignores the online occupancy-grid obstacle layer while local costmap,
TrajectoryManager collision checks, MPC, and velocity smoothing keep their normal behavior.

| Parameter | Default | Notes |
| --- | --- | --- |
| `enabled` | `true` | Enable stamping cached occupied cells |
| `topic` | `occupancy_grid` | Occupancy grid input |
| `occupied_threshold` | `65` | Occupancy value treated as lethal |
| `source_timeout` | `0.6` | Mark the layer non-current when the input snapshot is stale; does not clear the last valid snapshot |
| `stamp_source_cell_area` | `true` | Stamp the source grid cell area instead of only the center |
| `debug_logging` | `false` | Throttled layer diagnostics |

## IntensityVoxelLayer

`IntensityVoxelLayer` extends Nav2's obstacle layer with intensity filtering for
PointCloud2 observations. It is kept as an exported plugin for deployments that provide an
intensity-coded terrain or obstacle cloud.

Additional parameters beyond the usual Nav2 voxel/obstacle-layer parameters:

- `min_obstacle_intensity`
- `max_obstacle_intensity`

## BT Nodes

The package exports the BT plugin library `sirb_nav2_plugins_nodes`, which is listed in
`bt_navigator.plugin_lib_names`. The current behavior trees use these custom nodes for
trajectory candidate generation, committing trajectories, replanning gates, localization
readiness, nearby-goal recovery, and hole-pass mode control.

### HolePassModeController

`HolePassModeController` is a mode switch, not a velocity controller. It is ticked before
planning in the default behavior trees, so entering a configured hole port first switches the
navigation mode and only then lets normal Nav2 planning and tracking continue.

Each hole has two unordered port polygons:

```yaml
bt_navigator:
  ros__parameters:
    hole_pass:
      yaw_offset_deg: 0.0
      yaw_kp: 2.5
      max_v_yaw: 1.8
      raise_duration_sec: 1.0
      hole_ids: ["hole_1"]
      holes:
        hole_1:
          port_a_polygon: [x1, y1, x2, y2, x3, y3, x4, y4]
          port_b_polygon: [x1, y1, x2, y2, x3, y3, x4, y4]
```

The first port reached is treated as entry and the other port as exit. Entry sends
`HOLE_LOWER`, switches `navigation_mode_manager` to `hole_pass`, and keeps publishing
`HolePassCmd` on `mpc/hole_pass_cmd`. Exit sends `HOLE_RAISE`; after `raise_duration_sec` the
mode returns to `normal`. The controller then waits until the robot leaves both port polygons
before the same hole can trigger again.

Default bringup sets `navigation_mode_manager.semantic_layer_services` to only
`global_costmap/occupancy_grid_layer/set_semantic_layer_mode`. Therefore hole-pass mode
suppresses only the global costmap `OccupancyGridObstacleLayer`; local costmap, trajectory
manager collision checking, MPC, velocity smoother, and `fake_vel_transform` stay on the
normal navigation chain.

The target chassis heading is computed from entry-port center to exit-port center plus
`yaw_offset_deg`. `HolePassCmd.v_yaw` is the yaw-rate command in rad/s generated from that
target heading, current TF yaw, `yaw_kp`, and `max_v_yaw`.

## Build

```bash
colcon build --packages-select sirb_nav2_plugins
```
