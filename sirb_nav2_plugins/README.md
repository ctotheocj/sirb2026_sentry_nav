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

`OccupancyGridObstacleLayer` stamps occupied cells from a `nav_msgs/msg/OccupancyGrid`
into a Nav2 costmap without clearing free cells. In this workspace it consumes the online
`occupancy_grid` published by `plan_env/grid_map_node`, so local and global costmaps can
react to obstacle points from `lidar_preprocessor`.

The layer also exposes:

- `<layer_name>/set_enabled` (`std_srvs/srv/SetBool`)
- `<layer_name>/set_semantic_layer_mode`
  (`sentry_nav_interfaces/srv/SetSemanticLayerMode`)

The semantic mode service is used by hole-pass logic to mask a corridor while passing
through configured holes.

| Parameter | Default | Notes |
| --- | --- | --- |
| `enabled` | `true` | Enable stamping cached occupied cells |
| `topic` | `occupancy_grid` | Occupancy grid input |
| `occupied_threshold` | `65` | Occupancy value treated as lethal |
| `obstacle_keep_time` | `0.35` | Cached occupied-cell lifetime |
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
readiness, enemy/hole conditions, and hole-pass actions.

## Build

```bash
colcon build --packages-select sirb_nav2_plugins
```
