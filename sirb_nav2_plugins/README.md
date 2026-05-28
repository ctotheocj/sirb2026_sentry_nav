# sirb_nav2_plugins

`sirb_nav2_plugins` contains the Navigation2 extensions used by this workspace:

- behavior plugin: `pb_nav2_behaviors/BackUpFreeSpace`
- costmap layers: `pb_nav2_costmap_2d::IntensityVoxelLayer` and
  `pb_nav2_costmap_2d::OccupancyGridObstacleLayer`
- behavior-tree nodes exported through `sirb_nav2_plugins_nodes`

The current `sirb2026_nav_bringup` configuration uses `BackUpFreeSpace`,
`OccupancyGridObstacleLayer`, and the BT node library. `IntensityVoxelLayer` is still built
and exported for deployments that need intensity-filtered 3D obstacle clouds, but it is not
part of the default navigation YAML.

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

For hole passing the layer has two mechanisms:

- `clear_hole_corridors` removes dynamic occupancy-grid cells inside configured hole corridors
  during normal navigation and hole-pass navigation. The corridor is the axis-aligned envelope
  rectangle of all points from the two configured port polygons, expanded by `clear_hole_margin`.
- The semantic mode service suppresses the whole online occupancy-grid layer while the robot is
  physically lowered in `hole_pass` mode. Static map layers stay active.

`sirb2026_nav_bringup` keeps the hole geometry as a single source of truth under
`bt_navigator.ros__parameters.hole_pass`. The launch path expands that geometry into
`local_costmap` and `global_costmap` layer parameters before Nav2 nodes start, so users should
edit the hole polygons only in the `hole_pass.holes` block.

| Parameter | Default | Notes |
| --- | --- | --- |
| `enabled` | `true` | Enable stamping cached occupied cells |
| `topic` | `occupancy_grid` | Occupancy grid input |
| `occupied_threshold` | `65` | Occupancy value treated as lethal |
| `source_timeout` | `0.6` | Mark the layer non-current when the input snapshot is stale; does not clear the last valid snapshot |
| `stamp_source_cell_area` | `true` | Stamp the source grid cell area instead of only the center |
| `clear_hole_corridors` | `false` | Filter occupied cells inside configured hole corridors |
| `clear_hole_frame` | `map` | Frame of `clear_holes.*` polygons |
| `clear_hole_margin` | `0.15` | Extra clearance around each generated corridor polygon |
| `clear_hole_max_area` | `0.0` | Optional maximum generated envelope area in m^2; `0.0` disables the guard |
| `clear_hole_ids` | `[]` | Hole ids injected from `bt_navigator.hole_pass.hole_ids` by launch |
| `clear_holes.<id>.port_a_polygon` | `[]` | Injected port A polygon for corridor filtering |
| `clear_holes.<id>.port_b_polygon` | `[]` | Injected port B polygon for corridor filtering |
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
`bt_navigator.plugin_lib_names`. The current default behavior trees use custom nodes for
trajectory candidate generation, committing trajectories, replanning gates, nearby-goal
recovery, and hole-pass mode control. Localization is not a BT gate in the default trees; this
matches the pb2025 navigation behavior where Nav2 starts planning as soon as it receives a goal.

### HolePassModeController

`HolePassModeController` is a mode switch, not a velocity controller. It is ticked before
planning in the default behavior trees, so entering a configured hole port first switches the
navigation mode and only then lets normal Nav2 planning and tracking continue.

Each hole has two unordered port polygons:

```yaml
bt_navigator:
  ros__parameters:
    hole_pass:
      target_yaw_deg: 0.0
      yaw_kp: 2.5
      max_v_yaw: 1.8
      raise_duration_sec: 1.0
      require_navigation_intent: true
      min_goal_direction_cos: 0.2
      min_goal_exit_margin: 0.2
      hole_ids: ["hole_1"]
      holes:
        hole_1:
          port_a_polygon: [x1, y1, x2, y2, x3, y3, x4, y4]
          port_b_polygon: [x1, y1, x2, y2, x3, y3, x4, y4]
```

The first port reached is treated as entry and the other port as exit. With
`require_navigation_intent` enabled, this only triggers when the active navigation goal is in
the entry-to-exit direction and closer to the opposite port, so A/B stays unordered and
bidirectional without lowering for unrelated drive-by motion. Entry sends `HOLE_LOWER`,
switches `navigation_mode_manager` to `hole_pass`, and keeps publishing `HolePassCmd` on
`mpc/hole_pass_cmd`. Raising is allowed only while the robot is inside one of the configured
port polygons: normal completion raises inside the opposite exit port; if the active goal stops
matching the entry-to-exit intent while lowered, the controller replaces the BT planning goal
with the original entry-port center, keeps the chassis lowered, and raises only after the robot
returns to that original entry port. After `raise_duration_sec` the mode returns to `normal`.
The controller then waits until the robot leaves both port polygons before the same hole can
trigger again.

Default bringup sets `navigation_mode_manager.semantic_layer_services` for both global and
local `occupancy_grid_layer` services. Therefore hole-pass mode suppresses the dynamic
occupancy-grid obstacle layer in both costmaps. In addition,
`navigation_mode_manager/mode` is consumed by `TrajectoryManager` and `MpcController`; while the
mode is `hole_pass`, the trajectory-manager forward collision rejection and the MPC predicted
collision hard stop are bypassed. Velocity smoothing and `fake_vel_transform` remain on the
normal command chain.

The default trajectory product generated by `GenerateMincoCandidate` is `time_reference`.
It is an executable, time-parameterized MINCO polynomial reference for MPC, not a degraded
fallback. Products such as `collision_fallback`, `geometry_fallback`, `cached_minco`, and
`path_fallback` are degraded and can request that `TrajectoryManager` keep a healthy active
trajectory instead of replacing it.

The manager owns the active hole-pass command state. BT halt/destruction and watchdog expiry do
not restore normal mode by default, so a hard action cancel while lowered keeps publishing the
last lowered chassis state instead of raising outside a port. A later BT instance can resume the
active mode from `navigation_mode_manager/get_navigation_mode`; entry/exit direction is inferred
again from the configured hole polygons, current pose, and navigation target rather than stored
inside the generic mode service.

The target chassis heading is `target_yaw_deg`: an absolute yaw in the map frame, measured
counter-clockwise from map +x in degrees. `HolePassCmd.v_yaw` is the yaw-rate command in rad/s
generated from that target heading, current TF yaw, `yaw_kp`, and `max_v_yaw`.

## Build

```bash
colcon build --packages-select sirb_nav2_plugins
```
