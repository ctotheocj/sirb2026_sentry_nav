# ndt_omp_relocalization

`ndt_omp_relocalization_node` is the legacy/fallback prior-map relocalization backend used
by `sirb2026_nav_bringup` when `localization_backend:=ndt`. It registers the current
`registered_scan` against a prior PCD map with NDT-OMP and publishes a quality-gated
correction transform from `map` to `odom`. Point-LIO remains the high-rate `odom -> base`
source; NDT is the lower-rate global drift correction layer.

## Runtime Chain

```text
Point-LIO -> cloud_registered
  -> loam_interface -> registered_scan
  -> ndt_omp_relocalization -> map -> odom TF
```

The top-level launch passes `prior_pcd_file` from the selected world. The package-level
standalone launch also accepts `prior_pcd_file:=...` and uses the same default frame names as
the main navigation chain, so it can be used as a minimal NDT smoke-test entry point.

## Topics

Subscribed:

| Topic | Type | Notes |
| --- | --- | --- |
| `registered_scan` | `sensor_msgs/msg/PointCloud2` | Current scan/cloud in odom context |
| `initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | RViz initial pose reset |

Published:

| Output | Notes |
| --- | --- |
| `map -> odom` TF | Trusted NDT results update the correction; when gated, the node holds the last trusted correction. Before the first trusted result, it publishes the configured initial pose so the `map` frame exists |
| `ndt_omp_relocalization/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` consumed by `LocalizationReady` in the default BT |

## Parameters

| Parameter | Default | Notes |
| --- | --- | --- |
| `num_threads` | `4` | OpenMP worker threads |
| `ndt_resolution` | `1.0` | NDT voxel resolution |
| `ndt_step_size` | `0.1` | Optimization step size |
| `ndt_epsilon` | `0.01` | Convergence epsilon |
| `ndt_max_iterations` | `30` | Maximum optimization iterations |
| `ndt_search_method` | `1` | `0` KDTREE, `1` DIRECT7, `2` DIRECT1, `3` DIRECT26 |
| `fitness_score_threshold` | `1.0` | Max accepted NDT fitness score |
| `registration_period_ms` | `300` | Registration timer period; default bringup uses 250 ms |
| `global_leaf_size` | `0.25` | Prior map voxel filter leaf size |
| `registered_leaf_size` | `0.25` | Input scan voxel filter leaf size |
| `min_source_points` | `1000` | Minimum raw source points; default bringup uses 800 for MID360/simulation scans |
| `min_filtered_points` | `120` | Minimum filtered source points |
| `max_scan_age_sec` | `0.5` | Drop stale input cloud |
| `map_frame` | `map` | Map frame |
| `odom_frame` | `odom` | Odom frame |
| `base_frame` | `""` | Base frame used for lidar-to-odom context |
| `robot_base_frame` | `""` | Robot base frame used for initialpose handling |
| `prior_pcd_file` | `""` | Prior PCD path |
| `init_pose` | `[0,0,0,0,0,0]` | Initial `[x,y,z,roll,pitch,yaw]` |
| `enable_roll_pitch_fix` | `true` | Zero roll/pitch in published correction |
| `trust_ndt_threshold` | `5` | Consecutive converged registrations before trusting |
| `jump_threshold_xy` | `0.5` | XY jump gate; default bringup uses 0.30 m |
| `jump_threshold_yaw` | `0.3` | Yaw jump gate; default bringup uses 0.45 rad |
| `jump_threshold_rp` | `0.1` | Roll/pitch jump gate; default bringup uses 0.18 rad |
| `enable_quality_gate` | `true` | Enable residual/overlap quality checks |
| `quality_sample_points` | `1500` | Max sampled points for quality gate |
| `quality_max_corr_dist` | `1.0` | Nearest-neighbor correspondence range |
| `quality_min_valid_correspondences` | `120` | Minimum valid correspondences |
| `quality_min_overlap_ratio` | `0.35` | Minimum overlap ratio |
| `quality_max_median_residual` | `0.35` | Median residual gate |
| `quality_max_p90_residual` | `1.0` | 90th percentile residual gate |
| `publish_tf_only_when_trusted` | `false` | Default bringup sets true so untrusted NDT registrations cannot pull `map -> odom`; the initial pose is still published before the first trusted result |
| `freeze_tf_when_not_trusted` | `false` | Default bringup sets true to hold the last trusted correction while NDT is untrusted |

The diagnostic array contains a single status from `ndt_omp_relocalization`. Important
key/value fields are
`localization_state`, `trusted`, `fitness`, `overlap`, `source_points`,
`filtered_points`, `jump_rejected`, and `tf_mode`.

The current source does not declare `lidar_frame` or `use_scan_stamp_for_tf`; those are not
valid parameters for this package.

## Build

```bash
colcon build --packages-select ndt_omp_ros2 ndt_omp_relocalization
```

`ndt_omp_ros2` must provide the shared `pclomp` NDT library. The workspace launch also
requires a valid PCD for the selected `world` or an explicit `prior_pcd_file:=...`.
