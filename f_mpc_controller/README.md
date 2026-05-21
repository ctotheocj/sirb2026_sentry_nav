# f_mpc_controller

`f_mpc_controller` is the Nav2 `nav2_core::Controller` plugin used by this workspace as
`controller_server.FollowPath`. It tracks the path committed by the behavior tree, while
preferentially sampling the active MINCO trajectory published by `trajectory_manager`.

## Current Chain

```text
BT ComputePath* -> SafeGeometricSmoother GenerateMincoCandidate
  -> TrajectoryManager CommitTrajectory
  -> trajectory_manager/trajectory_for_mpc
  -> f_mpc_controller::MpcController
  -> cmd_vel_controller
```

`FollowPath` still receives a Nav2 path from the behavior tree. The controller uses that
path as the global path and as a fallback reference, but the normal high-quality tracking
branch is the active `sentry_nav_interfaces/msg/MincoTrajectory` from
`trajectory_manager/trajectory_for_mpc`.

## Runtime Behavior

- Samples the active MINCO trajectory in the controller global frame.
- Falls back to the Nav2 path only when `allow_path_fallback_without_minco` permits it.
- Stops and eventually fails `FollowPath` when no executable MINCO is available for longer
  than `minco_unavailable_failure_sec`.
- Scales both MINCO reference time and reference velocity when lateral tracking error grows,
  so the controller does not chase far-ahead points while trying to slow down.
- Limits per-horizon speed by braking distance and trajectory curvature before solving MPC.
- Uses a configurable measured-velocity anchor from odometry for the first input-rate
  constraint, falling back to the last command when odometry is stale or rejected.
- Solves an omnidirectional MPC QP with OSQP.
- Applies dynamic obstacle constraints when `enable_dynamic_obstacle_avoidance=true`.
- Publishes `local_plan` for MPC reference visualization.
- Checks the predicted MPC rollout against the Nav2 costmap and triggers recovery on
  persistent collision.
- Returns `geometry_msgs/msg/TwistStamped`; launch remaps controller output to
  `cmd_vel_controller`.

## Key Parameters

These are the parameters actually declared by the current implementation under
`controller_server.ros__parameters.FollowPath`.

| Parameter | Meaning |
| --- | --- |
| `QX`, `QY`, `R`, `S`, `Qv` | MPC tracking, input, input smoothness, and reference velocity weights |
| `horizon`, `control_frequency` | MPC horizon length and controller frequency |
| `v_ref_max`, `v_circle_max` | Reference speed cap and hard velocity cap |
| `ax_max`, `ay_max` | Acceleration limits used by the solver and speed-limit floor |
| `curvature_speed_limit_enabled` | Enable trajectory-curvature speed limiting before MPC solve |
| `lateral_accel_limit` | Lateral acceleration budget used by `v <= sqrt(a_lat / kappa)` |
| `curvature_speed_limit_min_speed` | Ignore curvature estimation below this MINCO speed |
| `use_odometry_state`, `odom_topic` | Use odometry pose/twist as the controller state source |
| `max_odom_age_sec`, `max_odom_predict_dt` | Odometry freshness and pose prediction limits |
| `enable_measured_velocity_anchor` | Blend odometry velocity into the MPC first-step acceleration anchor |
| `velocity_anchor_blend_alpha` | Weight of measured velocity versus last command in the control anchor |
| `velocity_anchor_lowpass_alpha` | Low-pass factor applied to measured odometry velocity |
| `velocity_anchor_max_age_sec` | Max allowed age for measured velocity anchoring |
| `velocity_anchor_max_jump` | Reject measured velocity jumps larger than this value |
| `minco_traj_topic` | Active MINCO trajectory topic, normally `trajectory_manager/trajectory_for_mpc` |
| `minco_timeout_sec` | Time-window grace for active MINCO execution |
| `minco_unavailable_failure_sec` | Max wait before failing `FollowPath` when MINCO is unavailable |
| `allow_path_fallback_without_minco` | Allow path-only reference tracking while waiting for MINCO |
| `minco_projection_*` | Projection search window and lag/advance limits on the active trajectory |
| `enable_lateral_error_ref_scaling` | Reduce reference progression when lateral tracking error grows |
| `lateral_error_slow_threshold`, `lateral_error_high_threshold` | Error band that maps to reference time scaling |
| `min_lateral_ref_time_scale` | Lower bound for lateral-error reference time scaling |
| `enable_pose_jump_damping` | Temporarily damp reference speed after localization jumps |
| `prevent_tracking_reverse` | Remove command component opposite to reference direction |
| `enable_dynamic_obstacle_avoidance` | Subscribe to predicted dynamic obstacles and add MPC constraints |
| `allow_obstacle_retry_without_constraints` | If true, retry an infeasible solve after clearing hard obstacle constraints |
| `allow_speed_limit_retry_without_limits` | If true, retry an infeasible solve after clearing per-horizon speed limits |
| `collision_stop_failure_sec` | Duration of predicted collision before throwing a Nav2 exception |
| `local_plan_publish_period_sec` | Visualization publish period for `local_plan` |

The default safety policy keeps both retry switches false: infeasible hard obstacle or
per-horizon speed constraints cause a stop/recovery path instead of silently removing the
constraint that made the problem safe.

The old `vx_max`, `vy_max`, `adaptive_step_*`, `enable_trapezoidal_accel`, `trap_*`,
`enable_velocity_reference`, and `TimestampedPath` parameters are not part of the current
controller path.

## Topics

| Topic | Type | Direction | Notes |
| --- | --- | --- | --- |
| `odometry` | `nav_msgs/msg/Odometry` | subscribe | Optional state source when `use_odometry_state=true` |
| `trajectory_manager/trajectory_for_mpc` | `sentry_nav_interfaces/msg/MincoTrajectory` | subscribe | Primary executable reference |
| `dynamic_obstacles` | `sentry_nav_interfaces/msg/TrackedObstacleArray` | subscribe | Only when dynamic obstacle avoidance is enabled |
| `local_plan` | `nav_msgs/msg/Path` | publish | Reference preview for RViz/debugging |

`trajectory_manager.input_topic` may still be configured to
`safe_geometric_smoother/legacy_trajectory_for_mpc` for compatibility, but the current
normal chain commits candidate MINCO trajectories through the `CommitTrajectory` action and
publishes the active trajectory on `trajectory_manager/trajectory_for_mpc`.

## Build

```bash
colcon build --packages-select f_mpc_controller
```

The package requires `OsqpEigen`, `Eigen3`, Nav2 controller interfaces, and
`sentry_nav_interfaces`.
