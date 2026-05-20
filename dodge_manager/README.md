# dodge_manager

`dodge_manager_node` listens for referee-system HP deduction events and can temporarily
take over velocity output to execute evasive moves inside a configured dodge zone.

The top-level navigation launch files start `dodge_manager_node` unconditionally. Whether it
actually performs evasive actions is controlled only by
`dodge_manager.ros__parameters.enable_dodge` in the YAML.

## Topics

Subscribed:

| Topic | Type | Notes |
| --- | --- | --- |
| `/referee/robot_status` | `pb_rm_interfaces/msg/RobotStatus` | Hit detection |
| `/tracker/target` | `auto_aim_interfaces/msg/Target` | FOUND_STOP mode target detection |
| `local_costmap/costmap_raw` | `nav2_msgs/msg/Costmap` | Direction safety sampling |
| `goal_pose` | `geometry_msgs/msg/PoseStamped` | Latest navigation goal |

Published:

| Topic | Type | Notes |
| --- | --- | --- |
| `cmd_vel` | `geometry_msgs/msg/Twist` | Direct evasion velocity output |

Action client:

| Action | Type | Notes |
| --- | --- | --- |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Pause/resume navigation around dodge execution |

## Parameters

The parameters can be loaded from `dodge_manager/config/dodge_manager.yaml` or from the
main `sirb2026_nav_bringup/config/*/nav2_params.yaml`.

```yaml
dodge_manager:
  ros__parameters:
    enable_dodge: false
    global_frame: odom
    robot_frame: gimbal_yaw
    control_frequency: 20.0

    dodge_zone:
      x1: -2.0
      y1: -2.0
      x2:  2.0
      y2: -2.0
      x3:  2.0
      y3:  2.0
      x4: -2.0
      y4:  2.0

    robot_status_topic: /referee/robot_status
    tracker_target_topic: /tracker/target
    cmd_vel_topic: cmd_vel
    costmap_topic: local_costmap/costmap_raw
    goal_pose_topic: goal_pose

    robot_radius: 0.20
    dodge_distance: 1.0
    dodge_velocity: 1.5
    dodge_count: 3
    safety_margin: 0.15
    direction_samples: 36
    costmap_check_resolution: 0.05
    arrive_threshold: 0.15
```

`dodge_zone` is a four-point polygon expressed as nested parameters
`dodge_zone.x1`, `dodge_zone.y1`, ..., `dodge_zone.y4`. It is not an 8-element YAML array.

## State Machine

- `IDLE`: no intervention
- `MONITORING`: robot is inside the dodge zone and waiting for hit events
- `DODGING`: non-blocking plan/move/done dodge execution
- `FOUND_STOP`: target detected and stop command active
