# fake_vel_transform

本功能包启动时，Fake Velocity Transform 会创建一个 `fake_robot_base_frame` 的坐标系，其 x, y, z 与 `robot_base_frame` 的坐标系一致，但 yaw 固定指向正前方，然后将生成的该坐标系发布到 tf2。同时，Fake Velocity Transform 会订阅 `input_cmd_vel_topic` 话题，将接收到的速度指令转换到 `robot_base_frame` 坐标系下，并发布到 `output_cmd_vel_topic` 话题。

主要目的是用于适配 NAV2 局部路径规划器，当速度参考坐标系 `robot_base_frame` 变化剧烈时，如云台处于自旋扫描时，NAV2 局部路径规划器会将机器人的方向视为与当前路径规划方向一致，导致机器人无法正常运动。而使用 `fake_robot_base_frame` 可以规避这个问题，实现较稳定的轨迹跟踪效果。

由于 NAV2 Humble 发行版仍主要使用 Twist 类型（不含时间戳），当前 bringup 使用 `input_cmd_vel_topic` 直接接收 `velocity_smoother` 输出并采用当前缓存 yaw 转换。若部署显式配置了 `input_cmd_vel_stamped_topic`，本节点会优先按速度命令时间戳查询 `odom_frame` 到 `robot_base_frame` 的 TF 并完成速度转换。
Related issue: [Switch from Twist to TwistStamped for cmd_vel #1594](https://github.com/ros-navigation/navigation2/issues/1594)

## Published Topics

* `tf` (`tf2_msgs/msg/TFMessage`) - 与机器人可移动关节相对应的变换
* `output_cmd_vel_topic` (`geometry_msgs/msg/Twist`) - 转换后的速度指令

## Subscribed Topics

* `input_cmd_vel_topic` (`geometry_msgs/msg/Twist`) - 机器人的 legacy 速度指令
* `input_cmd_vel_stamped_topic` (`geometry_msgs/msg/TwistStamped`) - 显式配置后订阅的带时间戳速度指令，优先使用
* `odom_topic` (`nav_msgs/msg/Odometry`) - 里程计数据
* `cmd_spin_topic` (`example_interfaces/msg/Float32`) - 控制底盘固定旋转速度，将会叠加到 `output_cmd_vel_topic` 中
* `nav_yaw_topic` (`std_msgs/msg/Float64`) - `use_nav_yaw=true` 时订阅，默认 `/Nav_yaw`

## Parameters

* `odom_topic` (`string`, default: "odom") - 里程计话题。里程计的 frame_id 与 `robot_base_frame` 参数保持一致
* `robot_base_frame` (`string`, default: "gimbal_link") - 速度参考坐标系
* `fake_robot_base_frame` (`string`, default: "gimbal_link_fake") - 伪速度参考坐标系
* `cmd_spin_topic` (`string`, default: "cmd_spin") - 控制底盘固定旋转速度的话题
* `input_cmd_vel_topic` (`string`, default: "") - 输入速度指令的话题
* `input_cmd_vel_stamped_topic` (`string`, default: "") - 输入带时间戳速度指令的话题，留空时不订阅 stamped 输入
* `output_cmd_vel_topic` (`string`, default: "") - 输出速度指令的话题。将原本基于 `fake_robot_base_frame` 的速度变换到 `robot_base_frame` 后发布
* `odom_frame` (`string`, default: "odom") - TF 查询目标坐标系
* `prefer_stamped_cmd_vel` (`bool`, default: true) - stamped 速度新鲜时抑制 legacy Twist 输出
* `allow_latest_tf_fallback` (`bool`, default: true) - stamped 时间戳 TF 外推失败时允许使用最新 TF 兜底
* `tf_lookup_timeout_sec` (`double`, default: 0.01) - TF 查询超时时间
* `stamped_cmd_timeout_sec` (`double`, default: 0.12) - stamped 速度新鲜度窗口
* `max_latest_cmd_age_sec` (`double`, default: 0.12) - legacy 与 stamped 新鲜度的共同上限；实际 stamped 窗口取该值与 `stamped_cmd_timeout_sec` 的较小值
* `init_spin_speed` (`float`, default: 0.0) - 若没有接收 `cmd_spin_topic`，则使用该值作为固定旋转速度
* `use_nav_yaw` (`bool`, default: false) - 使用融合 yaw 代替 `odom_frame -> robot_base_frame` TF yaw
* `nav_yaw_topic` (`string`, default: "/Nav_yaw") - 融合 yaw 话题

## Full Navigation Chain

在当前导航 bringup 中，速度链路为：

```text
controller_server -> cmd_vel_controller
velocity_smoother -> cmd_vel_selected
fake_vel_transform -> cmd_vel
```

`fake_vel_transform` 同时发布 `robot_base_frame -> fake_robot_base_frame` TF。Nav2 的
`robot_base_frame` 配置为 `gimbal_yaw_fake`，最终输出再转换回真实 `gimbal_yaw` 相关底盘速度。
