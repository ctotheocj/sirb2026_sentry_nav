# fake_vel_transform

本功能包启动时，Fake Velocity Transform 会创建一个 `fake_robot_base_frame` 的坐标系，其 x, y, z 与 `robot_base_frame` 的坐标系一致，但 yaw 固定指向正前方，然后将生成的该坐标系发布到 tf2。同时，Fake Velocity Transform 会订阅 `input_cmd_vel_topic` 话题，将接收到的速度指令转换到 `robot_base_frame` 坐标系下，并发布到 `output_cmd_vel_topic` 话题。

主要目的是用于适配 NAV2 局部路径规划器，当速度参考坐标系 `robot_base_frame` 变化剧烈时，如云台处于自旋扫描时，NAV2 局部路径规划器会将机器人的方向视为与当前路径规划方向一致，导致机器人无法正常运动。而使用 `fake_robot_base_frame` 可以规避这个问题，实现较稳定的轨迹跟踪效果。

由于 NAV2 Humble 发行版仍主要使用 Twist 类型（不含时间戳），直接按速度时间戳对齐 odometry/TF 会丢失信息。本功能包优先订阅 `input_cmd_vel_stamped_topic`，按速度命令时间戳查询 `odom_frame` 到 `robot_base_frame` 的 TF 并完成速度转换。若 stamped 速度暂时不可用，则使用 legacy `input_cmd_vel_topic` 并采用当前缓存 yaw 转换；该路径只作为未收到 stamped 指令时的直接输入路径，不再依赖 `local_plan` 间接同步。
Related issue: [Switch from Twist to TwistStamped for cmd_vel #1594](https://github.com/ros-navigation/navigation2/issues/1594)

## Published Topics

* `tf` (`tf2_msgs/msg/TFMessage`) - 与机器人可移动关节相对应的变换
* `output_cmd_vel_topic` (`geometry_msgs/msg/Twist`) - 转换后的速度指令

## Subscribed Topics

* `input_cmd_vel_topic` (`geometry_msgs/msg/Twist`) - 机器人的 legacy 速度指令
* `input_cmd_vel_stamped_topic` (`geometry_msgs/msg/TwistStamped`) - 机器人的带时间戳速度指令，优先使用
* `odom_topic` (`nav_msgs/msg/Odometry`) - 里程计数据
* `cmd_spin_topic` (`example_interfaces/msg/Float32`) - 控制底盘固定旋转速度，将会叠加到 `output_cmd_vel_topic` 中

## Parameters

* `odom_topic` (`string`, default: "odom") - 里程计话题。里程计的 frame_id 与 `robot_base_frame` 参数保持一致
* `robot_base_frame` (`string`, default: "gimbal_link") - 速度参考坐标系
* `fake_robot_base_frame` (`string`, default: "gimbal_link_fake") - 伪速度参考坐标系
* `cmd_spin_topic` (`string`, default: "cmd_spin") - 控制底盘固定旋转速度的话题
* `input_cmd_vel_topic` (`string`, default: "") - 输入速度指令的话题
* `input_cmd_vel_stamped_topic` (`string`, default: "") - 输入带时间戳速度指令的话题，留空时默认为 `input_cmd_vel_topic + "_stamped"`
* `output_cmd_vel_topic` (`string`, default: "") - 输出速度指令的话题。将原本基于 `fake_robot_base_frame` 的速度变换到 `robot_base_frame` 后发布
* `odom_frame` (`string`, default: "odom") - TF 查询目标坐标系
* `prefer_stamped_cmd_vel` (`bool`, default: true) - stamped 速度新鲜时抑制 legacy Twist 输出
* `allow_latest_tf_fallback` (`bool`, default: true) - stamped 时间戳 TF 外推失败时允许使用最新 TF 兜底
* `tf_lookup_timeout_sec` (`double`, default: 0.01) - TF 查询超时时间
* `stamped_cmd_timeout_sec` (`double`, default: 0.12) - stamped 速度新鲜度窗口
* `init_spin_speed` (`double`, default: 0.0) - 若没有接收 `cmd_spin_topic`，则使用该值作为固定旋转速度
