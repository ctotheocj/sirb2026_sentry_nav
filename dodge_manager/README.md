# dodge_manager

哨兵机器人受击闪避管理器。监听裁判系统血量扣减事件，在指定闪避区域内自动执行闪避动作。

## 节点

`dodge_manager_node`

## 话题

### 订阅

| 话题 | 类型 | 说明 |
|------|------|------|
| `/referee/robot_status` | `pb_rm_interfaces/RobotStatus` | 裁判系统状态，用于检测受击（HP扣减） |
| `/tracker/target` | `auto_aim_interfaces/Target` | 敌方目标跟踪（FOUND_STOP模式使用） |
| `local_costmap/costmap_raw` | `nav2_msgs/Costmap` | 本地代价地图，用于验证闪避方向安全性 |
| `goal_pose` | `geometry_msgs/PoseStamped` | 导航目标点 |

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `cmd_vel` | `geometry_msgs/Twist` | 闪避期间直接输出速度指令 |

## Action 客户端

| Action | 类型 | 说明 |
|--------|------|------|
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 暂停/恢复导航 |

## 控制频率

20 Hz

## 状态机

| 状态 | 说明 |
|------|------|
| `IDLE` | 正常导航，不介入 |
| `MONITORING` | 在闪避区域内，监听受击 |
| `DODGING` | 执行闪避（子状态: PLAN → MOVE → DONE），非阻塞 |
| `FOUND_STOP` | 发现敌人，发出停止指令 |

## 算法流程

1. 检测 HP 扣减（`hp_deduction_reason == 0`）
2. 检查机器人是否在 4 顶点多边形闪避区内
3. 通过 costmap 采样验证逃跑方向安全性（36 方向采样）
4. P 控制闭环移动，执行 3 次闪避
5. 闪避完成后恢复导航

## 参数

配置文件：`dodge_manager/config/dodge_manager.yaml`

```yaml
dodge_manager:
  ros__parameters:
    enable_dodge: false              # 是否启用闪避功能
    robot_radius: 0.20               # 机器人半径 (m)
    dodge_distance: 1.0              # 单次闪避距离 (m)
    dodge_velocity: 1.5              # 闪避速度 (m/s)
    dodge_count: 3                   # 每次触发最大闪避次数
    safety_margin: 0.15              # 安全裕度 (m)
    direction_samples: 36            # 方向采样数
    arrive_threshold: 0.15           # 到达判断阈值 (m)
    # 闪避区域（4顶点多边形，世界坐标系，单位m）
    dodge_zone: [x1, y1, x2, y2, x3, y3, x4, y4]
```

`dodge_zone` 按顺序给出 4 个顶点的 x、y 坐标，共 8 个浮点数，顶点顺序需构成合法凸多边形。

## 依赖

`rclcpp` `rclcpp_action` `geometry_msgs` `nav2_msgs` `pb_rm_interfaces` `auto_aim_interfaces` `tf2`
