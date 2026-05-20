# Yaw Fusion 功能包

## 功能说明

本功能包用于融合**电控云台yaw角度**和**odometry计算 yaw角度**，解决以下问题：
- odometry 准确但更新慢
- 电控 yaw 实时但有累积误差

## 工作原理

```
┌─────────────────────────────────────────────────────────────────┐
│                         YawFusion 节点                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   /odom (odometry yaw)                                          │
│         │                                                       │
│         ▼                                                       │
│   ┌─────────────────────────────────────────┐                  │
│   │  检测底盘是否静止 (角速度 < 阈值)         │                  │
│   └────────────────┬────────────────────────┘                  │
│                    │ 静止超过 calibration_timeout               │
│                    ▼                                            │
│   ┌─────────────────────────────────────────┐                  │
│   │  校准: yaw_offset = odom_yaw - TF_yaw   │                  │
│   └─────────────────────────────────────────┘                  │
│                                                                  │
│   /tf (base_link → gimbal_link)                                │
│         │                                                       │
│         ▼                                                       │
│   ┌─────────────────────────────────────────┐                  │
│   │  获取电控TF的原始yaw                      │                  │
│   └────────────────┬────────────────────────┘                  │
│                    │                                             │
│                    ▼                                             │
│   ┌─────────────────────────────────────────┐                  │
│   │  融合: fused_yaw = TF_yaw + yaw_offset  │                  │
│   └────────────────┬────────────────────────┘                  │
│                    │                                             │
│                    ▼                                             │
│              /Nav_yaw 发布                                      │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## 核心逻辑

| 状态 | 行为 |
|-----|------|
| **odom静止时** | 计算偏置 `offset = odom_yaw - TF_yaw` |
| **运行时** | `fused_yaw = TF_yaw + offset` |
| **数据平滑** | 使用一阶低通滤波器平滑输出 (Alpha = tolerance) |

## 订阅话题

| 话题名 | 类型 | 说明 |
|-------|------|-----|
| `/odom` | `nav_msgs/msg/Odometry` | 底盘odometry数据 |

## 发布话题

| 话题名 | 类型 | 说明 |
|-------|------|-----|
| `/Nav_yaw` | `std_msgs/msg/Float64` | **融合后的yaw（主要输出）** |
| `/yaw_fusion/debug` | `std_msgs/msg/Float64` | 调试数据 |
| `/yaw_fusion/status` | `std_msgs/msg/String` | 状态信息 |

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|-----|------|-------|------|
| `calibration_timeout` | `double` | `2.0` | 静止多长时间(秒)后触发校准 |
| `yaw_threshold` | `double` | `0.01` | 判定底盘静止的角速度阈值(rad/s) |
| `use_tolerance` | `bool` | `true` | 是否启用一阶低通滤波 |
| `tolerance` | `double` | `0.2` | 低通滤波器 Alpha 值(0.0~1.0)，越小越平滑 |
| `parent_frame` | `string` | `"base_link"` | TF父坐标系 |
| `child_frame` | `string` | `"gimbal_link"` | TF子坐标系（电控yaw来源） |

## 运行方式

```bash
# 编译
cd ~/ros_ws_vision
colcon build --packages-select yaw_fusion
source install/setup.bash

# 方式1: 使用launch文件
ros2 launch yaw_fusion yaw_fusion.launch.py

# 方式2: 直接运行节点
ros2 run yaw_fusion yaw_fusion_node

# 查看输出话题
ros2 topic echo /Nav_yaw

# 查看状态
ros2 topic echo /yaw_fusion/status
```

## 与其他节点配合

### 1. 确保 rm_serial_driver 已运行

`yaw_fusion` 需要 `rm_serial_driver` 发布的TF变换：

```bash
# 确保电控数据正常
ros2 topic list | grep tf
# 应该看到 /tf 或类似话题
```

### 2. 确保 odometry 正常

确保有节点发布 `/odom` 话题：

```bash
ros2 topic info /odom
```

## 调参建议

| 场景 | 参数调整 |
|-----|---------|
| odometry更新很慢 | 增大 `calibration_timeout` (如3.0秒) |
| 底盘微小抖动导致频繁校准 | 增大 `yaw_threshold` (如0.05 rad/s) |
| 电控数据跳变频繁 | 启用 `use_tolerance=true`，减小 `tolerance` (如0.1) |
| 需要更平滑的输出 | 减小 `tolerance` (如0.05) |
| 允许更快的动态响应 | 增大 `tolerance` (如0.5) |

## 坐标变换要求

本节点依赖以下TF变换存在：

```
base_link ──► gimbal_link
```

这通常由 `rm_serial_driver` 节点发布。如果TF缺失，节点会警告。

## 状态消息

`/yaw_fusion/status` 话题输出的状态：

| 状态 | 含义 |
|-----|------|
| `calibrated` | 校准完成，已开始融合输出 |

## 代码结构

```
yaw_fusion_node.cpp
├── YawFusion 节点类
│   ├── 构造函数
│   │   ├── 参数声明
│   │   ├── TF 初始化
│   │   └── 订阅/发布设置
│   │
│   ├── odomCallback()
│   │   └── 检测静止并触发校准
│   │
│   ├── performCalibration()
│   │   └── 计算 yaw_offset
│   │
│   ├── getRawTF()
│   │   └── 获取TF原始yaw（支持时间戳查询）
│   │
│   ├── timerCallback()
│   │   └── 融合发布（核心公式）
│   │
│   └── 工具函数
│       ├── quaternionToYaw()
│       └── normalizeAngle()
```

## 注意事项

1. **首次运行**：需要等待odom和TF数据都收到后才会开始工作
2. **校准时机**：必须在底盘静止时校准才准确，且每次静止只校准一次
3. **静止判定**：使用里程计的角速度(`twist.twist.angular.z`)判断静止，不受帧率影响
4. **坐标系**：确保 `parent_frame` 和 `child_frame` 与实际TF树匹配
5. **数据源**：
   - odom yaw 来自 `/odom` 话题的 `pose.pose.orientation`
   - 角速度来自 `/odom` 话题的 `twist.twist.angular.z`
   - 电控 yaw 来自 `base_link` → `gimbal_link` 的TF变换
6. **TF容差**：时间戳查找容差默认0.02秒，过小可能导致查找失败

## 更新日志

- **v0.2.0**:
  - 改用角速度判断静止，解决不同频率带来的静止误判
  - 将容错跳变处理替换为一阶低通滤波器，tolerance 参数改为 Alpha 值
- **v0.1.1**: 优化注释，增加Matrix3x3头文件显式包含
- **v0.1.0**: 初始版本，偏置融合核心逻辑
