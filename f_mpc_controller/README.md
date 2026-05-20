# f_mpc_controller

## 许可证

本项目遵循 Apache License 2.0。

---

## 简介

`f_mpc_controller` 是基于 ROS2 Nav2 框架的全向移动机器人控制器插件，采用简化模型预测控制（MPC）算法，实现对全局路径的高效跟踪。控制器以 `nav2_core::Controller` 接口实现，可无缝集成到 Nav2 导航栈。

**主要特性：**
- 全向速度输出（`vx`, `vy`），无旋转约束
- 基于 OSQP 的 QP 求解，支持热启动，每周期仅更新梯度和边界
- 直线段检测与梯形速度曲线加速（可选）
- MINCO 时间参数化速度参考比例映射（可选）
- 目标点接近减速（可选）
- 基于 costmap 的预测轨迹碰撞检查
- 动态障碍物预测推离参考轨迹（可选）
- 横向跟踪误差实时发布（可选）
- `setPlan()` 二阶段加锁，路径转换在锁外完成，控制周期不阻塞

---

## 速度控制架构

控制器采用**自适应步长算法**，根据路径曲率和机器人速度动态调整参考轨迹采样密度，实现弯道精确跟踪与直线高速通过的平衡。

### 自适应步长原理

参考轨迹在 `setPlan()` 时预计算一次（`precomputeReference()`），采样点间距根据局部曲率自适应调整：

```
基础步长 = (v_typical × lookahead_time) / horizon
  其中 v_typical = 0.7 × hypot(vx_max, vy_max)

自适应步长 = 基础步长 × speed_factor × curvature_factor
  speed_factor     = clamp(v_current / v_typical, 0.6, 1.4)
  curvature_factor = 1.0 / (1.0 + curvature_gain × κ)

最终步长 = clamp(自适应步长, step_min, step_max)
```

**效果**：
- **弯道**（高曲率 κ）→ `curvature_factor` 小 → 步长缩小（0.1m）→ 点密集 → 精确跟踪
- **直线**（低曲率）→ `curvature_factor` ≈ 1 → 步长放大（1.0m）→ 点稀疏 → 高速通过
- **高速**（v_current 大）→ `speed_factor` > 1 → 步长放大 → 预见距离增加

### 游标推进机制

`updateRefCursor()` 采用速度自适应推进阈值，确保高速时游标不落后：

```
threshold = max(0.3, cursor_advance_gain × v_current × dt)
```

当机器人距当前游标点 < threshold 时，游标自动推进到下一个预计算点。高速时阈值增大，游标推进更激进。

### 三级优先级速度控制（保留）

在参考轨迹生成时，仍支持三级优先级决定每个 horizon 步的前进距离：

```
优先级 1（最高）：在直线段 & enable_trapezoidal_accel=true
    使用梯形速度曲线（加速/巡航/制动）

优先级 2：enable_velocity_reference=true & 有效时间戳
    使用 MINCO 速度参考比例映射

优先级 3（默认）：
    使用预计算的自适应步长参考点
```

**设计原理**：MPC 求解器本身不变，通过控制参考点间距来引导 MPC 输出目标速度。参考点间距越大，MPC 为追上参考点而输出越大速度；间距越小，速度越低。

### 梯形速度曲线

直线段内根据剩余距离自动切换加速/制动：

```
brake_dist = (v_cur² - v_end²) / (2 × decel)

if dist_remaining ≤ brake_dist:   # 制动阶段
    v = max(v_end, v_cur - decel × t)
else:                              # 加速/巡航阶段
    v = min(v_max, v_cur + accel × t)
```

### MINCO 速度参考比例映射

将轨迹优化器的绝对速度归一化为比例，再映射到 MPC 速度空间：

```
minco_ratio = clamp(minco_speed / trajectory_optimizer_v_max, 0.0, 1.0)
step_speed  = hypot(vx_max, vy_max) × minco_ratio
```

---

## 代码结构

```
f_mpc_controller/
├── include/f_mpc_controller/
│   ├── mpc_controller.hpp    # 控制器主类定义
│   └── mpc.hpp               # MPC 求解器定义
├── src/
│   ├── mpc_controller.cpp    # 控制器主逻辑实现
│   └── mpc.cpp               # MPC 求解器实现（基于 OSQP）
├── f_mpc_controller.xml      # Nav2 插件描述文件
├── CMakeLists.txt
├── package.xml
└── README.md
```

### 核心模块

| 模块 | 功能 |
|------|------|
| `MpcController::configure()` | 参数声明与读取，订阅外部话题 |
| `MpcController::setPlan()` | 接收全局路径，odom 坐标转换，路径拒绝判断，MINCO 时间戳匹配，直线段检测 |
| `MpcController::computeVelocityCommands()` | 主控制回路：更新索引 → 生成参考轨迹 → MPC 求解 → 预测碰撞检查 → 速度后处理 → 发布 |
| `MpcController::updateTargetIndex()` | 在路径上搜索最近点，窗口随速度动态扩展 |
| `MpcController::generateReferenceTrajectory()` | 三级优先级步长决策，插值生成参考轨迹，动态障碍预测约束 |
| `MpcController::computeTrapezoidalVelocity()` | 梯形速度曲线计算，返回预测时刻 t 的期望速度 |
| `MpcController::applyGoalSlowdown()` | 线性目标点减速 |
| `MPC::solve()` | 基于 OSQP 的 QP 求解，热启动，bounds 脏标记优化 |

---

## 安装与构建

1. 安装依赖：

```bash
sudo apt install libeigen3-dev ros-humble-osqp-vendor
```

2. 安装 OSQP 及 OSQP-Eigen：

```bash
git clone https://github.com/osqp/osqp.git
cd osqp && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc) && sudo make install && sudo ldconfig

git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc) && sudo make install
```

3. 编译本包：

```bash
colcon build --packages-select f_mpc_controller
```

---

## 参数说明

### 基础 MPC 参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `vx_min` / `vx_max` | double | -1.0 / 1.0 | X 方向速度限制（m/s） |
| `vy_min` / `vy_max` | double | -1.0 / 1.0 | Y 方向速度限制（m/s） |
| `QX` | double | 50.0 | X 方向状态跟踪权重，10~100 |
| `QY` | double | 30.0 | Y 方向状态跟踪权重，10~100 |
| `R` | double | 0.1 | 控制输入权重（越大越节能但跟踪越差） |
| `S` | double | 1.0 | 控制量平滑权重（惩罚相邻时刻速度变化） |
| `horizon` | int | 10 | 预测时域步数 |
| `control_frequency` | double | 20.0 | 控制频率（Hz），应与 controller_server 一致 |

### 自适应步长参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `adaptive_step_lookahead_time` | double | 1.2 | 预见时间（秒），决定 horizon 覆盖的时间跨度 |
| `adaptive_step_curvature_gain` | double | 2.5 | 曲率敏感系数，越大弯道步长越小（建议 2.0~3.0） |
| `adaptive_step_min` | double | 0.1 | 最小步长（m），弯道密集采样下限 |
| `adaptive_step_max` | double | 1.0 | 最大步长（m），直线稀疏采样上限（建议 0.8~1.2） |
| `cursor_advance_gain` | double | 3.0 | 游标推进系数，越大高速时游标越激进（建议 2.5~3.5） |

**调优指南**：
- 高速直线仍打转 → 增大 `adaptive_step_max` 和 `cursor_advance_gain`
- 高速弯道跟不上 → 减小 `adaptive_step_curvature_gain`
- 低速不稳定 → 减小 `adaptive_step_min`
- 预见距离不足 → 增大 `adaptive_step_lookahead_time`

### 路径拒绝参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `reject_angle_deg` | double | 120.0 | 新路径与当前速度方向夹角超过此值则拒绝（度） |
| `reject_dist_threshold` | double | 1.5 | 距终点小于此距离时不拒绝新路径（m） |
| `max_consecutive_rejects` | int | 3 | 连续拒绝次数上限，超过后强制接受 |

### 直线段检测参数

直线段始终检测（用于梯形加速和可视化），内置常量：
- 直线判定余弦阈值：`0.94`（约 20°）
- 与上条直线终点距离阈值：`1.0 m`

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `straight_line_min_length` | double | 3.0 | 认定为直线段的最小长度（m） |

### 梯形速度曲线参数（`enable_trapezoidal_accel: true` 时在直线段内生效）

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `enable_trapezoidal_accel` | bool | false | 启用直线段梯形速度曲线 |
| `trap_v_max` | double | 5.0 | 直线段最高速度（m/s） |
| `trap_v_end` | double | 2.0 | 直线段末端目标速度（m/s） |
| `trap_accel` | double | 3.0 | 加速度（m/s²） |
| `trap_decel` | double | 2.0 | 制动减速度（m/s²） |
| `linear_lateral_damping` | double | 0.15 | 直线段横向阻尼系数（0=完全抑制横向振荡，1=不抑制） |

> 启用梯形加速后，自动启用速度方向稳定化：抑制横向分量（消除振荡）+ 增强纵向分量（保持加速性能）。

### MINCO 速度参考参数（`enable_velocity_reference: true` 时在非梯形段生效）

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `enable_velocity_reference` | bool | false | 启用 MINCO 时间参数化速度参考比例映射 |
| `velocity_reference_topic` | string | `GridBased/timestamped_path` | `TimestampedPath` 话题名（与 corridor_planner 插件名一致） |
| `trajectory_optimizer_v_max` | double | 3.0 | 轨迹优化器速度上限（m/s），用于比例归一化 |

> 启用后，弯道/障碍段参考步长 = `hypot(vx_max, vy_max) × clamp(minco_speed / trajectory_optimizer_v_max, 0, 1)`。

### 目标减速参数（`enable_goal_slowdown: true` 时生效）

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `enable_goal_slowdown` | bool | true | 启用目标点线性减速 |
| `goal_slowdown_distance` | double | 1.5 | 开始减速的距终点距离（m） |

### 动态障碍预测规避参数（`enable_dynamic_obstacle_avoidance: true` 时生效）

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `enable_dynamic_obstacle_avoidance` | bool | false | 启用动态障碍物预测推离 |
| `dynamic_obstacle_topic` | string | `/dynamic_obstacles` | `TrackedObstacleArray` 话题名 |
| `dynamic_safety_margin` | double | 0.3 | 在障碍物半径基础上叠加的安全裕度（m） |
| `robot_radius` | double | 0.2 | 机器人半径，用于碰撞半径计算（m） |

---

## 发布/订阅话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `local_plan` | `nav_msgs/Path` | 发布 | MPC 参考轨迹可视化（odom 坐标系） |
| `active_line_marker` | `visualization_msgs/Marker` | 发布 | 当前激活直线段可视化 |
| `{velocity_reference_topic}` | `sentry_nav_interfaces/TimestampedPath` | 订阅 | MINCO 时间参数化路径（仅启用速度参考时） |
| `{dynamic_obstacle_topic}` | `sentry_nav_interfaces/TrackedObstacleArray` | 订阅 | 动态障碍物数组（仅启用动态障碍时） |

---

## 控制流程

```
setPlan()
  ├─ TF 查询 & 路径坐标转换（锁外）
  ├─ 计算新路径起始方向 & 在新路径上找最近点（锁外）
  └─ plan_mutex_ 加锁
      ├─ 路径方向拒绝判断（角度 + 距终点距离）
      ├─ 连续拒绝超限后强制接受并重置状态
      ├─ 写入 global_plan_odom_ & path_accumulated_dist_
      ├─ MINCO 时间戳匹配
      └─ 直线段检测 → long_lines_（始终执行）

computeVelocityCommands()  [plan_mutex_ 全程持有]
  ├─ TF 查询 base→odom
  ├─ updateTargetIndex()          # 动态窗口最近点搜索
  ├─ generateReferenceTrajectory()
  │   ├─ 判断 target_index_ 是否在直线段内
  │   ├─ 预计算 MINCO 比例步速（弯道备用）
  │   ├─ 快照动态障碍（dyn_obs_mutex_）
  │   └─ 循环生成 horizon 个参考点
  │       ├─ 三级优先级确定 step_speed
  │       │   ├─ [P1] 梯形速度曲线（直线段 + 启用时）
  │       │   ├─ [P2] MINCO 比例映射（弯道/无梯形时）
  │       │   └─ [P3] 默认均匀步长
  │       ├─ 弧长插值取参考点
  │       ├─ 动态障碍预测约束
  │       └─ 转换到机器人坐标系
  ├─ MPC::solve()                 # OSQP QP，热启动
  ├─ publishLocalPath()           # 有订阅者时发布
  ├─ publishActiveLineMarker()    # 有订阅者时发布
  ├─ applyGoalSlowdown()          # 线性减速系数
  ├─ applyAvoidanceSlowdown()     # 方向突变减速系数
  └─ 输出 cmd_vel（vx, vy）
```

---

## 配置示例

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 40.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "f_mpc_controller::MpcController"
      # 基础速度
      vx_min: -5.0
      vx_max:  5.0
      vy_min: -5.0
      vy_max:  5.0
      # MPC 权重
      QX: 20.0
      QY: 40.0
      R: 0.5
      S: 1.0
      horizon: 40
      control_frequency: 40.0
      # 路径拒绝
      reject_angle_deg: 180.0
      reject_dist_threshold: 2.0
      max_consecutive_rejects: 5
      # 自适应步长
      adaptive_step_lookahead_time: 1.2
      adaptive_step_curvature_gain: 2.5
      adaptive_step_min: 0.1
      adaptive_step_max: 1.0
      cursor_advance_gain: 3.0
      # 直线段检测
      straight_line_min_length: 2.0
      # 梯形速度曲线（直线段）
      enable_trapezoidal_accel: true
      trap_v_max: 5.0
      trap_v_end: 2.0
      trap_accel: 3.0
      trap_decel: 2.0
      # MINCO 速度参考比例映射（弯道/障碍段）
      enable_velocity_reference: false
      velocity_reference_topic: "GridBased/timestamped_path"
      trajectory_optimizer_v_max: 3.0
      # 目标减速
      enable_goal_slowdown: true
      goal_slowdown_distance: 1.5
      # 动态障碍预测（可选）
      enable_dynamic_obstacle_avoidance: false
      dynamic_obstacle_topic: "/dynamic_obstacles"
      dynamic_safety_margin: 0.3
      robot_radius: 0.2

velocity_smoother:
  ros__parameters:
    smoothing_frequency: 40.0   # 与 control_frequency 一致
    scale_velocities: false
    feedback: "OPEN_LOOP"
    max_velocity: [8.0, 8.0, 3.0]
    min_velocity: [-8.0, -8.0, -3.0]
    max_accel: [4.5, 4.5, 5.0]
    max_decel: [-4.5, -4.5, -5.0]
```

---

## 依赖

| 依赖 | 用途 |
|------|------|
| `nav2_core` | 控制器插件接口 |
| `nav2_costmap_2d` | Costmap 障碍查询 |
| `OsqpEigen` | QP 求解 |
| `sentry_nav_interfaces` | `TimestampedPath`, `TrackedObstacleArray` 消息 |
| `tf2_ros` / `tf2_geometry_msgs` | 坐标变换 |

---

## 参考

- [Nav2 官方文档](https://navigation.ros.org/)
- [OSQP](https://osqp.org/)
- [OSQP-Eigen](https://github.com/robotology/osqp-eigen)
