# ndt_omp_relocalization

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

基于 [ndt_omp](https://github.com/koide3/ndt_omp) 多线程 NDT（Normal Distributions Transform）算法的点云重定位节点，专为 RoboMaster 哨兵导航系统设计。

给定一个已配准的点云（基于 odom 坐标系）和先验点云地图（使用 [Point-LIO](https://github.com/LihanChen2004/Point-LIO) 等建图工具生成），本节点将计算两组点云之间的变换，并发布从 `map` 坐标系到 `odom` 坐标系的修正 TF 变换。

## 架构概述

```
registered_scan (PointCloud2)  ──►  VoxelGrid 下采样  ──►  NDT-OMP 配准
                                                              │
                                                              ▼
                                                    performRegistration()
                                                     (跳变检查 + 信任策略)
                                                              │
                                                              ▼
prior PCD map  ──►  loadGlobalMap()  ──►  TF 变换到 odom 系  ──►  setInputTarget()
                                                              │
                                                              ▼
                                                    publishTransform()  ──►  map → odom TF
                                                      (20Hz 持续发布)
```

## 依赖

- ROS2 Humble
- [ndt_omp_ros2](https://github.com/koide3/ndt_omp)（需编译为共享库）
- PCL (common, io, filters)
- OpenMP
- tf2 / tf2_eigen / tf2_ros
- pcl_conversions

## 编译

### 1. 克隆仓库

```bash
mkdir -p ~/rm_2025_ws/src
cd ~/rm_2025_ws/src

# 克隆 ndt_omp_ros2 依赖（如果尚未存在）
git clone https://github.com/koide3/ndt_omp.git ndt_omp_ros2

# 确保 ndt_omp_relocalization 已放置在工作空间中
# 例如: src/sirb2026_sentry_nav/ndt_omp_relocalization/
```

### 2. 确保 ndt_omp_ros2 编译为共享库

`ndt_omp_ros2` 的 `CMakeLists.txt` 需要将 `add_library` 改为 `SHARED` 并添加安装目标：

```cmake
add_library(ndt_omp SHARED ...)
target_link_libraries(ndt_omp ${PCL_LIBRARIES})
ament_export_libraries(ndt_omp)

install(TARGETS ndt_omp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
```

### 3. 安装依赖

```bash
cd ~/rm_2025_ws
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

### 4. 编译

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select ndt_omp_ros2 ndt_omp_relocalization
```

## 使用方法

### 独立启动

```bash
ros2 launch ndt_omp_relocalization ndt_omp_relocalization_launch.py
```

### 通过 sirb2026_nav_bringup 启动（推荐）

`sirb2026_nav_bringup` 固定启动轻量 NDT-OMP 后端：

```bash
ros2 launch sirb2026_nav_bringup rm_navigation_reality_launch.py
```

### 手动初始位姿

在 RViz2 中使用 `2D Pose Estimate` 工具发布 `/initialpose`，节点会自动计算 `map→odom` 变换并重置信任状态。

## 参数说明

### NDT 配准参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `num_threads` | int | `4` | NDT-OMP 使用的 OpenMP 线程数 |
| `ndt_resolution` | float | `1.0` | NDT 体素网格分辨率 (m)，越小越精确但越慢 |
| `ndt_step_size` | float | `0.1` | NDT 优化步长，More-Thuente 线搜索的最大步长 |
| `ndt_epsilon` | float | `0.01` | NDT 收敛判据阈值 |
| `ndt_max_iterations` | int | `30` | NDT 最大迭代次数 |
| `ndt_search_method` | int | `1` | 邻域搜索方法: `0`=KDTREE, `1`=DIRECT7, `2`=DIRECT1, `3`=DIRECT26 |
| `fitness_score_threshold` | float | `1.0` | 适配度分数阈值，超过此值视为未收敛 |

### 点云处理参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `global_leaf_size` | float | `0.25` | 全局地图体素下采样分辨率 (m) |
| `registered_leaf_size` | float | `0.25` | 输入扫描点云体素下采样分辨率 (m) |
| `prior_pcd_file` | string | `""` | 先验 PCD 地图文件的绝对路径 |

### 坐标系参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `map_frame` | string | `"map"` | 地图坐标系名称 |
| `odom_frame` | string | `"odom"` | 里程计坐标系名称 |
| `base_frame` | string | `""` | 速度参考坐标系（用于 lidar→odom 变换） |
| `robot_base_frame` | string | `""` | 机器人底盘坐标系（用于 initialpose 计算） |
| `lidar_frame` | string | `""` | 激光雷达坐标系名称 |

### 位姿修正参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `init_pose` | double[] | `[0,0,0,0,0,0]` | 初始位姿 `[x, y, z, roll, pitch, yaw]` |
| `enable_roll_pitch_fix` | bool | `true` | 是否将 roll/pitch 强制归零（适用于 2D 导航） |
| `trust_ndt_threshold` | int | `5` | 连续收敛多少帧后信任 NDT 结果 |
| `jump_threshold_xy` | float | `0.5` | XY 方向跳变阈值 (m) |
| `jump_threshold_yaw` | float | `0.3` | Yaw 方向跳变阈值 (rad) |
| `jump_threshold_rp` | float | `0.1` | Roll/Pitch 方向跳变阈值 (rad) |

### 发布策略参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `publish_tf_only_when_trusted` | bool | `false` | ~~仅在信任时发布 TF~~ （当前版本始终发布以保证 `map` 帧存在） |
| `freeze_tf_when_not_trusted` | bool | `false` | 不可信时冻结在最后一次可信位姿 |
| `use_scan_stamp_for_tf` | bool | `true` | 使用扫描时间戳（而非当前时间）作为 TF 时间戳 |

## 订阅话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `registered_scan` | `sensor_msgs/msg/PointCloud2` | 已配准的点云（基于 odom 系） |
| `initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 手动初始位姿（来自 RViz2） |

## 发布内容

| 内容 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `map` → `odom` TF | `tf2_msgs/msg/TFMessage` | 20 Hz | 地图到里程计坐标系的校正变换 |

## NDT 搜索方法对比

| 方法 | 说明 | 速度 | 适用场景 |
|------|------|------|----------|
| `KDTREE` (0) | 标准 KD-Tree 搜索 | 较慢 | 通用场景 |
| `DIRECT7` (1) | 直接搜索 7 个相邻体素 |  快 |
| `DIRECT1` (2) | 仅搜索最近的 1 个体素 | 最快 | 点云质量好、初始猜测准确 |
| `DIRECT26` (3) | 搜索 26 个相邻体素 | 最慢 | 需要极高鲁棒性 |

## 参数调优指南

### 快速入门（RM 比赛推荐配置）

```yaml
ndt_omp_relocalization:
  ros__parameters:
    num_threads: 4
    ndt_resolution: 1.0
    ndt_step_size: 0.1
    ndt_epsilon: 0.01
    ndt_max_iterations: 30
    ndt_search_method: 1          # DIRECT7
    fitness_score_threshold: 1.0
    global_leaf_size: 0.25
    registered_leaf_size: 0.25
    enable_roll_pitch_fix: true
    trust_ndt_threshold: 5
    jump_threshold_xy: 0.5
    jump_threshold_yaw: 0.3
    jump_threshold_rp: 0.1
    publish_tf_only_when_trusted: false
    freeze_tf_when_not_trusted: false
    use_scan_stamp_for_tf: true
```

### 高精度调优路线

1. **降低 `ndt_resolution`** (0.5~0.8) → 提高精度，但增加计算量
2. **增加 `ndt_max_iterations`** (50) → 确保充分收敛
3. **降低 `fitness_score_threshold`** (0.3~0.5) → 更严格的收敛判定
4. **增加 `trust_ndt_threshold`** (8~10) → 更保守的信任策略
5. **降低 `jump_threshold_xy`** (0.2~0.3) → 更灵敏的跳变检测

### 速度优先调优路线

1. **增大 `ndt_resolution`** (2.0~3.0) → 减少体素数量
2. **增大 `registered_leaf_size`** (0.5) → 减少输入点数
3. **减少 `ndt_max_iterations`** (15~20) → 提前终止
4. **使用 `ndt_search_method: 2`** (DIRECT1) → 最快搜索

如需在其他项目中使用本包，需满足以下条件：

1. **依赖**：确保 `ndt_omp_ros2` 已编译并安装为共享库
2. **TF 树**：需要存在 `odom → base_link` 的 TF 变换（通常由里程计提供）
3. **输入话题**：发布已配准的点云到 `registered_scan`（基于 odom 坐标系）
4. **先验地图**：提供 PCD 格式的先验点云地图文件
5. **参数配置**：正确设置 `base_frame`、`robot_base_frame`、`lidar_frame` 等坐标系名称

## 许可证

Apache License 2.0 - 详见 [LICENSE](../../LICENSE) 文件。
