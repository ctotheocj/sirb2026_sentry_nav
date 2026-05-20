# dynamic_obstacle_tracker

基于 LiDAR 点云的动态障碍物（敌方机器人）实时检测与跟踪。
直接订阅原始点云，内部完成地面分割 + 网格哈希 DBSCAN 聚类 + Gap 分裂 + EKF-SORT 多目标跟踪。

## 节点

`obstacle_tracker_node`（类名：`ObstacleTrackerNode`）

## 话题

### 订阅

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cloud_registered` | `sensor_msgs/PointCloud2` | Point-LIO map 帧点云（实车）；仿真改为 `/pointcloud` |

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `/dynamic_obstacles` | `sentry_nav_interfaces/TrackedObstacleArray` | 跟踪障碍物数组（ID、位置、速度、半径、置信度） |
| `/dynamic_obstacles_viz` | `visualization_msgs/MarkerArray` | 圆柱 + 速度箭头 markers |
| `/dynamic_obstacles_cloud` | `sensor_msgs/PointCloud2` | 彩色聚类点云（每个障碍物簇独立颜色，用于 RViz 调试） |

## 算法流程

```
原始点云 (/cloud_registered)
  │
  ├─ 1. 体素下采样 (voxel_leaf_size)
  │
  ├─ 2. 自适应网格地面分割 (ground_filter.hpp)
  │      • XY 平面划格 (cell_size=0.4m)
  │      • 每格取低 5% z 分位 → 本地地面高度
  │      • 3×3 邻居均值平滑
  │      • 保留 z ∈ [地面+min_h, 地面+max_h] 的点
  │      → 天然适配 15-20° 坡面，无需全局平面假设
  │
  ├─ 3. 网格哈希 DBSCAN 2D 聚类，O(N) 均摊
  │
  ├─ 4. Gap-Split 后处理：PCA 投影 + 最大间隙检测，分离多机器人合并簇
  │
  ├─ 5. EKF-SORT 多目标跟踪：状态 [px, py, vx, vy]，贪心匈牙利关联
  │
  └─ 发布三路输出
```

## EKF 状态与轨迹生命周期

- 状态向量：`[px, py, vx, vy]`，匀速运动模型
- 轨迹：`New` → `Confirmed`（`age >= confirm_frames`）→ `Deleted`（`missed > max_missed`）

## 参数

```yaml
obstacle_tracker_node:
  ros__parameters:
    # 输入话题
    pointcloud_topic: /cloud_registered   # 实车; 仿真改 /pointcloud

    # 体素下采样
    voxel_leaf_size: 0.07                 # (m)

    # 地面分割
    ground_cell_size: 0.4                 # 网格尺寸 (m)
    ground_low_percentile: 0.05           # 低百分位（5%）作为地面估计
    obstacle_min_height: 0.10             # 地面以上最小有效高度 (m)
    obstacle_max_height: 1.80             # 地面以上最大有效高度 (m)

    # DBSCAN 聚类
    dbscan_epsilon: 0.5                   # 邻域半径 (m)
    dbscan_min_pts: 3                     # 核心点最小邻居数

    # Gap 分裂
    gap_split_max_single_radius: 0.28     # 单个机器人最大半径 (m)
    gap_split_threshold: 0.06             # 分裂最小间隙 (m)

    # EKF-SORT
    q_pos: 0.01
    q_vel: 2.0                            # 全向轮大机动，速度噪声需大
    r_pos: 0.05
    association_threshold: 1.5            # 匹配距离阈值 (m)
    confirm_frames: 3
    max_missed_frames: 5
    obstacle_radius_default: 0.35         # (m)
    max_output_obstacles: 7
```

## 输出消息格式

`sentry_nav_interfaces/TrackedObstacle`：

```
int32 id
float64 x, y       # 位置 (map 坐标系)
float64 vx, vy     # 速度 (m/s)
float64 radius     # 障碍物半径 (m)
float32 confidence # 置信度 [0, 1]
```

## 依赖

`rclcpp` `sensor_msgs` `std_msgs` `visualization_msgs` `pcl` `eigen` `sentry_nav_interfaces`
