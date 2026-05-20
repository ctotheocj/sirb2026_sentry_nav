// Copyright 2025 Pan
// Licensed under the Apache License, Version 2.0
//
// 自适应网格地面分割
//
// 算法: 将 XY 平面划分为均匀格子，每格取低百分位 z 值作为本地地面高度估计，
//       再经 3×3 邻居均值平滑，最后按 [ground_z + min_h, ground_z + max_h]
//       保留障碍物候选点。
//
// 优点:
//   - O(N) 均摊复杂度（哈希网格）
//   - 天然适应 15~20° 坡面，无需全局平面假设
//   - 无需 IMU / 里程计信息
//
// 参数推荐 (RM 室内赛场 + MID360):
//   cell_size          = 0.4 m   (约 2× 机器人体宽)
//   low_percentile     = 0.05    (5% 分位，抑制单点噪声)
//   obstacle_min_height= 0.10 m  (轮子以上)
//   obstacle_max_height= 1.80 m  (机器人最高点以上留余量)

#pragma once

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

namespace dynamic_obstacle_tracker
{

struct GroundFilterParams
{
  double cell_size{0.4};           // 网格尺寸 (m)
  double low_percentile{0.05};     // 取格内最低 p% z 值为地面
  double obstacle_min_height{0.10}; // 地面以上最小有效高度 (m)
  double obstacle_max_height{1.80}; // 地面以上最大有效高度 (m)
};

// ----------------------------------------------------------------
//  filterGround
//
//  输入:  pts           - 原始 3D 点云 (Eigen::Vector3d)
//  输入:  params        - 参数
//  输出:  obstacle_pts  - 过滤地面后的障碍物候选点（仅 XY 用于后续聚类）
//         obstacle_z    - 对应的平均 z（用于可视化高度）
//  返回值: obstacle_pts 对应原始点云的下标（用于彩色可视化）
// ----------------------------------------------------------------
inline std::vector<int> filterGround(
  const std::vector<Eigen::Vector3d> & pts,
  const GroundFilterParams & params,
  std::vector<Eigen::Vector2d> & obstacle_pts_2d)
{
  obstacle_pts_2d.clear();
  if (pts.empty()) { return {}; }

  const double cs = params.cell_size;

  // ---- Step 1: 建网格，收集每格 z 值 ----
  struct CellKey {
    int gx, gy;
    bool operator==(const CellKey & o) const { return gx == o.gx && gy == o.gy; }
  };
  struct CellKeyHash {
    size_t operator()(const CellKey & k) const {
      return std::hash<int>{}(k.gx) ^ (std::hash<int>{}(k.gy) * 2654435761ULL);
    }
  };

  std::unordered_map<CellKey, std::vector<float>, CellKeyHash> cell_zs;
  cell_zs.reserve(pts.size() / 4);

  for (const auto & p : pts) {
    if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) { continue; }
    CellKey k{
      static_cast<int>(std::floor(p.x() / cs)),
      static_cast<int>(std::floor(p.y() / cs))};
    cell_zs[k].push_back(static_cast<float>(p.z()));
  }

  // ---- Step 2: 每格计算低百分位 z（地面高度估计）----
  std::unordered_map<CellKey, float, CellKeyHash> ground_z;
  ground_z.reserve(cell_zs.size());

  for (auto & [k, zvals] : cell_zs) {
    if (zvals.empty()) { continue; }
    // 排序后取低 percentile
    // 单点(size=1)时直接用该点 z 值; ≥2 点时用百分位抑制高点噪声
    // 不跳过单点格：坡面边缘 cell 往往只有 1-2 个点, 跳过会破坏邻居平滑
    std::sort(zvals.begin(), zvals.end());
    int idx = std::max(0, static_cast<int>(params.low_percentile * zvals.size()) - 1);
    ground_z[k] = zvals[static_cast<size_t>(idx)];
  }

  // ---- Step 3: 3×3 邻居均值平滑（处理无数据格）----
  // 对每个有数据的格，查询其 3×3 邻居的 ground_z 均值
  auto getGroundZ = [&](int gx, int gy) -> double {
    float sum = 0.0f;
    int cnt = 0;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        auto it = ground_z.find(CellKey{gx + dx, gy + dy});
        if (it != ground_z.end()) {
          sum += it->second;
          ++cnt;
        }
      }
    }
    if (cnt == 0) { return -1e9; }  // 无邻居，不过滤（保留该点）
    return static_cast<double>(sum) / cnt;
  };

  // ---- Step 4: 逐点分类 ----
  std::vector<int> result_indices;
  result_indices.reserve(pts.size());
  obstacle_pts_2d.reserve(pts.size());

  const int N = static_cast<int>(pts.size());
  for (int i = 0; i < N; ++i) {
    const auto & p = pts[i];
    if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) { continue; }

    int gx = static_cast<int>(std::floor(p.x() / cs));
    int gy = static_cast<int>(std::floor(p.y() / cs));
    double gz = getGroundZ(gx, gy);

    double rel_z = p.z() - gz;
    if (rel_z < params.obstacle_min_height) { continue; }   // 地面点
    if (rel_z > params.obstacle_max_height) { continue; }   // 超高噪声

    obstacle_pts_2d.emplace_back(p.x(), p.y());
    result_indices.push_back(i);
  }

  return result_indices;
}

}  // namespace dynamic_obstacle_tracker
