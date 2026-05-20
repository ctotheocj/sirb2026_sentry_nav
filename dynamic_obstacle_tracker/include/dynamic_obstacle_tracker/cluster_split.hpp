// Copyright 2025 Pan
// Licensed under the Apache License, Version 2.0
//
// Gap-Split 后处理: 对 DBSCAN 合并的两台机器人进行分裂
//
// 算法: PCA 主轴投影 + 最大间隙检测
// 时间复杂度: O(k log k)，k = 簇内点数（通常 5-30）
//
// 物理参数推导（RM 机器人 + VLP-16）:
//   激光在 D 米处水平角分辨率 0.2°:
//     within-robot 点间距 ≈ D * sin(0.2°) ≈ D * 0.0035m
//     D=3m  → 0.010m;  D=10m → 0.035m;  D=15m → 0.052m
//   两台 RM 机器人体表间距:
//     center-to-center 0.40m - 2×0.15m 体半径 = 0.10m
//   GAP_THRESHOLD = 0.06m 覆盖范围:
//     0.010m(3m) << 0.06m << 0.10m  → 可靠区分 (3~8m 范围)
//     0.052m(15m) < 0.06m           → 10m 以上裕量不足 ~1.15×
//   若需 10m 以上覆盖: gap_threshold = 0.04 + 0.002 * dist_to_lidar（距离自适应）
//
// 终止条件:
//   1. 子簇 radius <= max_single_robot_radius → 已是单台，不再分裂
//   2. 子簇点数 < min_cluster_pts → 无效子簇，丢弃
//   3. 递归深度 >= max_depth → 防异常点云死循环（RM 场地 ≤7 台，max_depth=3 充足）

#pragma once

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "dynamic_obstacle_tracker/dbscan.hpp"

namespace dynamic_obstacle_tracker
{

// ----------------------------------------------------------------
//  splitOversizedCluster（内部递归函数）
//
//  参数:
//    pts                    - 完整点云
//    indices                - 本簇点下标列表
//    gap_threshold          - 最小分裂间隙 (m)，建议 0.06
//    max_single_robot_radius - 单台机器人最大 DBSCAN 半径 (m)，建议 0.28
//    min_cluster_pts        - 子簇最少点数（建议 3 = DBSCAN min_pts）
//    depth                  - 当前递归深度（初始 0，上限 max_depth）
//    max_depth              - 最大递归深度（建议 3，对应 ≤8 台机器人合并）
// ----------------------------------------------------------------
inline std::vector<ClusterResult> splitOversizedCluster(
  const std::vector<Eigen::Vector2d> & pts,
  const std::vector<int> & indices,
  double gap_threshold,
  double max_single_robot_radius,
  int min_cluster_pts,
  int depth,
  int max_depth)
{
  const int k = static_cast<int>(indices.size());

  // --- 终止条件 2: 点数不足 → 丢弃（不产生有效簇）---
  if (k < min_cluster_pts) { return {}; }

  // 计算质心和包围半径
  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (int idx : indices) { centroid += pts[idx]; }
  centroid /= k;

  double radius_sq = 0.0;
  for (int idx : indices) {
    double d2 = (pts[idx] - centroid).squaredNorm();
    if (d2 > radius_sq) { radius_sq = d2; }
  }
  const double radius = std::sqrt(radius_sq);

  // --- 终止条件 1: 子簇已是单台机器人大小 → 直接返回 ---
  // --- 终止条件 3: 递归深度上限 ---
  if (radius <= max_single_robot_radius || depth >= max_depth) {
    ClusterResult r;
    r.centroid = centroid;
    r.radius = radius;
    r.point_count = k;
    return {r};
  }

  // --- PCA: 计算 2×2 协方差矩阵 → 主轴 ---
  Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
  for (int idx : indices) {
    Eigen::Vector2d d = pts[idx] - centroid;
    cov += d * d.transpose();
  }
  cov /= k;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
  Eigen::Vector2d axis = solver.eigenvectors().col(1);  // 最大特征值对应的主轴

  // --- 投影到主轴并排序 ---
  std::vector<std::pair<double, int>> proj;
  proj.reserve(k);
  for (int idx : indices) {
    proj.push_back({(pts[idx] - centroid).dot(axis), idx});
  }
  std::sort(proj.begin(), proj.end());

  // --- 找最大间隙 ---
  double max_gap = 0.0;
  int split_pos = -1;
  for (int i = 0; i < k - 1; ++i) {
    double gap = proj[i + 1].first - proj[i].first;
    if (gap > max_gap) { max_gap = gap; split_pos = i; }
  }

  // --- 无显著间隙 → 不分裂（单个大障碍物）---
  if (max_gap < gap_threshold || split_pos < 0) {
    ClusterResult r;
    r.centroid = centroid;
    r.radius = radius;
    r.point_count = k;
    return {r};
  }

  // --- 分裂：递归处理两个子组 ---
  std::vector<int> group_a, group_b;
  for (int i = 0; i <= split_pos; ++i) { group_a.push_back(proj[i].second); }
  for (int i = split_pos + 1; i < k; ++i) { group_b.push_back(proj[i].second); }

  auto results_a = splitOversizedCluster(
    pts, group_a, gap_threshold, max_single_robot_radius, min_cluster_pts, depth + 1, max_depth);
  auto results_b = splitOversizedCluster(
    pts, group_b, gap_threshold, max_single_robot_radius, min_cluster_pts, depth + 1, max_depth);

  std::vector<ClusterResult> out;
  out.insert(out.end(), results_a.begin(), results_a.end());
  out.insert(out.end(), results_b.begin(), results_b.end());
  return out;
}

// ----------------------------------------------------------------
//  postProcessClusters
//
//  对 dbscan2D() 的输出进行 Gap-Split 后处理。
//  只处理 radius > max_single_robot_radius 的过大簇。
//
//  参数:
//    clusters               - dbscan2D() 输出
//    pts                    - 原始点云
//    labels                 - dbscan2D 输出的点标签（与 pts 等长）
//    max_single_robot_radius - 单台机器人最大包围半径，建议 0.28m
//    gap_threshold          - 分裂间隙阈值，建议 0.06m
//    min_cluster_pts        - 子簇最少点数，建议 3
// ----------------------------------------------------------------
inline std::vector<ClusterResult> postProcessClusters(
  const std::vector<ClusterResult> & clusters,
  const std::vector<Eigen::Vector2d> & pts,
  const std::vector<int> & labels,
  double max_single_robot_radius = 0.28,
  double gap_threshold = 0.06,
  int min_cluster_pts = 3)
{
  const int num_clusters = static_cast<int>(clusters.size());

  // 将点云按簇 ID 分组
  std::vector<std::vector<int>> cluster_indices(num_clusters);
  for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
    if (labels[i] >= 0 && labels[i] < num_clusters) {
      cluster_indices[labels[i]].push_back(i);
    }
  }

  std::vector<ClusterResult> output;
  output.reserve(num_clusters);

  for (int c = 0; c < num_clusters; ++c) {
    if (clusters[c].radius <= max_single_robot_radius) {
      output.push_back(clusters[c]);
    } else {
      // 过大 → Gap-Split（初始 depth=0, max_depth=3）
      auto sub = splitOversizedCluster(
        pts, cluster_indices[c], gap_threshold, max_single_robot_radius,
        min_cluster_pts, 0, 3);
      for (auto & s : sub) { output.push_back(s); }
    }
  }

  return output;
}

// ----------------------------------------------------------------
//  dbscan2DWithSplit
//
//  完整流程: dbscan2D → postProcessClusters(Gap-Split)
//
//  额外参数（相比 dbscan2D）:
//    max_single_robot_radius - 单台机器人最大包围半径 (m)，建议 0.28
//    gap_threshold           - 分裂间隙阈值 (m)，建议 0.06
// ----------------------------------------------------------------
inline std::vector<ClusterResult> dbscan2DWithSplit(
  const std::vector<Eigen::Vector2d> & pts,
  double epsilon,
  int min_pts,
  double max_single_robot_radius = 0.28,
  double gap_threshold = 0.06)
{
  if (pts.empty()) { return {}; }

  const int N = static_cast<int>(pts.size());
  const double cell_size = epsilon;

  struct GridKey { int gx, gy;
    bool operator==(const GridKey & o) const { return gx == o.gx && gy == o.gy; }
  };
  struct GridKeyHash {
    size_t operator()(const GridKey & k) const {
      return std::hash<int>{}(k.gx) ^ (std::hash<int>{}(k.gy) * 2654435761ULL);
    }
  };

  std::unordered_map<GridKey, std::vector<int>, GridKeyHash> grid;
  grid.reserve(N);
  for (int i = 0; i < N; ++i) {
    int gx = static_cast<int>(std::floor(pts[i].x() / cell_size));
    int gy = static_cast<int>(std::floor(pts[i].y() / cell_size));
    grid[GridKey{gx, gy}].push_back(i);
  }

  auto getNeighbors = [&](int i) -> std::vector<int> {
    std::vector<int> nb;
    int gx = static_cast<int>(std::floor(pts[i].x() / cell_size));
    int gy = static_cast<int>(std::floor(pts[i].y() / cell_size));
    const double eps2 = epsilon * epsilon;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        auto it = grid.find(GridKey{gx + dx, gy + dy});
        if (it == grid.end()) { continue; }
        for (int j : it->second) {
          if ((pts[i] - pts[j]).squaredNorm() <= eps2) { nb.push_back(j); }
        }
      }
    }
    return nb;
  };

  std::vector<int> label(N, -2);
  int cluster_id = 0;

  for (int i = 0; i < N; ++i) {
    if (label[i] != -2) { continue; }
    auto neighbors = getNeighbors(i);
    if (static_cast<int>(neighbors.size()) < min_pts) { label[i] = -1; continue; }
    label[i] = cluster_id;
    std::vector<int> queue = neighbors;
    for (int qi = 0; qi < static_cast<int>(queue.size()); ++qi) {
      int q = queue[qi];
      if (label[q] == -1) { label[q] = cluster_id; }
      if (label[q] != -2) { continue; }
      label[q] = cluster_id;
      auto qn = getNeighbors(q);
      if (static_cast<int>(qn.size()) >= min_pts) {
        for (int nb : qn) {
          if (label[nb] == -2 || label[nb] == -1) { queue.push_back(nb); }
        }
      }
    }
    ++cluster_id;
  }

  // 计算初始簇的质心和半径
  std::vector<ClusterResult> results(cluster_id);
  std::vector<int> counts(cluster_id, 0);
  for (int i = 0; i < N; ++i) {
    if (label[i] < 0) { continue; }
    results[label[i]].centroid += pts[i];
    ++counts[label[i]];
  }
  for (int c = 0; c < cluster_id; ++c) {
    if (counts[c] > 0) { results[c].centroid /= counts[c]; results[c].point_count = counts[c]; }
  }
  for (int i = 0; i < N; ++i) {
    if (label[i] < 0) { continue; }
    double d2 = (pts[i] - results[label[i]].centroid).squaredNorm();
    if (d2 > results[label[i]].radius) { results[label[i]].radius = d2; }
  }
  for (auto & r : results) { r.radius = std::sqrt(r.radius); }

  // Gap-Split 后处理
  return postProcessClusters(results, pts, label, max_single_robot_radius, gap_threshold, min_pts);
}

}  // namespace dynamic_obstacle_tracker
