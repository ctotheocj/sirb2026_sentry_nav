// Copyright 2025 Pan
// Licensed under the Apache License, Version 2.0
//
// DBSCAN 2D 聚类 - 网格哈希加速，O(N) 平均复杂度
// 输入: 2D 点集（已经过 z 过滤和体素下采样）
// 输出: 簇的质心 + 包围半径 + 点数

#pragma once

#include <cmath>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

namespace dynamic_obstacle_tracker
{

struct ClusterResult
{
  Eigen::Vector2d centroid;   // 质心
  double radius;              // 包围半径: sqrt(max(|p-centroid|²))
  int point_count;
};

// 网格哈希键
struct GridKey
{
  int gx, gy;
  bool operator==(const GridKey & other) const { return gx == other.gx && gy == other.gy; }
};

struct GridKeyHash
{
  size_t operator()(const GridKey & k) const
  {
    // Cantor 配对函数
    size_t hx = std::hash<int>{}(k.gx);
    size_t hy = std::hash<int>{}(k.gy);
    return hx ^ (hy * 2654435761ULL);
  }
};

// DBSCAN 2D
// epsilon: 邻域半径 (m)
// min_pts: 核心点最小邻域点数
inline std::vector<ClusterResult> dbscan2D(
  const std::vector<Eigen::Vector2d> & pts,
  double epsilon,
  int min_pts)
{
  if (pts.empty()) { return {}; }

  const int N = static_cast<int>(pts.size());
  const double cell_size = epsilon;

  // 建立网格哈希: cell → 点索引列表
  std::unordered_map<GridKey, std::vector<int>, GridKeyHash> grid;
  grid.reserve(N);
  for (int i = 0; i < N; ++i) {
    GridKey k;
    k.gx = static_cast<int>(std::floor(pts[i].x() / cell_size));
    k.gy = static_cast<int>(std::floor(pts[i].y() / cell_size));
    grid[k].push_back(i);
  }

  // 查询点 i 的 epsilon 邻域（3×3 块搜索）
  auto getNeighbors = [&](int i) -> std::vector<int> {
    std::vector<int> neighbors;
    int gx = static_cast<int>(std::floor(pts[i].x() / cell_size));
    int gy = static_cast<int>(std::floor(pts[i].y() / cell_size));
    const double eps2 = epsilon * epsilon;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        GridKey k{gx + dx, gy + dy};
        auto it = grid.find(k);
        if (it == grid.end()) { continue; }
        for (int j : it->second) {
          if ((pts[i] - pts[j]).squaredNorm() <= eps2) {
            neighbors.push_back(j);
          }
        }
      }
    }
    return neighbors;
  };

  // DBSCAN BFS
  std::vector<int> label(N, -2);  // -2=未访问, -1=噪声, >=0=簇ID
  int cluster_id = 0;

  for (int i = 0; i < N; ++i) {
    if (label[i] != -2) { continue; }
    auto neighbors = getNeighbors(i);
    if (static_cast<int>(neighbors.size()) < min_pts) {
      label[i] = -1;  // 噪声
      continue;
    }
    label[i] = cluster_id;
    // BFS 扩展
    std::vector<int> queue = neighbors;
    for (int qi = 0; qi < static_cast<int>(queue.size()); ++qi) {
      int q = queue[qi];
      if (label[q] == -1) { label[q] = cluster_id; }  // 噪声→边界点
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

  // 计算每簇质心和包围半径
  std::vector<ClusterResult> results(cluster_id);
  std::vector<int> counts(cluster_id, 0);

  for (int i = 0; i < N; ++i) {
    if (label[i] < 0) { continue; }
    int cid = label[i];
    results[cid].centroid += pts[i];
    ++counts[cid];
  }
  for (int c = 0; c < cluster_id; ++c) {
    if (counts[c] > 0) {
      results[c].centroid /= counts[c];
      results[c].point_count = counts[c];
    }
  }
  for (int i = 0; i < N; ++i) {
    if (label[i] < 0) { continue; }
    int cid = label[i];
    double d2 = (pts[i] - results[cid].centroid).squaredNorm();
    if (d2 > results[cid].radius) { results[cid].radius = d2; }
  }
  for (auto & r : results) { r.radius = std::sqrt(r.radius); }

  return results;
}

}  // namespace dynamic_obstacle_tracker
