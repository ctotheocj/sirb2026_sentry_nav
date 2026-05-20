#pragma once
#ifndef LIDAR_PREPROCESSOR__GROUND_FILTER_HPP_
#define LIDAR_PREPROCESSOR__GROUND_FILTER_HPP_

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace lidar_preprocessor
{

struct GroundFilterParams
{
  double cell_size{0.4};
  double low_percentile{0.05};
  int    cell_min_points{5};
  double obstacle_min_height{0.05};
  double obstacle_max_height{1.80};
  bool   enable_normal_check{true};
  int    normal_min_neighbors{5};
  double ground_normal_cos_thresh{0.966};
  double slope_tolerance_height{0.15};
};

struct GroundFilterPoint
{
  int index{0};
  double height{0.0};
};

namespace internal
{

struct CellKey {
  int cx, cy;
  bool operator==(const CellKey & o) const { return cx == o.cx && cy == o.cy; }
};

struct CellHash {
  size_t operator()(const CellKey & k) const {
    size_t h = static_cast<size_t>(k.cx) * 2654435761ULL;
    h ^= static_cast<size_t>(k.cy) * 2246822519ULL;
    return h;
  }
};

inline Eigen::Vector3d estimateNormal(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const std::vector<int> & neighbors)
{
  if (static_cast<int>(neighbors.size()) < 3) return Eigen::Vector3d(0, 0, 1);
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (int idx : neighbors)
    centroid += Eigen::Vector3d(cloud.points[idx].x, cloud.points[idx].y, cloud.points[idx].z);
  centroid /= static_cast<double>(neighbors.size());
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (int idx : neighbors) {
    Eigen::Vector3d d(cloud.points[idx].x - centroid.x(),
                      cloud.points[idx].y - centroid.y(),
                      cloud.points[idx].z - centroid.z());
    cov += d * d.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
  if (solver.info() != Eigen::Success) return Eigen::Vector3d(0, 0, 1);
  Eigen::Vector3d n = solver.eigenvectors().col(0);
  if (n.z() < 0) n = -n;
  return n.normalized();
}

}  // namespace internal

inline std::vector<GroundFilterPoint> filterGroundDetailed(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const GroundFilterParams & p)
{
  using internal::CellKey;
  using internal::CellHash;

  const double inv = 1.0 / p.cell_size;

  std::unordered_map<CellKey, std::vector<int>, CellHash> cell_indices;
  cell_indices.reserve(cloud.size() / 4);
  for (int i = 0; i < static_cast<int>(cloud.size()); ++i) {
    const auto & pt = cloud.points[i];
    CellKey k{static_cast<int>(std::floor(static_cast<double>(pt.x) * inv)),
              static_cast<int>(std::floor(static_cast<double>(pt.y) * inv))};
    cell_indices[k].push_back(i);
  }

  std::unordered_map<CellKey, double, CellHash> cell_ref_z;
  cell_ref_z.reserve(cell_indices.size());
  for (const auto & [k, idxs] : cell_indices) {
    if (static_cast<int>(idxs.size()) < p.cell_min_points) continue;
    std::vector<double> zvals;
    zvals.reserve(idxs.size());
    for (int idx : idxs) zvals.push_back(static_cast<double>(cloud.points[idx].z));
    std::sort(zvals.begin(), zvals.end());
    const int pidx = std::clamp(
      static_cast<int>(std::floor(p.low_percentile * static_cast<double>(zvals.size() - 1))),
      0, static_cast<int>(zvals.size()) - 1);
    cell_ref_z[k] = zvals[pidx];
  }

  std::unordered_map<CellKey, bool, CellHash> cell_is_slope;
  if (p.enable_normal_check) {
    cell_is_slope.reserve(cell_ref_z.size());
    for (const auto & [k, ref_z] : cell_ref_z) {
      std::vector<int> neighbors;
      neighbors.reserve(50);
      for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy) {
          auto it = cell_indices.find({k.cx + dx, k.cy + dy});
          if (it != cell_indices.end())
            for (int idx : it->second) neighbors.push_back(idx);
        }
      if (static_cast<int>(neighbors.size()) < p.normal_min_neighbors) {
        cell_is_slope[k] = false;
        continue;
      }
      const Eigen::Vector3d n = internal::estimateNormal(cloud, neighbors);
      cell_is_slope[k] = (n.z() < p.ground_normal_cos_thresh);
    }
  }

  std::vector<GroundFilterPoint> result;
  result.reserve(cloud.size() / 2);
  for (const auto & [k, idxs] : cell_indices) {
    auto ref_it = cell_ref_z.find(k);
    if (ref_it == cell_ref_z.end()) continue;
    const double ref_z = ref_it->second;

    double min_h = p.obstacle_min_height;
    if (p.enable_normal_check) {
      auto slope_it = cell_is_slope.find(k);
      if (slope_it != cell_is_slope.end() && slope_it->second)
        min_h += p.slope_tolerance_height;
    }

    for (int idx : idxs) {
      const double dz = static_cast<double>(cloud.points[idx].z) - ref_z;
      if (dz >= min_h && dz <= p.obstacle_max_height) result.push_back({idx, dz});
    }
  }
  return result;
}

inline std::vector<GroundFilterPoint> collectGroundHeightPoints(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const GroundFilterParams & p,
  double min_height,
  double max_height)
{
  using internal::CellKey;
  using internal::CellHash;

  const double inv = 1.0 / p.cell_size;

  std::unordered_map<CellKey, std::vector<int>, CellHash> cell_indices;
  cell_indices.reserve(cloud.size() / 4);
  for (int i = 0; i < static_cast<int>(cloud.size()); ++i) {
    const auto & pt = cloud.points[i];
    CellKey k{static_cast<int>(std::floor(static_cast<double>(pt.x) * inv)),
              static_cast<int>(std::floor(static_cast<double>(pt.y) * inv))};
    cell_indices[k].push_back(i);
  }

  std::unordered_map<CellKey, double, CellHash> cell_ref_z;
  cell_ref_z.reserve(cell_indices.size());
  for (const auto & [k, idxs] : cell_indices) {
    if (static_cast<int>(idxs.size()) < p.cell_min_points) continue;
    std::vector<double> zvals;
    zvals.reserve(idxs.size());
    for (int idx : idxs) zvals.push_back(static_cast<double>(cloud.points[idx].z));
    std::sort(zvals.begin(), zvals.end());
    const int pidx = std::clamp(
      static_cast<int>(std::floor(p.low_percentile * static_cast<double>(zvals.size() - 1))),
      0, static_cast<int>(zvals.size()) - 1);
    cell_ref_z[k] = zvals[pidx];
  }

  std::vector<GroundFilterPoint> result;
  result.reserve(cloud.size());
  for (const auto & [k, idxs] : cell_indices) {
    auto ref_it = cell_ref_z.find(k);
    if (ref_it == cell_ref_z.end()) continue;
    const double ref_z = ref_it->second;
    for (int idx : idxs) {
      const double dz = static_cast<double>(cloud.points[idx].z) - ref_z;
      if (dz >= min_height && dz <= max_height) result.push_back({idx, dz});
    }
  }
  return result;
}

inline std::vector<int> filterGround(
  const pcl::PointCloud<pcl::PointXYZI> & cloud,
  const GroundFilterParams & p)
{
  const auto detailed = filterGroundDetailed(cloud, p);
  std::vector<int> result;
  result.reserve(detailed.size());
  for (const auto & point : detailed) {
    result.push_back(point.index);
  }
  return result;
}

}  // namespace lidar_preprocessor
#endif  // LIDAR_PREPROCESSOR__GROUND_FILTER_HPP_
