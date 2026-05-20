// Copyright 2025 Pan — Apache-2.0
// DynObjCluster: ROS2 adaptation of M-Detector cluster_predict

#pragma once
#ifndef CLUSTER_PREDICT__VOXEL_CLUSTER_H_
#define CLUSTER_PREDICT__VOXEL_CLUSTER_H_

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "m-detector/types.h"

// EA_disk: axis-aligned ellipse prediction for a dynamic cluster
// Predicts future positions as an ellipse (semi-axes a,b) centred at (cx,cy)
// propagated by velocity (vx,vy) over dt seconds.
struct EA_disk
{
    double cx{0}, cy{0};   // centre
    double vx{0}, vy{0};   // velocity (m/s)
    double a{0.5}, b{0.5}; // semi-axes (m)

    // Returns predicted centre at time dt ahead
    Eigen::Vector2d predict(double dt) const {
        return {cx + vx * dt, cy + vy * dt};
    }

    // Returns true if point (px,py) is inside the predicted ellipse at dt
    bool contains(double px, double py, double dt) const {
        auto c = predict(dt);
        double dx = px - c.x(), dy = py - c.y();
        return (dx*dx)/(a*a) + (dy*dy)/(b*b) <= 1.0;
    }
};

class DynObjCluster
{
public:
    float Voxel_resolution{0.3f};
    int   min_cluster_size{8};
    int   max_cluster_size{25000};
    bool  debug_en{false};

    // EA_disk predictions from last Clusterprocess call
    std::vector<EA_disk> predictions;

    DynObjCluster() = default;
    ~DynObjCluster() = default;

    void Init();

    // Main entry: classify dyn_tag points via voxel clustering + EA_disk prediction
    void Clusterprocess(
        std::vector<int> & dyn_tag,
        const pcl::PointCloud<PointType> & event_point,
        const pcl::PointCloud<PointType> & raw_point,
        const Eigen::Matrix3d & rot,
        const Eigen::Vector3d & pos);

private:
    struct VoxelKey {
        int x, y, z;
        bool operator==(const VoxelKey & o) const {
            return x == o.x && y == o.y && z == o.z;
        }
    };
    struct VoxelHash {
        size_t operator()(const VoxelKey & k) const {
            return std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 11)
                   ^ (std::hash<int>()(k.z) << 22);
        }
    };

    // Previous frame cluster centres for velocity estimation
    std::vector<Eigen::Vector2d> prev_centres_;
    double prev_time_{-1.0};

    void buildVoxelMap(
        const pcl::PointCloud<PointType> & cloud,
        std::unordered_map<VoxelKey, std::vector<int>, VoxelHash> & vmap);

    void clusterBFS(
        const std::unordered_map<VoxelKey, std::vector<int>, VoxelHash> & vmap,
        std::vector<std::vector<int>> & clusters);
};

#endif  // CLUSTER_PREDICT__VOXEL_CLUSTER_H_
