// Copyright 2025 Pan — Apache-2.0
// M-Detector types: ROS2 adaptation of m-detector/include/types.h

#pragma once
#ifndef M_DETECTOR__TYPES_H_
#define M_DETECTOR__TYPES_H_

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Eigen type aliases matching original M-Detector
using V3D = Eigen::Vector3d;
using V3F = Eigen::Vector3f;
using M3D = Eigen::Matrix3d;
using M3F = Eigen::Matrix3f;

// Point type used throughout M-Detector
using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<PointType>;

// Voxel cluster point container (used by DynObjCluster)
struct Point_Cloud
{
    std::vector<int> point_index;
    int voxel_index{-1};
};

#endif  // M_DETECTOR__TYPES_H_
