// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NDT_OMP_RELOCALIZATION__NDT_OMP_RELOCALIZATION_HPP_
#define NDT_OMP_RELOCALIZATION__NDT_OMP_RELOCALIZATION_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pclomp/ndt_omp.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace ndt_omp_relocalization
{

class NdtOmpRelocalizationNode : public rclcpp::Node
{
public:
  explicit NdtOmpRelocalizationNode(const rclcpp::NodeOptions & options);

private:
  struct QualityMetrics
  {
    int sampled_points{0};
    int valid_correspondences{0};
    double overlap_ratio{0.0};
    double median_residual{0.0};
    double p90_residual{0.0};
  };

  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadGlobalMap(const std::string & file_name);
  void performRegistration();
  void publishTransform();
  void broadcastTransform(const Eigen::Isometry3d & pose);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  bool checkJump(const Eigen::Isometry3d & new_pose, const Eigen::Isometry3d & prev_pose) const;
  Eigen::Isometry3d applyZeroRollPitch(const Eigen::Isometry3d & ndt_pose) const;
  bool setupGlobalMap();
  bool evaluateAlignmentQuality(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
    const Eigen::Isometry3d & map_to_odom,
    QualityMetrics & metrics) const;

  /// Extract yaw from an Eigen rotation matrix (no tf2 conversion overhead).
  static double getYaw(const Eigen::Matrix3d & rot);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  // NDT parameters
  int num_threads_;
  float ndt_resolution_;
  float ndt_step_size_;
  float ndt_epsilon_;
  int ndt_max_iterations_;
  int ndt_search_method_;  // 0: KDTREE, 1: DIRECT7, 2: DIRECT1, 3: DIRECT26
  int registration_period_ms_;

  float global_leaf_size_;
  float registered_leaf_size_;
  int min_source_points_;
  int min_filtered_points_;
  double max_scan_age_sec_;
  std::vector<double> init_pose_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string prior_pcd_file_;
  std::string base_frame_;
  std::string robot_base_frame_;

  Eigen::Isometry3d result_t_;
  Eigen::Isometry3d previous_result_t_;

  // last trusted map->odom (freeze output when not trusted)
  Eigen::Isometry3d last_trusted_result_t_{Eigen::Isometry3d::Identity()};
  bool has_last_trusted_result_{false};

  bool converged_;
  bool trust_ndt_;
  int consecutive_converged_count_;
  int trust_ndt_threshold_;

  // publish policy
  bool publish_tf_only_when_trusted_{false};
  bool freeze_tf_when_not_trusted_{false};

  double jump_threshold_xy_;
  double jump_threshold_yaw_;
  double jump_threshold_rp_;

  bool enable_roll_pitch_fix_;

  // fitness score threshold for convergence quality check
  double fitness_score_threshold_;
  bool enable_quality_gate_;
  int quality_sample_points_;
  double quality_max_corr_dist_;
  int quality_min_valid_correspondences_;
  double quality_min_overlap_ratio_;
  double quality_max_median_residual_;
  double quality_max_p90_residual_;

  std::mutex cloud_mutex_;
  std::mutex pose_mutex_;
  bool map_ready_ = false;
  bool has_latest_cloud_stamp_{false};
  rclcpp::Time latest_cloud_stamp_{0, 0, RCL_ROS_TIME};

  /// Maximum number of accumulated points before oldest data is discarded.
  static constexpr size_t kMaxAccumulatedPoints = 500000;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;

  // NDT OMP registration object
  pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_;

  // Downsampled target cloud for NDT
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr target_kdtree_;

  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace ndt_omp_relocalization

#endif  // NDT_OMP_RELOCALIZATION__NDT_OMP_RELOCALIZATION_HPP_
