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

#ifndef SENSOR_SCAN_GENERATION__SENSOR_SCAN_GENERATION_HPP_
#define SENSOR_SCAN_GENERATION__SENSOR_SCAN_GENERATION_HPP_

#include <memory>
#include <string>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace sensor_scan_generation
{

class SensorScanGenerationNode : public rclcpp::Node
{
public:
  explicit SensorScanGenerationNode(const rclcpp::NodeOptions & options);

private:
  void laserCloudAndOdometryHandler(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odometry,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & laserCloud2);

  bool getTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
    tf2::Transform & transform);

  void publishTransform(
    const tf2::Transform & transform, const std::string & parent_frame,
    const std::string & child_frame, const rclcpp::Time & stamp);

  void publishOdometry(
    const tf2::Transform & transform, std::string parent_frame, const std::string & child_frame,
    const rclcpp::Time & stamp);

  tf2::Transform makePlanarBaseTransform(const tf2::Transform & transform) const;

  static double normalizeAngle(double angle);

  void diagnoseOdomJump(
    const tf2::Transform & tf_odom_to_lidar, const tf2::Transform & tf_lidar_to_chassis,
    const tf2::Transform & tf_odom_to_chassis, const rclcpp::Time & stamp);

  std::string lidar_frame_;
  std::string base_frame_;
  std::string robot_base_frame_;
  double max_sync_interval_sec_{0.03};
  bool use_initial_lidar_to_base_transform_{false};
  bool flatten_base_pose_{true};
  bool flatten_base_pose_z_{true};
  bool log_odom_jump_diagnostics_{true};
  double odom_jump_yaw_threshold_{0.35};
  double odom_jump_yaw_rate_threshold_{5.0};
  double odom_jump_distance_threshold_{0.20};
  double odom_jump_speed_threshold_{6.0};
  double odom_pose_cov_xy_{0.02};
  double odom_pose_cov_z_{0.05};
  double odom_pose_cov_rp_{0.05};
  double odom_pose_cov_yaw_{0.03};
  double odom_twist_cov_xy_{0.05};
  double odom_twist_cov_z_{0.10};
  double odom_twist_cov_rp_{0.10};
  double odom_twist_cov_yaw_{0.05};

  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_chassis_odometry_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> laser_cloud_sub_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  tf2::Transform tf_lidar_to_robot_base_;
  tf2::Transform tf_lidar_to_chassis_;
  bool has_lidar_to_chassis_{false};
  bool has_lidar_to_robot_base_{false};
  tf2::Transform previous_odometry_transform_;
  rclcpp::Time previous_odometry_stamp_;
  bool has_previous_odometry_{false};

  tf2::Transform previous_lidar_odometry_transform_;
  tf2::Transform previous_lidar_to_chassis_transform_;
  tf2::Transform previous_chassis_diagnostic_transform_;
  rclcpp::Time previous_chassis_diagnostic_stamp_;
  bool has_previous_jump_diagnostic_{false};
};

}  // namespace sensor_scan_generation

#endif  // SENSOR_SCAN_GENERATION__SENSOR_SCAN_GENERATION_HPP_
