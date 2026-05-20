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

#include "sensor_scan_generation/sensor_scan_generation.hpp"

#include <cmath>

#include "pcl_ros/transforms.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sensor_scan_generation
{

SensorScanGenerationNode::SensorScanGenerationNode(const rclcpp::NodeOptions & options)
: Node("sensor_scan_generation", options)
{
  this->declare_parameter<std::string>("lidar_frame", "");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<std::string>("robot_base_frame", "");
  this->declare_parameter<double>("max_sync_interval_sec", 0.03);
  this->declare_parameter<bool>("use_initial_lidar_to_base_transform", false);
  this->declare_parameter<bool>("flatten_base_pose", true);
  this->declare_parameter<bool>("flatten_base_pose_z", true);
  this->declare_parameter<bool>("log_odom_jump_diagnostics", true);
  this->declare_parameter<double>("odom_jump_yaw_threshold", 0.35);
  this->declare_parameter<double>("odom_jump_yaw_rate_threshold", 5.0);
  this->declare_parameter<double>("odom_jump_distance_threshold", 0.20);
  this->declare_parameter<double>("odom_jump_speed_threshold", 6.0);
  this->declare_parameter<double>("odom_pose_cov_xy", 0.02);
  this->declare_parameter<double>("odom_pose_cov_z", 0.05);
  this->declare_parameter<double>("odom_pose_cov_rp", 0.05);
  this->declare_parameter<double>("odom_pose_cov_yaw", 0.03);
  this->declare_parameter<double>("odom_twist_cov_xy", 0.05);
  this->declare_parameter<double>("odom_twist_cov_z", 0.10);
  this->declare_parameter<double>("odom_twist_cov_rp", 0.10);
  this->declare_parameter<double>("odom_twist_cov_yaw", 0.05);

  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("max_sync_interval_sec", max_sync_interval_sec_);
  this->get_parameter("use_initial_lidar_to_base_transform", use_initial_lidar_to_base_transform_);
  this->get_parameter("flatten_base_pose", flatten_base_pose_);
  this->get_parameter("flatten_base_pose_z", flatten_base_pose_z_);
  this->get_parameter("log_odom_jump_diagnostics", log_odom_jump_diagnostics_);
  this->get_parameter("odom_jump_yaw_threshold", odom_jump_yaw_threshold_);
  this->get_parameter("odom_jump_yaw_rate_threshold", odom_jump_yaw_rate_threshold_);
  this->get_parameter("odom_jump_distance_threshold", odom_jump_distance_threshold_);
  this->get_parameter("odom_jump_speed_threshold", odom_jump_speed_threshold_);
  this->get_parameter("odom_pose_cov_xy", odom_pose_cov_xy_);
  this->get_parameter("odom_pose_cov_z", odom_pose_cov_z_);
  this->get_parameter("odom_pose_cov_rp", odom_pose_cov_rp_);
  this->get_parameter("odom_pose_cov_yaw", odom_pose_cov_yaw_);
  this->get_parameter("odom_twist_cov_xy", odom_twist_cov_xy_);
  this->get_parameter("odom_twist_cov_z", odom_twist_cov_z_);
  this->get_parameter("odom_twist_cov_rp", odom_twist_cov_rp_);
  this->get_parameter("odom_twist_cov_yaw", odom_twist_cov_yaw_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pub_laser_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_scan", 2);
  pub_chassis_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 2);

  rmw_qos_profile_t qos_profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

  odometry_sub_.subscribe(this, "lidar_odometry", qos_profile);
  laser_cloud_sub_.subscribe(this, "registered_scan", qos_profile);

  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odometry_sub_, laser_cloud_sub_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_sync_interval_sec_));
  sync_->registerCallback(std::bind(
    &SensorScanGenerationNode::laserCloudAndOdometryHandler, this, std::placeholders::_1,
    std::placeholders::_2));
}

void SensorScanGenerationNode::laserCloudAndOdometryHandler(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcd_msg)
{
  tf2::Transform tf_lidar_to_chassis;
  tf2::Transform tf_odom_to_chassis;
  tf2::Transform tf_odom_to_robot_base;
  tf2::Transform tf_odom_to_lidar;

  tf2::fromMsg(odometry_msg->pose.pose, tf_odom_to_lidar);

  if (!has_lidar_to_robot_base_ || !use_initial_lidar_to_base_transform_) {
    if (getTransform(lidar_frame_, robot_base_frame_, pcd_msg->header.stamp, tf_lidar_to_robot_base_)) {
      has_lidar_to_robot_base_ = true;
    } else if (!has_lidar_to_robot_base_) {
      return;
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Using cached lidar-to-robot-base transform after TF lookup failure.");
    }
  }

  if (!has_lidar_to_chassis_ || !use_initial_lidar_to_base_transform_) {
    if (getTransform(lidar_frame_, base_frame_, pcd_msg->header.stamp, tf_lidar_to_chassis_)) {
      has_lidar_to_chassis_ = true;
    } else if (!has_lidar_to_chassis_) {
      return;
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Using cached lidar-to-chassis transform after TF lookup failure.");
    }
  }

  tf_lidar_to_chassis = tf_lidar_to_chassis_;
  tf_odom_to_chassis = tf_odom_to_lidar * tf_lidar_to_chassis;
  tf_odom_to_robot_base = tf_odom_to_lidar * tf_lidar_to_robot_base_;

  const tf2::Transform tf_odom_to_chassis_out = makePlanarBaseTransform(tf_odom_to_chassis);
  const tf2::Transform tf_odom_to_robot_base_out = makePlanarBaseTransform(tf_odom_to_robot_base);

  diagnoseOdomJump(
    tf_odom_to_lidar, tf_lidar_to_chassis, tf_odom_to_chassis_out, pcd_msg->header.stamp);

  publishTransform(
    tf_odom_to_chassis_out, odometry_msg->header.frame_id, base_frame_, pcd_msg->header.stamp);
  publishOdometry(
    tf_odom_to_robot_base_out, odometry_msg->header.frame_id, robot_base_frame_, pcd_msg->header.stamp);

  sensor_msgs::msg::PointCloud2 out;
  pcl_ros::transformPointCloud(lidar_frame_, tf_odom_to_lidar.inverse(), *pcd_msg, out);
  pub_laser_cloud_->publish(out);
}

bool SensorScanGenerationNode::getTransform(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
  tf2::Transform & transform)
{
  try {
    auto transform_stamped = tf_buffer_->lookupTransform(
      target_frame, source_frame, time, rclcpp::Duration::from_seconds(0.5));
    tf2::fromMsg(transform_stamped.transform, transform);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "TF lookup failed: %s.", ex.what());
    return false;
  }
}

tf2::Transform SensorScanGenerationNode::makePlanarBaseTransform(
  const tf2::Transform & transform) const
{
  if (!flatten_base_pose_) {
    return transform;
  }

  tf2::Transform out = transform;
  tf2::Vector3 origin = out.getOrigin();
  if (flatten_base_pose_z_) {
    origin.setZ(0.0);
  }
  out.setOrigin(origin);

  tf2::Quaternion yaw_only;
  yaw_only.setRPY(0.0, 0.0, tf2::getYaw(transform.getRotation()));
  yaw_only.normalize();
  out.setRotation(yaw_only);
  return out;
}

void SensorScanGenerationNode::publishTransform(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;
  transform_msg.transform = tf2::toMsg(transform);
  br_->sendTransform(transform_msg);
}

void SensorScanGenerationNode::publishOdometry(
  const tf2::Transform & transform, std::string parent_frame, const std::string & child_frame,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry out;
  out.header.stamp = stamp;
  out.header.frame_id = parent_frame;
  out.child_frame_id = child_frame;

  const auto & origin = transform.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(transform.getRotation());

  out.pose.covariance[0] = odom_pose_cov_xy_;
  out.pose.covariance[7] = odom_pose_cov_xy_;
  out.pose.covariance[14] = odom_pose_cov_z_;
  out.pose.covariance[21] = odom_pose_cov_rp_;
  out.pose.covariance[28] = odom_pose_cov_rp_;
  out.pose.covariance[35] = odom_pose_cov_yaw_;

  out.twist.covariance[0] = odom_twist_cov_xy_;
  out.twist.covariance[7] = odom_twist_cov_xy_;
  out.twist.covariance[14] = odom_twist_cov_z_;
  out.twist.covariance[21] = odom_twist_cov_rp_;
  out.twist.covariance[28] = odom_twist_cov_rp_;
  out.twist.covariance[35] = odom_twist_cov_yaw_;

  if (has_previous_odometry_) {
    const double dt = (stamp - previous_odometry_stamp_).seconds();
    if (dt > 1.0e-4 && dt < 0.5) {
      const auto linear_velocity_parent =
        (transform.getOrigin() - previous_odometry_transform_.getOrigin()) / dt;
      const tf2::Vector3 linear_velocity_child =
        tf2::quatRotate(transform.getRotation().inverse(), linear_velocity_parent);

      // The downstream controller uses a planar ground-robot model: publish twist in child frame.
      const double yaw = tf2::getYaw(transform.getRotation());
      const double previous_yaw = tf2::getYaw(previous_odometry_transform_.getRotation());
      const double yaw_rate = normalizeAngle(yaw - previous_yaw) / dt;

      out.twist.twist.linear.x = linear_velocity_child.x();
      out.twist.twist.linear.y = linear_velocity_child.y();
      out.twist.twist.linear.z = linear_velocity_child.z();
      out.twist.twist.angular.x = 0.0;
      out.twist.twist.angular.y = 0.0;
      out.twist.twist.angular.z = yaw_rate;
    }
  }

  previous_odometry_transform_ = transform;
  previous_odometry_stamp_ = stamp;
  has_previous_odometry_ = true;

  pub_chassis_odometry_->publish(out);
}

double SensorScanGenerationNode::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

void SensorScanGenerationNode::diagnoseOdomJump(
  const tf2::Transform & tf_odom_to_lidar, const tf2::Transform & tf_lidar_to_chassis,
  const tf2::Transform & tf_odom_to_chassis, const rclcpp::Time & stamp)
{
  if (!log_odom_jump_diagnostics_) {
    return;
  }

  if (has_previous_jump_diagnostic_) {
    const double dt = (stamp - previous_chassis_diagnostic_stamp_).seconds();
    if (dt > 1.0e-4 && dt < 1.0) {
      const double base_dxy =
        std::hypot(
          tf_odom_to_chassis.getOrigin().x() - previous_chassis_diagnostic_transform_.getOrigin().x(),
          tf_odom_to_chassis.getOrigin().y() - previous_chassis_diagnostic_transform_.getOrigin().y());
      const double lidar_dyaw = normalizeAngle(
        tf2::getYaw(tf_odom_to_lidar.getRotation()) -
        tf2::getYaw(previous_lidar_odometry_transform_.getRotation()));
      const double extrinsic_dyaw = normalizeAngle(
        tf2::getYaw(tf_lidar_to_chassis.getRotation()) -
        tf2::getYaw(previous_lidar_to_chassis_transform_.getRotation()));
      const double base_dyaw = normalizeAngle(
        tf2::getYaw(tf_odom_to_chassis.getRotation()) -
        tf2::getYaw(previous_chassis_diagnostic_transform_.getRotation()));
      const double base_speed = base_dxy / dt;
      const double base_yaw_rate = std::abs(base_dyaw) / dt;

      if (
        (std::abs(base_dyaw) > odom_jump_yaw_threshold_ &&
        base_yaw_rate > odom_jump_yaw_rate_threshold_) ||
        (base_dxy > odom_jump_distance_threshold_ &&
        base_speed > odom_jump_speed_threshold_)) {
        RCLCPP_WARN(
          this->get_logger(),
          "sensor_scan_generation odom jump diagnostic: base_dxy=%.3fm base_dyaw=%.3frad "
          "dt=%.3fs base_speed=%.2fm/s base_yaw_rate=%.2frad/s lidar_dyaw=%.3frad "
          "lidar_to_base_dyaw=%.3frad cached_extrinsic=%d",
          base_dxy, base_dyaw, dt, base_speed, base_yaw_rate, lidar_dyaw, extrinsic_dyaw,
          use_initial_lidar_to_base_transform_ ? 1 : 0);
      }
    }
  }

  previous_lidar_odometry_transform_ = tf_odom_to_lidar;
  previous_lidar_to_chassis_transform_ = tf_lidar_to_chassis;
  previous_chassis_diagnostic_transform_ = tf_odom_to_chassis;
  previous_chassis_diagnostic_stamp_ = stamp;
  has_previous_jump_diagnostic_ = true;
}

}  // namespace sensor_scan_generation

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor_scan_generation::SensorScanGenerationNode)
