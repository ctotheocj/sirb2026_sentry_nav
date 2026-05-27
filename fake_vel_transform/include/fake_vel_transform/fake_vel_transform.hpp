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

#ifndef FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
#define FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "example_interfaces/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentry_nav_interfaces/msg/hole_pass_cmd.hpp"
#include "sentry_nav_interfaces/msg/hole_pass_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace fake_vel_transform
{
class FakeVelTransform : public rclcpp::Node
{
public:
  explicit FakeVelTransform(const rclcpp::NodeOptions & options);

private:
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void cmdVelStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void cmdSpinCallback(example_interfaces::msg::Float32::SharedPtr msg);
  void holePassCommandCallback(const sentry_nav_interfaces::msg::HolePassCmd::SharedPtr msg);
  void holePassStateCallback(const sentry_nav_interfaces::msg::HolePassState::SharedPtr msg);
  void publishTransform();
  void navYawCallback(const std_msgs::msg::Float64::SharedPtr msg);
  bool shouldBlockForHolePassLowering(const rclcpp::Time & now) const;
  geometry_msgs::msg::Twist transformVelocity(
    const geometry_msgs::msg::Twist & twist, double yaw_diff) const;
  bool lookupRobotBaseYaw(const rclcpp::Time & stamp, double * yaw);
  rclcpp::Time normalizedStamp(const builtin_interfaces::msg::Time & stamp);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_sub_;
  rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr cmd_spin_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr nav_yaw_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sentry_nav_interfaces::msg::HolePassCmd>::SharedPtr hole_pass_cmd_sub_;
  rclcpp::Subscription<sentry_nav_interfaces::msg::HolePassState>::SharedPtr hole_pass_state_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_chassis_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string robot_base_frame_;
  std::string fake_robot_base_frame_;
  std::string odom_topic_;
  std::string cmd_spin_topic_;
  std::string input_cmd_vel_topic_;
  std::string input_cmd_vel_stamped_topic_;
  std::string output_cmd_vel_topic_;
  std::string odom_frame_;
  bool prefer_stamped_cmd_vel_;
  bool debug_logging_{false};
  bool allow_latest_tf_fallback_;
  double tf_lookup_timeout_sec_;
  double stamped_cmd_timeout_sec_;
  std::atomic<float> spin_speed_{0.0F};

  bool hole_pass_lower_protection_enabled_{false};
  std::string hole_pass_cmd_topic_;
  std::string hole_pass_state_topic_;
  double hole_pass_state_timeout_sec_{0.25};

  std::mutex cmd_vel_mutex_;
  double current_robot_base_angle_;
  bool has_robot_base_angle_{false};
  rclcpp::Time last_stamped_cmd_time_;

  // nav_yaw 融合角度
  bool use_nav_yaw_;
  std::string nav_yaw_topic_;
  double nav_yaw_;
  bool nav_yaw_received_;

  bool hole_pass_command_is_lower_{false};
  uint8_t hole_pass_state_{sentry_nav_interfaces::msg::HolePassState::STATE_STAND};
  rclcpp::Time last_hole_pass_state_time_;
};

}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
