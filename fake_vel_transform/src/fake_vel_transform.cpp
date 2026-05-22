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

#include "fake_vel_transform/fake_vel_transform.hpp"

#include <algorithm>
#include <cmath>

#include "std_msgs/msg/float64.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{

constexpr double CONTROLLER_TIMEOUT = 0.5;

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  this->declare_parameter<std::string>("robot_base_frame", "gimbal_link");
  this->declare_parameter<std::string>("fake_robot_base_frame", "gimbal_link_fake");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  this->declare_parameter<std::string>("input_cmd_vel_stamped_topic", "");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<bool>("prefer_stamped_cmd_vel", true);
  this->declare_parameter<bool>("debug_logging", false);
  this->declare_parameter<bool>("allow_latest_tf_fallback", true);
  this->declare_parameter<double>("tf_lookup_timeout_sec", 0.01);
  this->declare_parameter<double>("stamped_cmd_timeout_sec", 0.12);
  this->declare_parameter<double>("max_latest_cmd_age_sec", 0.12);
  this->declare_parameter<float>("init_spin_speed", 0.0);
  this->declare_parameter<bool>("use_nav_yaw", false);
  this->declare_parameter<std::string>("nav_yaw_topic", "/Nav_yaw");

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("cmd_spin_topic", cmd_spin_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("input_cmd_vel_stamped_topic", input_cmd_vel_stamped_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("prefer_stamped_cmd_vel", prefer_stamped_cmd_vel_);
  this->get_parameter("debug_logging", debug_logging_);
  this->get_parameter("allow_latest_tf_fallback", allow_latest_tf_fallback_);
  this->get_parameter("tf_lookup_timeout_sec", tf_lookup_timeout_sec_);
  this->get_parameter("stamped_cmd_timeout_sec", stamped_cmd_timeout_sec_);
  double max_latest_cmd_age_sec = stamped_cmd_timeout_sec_;
  this->get_parameter("max_latest_cmd_age_sec", max_latest_cmd_age_sec);
  float initial_spin_speed = 0.0F;
  this->get_parameter("init_spin_speed", initial_spin_speed);
  spin_speed_.store(initial_spin_speed);
  this->get_parameter("use_nav_yaw", use_nav_yaw_);
  this->get_parameter("nav_yaw_topic", nav_yaw_topic_);

  stamped_cmd_timeout_sec_ = std::min(stamped_cmd_timeout_sec_, max_latest_cmd_age_sec);
  tf_lookup_timeout_sec_ = std::max(0.0, tf_lookup_timeout_sec_);
  stamped_cmd_timeout_sec_ = std::max(0.0, stamped_cmd_timeout_sec_);

  nav_yaw_ = 0.0;
  nav_yaw_received_ = false;
  current_robot_base_angle_ = 0.0;
  last_controller_activate_time_ = this->get_clock()->now();
  last_stamped_cmd_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);

  cmd_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
    cmd_spin_topic_, 1, std::bind(&FakeVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));
  if (!input_cmd_vel_stamped_topic_.empty()) {
    cmd_vel_stamped_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      input_cmd_vel_stamped_topic_, 10,
      std::bind(&FakeVelTransform::cmdVelStampedCallback, this, std::placeholders::_1));
  }

  // 根据参数决定是否订阅融合 yaw（替代 odom yaw）
  if (use_nav_yaw_) {
    // 队列深度=1 + BestEffort：永远只处理最新的 yaw，避免队列积压导致过时数据
    auto qos = rclcpp::QoS(1).best_effort();
    nav_yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      nav_yaw_topic_, qos,
      std::bind(&FakeVelTransform::navYawCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "use_nav_yaw=true, subscribing to: %s", nav_yaw_topic_.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "use_nav_yaw=false, using raw odom yaw");
  }

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "FakeVelTransform cmd chain: input='%s' stamped_input='%s' output='%s' odom_frame='%s' "
    "robot_frame='%s' fake_frame='%s' prefer_stamped=%d linear transform=fake_to_real",
    input_cmd_vel_topic_.c_str(), input_cmd_vel_stamped_topic_.c_str(),
    output_cmd_vel_topic_.c_str(), odom_frame_.c_str(), robot_base_frame_.c_str(),
    fake_robot_base_frame_.c_str(), prefer_stamped_cmd_vel_ ? 1 : 0);

  // 50Hz Timer to send transform from `robot_base_frame` to `fake_robot_base_frame`
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&FakeVelTransform::publishTransform, this));
}

void FakeVelTransform::cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  spin_speed_.store(msg->data);
}

void FakeVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  if ((this->get_clock()->now() - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT) {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    if (use_nav_yaw_ && nav_yaw_received_) {
      current_robot_base_angle_ = nav_yaw_;
    } else {
      current_robot_base_angle_ = tf2::getYaw(msg->pose.pose.orientation);
    }
  }
}

void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  const auto now = this->get_clock()->now();
  const bool stamped_cmd_recent = prefer_stamped_cmd_vel_ &&
    last_stamped_cmd_time_.nanoseconds() != 0 &&
    (now - last_stamped_cmd_time_).seconds() <= stamped_cmd_timeout_sec_;
  if (stamped_cmd_recent) {
    return;
  }

  auto aft_tf_vel = transformVelocity(*msg, current_robot_base_angle_);
  cmd_vel_chassis_pub_->publish(aft_tf_vel);
}

void FakeVelTransform::cmdVelStampedCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (!msg->header.frame_id.empty() && msg->header.frame_id != fake_robot_base_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Stamped cmd_vel frame_id '%s' does not match fake_robot_base_frame '%s'",
      msg->header.frame_id.c_str(), fake_robot_base_frame_.c_str());
  }

  const auto cmd_stamp = normalizedStamp(msg->header.stamp);
  double yaw = 0.0;
  bool have_yaw = false;

  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    if (use_nav_yaw_ && nav_yaw_received_) {
      yaw = nav_yaw_;
      have_yaw = true;
    }
  }

  if (!have_yaw && !lookupRobotBaseYaw(cmd_stamp, &yaw)) {
    return;
  }

  auto aft_tf_vel = transformVelocity(msg->twist, yaw);
  if (debug_logging_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "FakeVelTransform stamped: in_frame='%s' yaw=%.3f in=(%.3f, %.3f, %.3f) "
      "out=(%.3f, %.3f, %.3f)",
      msg->header.frame_id.c_str(), yaw, msg->twist.linear.x, msg->twist.linear.y,
      msg->twist.angular.z, aft_tf_vel.linear.x, aft_tf_vel.linear.y, aft_tf_vel.angular.z);
  }
  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    current_robot_base_angle_ = yaw;
    last_controller_activate_time_ = this->get_clock()->now();
    last_stamped_cmd_time_ = last_controller_activate_time_;
  }

  cmd_vel_chassis_pub_->publish(aft_tf_vel);
}

void FakeVelTransform::navYawCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  nav_yaw_ = msg->data;
  nav_yaw_received_ = true;
}

void FakeVelTransform::publishTransform()
{
  double robot_base_angle = 0.0;
  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    robot_base_angle = current_robot_base_angle_;
  }

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion q;
  q.setRPY(0, 0, -robot_base_angle);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

geometry_msgs::msg::Twist FakeVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist & twist, double yaw_diff) const
{
  geometry_msgs::msg::Twist aft_tf_vel;
  aft_tf_vel.angular.z = twist.angular.z + spin_speed_.load();
  aft_tf_vel.linear.x = twist.linear.x * std::cos(yaw_diff) + twist.linear.y * std::sin(yaw_diff);
  aft_tf_vel.linear.y = -twist.linear.x * std::sin(yaw_diff) + twist.linear.y * std::cos(yaw_diff);
  return aft_tf_vel;
}

bool FakeVelTransform::lookupRobotBaseYaw(const rclcpp::Time & stamp, double * yaw)
{
  const auto timeout = rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_);
  try {
    const auto tf = tf_buffer_->lookupTransform(odom_frame_, robot_base_frame_, stamp, timeout);
    *yaw = tf2::getYaw(tf.transform.rotation);
    return true;
  } catch (const tf2::ExtrapolationException & ex) {
    if (!allow_latest_tf_fallback_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "TF extrapolation for %s -> %s at %.9f: %s",
        odom_frame_.c_str(), robot_base_frame_.c_str(), stamp.seconds(), ex.what());
      return false;
    }

    try {
      const rclcpp::Time latest(0, 0, stamp.get_clock_type());
      const auto tf = tf_buffer_->lookupTransform(odom_frame_, robot_base_frame_, latest, timeout);
      *yaw = tf2::getYaw(tf.transform.rotation);
      RCLCPP_DEBUG(
        get_logger(), "TF extrapolation for stamped cmd, using latest %s -> %s yaw",
        odom_frame_.c_str(), robot_base_frame_.c_str());
      return true;
    } catch (const tf2::TransformException & latest_ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "TF latest fallback failed for %s -> %s: %s",
        odom_frame_.c_str(), robot_base_frame_.c_str(), latest_ex.what());
      return false;
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "TF lookup failed for %s -> %s at %.9f: %s",
      odom_frame_.c_str(), robot_base_frame_.c_str(), stamp.seconds(), ex.what());
    return false;
  }
}

rclcpp::Time FakeVelTransform::normalizedStamp(
  const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec == 0 && stamp.nanosec == 0U) {
    return this->get_clock()->now();
  }
  return rclcpp::Time(stamp, this->get_clock()->get_clock_type());
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
