#include "f_mpc_controller/mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace f_mpc_controller
{
bool MpcController::getOdomControlState(
  tf2::Transform & base_to_odom_tf, double & state_time_sec)
{
  // 优先使用新鲜里程计外推控制状态，失败时回退到 TF 位姿。
  if (!use_odometry_state_) {
    return false;
  }

  nav_msgs::msg::Odometry odom;
  rclcpp::Time odom_receive_time;
  rclcpp::Time odom_stamp;
  {
    std::lock_guard<std::mutex> lk(odom_mutex_);
    if (!has_latest_odom_) {
      auto node = node_.lock();
      if (node) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *clock_, 1000,
          "computeVelocityCommands: odometry state unavailable, no message on '%s'",
          odom_topic_.c_str());
      }
      return false;
    }
    odom = latest_odom_;
    odom_receive_time = latest_odom_receive_time_;
    odom_stamp = latest_odom_stamp_;
  }

  const std::string global_frame = costmap_ros_->getGlobalFrameID();
  const std::string control_base_frame = costmap_ros_->getBaseFrameID();

  const rclcpp::Time now = clock_->now();
  double age = 0.0;
  try {
    age = (now - odom_receive_time).seconds();
  } catch (const std::runtime_error &) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *clock_, 1000,
        "computeVelocityCommands: odometry state unavailable, incompatible time source");
    }
    return false;
  }

  if (age < -0.05 || age > max_odom_age_sec_) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *clock_, 1000,
        "computeVelocityCommands: odometry state stale, age=%.3fs limit=%.3fs frame='%s'",
        age, max_odom_age_sec_, odom.header.frame_id.c_str());
    }
    return false;
  }

  rclcpp::Time state_stamp = odom_stamp;
  if (state_stamp.nanoseconds() == 0) {
    state_stamp = odom_receive_time;
  }

  double predict_dt = 0.0;
  try {
    predict_dt = std::clamp((now - state_stamp).seconds(), 0.0, max_odom_predict_dt_);
  } catch (const std::runtime_error &) {
    predict_dt = std::clamp(age, 0.0, max_odom_predict_dt_);
    state_stamp = odom_receive_time;
  }
  state_time_sec = state_stamp.seconds() + predict_dt;
  tf2::Transform odom_tf;
  tf2::fromMsg(odom.pose.pose, odom_tf);
  if (!odom.header.frame_id.empty() && odom.header.frame_id != global_frame) {
    try {
      auto frame_tf_msg = tf_->lookupTransform(
        global_frame, odom.header.frame_id, tf2::TimePointZero);
      tf2::Transform frame_tf;
      tf2::fromMsg(frame_tf_msg.transform, frame_tf);
      odom_tf = frame_tf * odom_tf;
    } catch (tf2::TransformException &) {
      auto node = node_.lock();
      if (node) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *clock_, 1000,
          "computeVelocityCommands: odometry state frame transform failed '%s' -> '%s'",
          odom.header.frame_id.c_str(), global_frame.c_str());
      }
      return false;
    }
  }

  const double yaw = tf2::getYaw(odom_tf.getRotation());
  // nav_msgs/Odometry twist is expressed in child_frame_id by convention, and
  // sensor_scan_generation publishes it in the robot child frame. Convert it to
  // the global frame before predicting pose.
  const double vx_child = odom.twist.twist.linear.x;
  const double vy_child = odom.twist.twist.linear.y;
  const double wz = odom.twist.twist.angular.z;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double vx_global = c * vx_child - s * vy_child;
  const double vy_global = s * vx_child + c * vy_child;

  tf2::Vector3 origin = odom_tf.getOrigin();
  origin.setX(origin.x() + vx_global * predict_dt);
  origin.setY(origin.y() + vy_global * predict_dt);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw + wz * predict_dt);
  odom_tf.setOrigin(origin);
  odom_tf.setRotation(q);

  const std::string odom_child_frame = odom.child_frame_id;
  if (!odom_child_frame.empty() && odom_child_frame != control_base_frame) {
    try {
      auto child_to_base_msg = tf_->lookupTransform(
        odom_child_frame, control_base_frame, tf2::TimePointZero);
      tf2::Transform child_to_base_tf;
      tf2::fromMsg(child_to_base_msg.transform, child_to_base_tf);
      odom_tf = odom_tf * child_to_base_tf;
    } catch (tf2::TransformException &) {
      auto node = node_.lock();
      if (node) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *clock_, 1000,
          "computeVelocityCommands: odometry state base transform failed '%s' -> '%s'",
          odom_child_frame.c_str(), control_base_frame.c_str());
      }
      return false;
    }
  }

  base_to_odom_tf = odom_tf;
  return true;
}

}  // namespace f_mpc_controller
