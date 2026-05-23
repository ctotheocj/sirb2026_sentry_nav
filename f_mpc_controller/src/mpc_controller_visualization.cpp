#include "f_mpc_controller/mpc_controller.hpp"

#include <vector>

namespace f_mpc_controller
{
void MpcController::publishLocalPath(const tf2::Transform & base_to_odom_tf)
{
  // 按限频发布当前 MPC 参考路径，供可视化和下游速度同步模块使用。
  if (!local_path_pub_) return;
  const rclcpp::Time now = clock_->now();
  if (last_local_plan_publish_time_.nanoseconds() != 0 &&
    (now - last_local_plan_publish_time_).seconds() < local_plan_publish_period_sec_)
  {
    return;
  }
  last_local_plan_publish_time_ = now;
  nav_msgs::msg::Path local_path;
  local_path.header.frame_id = costmap_ros_->getGlobalFrameID();
  local_path.header.stamp = now;
  local_path.poses.reserve(ref.size() + 1);

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header = local_path.header;
  robot_pose.pose.position.x = base_to_odom_tf.getOrigin().x();
  robot_pose.pose.position.y = base_to_odom_tf.getOrigin().y();
  robot_pose.pose.position.z = 0.0;
  robot_pose.pose.orientation.w = 1.0;
  local_path.poses.push_back(robot_pose);

  for (const auto & p : ref) {
    geometry_msgs::msg::PoseStamped lp;
    lp.header = local_path.header;
    lp.pose.position.x = p.x;
    lp.pose.position.y = p.y;
    lp.pose.position.z = 0.0;
    lp.pose.orientation.w = 1.0;
    local_path.poses.push_back(lp);
  }
  local_path_pub_->publish(local_path);
  if (auto node = node_.lock()) {
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *clock_, 1000,
      "publishLocalPath: published local_plan poses=%zu frame=%s",
      local_path.poses.size(), local_path.header.frame_id.c_str());
  }
}

}  // namespace f_mpc_controller
