#include "f_mpc_controller/mpc_controller.hpp"

#include <algorithm>
#include <utility>
#include <vector>

#include <Eigen/Dense>

namespace f_mpc_controller
{
bool MpcController::buildTrajFromMsg(
  const sentry_nav_interfaces::msg::MincoTrajectory & msg, Trajectory<5> & traj)
{
  // 将轨迹管理器消息重建为 GCOPTER MINCO 轨迹，供控制周期按时间连续采样。
  const size_t n = msg.waypoints.size();
  if (n < 2 || msg.segment_times.size() != n - 1) return false;

  std::vector<Eigen::Vector3d> pts;
  pts.reserve(n);
  for (const auto & p : msg.waypoints) pts.emplace_back(p.x, p.y, p.z);

  Eigen::VectorXd times(static_cast<int>(n - 1));
  for (size_t i = 0; i < n - 1; ++i) times(i) = msg.segment_times[i];

  const int M = static_cast<int>(n) - 1;
  Eigen::Matrix3d head = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d tail = Eigen::Matrix3d::Zero();
  head.col(0) = pts.front();
  tail.col(0) = pts.back();
  head.col(1) = Eigen::Vector3d(
    msg.initial_velocity.x, msg.initial_velocity.y, msg.initial_velocity.z);
  head.col(2) = Eigen::Vector3d(
    msg.initial_acceleration.x, msg.initial_acceleration.y, msg.initial_acceleration.z);
  tail.col(1) = Eigen::Vector3d(
    msg.terminal_velocity.x, msg.terminal_velocity.y, msg.terminal_velocity.z);
  tail.col(2) = Eigen::Vector3d(
    msg.terminal_acceleration.x, msg.terminal_acceleration.y, msg.terminal_acceleration.z);

  Eigen::Matrix3Xd inner(3, std::max(0, M - 1));
  for (int i = 1; i < M; ++i) inner.col(i - 1) = pts[i];

  minco::MINCO_S3NU solver;
  solver.setConditions(head, tail, M);
  solver.setParameters(inner, times);
  solver.getTrajectory(traj);
  return true;
}
void MpcController::mincoTrajCallback(
  const sentry_nav_interfaces::msg::MincoTrajectory::SharedPtr msg)
{
  // 接收并缓存最新 MINCO 轨迹，同时维护轨迹 id、时间基准和投影重置状态。
  auto node = node_.lock();
  if (node) {
    RCLCPP_INFO_THROTTLE(node->get_logger(), *clock_, 1000,
      "mincoTrajCallback: received, id=%lu goal=%lu waypoints=%zu, freq_count=%d",
      static_cast<unsigned long>(msg->trajectory_id),
      static_cast<unsigned long>(msg->goal_id),
      msg->waypoints.size(), ++minco_cb_count_);
  }

  Trajectory<5> traj;
  if (!buildTrajFromMsg(*msg, traj)) {
    if (node) RCLCPP_INFO_THROTTLE(node->get_logger(), *clock_, 1000,
      "mincoTrajCallback: rejected — buildTrajFromMsg failed");
    return;
  }

  std::lock_guard<std::mutex> lk(traj_mutex_);
  const bool new_trajectory = msg->trajectory_id != 0 && msg->trajectory_id != minco_trajectory_id_;
  minco_traj_ = std::move(traj);
  const rclcpp::Time receive_stamp = clock_->now();
  if (new_trajectory || minco_first_receive_stamp_.nanoseconds() == 0) {
    minco_first_receive_stamp_ = receive_stamp;
  }
  traj_stamp_ = receive_stamp;
  minco_start_time_ = msg->start_time.sec == 0 && msg->start_time.nanosec == 0 ?
    traj_stamp_ : rclcpp::Time(msg->start_time);
  if (new_trajectory || minco_trajectory_id_ == 0) {
    try {
      const rclcpp::Time msg_stamp(msg->header.stamp);
      minco_start_offset_sec_ = (msg_stamp - minco_start_time_).seconds();
    } catch (const std::runtime_error &) {
      minco_start_offset_sec_ = 0.0;
      minco_start_time_ = traj_stamp_;
    }
    minco_time_base_offset_sec_ = minco_start_offset_sec_;
  }
  traj_duration_ = minco_traj_.getTotalDuration();
  minco_traj_frame_ = msg->header.frame_id;
  minco_trajectory_id_ = msg->trajectory_id;
  minco_goal_id_ = msg->goal_id;
  has_minco_traj_ = true;
  if (new_trajectory) {
    last_t_proj_ = 0.0;
    last_sampled_trajectory_id_ = 0;
    last_projection_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
}

}  // namespace f_mpc_controller
