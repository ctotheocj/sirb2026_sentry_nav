#ifndef SIRB_SMOOTHER__TRAJECTORY_MANAGER_NODE_HPP_
#define SIRB_SMOOTHER__TRAJECTORY_MANAGER_NODE_HPP_

#include <atomic>
#include <mutex>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sentry_nav_interfaces/action/commit_trajectory.hpp"
#include "sentry_nav_interfaces/msg/minco_trajectory.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "gcopter/trajectory.hpp"

namespace sirb_smoother
{

class TrajectoryManagerNode : public rclcpp::Node
{
public:
  explicit TrajectoryManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using CommitTrajectory = sentry_nav_interfaces::action::CommitTrajectory;
  using GoalHandleCommitTrajectory = rclcpp_action::ServerGoalHandle<CommitTrajectory>;

  enum class State
  {
    IDLE,
    TRACKING,
    REPLAN_PENDING,
    SWITCHING,
    FINISHING,
    EMERGENCY_STOP,
  };

  void trajectoryCallback(const sentry_nav_interfaces::msg::MincoTrajectory::SharedPtr msg);
  rclcpp_action::GoalResponse handleCommitGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CommitTrajectory::Goal> goal);
  rclcpp_action::CancelResponse handleCommitCancel(
    const std::shared_ptr<GoalHandleCommitTrajectory> goal_handle);
  void handleCommitAccepted(const std::shared_ptr<GoalHandleCommitTrajectory> goal_handle);
  void executeCommit(const std::shared_ptr<GoalHandleCommitTrajectory> goal_handle);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
  void publishTimerCallback();
  void publishStatus(const rclcpp::Time & stamp, const char * reason);
  void publishEmergencyStop(const rclcpp::Time & stamp, const char * reason);
  void publishLastEmergencyStop(const rclcpp::Time & stamp);
  void acceptTrajectory(
    const sentry_nav_interfaces::msg::MincoTrajectory & msg,
    double projected_time,
    State next_state,
    const char * reason,
    bool connect_from_active_override = true);
  void stampActiveTrajectoryTimeOrigin(const rclcpp::Time & now, double projected_time);
  bool shouldSwitchToCandidate(
    const sentry_nav_interfaces::msg::MincoTrajectory & candidate_msg,
    double & candidate_projected_time,
    bool & connect_from_active);
  void publishActiveTrajectory(const rclcpp::Time & stamp);
  void clearActiveTrajectory(const char * reason);
  void transitionTo(State next, const char * reason);
  const char * stateName(State state) const;
  bool validateTrajectory(const sentry_nav_interfaces::msg::MincoTrajectory & msg) const;
  double trajectoryDuration(const sentry_nav_interfaces::msg::MincoTrajectory & msg) const;
  double endpointDistance(
    const sentry_nav_interfaces::msg::MincoTrajectory & a,
    const sentry_nav_interfaces::msg::MincoTrajectory & b) const;
  bool cropTrajectory(
    const sentry_nav_interfaces::msg::MincoTrajectory & src,
    double crop_time,
    sentry_nav_interfaces::msg::MincoTrajectory & dst) const;
  bool projectOdomToTrajectory(
    const sentry_nav_interfaces::msg::MincoTrajectory & msg,
    const Trajectory<5> & traj,
    double seed_time,
    double & projected_time);
  bool getOdomPositionInTrajectoryFrame(
    const sentry_nav_interfaces::msg::MincoTrajectory & msg,
    double & x,
    double & y);
  bool getOdomStateInTrajectoryFrame(
    const sentry_nav_interfaces::msg::MincoTrajectory & msg,
    Eigen::Vector3d & position,
    Eigen::Vector3d & velocity) const;
  bool trimConnectedTrajectoryStart(
    sentry_nav_interfaces::msg::MincoTrajectory & msg,
    const Eigen::Vector3d & start,
    const Eigen::Vector3d & velocity) const;
  bool buildTrajectory(
    const sentry_nav_interfaces::msg::MincoTrajectory & msg,
    Trajectory<5> & traj) const;
  bool trajectoryIsCollisionFree(
    const sentry_nav_interfaces::msg::MincoTrajectory & msg,
    const Trajectory<5> & traj,
    double start_time,
    double horizon_sec,
    double & collision_time,
    int & collision_cost);
  bool holePassModeActive() const;
  bool costmapPointIsCollisionFree(
    double wx,
    double wy,
    double & collision_x,
    double & collision_y,
    int & collision_cost) const;
  bool worldToCostmap(double wx, double wy, unsigned int & mx, unsigned int & my) const;
  bool isNewGoal(const sentry_nav_interfaces::msg::MincoTrajectory & msg) const;

  rclcpp::Subscription<sentry_nav_interfaces::msg::MincoTrajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_mode_sub_;
  rclcpp::Publisher<sentry_nav_interfaces::msg::MincoTrajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp_action::Server<CommitTrajectory>::SharedPtr commit_action_server_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::atomic_bool commit_in_flight_{false};
  mutable std::mutex mutex_;
  State state_{State::IDLE};
  sentry_nav_interfaces::msg::MincoTrajectory active_traj_;
  sentry_nav_interfaces::msg::MincoTrajectory emergency_stop_traj_;
  rclcpp::Time active_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Odometry latest_odom_;
  nav2_msgs::msg::Costmap latest_costmap_;
  bool has_active_traj_{false};
  bool has_emergency_stop_{false};
  bool has_odom_{false};
  bool has_costmap_{false};
  int64_t active_seq_{0};
  double last_projected_time_{0.0};
  int tracking_error_replan_count_{0};
  uint64_t active_goal_id_{0};
  bool publish_full_active_trajectory_{true};
  std::string last_transition_reason_{"init"};

  std::string input_topic_{"safe_geometric_smoother/trajectory_for_mpc"};
  std::string output_topic_{"trajectory_manager/trajectory_for_mpc"};
  std::string odom_topic_{"odometry"};
  std::string costmap_topic_{"local_costmap/costmap_raw"};
  std::string navigation_mode_topic_{"navigation_mode_manager/mode"};
  double publish_rate_hz_{20.0};
  double active_timeout_sec_{6.0};
  double min_remaining_time_sec_{0.15};
  double crop_epsilon_sec_{0.02};
  double projection_search_window_sec_{0.8};
  double projection_max_tracking_error_{0.80};
  double candidate_projection_window_sec_{0.35};
  double switch_goal_change_distance_{0.5};
  double switch_max_position_error_{0.45};
  double switch_max_velocity_error_{1.8};
  double switch_max_acceleration_error_{4.0};
  double switch_min_direction_dot_{-0.2};
  double switch_min_candidate_duration_sec_{0.2};
  double switch_min_interval_sec_{3.0};
  double switch_immediate_deviation_{1.2};
  double switch_immediate_length_ratio_{0.25};
  double switch_allow_when_remaining_sec_{1.0};
  double emergency_stop_remaining_time_sec_{0.10};
  double emergency_stop_duration_sec_{0.20};
  bool enable_forward_collision_check_{true};
  bool allow_unknown_costmap_{true};
  int collision_cost_threshold_{253};
  double collision_check_horizon_sec_{1.2};
  double collision_check_step_sec_{0.10};
  double collision_check_radius_{0.20};
  bool collision_tf_failure_is_safe_{false};
  bool require_odom_{true};
  int tracking_error_replan_frames_{3};
  std::atomic_bool hole_pass_mode_active_{false};
};

}  // namespace sirb_smoother

#endif  // SIRB_SMOOTHER__TRAJECTORY_MANAGER_NODE_HPP_
