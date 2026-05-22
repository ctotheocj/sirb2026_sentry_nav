#pragma once

#ifndef F_MPC_CONTROLLER__MPC_CONTROLLER_HPP_
#define F_MPC_CONTROLLER__MPC_CONTROLLER_HPP_

#include <mutex>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "sentry_nav_interfaces/msg/minco_trajectory.hpp"
#include "sentry_nav_interfaces/msg/tracked_obstacle_array.hpp"

#include "gcopter/trajectory.hpp"
#include "gcopter/minco.hpp"

#include "f_mpc_controller/mpc.hpp"

namespace f_mpc_controller
{

namespace constants {
  constexpr double kMinStepSize = 0.01;
  constexpr double kMinVelocityThreshold = 0.1;
}

class MpcController : public nav2_core::Controller
{
public:
  MpcController() { setlocale(LC_ALL, ""); }
  ~MpcController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override {
    std::lock_guard<std::mutex> lk(plan_mutex_);
    v_ref_max_ = percentage ? v_ref_max_base_ * speed_limit / 100.0 : speed_limit;
    for (auto & v : path_speed_limit_) v = std::min(v, v_ref_max_);
  }

protected:
  double computeDistanceToGoal(const geometry_msgs::msg::PoseStamped & current_pose);
  void publishLocalPath(const tf2::Transform & base_to_odom_tf);
  bool getOdomControlState(tf2::Transform & base_to_odom_tf, double & state_time_sec);
  void updateTargetIndex();
  void generateReferenceTrajectory(const tf2::Transform & base_to_odom_tf);
  double computeReferenceTimeScale() const;
  bool sampleMincoReference(double r_x, double r_y, double ref_time_scale);
  void samplePathFallback(double r_x, double r_y, double ref_time_scale);
  void enforceNoReverseTrackingCommand(
    geometry_msgs::msg::TwistStamped & cmd,
    double r_x, double r_y, double cp, double sp);
  double computeLateralError(const geometry_msgs::msg::PoseStamped & pose, size_t idx) const;
  void mincoTrajCallback(const sentry_nav_interfaces::msg::MincoTrajectory::SharedPtr msg);
  bool buildTrajFromMsg(const sentry_nav_interfaces::msg::MincoTrajectory & msg, Trajectory<5> & traj);
  std::vector<ObstacleConstraint> buildObstacleConstraints(
    const State & current, const std::vector<State> & ref);
  void appendDynamicObstacleConstraints(
    const State & current, std::vector<ObstacleConstraint> & result);

private:
  struct MincoReferenceContext
  {
    Trajectory<5> traj;
    tf2::Transform map_to_odom;
    double traj_dur{0.0};
    double msg_age{0.0};
    double exec_time{0.0};
    double start_offset_sec{0.0};
    uint64_t traj_id{0};
    std::string frame;
  };

  void declareParameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);
  void loadParameters(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    double & v_circle_max,
    double & terminal_weight,
    int & terminal_horizon,
    double & Qv,
    double & ax_max,
    double & ay_max);
  void createRosInterfaces(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);
  void validateParameters(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    double v_circle_max,
    double ay_max) const;
  void createMpcSolver(
    double v_circle_max,
    double terminal_weight,
    int terminal_horizon,
    double Qv,
    double ax_max,
    double ay_max);

  bool lookupControlTransform(
    tf2::Transform & base_to_odom_tf,
    rclcpp::Time & state_tf_stamp);
  double applyPoseJumpDamping(
    const tf2::Transform & base_to_odom_tf,
    const rclcpp::Time & state_tf_stamp,
    bool using_odom_state,
    double state_time_sec);
  void updateEffectiveReferenceSpeed(
    double r_x,
    double r_y,
    double pose_jump_speed_scale);
  std::vector<Eigen::Vector2d> buildPreviousPrediction(double r_x, double r_y) const;
  int applyObstacleAndEsdfConstraints(
    double r_x,
    double r_y,
    const std::vector<Eigen::Vector2d> & p_prev);
  void updateMeasuredVelocityAnchor(
    double vx_global,
    double vy_global,
    const rclcpp::Time & now,
    double odom_age_sec);
  void updateMpcVelocityAnchor();
  double currentVelocityAnchorSpeed() const;
  void applyHorizonSpeedLimitFloor();
  SolveResult solveMpcWithFallbacks(double r_x, double r_y, int active_count);
  bool handleInvalidOrFailedSolve(
    const SolveResult & solve,
    int active_count,
    const tf2::Transform & base_to_odom_tf,
    const std_msgs::msg::Header & cmd_header,
    geometry_msgs::msg::TwistStamped & cmd_out);
  bool checkPredictedCollision(
    double r_x,
    double r_y,
    geometry_msgs::msg::TwistStamped &);
  bool holePassModeActive() const;
  void applyGoalStopProtection(
    geometry_msgs::msg::TwistStamped & cmd,
    double r_x,
    double r_y);
  void finalizeCommand(
    geometry_msgs::msg::TwistStamped & cmd,
    double cp,
    double sp);
  bool loadActiveMincoReference(MincoReferenceContext & ctx);
  bool resolveMincoFrame(MincoReferenceContext & ctx);
  double updateMincoProjection(
    MincoReferenceContext & ctx,
    double r_x,
    double r_y);
  void fillTerminalMincoReference(const MincoReferenceContext & ctx);
  void sampleMincoHorizon(
    const MincoReferenceContext & ctx,
    double t_proj,
    double ref_time_scale);

  std::shared_ptr<MPC> mpc_;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  nav_msgs::msg::Path global_plan_odom_;
  geometry_msgs::msg::PoseStamped pose_;
  int consecutive_reject_count_ = 0;
  std::vector<State> ref;
  std::vector<State> v_ref_;

  size_t target_index_{0};
  size_t last_target_index_{0};
  int stuck_count_{0};
  std::vector<double> path_accumulated_dist_;
  double path_total_dist_{0.0};
  double current_s_{0.0};
  double last_ux = 0.0;
  double last_uy = 0.0;

  double v_ref_max_{3.0};
  double v_ref_max_base_{3.0};
  double v_ref_max_effective_{3.0};
  double k_curvature_{2.0};

  double QX_, QY_, R_, S_;
  int horizon_;
  double control_frequency_;
  double control_dt_;

  double reject_angle_deg_;
  double reject_dist_threshold_;
  int max_consecutive_rejects_;
  double local_plan_publish_period_sec_{0.10};
  rclcpp::Time last_local_plan_publish_time_{0, 0, RCL_ROS_TIME};

  bool enable_goal_slowdown_;
  double goal_slowdown_distance_;
  int stuck_threshold_frames_{50};
  double stuck_lateral_threshold_{0.5};
  double goal_stop_distance_{0.10};
  double brake_safety_factor_{0.7};
  double ax_max_{5.0};
  double ay_max_{5.0};
  bool curvature_speed_limit_enabled_{true};
  double lateral_accel_limit_{0.0};
  double curvature_speed_limit_min_speed_{0.15};

  mutable std::mutex plan_mutex_;

  // MINCO trajectory
  rclcpp::Subscription<sentry_nav_interfaces::msg::MincoTrajectory>::SharedPtr minco_traj_sub_;
  Trajectory<5> minco_traj_;
  rclcpp::Time traj_stamp_;
  rclcpp::Time minco_first_receive_stamp_{0, 0, RCL_ROS_TIME};
  double traj_duration_{0.0};
  bool has_minco_traj_{false};
  mutable std::mutex traj_mutex_;
  std::string minco_traj_topic_;
  std::string minco_traj_frame_;
  rclcpp::Time minco_start_time_{0, 0, RCL_ROS_TIME};
  double minco_start_offset_sec_{0.0};
  double minco_time_base_offset_sec_{0.0};
  uint64_t minco_trajectory_id_{0};
  uint64_t minco_goal_id_{0};
  int minco_cb_count_{0};
  double minco_timeout_sec_{4.0};
  double minco_goal_reset_tolerance_{0.5};
  bool reset_warm_start_on_goal_change_only_{true};
  bool preserve_warm_start_on_similar_plan_{true};
  double similar_plan_goal_tolerance_{0.5};
  double similar_plan_anchor_tolerance_{0.5};
  double similar_plan_forward_distance_{1.5};
  double similar_plan_max_deviation_{0.4};

  // arc-length speed profile
  std::vector<double> path_speed_limit_;
  double fallback_sample_speed_{1.5};
  bool allow_path_fallback_without_minco_{false};
  double minco_unavailable_failure_sec_{0.8};
  rclcpp::Time minco_unavailable_since_{0, 0, RCL_ROS_TIME};
  bool reference_waiting_for_minco_{false};
  bool reference_uses_minco_{false};

  double last_t_proj_{0.0};
  rclcpp::Time last_projection_update_time_{0, 0, RCL_ROS_TIME};
  uint64_t last_sampled_trajectory_id_{0};
  double minco_projection_search_ahead_sec_{0.30};
  double minco_projection_max_advance_sec_{0.12};
  double minco_projection_max_lag_sec_{0.80};
  double cmd_lookahead_sec_{0.01};
  bool enable_lateral_error_ref_scaling_{true};
  double lateral_error_slow_threshold_{0.30};
  double lateral_error_high_threshold_{0.60};
  double min_lateral_ref_time_scale_{0.60};
  double last_ref_time_scale_{1.0};
  double last_min_curvature_speed_limit_{0.0};

  std::vector<double> horizon_speed_limits_;  // per-step speed limits for MPC hard constraint

  bool prevent_tracking_reverse_{true};
  double reverse_guard_min_ref_speed_{0.05};
  double reverse_guard_allowance_{0.03};
  double collision_stop_failure_sec_{0.8};
  rclcpp::Time collision_stop_since_{0, 0, RCL_ROS_TIME};

  bool enable_pose_jump_damping_{true};
  double pose_jump_distance_threshold_{0.25};
  double pose_jump_speed_threshold_{6.0};
  double pose_jump_damping_scale_{0.35};
  int pose_jump_damping_frames_{6};
  int pose_jump_damping_count_{0};
  bool has_last_tf_pose_{false};
  double last_tf_x_{0.0};
  double last_tf_y_{0.0};
  rclcpp::Time last_tf_stamp_{0, 0, RCL_ROS_TIME};
  double last_state_time_sec_{0.0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry latest_odom_;
  rclcpp::Time latest_odom_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_odom_stamp_{0, 0, RCL_ROS_TIME};
  mutable std::mutex odom_mutex_;
  bool has_latest_odom_{false};
  std::string odom_topic_{"odometry"};
  bool use_odometry_state_{true};
  double max_odom_age_sec_{0.80};
  double max_odom_predict_dt_{0.10};

  bool enable_measured_velocity_anchor_{true};
  double velocity_anchor_blend_alpha_{0.30};
  double velocity_anchor_lowpass_alpha_{0.50};
  double velocity_anchor_max_age_sec_{0.15};
  double velocity_anchor_max_jump_{1.5};
  double measured_velocity_raw_x_{0.0};
  double measured_velocity_raw_y_{0.0};
  double filtered_velocity_anchor_x_{0.0};
  double filtered_velocity_anchor_y_{0.0};
  double velocity_anchor_x_{0.0};
  double velocity_anchor_y_{0.0};
  double measured_velocity_anchor_age_sec_{0.0};
  rclcpp::Time measured_velocity_anchor_time_{0, 0, RCL_ROS_TIME};
  bool has_measured_velocity_anchor_{false};
  bool has_filtered_velocity_anchor_{false};
  bool current_velocity_anchor_valid_{false};

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;

  // Dynamic obstacles from tracker
  rclcpp::Subscription<sentry_nav_interfaces::msg::TrackedObstacleArray>::SharedPtr dyn_obs_sub_;
  std::vector<sentry_nav_interfaces::msg::TrackedObstacle> dynamic_obstacles_;
  mutable std::mutex obs_mutex_;
  int prev_active_obs_count_{0};

  // Obstacle constraint parameters
  bool enable_dynamic_obstacle_avoidance_{false};
  std::string dynamic_obstacle_topic_{"dynamic_obstacles"};
  double dynamic_safety_margin_{0.3};
  double robot_radius_{0.2};
  int max_dynamic_obs_{5};

  bool allow_obstacle_retry_without_constraints_{false};
  bool allow_speed_limit_retry_without_limits_{false};

  std::string navigation_mode_topic_{"navigation_mode_manager/mode"};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_mode_sub_;
  mutable std::mutex navigation_mode_mutex_;
  bool hole_pass_mode_active_{false};

  bool debug_logging_{false};

  std::chrono::steady_clock::time_point last_compute_start_wall_{};
  bool has_last_compute_start_wall_{false};
};

}  // namespace f_mpc_controller
#endif  // F_MPC_CONTROLLER__MPC_CONTROLLER_HPP_
