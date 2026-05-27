#include "f_mpc_controller/mpc_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <Eigen/Dense>

using nav2_util::declare_parameter_if_not_declared;
using SteadyClock = std::chrono::steady_clock;

namespace f_mpc_controller
{

void MpcController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // 初始化控制器依赖、参数、ROS 接口和 MPC 求解器，确保运行前所有尺寸相关配置已经确定。
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  clock_ = node->get_clock();

  declareParameters(node);

  double v_circle_max = 3.0;
  double terminal_weight = 5.0;
  int terminal_horizon = 2;
  double Qv = 0.5;
  double ax_max = 5.0;
  double ay_max = 5.0;
  loadParameters(node, v_circle_max, terminal_weight, terminal_horizon, Qv, ax_max, ay_max);

  control_dt_ = 1.0 / control_frequency_;
  createRosInterfaces(node);
  validateParameters(node, v_circle_max, ay_max);
  createMpcSolver(v_circle_max, terminal_weight, terminal_horizon, Qv, ax_max, ay_max);

  RCLCPP_INFO(node->get_logger(), "MpcController [%s] configured with horizon %d and dt %.3f",
              plugin_name_.c_str(), horizon_, control_dt_);
}

void MpcController::declareParameters(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  declare_parameter_if_not_declared(node, plugin_name_ + ".QX", rclcpp::ParameterValue(50.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".QY", rclcpp::ParameterValue(30.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".R", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".S", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".reject_angle_deg", rclcpp::ParameterValue(120.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".reject_dist_threshold", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_consecutive_rejects", rclcpp::ParameterValue(3));
  declare_parameter_if_not_declared(node, plugin_name_ + ".local_plan_publish_period_sec",
    rclcpp::ParameterValue(0.10));
  declare_parameter_if_not_declared(node, plugin_name_ + ".horizon", rclcpp::ParameterValue(10));
  declare_parameter_if_not_declared(node, plugin_name_ + ".control_frequency", rclcpp::ParameterValue(20.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".enable_goal_slowdown", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".goal_slowdown_distance", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".terminal_weight", rclcpp::ParameterValue(5.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".terminal_horizon", rclcpp::ParameterValue(2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".Qv", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".v_ref_max", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".k_curvature", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".stuck_threshold_frames", rclcpp::ParameterValue(50));
  declare_parameter_if_not_declared(node, plugin_name_ + ".stuck_lateral_threshold", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".ax_max", rclcpp::ParameterValue(5.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".ay_max", rclcpp::ParameterValue(5.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".curvature_speed_limit_enabled",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".lateral_accel_limit",
    rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".curvature_speed_limit_min_speed",
    rclcpp::ParameterValue(0.15));
  declare_parameter_if_not_declared(node, plugin_name_ + ".v_circle_max", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".goal_stop_distance", rclcpp::ParameterValue(0.10));
  declare_parameter_if_not_declared(node, plugin_name_ + ".brake_safety_factor", rclcpp::ParameterValue(0.7));
  declare_parameter_if_not_declared(node, plugin_name_ + ".use_odometry_state",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".odom_topic",
    rclcpp::ParameterValue("odometry"));
  declare_parameter_if_not_declared(node, plugin_name_ + ".state_frame",
    rclcpp::ParameterValue(std::string("")));
  declare_parameter_if_not_declared(node, plugin_name_ + ".command_frame",
    rclcpp::ParameterValue(std::string("")));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_odom_age_sec",
    rclcpp::ParameterValue(0.80));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_odom_predict_dt",
    rclcpp::ParameterValue(0.10));
  declare_parameter_if_not_declared(node, plugin_name_ + ".enable_measured_velocity_anchor",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".velocity_anchor_blend_alpha",
    rclcpp::ParameterValue(0.30));
  declare_parameter_if_not_declared(node, plugin_name_ + ".velocity_anchor_lowpass_alpha",
    rclcpp::ParameterValue(0.50));
  declare_parameter_if_not_declared(node, plugin_name_ + ".velocity_anchor_max_age_sec",
    rclcpp::ParameterValue(0.15));
  declare_parameter_if_not_declared(node, plugin_name_ + ".velocity_anchor_max_jump",
    rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".minco_traj_topic",
    rclcpp::ParameterValue("trajectory_manager/trajectory_for_mpc"));
  declare_parameter_if_not_declared(node, plugin_name_ + ".fallback_sample_speed",
    rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".allow_path_fallback_without_minco",
                                    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, plugin_name_ + ".minco_unavailable_failure_sec",
                                    rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".prevent_tracking_reverse",
                                    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".reverse_guard_min_ref_speed",
                                    rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, plugin_name_ + ".reverse_guard_allowance",
                                    rclcpp::ParameterValue(0.03));
  declare_parameter_if_not_declared(node, plugin_name_ + ".reference_direction_constraints_enabled",
                                    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_reverse_speed",
                                    rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_lateral_correction_speed",
                                    rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(node, plugin_name_ + ".collision_stop_failure_sec",
                                    rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".minco_timeout_sec",
    rclcpp::ParameterValue(4.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".minco_goal_reset_tolerance",
    rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".reset_warm_start_on_goal_change_only",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".preserve_warm_start_on_similar_plan",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".similar_plan_goal_tolerance",
    rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".similar_plan_anchor_tolerance",
    rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".similar_plan_forward_distance",
    rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".similar_plan_max_deviation",
    rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(node, plugin_name_ + ".cmd_lookahead_sec",
    rclcpp::ParameterValue(0.01));
  declare_parameter_if_not_declared(node, plugin_name_ + ".minco_projection_search_ahead_sec",
    rclcpp::ParameterValue(0.30));
  declare_parameter_if_not_declared(node, plugin_name_ + ".minco_projection_max_advance_sec",
    rclcpp::ParameterValue(0.12));
  declare_parameter_if_not_declared(node, plugin_name_ + ".minco_projection_max_lag_sec",
    rclcpp::ParameterValue(0.80));
  declare_parameter_if_not_declared(node, plugin_name_ + ".enable_lateral_error_ref_scaling",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".lateral_error_slow_threshold",
    rclcpp::ParameterValue(0.30));
  declare_parameter_if_not_declared(node, plugin_name_ + ".lateral_error_high_threshold",
    rclcpp::ParameterValue(0.60));
  declare_parameter_if_not_declared(node, plugin_name_ + ".min_lateral_ref_time_scale",
    rclcpp::ParameterValue(0.60));
  declare_parameter_if_not_declared(node, plugin_name_ + ".enable_pose_jump_damping",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".pose_jump_distance_threshold",
    rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(node, plugin_name_ + ".pose_jump_speed_threshold",
    rclcpp::ParameterValue(6.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".pose_jump_damping_scale",
    rclcpp::ParameterValue(0.35));
  declare_parameter_if_not_declared(node, plugin_name_ + ".pose_jump_damping_frames",
    rclcpp::ParameterValue(6));
  declare_parameter_if_not_declared(node, plugin_name_ + ".enable_dynamic_obstacle_avoidance", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, plugin_name_ + ".dynamic_obstacle_topic", rclcpp::ParameterValue(std::string("dynamic_obstacles")));
  declare_parameter_if_not_declared(node, plugin_name_ + ".dynamic_safety_margin", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(node, plugin_name_ + ".robot_radius", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_dynamic_obstacles", rclcpp::ParameterValue(5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".allow_obstacle_retry_without_constraints",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, plugin_name_ + ".allow_speed_limit_retry_without_limits",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, plugin_name_ + ".navigation_mode_topic",
    rclcpp::ParameterValue(std::string("navigation_mode_manager/mode")));
  declare_parameter_if_not_declared(node, plugin_name_ + ".debug_logging",
    rclcpp::ParameterValue(false));
}

void MpcController::loadParameters(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  double & v_circle_max,
  double & terminal_weight,
  int & terminal_horizon,
  double & Qv,
  double & ax_max,
  double & ay_max)
{
  node->get_parameter(plugin_name_ + ".v_circle_max", v_circle_max);
  node->get_parameter(plugin_name_ + ".QX", QX_);
  node->get_parameter(plugin_name_ + ".QY", QY_);
  node->get_parameter(plugin_name_ + ".R", R_);
  node->get_parameter(plugin_name_ + ".S", S_);
  node->get_parameter(plugin_name_ + ".horizon", horizon_);
  node->get_parameter(plugin_name_ + ".control_frequency", control_frequency_);
  node->get_parameter(plugin_name_ + ".reject_angle_deg", reject_angle_deg_);
  node->get_parameter(plugin_name_ + ".reject_dist_threshold", reject_dist_threshold_);
  node->get_parameter(plugin_name_ + ".max_consecutive_rejects", max_consecutive_rejects_);
  node->get_parameter(plugin_name_ + ".local_plan_publish_period_sec", local_plan_publish_period_sec_);
  node->get_parameter(plugin_name_ + ".enable_goal_slowdown", enable_goal_slowdown_);
  node->get_parameter(plugin_name_ + ".goal_slowdown_distance", goal_slowdown_distance_);
  node->get_parameter(plugin_name_ + ".terminal_weight", terminal_weight);
  node->get_parameter(plugin_name_ + ".terminal_horizon", terminal_horizon);
  node->get_parameter(plugin_name_ + ".Qv", Qv);
  node->get_parameter(plugin_name_ + ".v_ref_max", v_ref_max_);
  v_ref_max_base_ = v_ref_max_;
  node->get_parameter(plugin_name_ + ".k_curvature", k_curvature_);
  node->get_parameter(plugin_name_ + ".stuck_threshold_frames", stuck_threshold_frames_);
  node->get_parameter(plugin_name_ + ".stuck_lateral_threshold", stuck_lateral_threshold_);
  node->get_parameter(plugin_name_ + ".ax_max", ax_max);
  node->get_parameter(plugin_name_ + ".ay_max", ay_max);
  ax_max_ = ax_max;
  ay_max_ = ay_max;
  node->get_parameter(
    plugin_name_ + ".curvature_speed_limit_enabled", curvature_speed_limit_enabled_);
  node->get_parameter(plugin_name_ + ".lateral_accel_limit", lateral_accel_limit_);
  if (lateral_accel_limit_ <= 0.0) {
    lateral_accel_limit_ = ay_max_;
  }
  node->get_parameter(
    plugin_name_ + ".curvature_speed_limit_min_speed", curvature_speed_limit_min_speed_);
  node->get_parameter(plugin_name_ + ".goal_stop_distance", goal_stop_distance_);
  node->get_parameter(plugin_name_ + ".brake_safety_factor", brake_safety_factor_);
  node->get_parameter(plugin_name_ + ".use_odometry_state", use_odometry_state_);
  node->get_parameter(plugin_name_ + ".odom_topic", odom_topic_);
  node->get_parameter(plugin_name_ + ".state_frame", state_frame_);
  node->get_parameter(plugin_name_ + ".command_frame", command_frame_);
  if (state_frame_.empty()) {
    state_frame_ = costmap_ros_->getBaseFrameID();
    RCLCPP_WARN(
      node->get_logger(),
      "FollowPath.state_frame is empty; falling back to Nav2 base frame '%s'. "
      "Set state_frame explicitly when using a fake command frame.",
      state_frame_.c_str());
  }
  if (command_frame_.empty()) {
    command_frame_ = costmap_ros_->getBaseFrameID();
  }
  node->get_parameter(plugin_name_ + ".max_odom_age_sec", max_odom_age_sec_);
  node->get_parameter(plugin_name_ + ".max_odom_predict_dt", max_odom_predict_dt_);
  node->get_parameter(
    plugin_name_ + ".enable_measured_velocity_anchor", enable_measured_velocity_anchor_);
  node->get_parameter(
    plugin_name_ + ".velocity_anchor_blend_alpha", velocity_anchor_blend_alpha_);
  velocity_anchor_blend_alpha_ = std::clamp(velocity_anchor_blend_alpha_, 0.0, 1.0);
  node->get_parameter(
    plugin_name_ + ".velocity_anchor_lowpass_alpha", velocity_anchor_lowpass_alpha_);
  velocity_anchor_lowpass_alpha_ = std::clamp(velocity_anchor_lowpass_alpha_, 0.0, 1.0);
  node->get_parameter(
    plugin_name_ + ".velocity_anchor_max_age_sec", velocity_anchor_max_age_sec_);
  node->get_parameter(plugin_name_ + ".velocity_anchor_max_jump", velocity_anchor_max_jump_);
  node->get_parameter(plugin_name_ + ".minco_traj_topic", minco_traj_topic_);
  node->get_parameter(plugin_name_ + ".fallback_sample_speed", fallback_sample_speed_);
  node->get_parameter(
    plugin_name_ + ".allow_path_fallback_without_minco", allow_path_fallback_without_minco_);
  node->get_parameter(
    plugin_name_ + ".minco_unavailable_failure_sec", minco_unavailable_failure_sec_);
  node->get_parameter(plugin_name_ + ".prevent_tracking_reverse", prevent_tracking_reverse_);
  node->get_parameter(plugin_name_ + ".reverse_guard_min_ref_speed", reverse_guard_min_ref_speed_);
  node->get_parameter(plugin_name_ + ".reverse_guard_allowance", reverse_guard_allowance_);
  node->get_parameter(
    plugin_name_ + ".reference_direction_constraints_enabled",
    reference_direction_constraints_enabled_);
  node->get_parameter(plugin_name_ + ".max_reverse_speed", max_reverse_speed_);
  max_reverse_speed_ = std::max(0.0, max_reverse_speed_);
  node->get_parameter(
    plugin_name_ + ".max_lateral_correction_speed", max_lateral_correction_speed_);
  max_lateral_correction_speed_ = std::max(0.0, max_lateral_correction_speed_);
  node->get_parameter(plugin_name_ + ".collision_stop_failure_sec", collision_stop_failure_sec_);
  node->get_parameter(plugin_name_ + ".minco_timeout_sec", minco_timeout_sec_);
  node->get_parameter(plugin_name_ + ".minco_goal_reset_tolerance", minco_goal_reset_tolerance_);
  node->get_parameter(
    plugin_name_ + ".reset_warm_start_on_goal_change_only",
    reset_warm_start_on_goal_change_only_);
  node->get_parameter(
    plugin_name_ + ".preserve_warm_start_on_similar_plan", preserve_warm_start_on_similar_plan_);
  node->get_parameter(plugin_name_ + ".similar_plan_goal_tolerance", similar_plan_goal_tolerance_);
  node->get_parameter(plugin_name_ + ".similar_plan_anchor_tolerance", similar_plan_anchor_tolerance_);
  node->get_parameter(plugin_name_ + ".similar_plan_forward_distance", similar_plan_forward_distance_);
  node->get_parameter(plugin_name_ + ".similar_plan_max_deviation", similar_plan_max_deviation_);
  node->get_parameter(plugin_name_ + ".cmd_lookahead_sec", cmd_lookahead_sec_);
  node->get_parameter(
    plugin_name_ + ".minco_projection_search_ahead_sec", minco_projection_search_ahead_sec_);
  node->get_parameter(
    plugin_name_ + ".minco_projection_max_advance_sec", minco_projection_max_advance_sec_);
  node->get_parameter(
    plugin_name_ + ".minco_projection_max_lag_sec", minco_projection_max_lag_sec_);
  node->get_parameter(
    plugin_name_ + ".enable_lateral_error_ref_scaling", enable_lateral_error_ref_scaling_);
  node->get_parameter(plugin_name_ + ".lateral_error_slow_threshold", lateral_error_slow_threshold_);
  node->get_parameter(plugin_name_ + ".lateral_error_high_threshold", lateral_error_high_threshold_);
  node->get_parameter(plugin_name_ + ".min_lateral_ref_time_scale", min_lateral_ref_time_scale_);
  node->get_parameter(plugin_name_ + ".enable_pose_jump_damping", enable_pose_jump_damping_);
  node->get_parameter(plugin_name_ + ".pose_jump_distance_threshold", pose_jump_distance_threshold_);
  node->get_parameter(plugin_name_ + ".pose_jump_speed_threshold", pose_jump_speed_threshold_);
  node->get_parameter(plugin_name_ + ".pose_jump_damping_scale", pose_jump_damping_scale_);
  node->get_parameter(plugin_name_ + ".pose_jump_damping_frames", pose_jump_damping_frames_);
  node->get_parameter(plugin_name_ + ".enable_dynamic_obstacle_avoidance", enable_dynamic_obstacle_avoidance_);
  node->get_parameter(plugin_name_ + ".dynamic_obstacle_topic", dynamic_obstacle_topic_);
  node->get_parameter(plugin_name_ + ".dynamic_safety_margin", dynamic_safety_margin_);
  node->get_parameter(plugin_name_ + ".robot_radius", robot_radius_);
  node->get_parameter(plugin_name_ + ".max_dynamic_obstacles", max_dynamic_obs_);
  node->get_parameter(
    plugin_name_ + ".allow_obstacle_retry_without_constraints",
    allow_obstacle_retry_without_constraints_);
  node->get_parameter(
    plugin_name_ + ".allow_speed_limit_retry_without_limits",
    allow_speed_limit_retry_without_limits_);
  node->get_parameter(plugin_name_ + ".navigation_mode_topic", navigation_mode_topic_);
  node->get_parameter(plugin_name_ + ".debug_logging", debug_logging_);
}

void MpcController::createRosInterfaces(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", rclcpp::QoS(10));
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS(1),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(odom_mutex_);
      latest_odom_ = *msg;
      latest_odom_receive_time_ = clock_->now();
      latest_odom_stamp_ = rclcpp::Time(msg->header.stamp);
      has_latest_odom_ = true;
    });

  minco_traj_sub_ = node->create_subscription<sentry_nav_interfaces::msg::MincoTrajectory>(
    minco_traj_topic_, rclcpp::QoS(1),
    std::bind(&MpcController::mincoTrajCallback, this, std::placeholders::_1));

  navigation_mode_sub_ = node->create_subscription<std_msgs::msg::String>(
    navigation_mode_topic_, rclcpp::QoS(10),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(navigation_mode_mutex_);
      hole_pass_mode_active_ = msg->data == "hole_pass";
    });

  if (enable_dynamic_obstacle_avoidance_) {
    dyn_obs_sub_ = node->create_subscription<sentry_nav_interfaces::msg::TrackedObstacleArray>(
      dynamic_obstacle_topic_, rclcpp::SensorDataQoS(),
      [this](const sentry_nav_interfaces::msg::TrackedObstacleArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(obs_mutex_);
        dynamic_obstacles_ = msg->obstacles;
      });
  }
}

void MpcController::validateParameters(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  double v_circle_max,
  double ay_max) const
{
  if (pose_jump_speed_threshold_ < v_ref_max_ * 2.0) {
    RCLCPP_WARN(
      node->get_logger(),
      "Motion profile mismatch: pose_jump_speed_threshold %.2f is too close to v_ref_max %.2f",
      pose_jump_speed_threshold_, v_ref_max_);
  }
  if (fallback_sample_speed_ > v_ref_max_) {
    RCLCPP_WARN(
      node->get_logger(),
      "Motion profile mismatch: fallback_sample_speed %.2f > v_ref_max %.2f",
      fallback_sample_speed_, v_ref_max_);
  }
  if (v_ref_max_ > v_circle_max + 1.0e-6) {
    RCLCPP_WARN(
      node->get_logger(),
      "Motion profile mismatch: v_ref_max %.2f > v_circle_max %.2f; MPC hard speed limit will dominate",
      v_ref_max_, v_circle_max);
  }
  if (ax_max_ <= 0.0 || ay_max <= 0.0) {
    RCLCPP_WARN(
      node->get_logger(),
      "Motion profile mismatch: non-positive acceleration limit ax=%.2f ay=%.2f",
      ax_max_, ay_max);
  }
  if (lateral_accel_limit_ <= 0.0) {
    RCLCPP_WARN(
      node->get_logger(),
      "Motion profile mismatch: non-positive lateral_accel_limit %.2f",
      lateral_accel_limit_);
  }
  if (velocity_anchor_max_age_sec_ <= 0.0 && enable_measured_velocity_anchor_) {
    RCLCPP_WARN(
      node->get_logger(),
      "velocity_anchor_max_age_sec %.3f disables measured velocity anchoring",
      velocity_anchor_max_age_sec_);
  }
  if (state_frame_.empty() || command_frame_.empty()) {
    RCLCPP_WARN(
      node->get_logger(),
      "MPC frame contract incomplete: state_frame='%s' command_frame='%s'",
      state_frame_.c_str(), command_frame_.c_str());
  }
  if (state_frame_ != costmap_ros_->getBaseFrameID() ||
      command_frame_ != costmap_ros_->getBaseFrameID())
  {
    RCLCPP_INFO(
      node->get_logger(),
      "MPC frame contract: Nav2 base='%s' state_frame='%s' command_frame='%s'",
      costmap_ros_->getBaseFrameID().c_str(), state_frame_.c_str(), command_frame_.c_str());
  }
  if (reference_direction_constraints_enabled_ &&
      max_lateral_correction_speed_ <= 1.0e-6)
  {
    RCLCPP_WARN(
      node->get_logger(),
      "reference_direction_constraints_enabled=true but max_lateral_correction_speed is %.3f; "
      "tracking may become infeasible on lateral error",
      max_lateral_correction_speed_);
  }
  if (reference_direction_constraints_enabled_ &&
      max_lateral_correction_speed_ > v_ref_max_base_)
  {
    RCLCPP_WARN(
      node->get_logger(),
      "max_lateral_correction_speed %.2f is above v_ref_max %.2f; direction constraints will not "
      "meaningfully limit sideways tracking",
      max_lateral_correction_speed_, v_ref_max_base_);
  }
  if (minco_timeout_sec_ < 2.0 / std::max(control_frequency_, 1.0)) {
    RCLCPP_WARN(
      node->get_logger(),
      "Motion profile mismatch: minco_timeout_sec %.3f is close to controller period %.3f",
      minco_timeout_sec_, control_dt_);
  }
}

void MpcController::createMpcSolver(
  double v_circle_max,
  double terminal_weight,
  int terminal_horizon,
  double Qv,
  double ax_max,
  double ay_max)
{
  // 在所有参数读取完成后创建求解器，避免障碍物约束矩阵尺寸使用未初始化配置。
  mpc_ = std::make_shared<MPC>(control_dt_, horizon_,
                                QX_, QY_, R_, S_, terminal_weight, terminal_horizon, Qv,
                                ax_max, ay_max, v_circle_max,
                                max_dynamic_obs_);
}

void MpcController::cleanup()
{
  auto node = node_.lock();
  if (!node) return;
  local_path_pub_.reset();
  minco_traj_sub_.reset();
  dyn_obs_sub_.reset();
  navigation_mode_sub_.reset();
  RCLCPP_INFO(node->get_logger(), "MpcController cleanup");
}

void MpcController::activate()
{
  auto node = node_.lock();
  if (!node) return;
  local_path_pub_->on_activate();
  RCLCPP_INFO(node->get_logger(), "MpcController activated");
}

void MpcController::deactivate()
{
  auto node = node_.lock();
  if (!node) return;
  local_path_pub_->on_deactivate();
  RCLCPP_INFO(node->get_logger(), "MpcController deactivated");
}

bool MpcController::lookupControlTransform(
  tf2::Transform & state_to_odom_tf,
  tf2::Transform & command_to_odom_tf,
  rclcpp::Time & state_tf_stamp)
{
  // 获取当前控制周期使用的机器人位姿 TF，失败时直接输出停止命令。
  auto node = node_.lock();
  try {
    const std::string global_frame = costmap_ros_->getGlobalFrameID();
    auto state_ts = tf_->lookupTransform(global_frame, state_frame_, tf2::TimePointZero);
    tf2::fromMsg(state_ts.transform, state_to_odom_tf);
    state_tf_stamp = rclcpp::Time(state_ts.header.stamp);

    if (command_frame_ == state_frame_) {
      command_to_odom_tf = state_to_odom_tf;
    } else {
      auto command_ts = tf_->lookupTransform(global_frame, command_frame_, tf2::TimePointZero);
      tf2::fromMsg(command_ts.transform, command_to_odom_tf);
    }
  } catch (tf2::TransformException & ex) {
    if (node) {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *clock_, 500,
        "computeVelocityCommands: TF failed for state_frame='%s' command_frame='%s': %s — stopping",
        state_frame_.c_str(), command_frame_.c_str(), ex.what());
    }
    return false;
  }
  return true;
}

double MpcController::applyPoseJumpDamping(
  const tf2::Transform & base_to_odom_tf,
  const rclcpp::Time & state_tf_stamp,
  bool using_odom_state,
  double state_time_sec)
{
  // 检测定位位姿突跳并短时间降低参考速度，避免 MPC 追踪瞬时错误状态。
  double pose_jump_speed_scale = 1.0;
  if (!enable_pose_jump_damping_) {
    return pose_jump_speed_scale;
  }

  auto node = node_.lock();
  const rclcpp::Time now = clock_->now();
  const double tf_x = base_to_odom_tf.getOrigin().x();
  const double tf_y = base_to_odom_tf.getOrigin().y();
  if (has_last_tf_pose_) {
    double dt = 0.0;
    if (using_odom_state) {
      dt = state_time_sec - last_state_time_sec_;
    } else {
      try {
        dt = (state_tf_stamp - last_tf_stamp_).seconds();
      } catch (const std::runtime_error &) {
        dt = (now - last_tf_stamp_).seconds();
      }
    }
    const double dist = std::hypot(tf_x - last_tf_x_, tf_y - last_tf_y_);
    const double implied_speed = dt > 1.0e-3 ? dist / dt : 0.0;
    if (dt > 1.0e-3 && dt < 1.0 &&
        (dist > pose_jump_distance_threshold_ && implied_speed > pose_jump_speed_threshold_))
    {
      pose_jump_damping_count_ = std::max(1, pose_jump_damping_frames_);
      if (node) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *clock_, 500,
          "Pose jump detected: dist=%.2fm dt=%.3fs speed=%.2fm/s — damping ref speed",
          dist, dt, implied_speed);
      }
    }
  }
  last_tf_x_ = tf_x;
  last_tf_y_ = tf_y;
  last_tf_stamp_ = using_odom_state ? now : state_tf_stamp;
  last_state_time_sec_ = state_time_sec;
  has_last_tf_pose_ = true;

  if (pose_jump_damping_count_ > 0) {
    pose_jump_speed_scale = std::clamp(pose_jump_damping_scale_, 0.05, 1.0);
    --pose_jump_damping_count_;
  }
  return pose_jump_speed_scale;
}

void MpcController::updateEffectiveReferenceSpeed(
  double r_x,
  double r_y,
  double pose_jump_speed_scale)
{
  // 控制环速度上限只受导航速度限制和位姿跳变阻尼影响；静态障碍避让由上游轨迹和 costmap 负责。
  v_ref_max_effective_ = v_ref_max_;
  (void)r_x;
  (void)r_y;
  v_ref_max_effective_ *= pose_jump_speed_scale;
}

std::vector<Eigen::Vector2d> MpcController::buildPreviousPrediction(
  double r_x,
  double r_y) const
{
  // 用上一帧控制序列前向积分，作为动态障碍线性化轨迹。
  std::vector<Eigen::Vector2d> p_prev;
  if (!enable_dynamic_obstacle_avoidance_) {
    return p_prev;
  }

  p_prev.resize(horizon_);
  double px = r_x;
  double py = r_y;
  const auto & last_U = mpc_->getLastU();
  for (int k = 0; k < horizon_; ++k) {
    if (2 * k + 1 < static_cast<int>(last_U.size())) {
      px += last_U(2 * k) * control_dt_;
      py += last_U(2 * k + 1) * control_dt_;
    }
    p_prev[k] = Eigen::Vector2d(px, py);
  }
  return p_prev;
}

int MpcController::applyObstacleAndEsdfConstraints(
  double r_x,
  double r_y,
  const std::vector<Eigen::Vector2d> & p_prev)
{
  // 将动态障碍转换为本周期 MPC 约束。
  int active_count = 0;

  if (enable_dynamic_obstacle_avoidance_) {
    auto obstacles = buildObstacleConstraints({r_x, r_y}, ref);
    active_count = 0;
    for (const auto & o : obstacles) {
      for (double r : o.radii) {
        if (r > 0) {
          ++active_count;
          break;
        }
      }
    }
    if (active_count != prev_active_obs_count_) {
      mpc_->resetWarmStart();
    }
    prev_active_obs_count_ = active_count;
    mpc_->setObstacles(obstacles, Eigen::Vector2d(r_x, r_y), p_prev, ref);
  }

  return active_count;
}

void MpcController::updateMeasuredVelocityAnchor(
  double vx_global,
  double vy_global,
  const rclcpp::Time & now,
  double odom_age_sec)
{
  measured_velocity_raw_x_ = vx_global;
  measured_velocity_raw_y_ = vy_global;
  measured_velocity_anchor_age_sec_ = odom_age_sec;
  measured_velocity_anchor_time_ = now;

  if (!std::isfinite(vx_global) || !std::isfinite(vy_global)) {
    has_measured_velocity_anchor_ = false;
    return;
  }

  if (!has_filtered_velocity_anchor_) {
    filtered_velocity_anchor_x_ = vx_global;
    filtered_velocity_anchor_y_ = vy_global;
    has_filtered_velocity_anchor_ = true;
    has_measured_velocity_anchor_ = true;
    return;
  }

  const double jump = std::hypot(
    vx_global - filtered_velocity_anchor_x_,
    vy_global - filtered_velocity_anchor_y_);
  if (velocity_anchor_max_jump_ > 0.0 && jump > velocity_anchor_max_jump_) {
    has_measured_velocity_anchor_ = false;
    has_filtered_velocity_anchor_ = false;
    if (auto node = node_.lock()) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *clock_, 500,
        "Measured velocity anchor rejected: jump=%.2fm/s limit=%.2fm/s raw=(%.2f, %.2f) filtered=(%.2f, %.2f)",
        jump, velocity_anchor_max_jump_, vx_global, vy_global,
        filtered_velocity_anchor_x_, filtered_velocity_anchor_y_);
    }
    return;
  }

  filtered_velocity_anchor_x_ =
    velocity_anchor_lowpass_alpha_ * vx_global +
    (1.0 - velocity_anchor_lowpass_alpha_) * filtered_velocity_anchor_x_;
  filtered_velocity_anchor_y_ =
    velocity_anchor_lowpass_alpha_ * vy_global +
    (1.0 - velocity_anchor_lowpass_alpha_) * filtered_velocity_anchor_y_;
  has_measured_velocity_anchor_ = true;
}

bool MpcController::computeReferenceTangent(
  Eigen::Vector2d & tangent, int index, double r_x, double r_y) const
{
  tangent = Eigen::Vector2d::Zero();
  if (index < 0) {
    return false;
  }

  if (index < static_cast<int>(v_ref_.size())) {
    const auto & v = v_ref_[index];
    tangent = Eigen::Vector2d(v.x, v.y);
    if (tangent.norm() > reverse_guard_min_ref_speed_) {
      tangent.normalize();
      return true;
    }
  }

  if (index + 1 < static_cast<int>(ref.size())) {
    tangent = Eigen::Vector2d(ref[index + 1].x - ref[index].x, ref[index + 1].y - ref[index].y);
    if (tangent.norm() > 1.0e-4) {
      tangent.normalize();
      return true;
    }
  }

  if (index < static_cast<int>(ref.size())) {
    tangent = Eigen::Vector2d(ref[index].x - r_x, ref[index].y - r_y);
    if (tangent.norm() > 1.0e-4) {
      tangent.normalize();
      return true;
    }
  }

  if (ref.size() >= 2) {
    tangent = Eigen::Vector2d(ref.back().x - ref.front().x, ref.back().y - ref.front().y);
    if (tangent.norm() > 1.0e-4) {
      tangent.normalize();
      return true;
    }
  }

  return false;
}

std::vector<Eigen::Vector2d> MpcController::buildReferenceTangents(double r_x, double r_y) const
{
  std::vector<Eigen::Vector2d> tangents;
  tangents.reserve(horizon_);
  Eigen::Vector2d last_valid = Eigen::Vector2d::Zero();
  bool has_last_valid = false;
  for (int i = 0; i < horizon_; ++i) {
    Eigen::Vector2d tangent;
    if (computeReferenceTangent(tangent, i, r_x, r_y)) {
      last_valid = tangent;
      has_last_valid = true;
      tangents.push_back(tangent);
    } else if (has_last_valid) {
      tangents.push_back(last_valid);
    } else {
      tangents.push_back(Eigen::Vector2d::Zero());
    }
  }
  return tangents;
}

Eigen::Vector2d MpcController::constrainReferenceVelocity(
  const Eigen::Vector2d & velocity,
  double r_x,
  double r_y,
  bool * changed) const
{
  if (changed) {
    *changed = false;
  }
  if (!reference_direction_constraints_enabled_) {
    return velocity;
  }

  Eigen::Vector2d tangent;
  if (!computeReferenceTangent(tangent, 0, r_x, r_y)) {
    return velocity;
  }

  const Eigen::Vector2d normal(-tangent.y(), tangent.x());
  double longitudinal = velocity.dot(tangent);
  double lateral = velocity.dot(normal);
  const double constrained_longitudinal = std::max(longitudinal, -max_reverse_speed_);
  const double constrained_lateral =
    std::clamp(lateral, -max_lateral_correction_speed_, max_lateral_correction_speed_);
  if (changed) {
    *changed =
      std::abs(constrained_longitudinal - longitudinal) > 1.0e-6 ||
      std::abs(constrained_lateral - lateral) > 1.0e-6;
  }
  return constrained_longitudinal * tangent + constrained_lateral * normal;
}

void MpcController::updateMpcVelocityAnchor(double r_x, double r_y)
{
  current_velocity_anchor_valid_ = false;
  velocity_anchor_x_ = last_ux;
  velocity_anchor_y_ = last_uy;

  bool measured_valid = enable_measured_velocity_anchor_ && has_measured_velocity_anchor_;
  if (measured_valid) {
    const rclcpp::Time now = clock_->now();
    double anchor_age = measured_velocity_anchor_age_sec_;
    try {
      anchor_age = std::max(anchor_age, (now - measured_velocity_anchor_time_).seconds());
    } catch (const std::runtime_error &) {
      measured_valid = false;
    }
    if (anchor_age < -0.05 || anchor_age > velocity_anchor_max_age_sec_) {
      measured_valid = false;
    }
  }

  if (measured_valid) {
    const double alpha = velocity_anchor_blend_alpha_;
    velocity_anchor_x_ = alpha * filtered_velocity_anchor_x_ + (1.0 - alpha) * last_ux;
    velocity_anchor_y_ = alpha * filtered_velocity_anchor_y_ + (1.0 - alpha) * last_uy;
    current_velocity_anchor_valid_ = true;
  }

  bool anchor_changed = false;
  const Eigen::Vector2d constrained_anchor = constrainReferenceVelocity(
    Eigen::Vector2d(velocity_anchor_x_, velocity_anchor_y_), r_x, r_y, &anchor_changed);
  velocity_anchor_x_ = constrained_anchor.x();
  velocity_anchor_y_ = constrained_anchor.y();
  if (anchor_changed) {
    mpc_->resetWarmStart();
    if (auto node = node_.lock()) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *clock_, 500,
        "MPC velocity anchor constrained to reference direction: raw=(%.2f, %.2f) constrained=(%.2f, %.2f)",
        measured_valid ? filtered_velocity_anchor_x_ : last_ux,
        measured_valid ? filtered_velocity_anchor_y_ : last_uy,
        velocity_anchor_x_, velocity_anchor_y_);
    }
  }

  mpc_->setControlAnchorU(velocity_anchor_x_, velocity_anchor_y_);
}

void MpcController::updateReferenceDirectionConstraints(double r_x, double r_y)
{
  if (!mpc_) {
    return;
  }
  mpc_->setReferenceDirectionConstraints(
    buildReferenceTangents(r_x, r_y),
    max_reverse_speed_,
    max_lateral_correction_speed_,
    reference_direction_constraints_enabled_);
}

double MpcController::currentVelocityAnchorSpeed() const
{
  return std::hypot(velocity_anchor_x_, velocity_anchor_y_);
}

void MpcController::applyHorizonSpeedLimitFloor()
{
  // 对每步速度上限加入可达性下界，防止约束比当前速度刹停能力还激进。
  if (horizon_speed_limits_.empty()) {
    return;
  }

  double min_reachable_speed = currentVelocityAnchorSpeed();
  const double dv = std::max(ax_max_, 0.5) * control_dt_;
  const double octagon_scale = std::max(std::cos(M_PI / 8.0), 1.0e-3);
  for (size_t i = 0; i < horizon_speed_limits_.size(); ++i) {
    if (i > 0) {
      min_reachable_speed = std::max(0.0, min_reachable_speed - dv);
    }
    auto & v_limit = horizon_speed_limits_[i];
    const double required_limit = min_reachable_speed / octagon_scale;
    v_limit = std::clamp(std::max(v_limit, required_limit), 0.0, v_ref_max_base_);
  }
  mpc_->setHorizonSpeedLimits(horizon_speed_limits_);
}

SolveResult MpcController::solveMpcWithFallbacks(
  double r_x,
  double r_y,
  int active_count)
{
  // 先按完整约束求解，必要时按障碍约束和逐步速度限制的优先级降级重试。
  auto node = node_.lock();
  SolveResult solve = mpc_->solve({r_x, r_y}, ref, v_ref_);
  if (!solve.success && active_count > 0 && allow_obstacle_retry_without_constraints_) {
    mpc_->clearObstacles();
    mpc_->resetWarmStart();
    solve = mpc_->solve({r_x, r_y}, ref, v_ref_);
    if (node) {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *clock_, 500,
        "MPC hard obstacle constraints infeasible — retrying without hard obstacles "
        "(status=%d, active_obs=%d)",
        static_cast<int>(solve.status), active_count);
    }
  }
  if (!solve.success && active_count > 0 && !allow_obstacle_retry_without_constraints_) {
    if (node) {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *clock_, 500,
        "MPC hard obstacle constraints infeasible — stopping instead of clearing constraints "
        "(status=%d, active_obs=%d)",
        static_cast<int>(solve.status), active_count);
    }
  }
  if (!solve.success && active_count == 0 && !horizon_speed_limits_.empty() &&
      allow_speed_limit_retry_without_limits_)
  {
    mpc_->clearHorizonSpeedLimits();
    mpc_->resetWarmStart();
    solve = mpc_->solve({r_x, r_y}, ref, v_ref_);
    if (node) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *clock_, 500,
        "MPC speed-limited solve failed — retrying without per-horizon speed limits "
        "(status=%d)",
        static_cast<int>(solve.status));
    }
  } else if (!solve.success && active_count == 0 && !horizon_speed_limits_.empty() &&
             !allow_speed_limit_retry_without_limits_)
  {
    if (node) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *clock_, 500,
        "MPC speed-limited solve failed — stopping instead of clearing per-horizon speed limits "
        "(status=%d)",
        static_cast<int>(solve.status));
    }
  }
  return solve;
}

bool MpcController::handleInvalidOrFailedSolve(
  const SolveResult & solve,
  int active_count,
  const tf2::Transform & base_to_odom_tf,
  const std_msgs::msg::Header & cmd_header,
  geometry_msgs::msg::TwistStamped & cmd_out)
{
  // 统一处理 NaN、无解和求解失败，清空 warm start 并输出安全停止。
  auto node = node_.lock();
  const Control u = solve.control;
  if (std::isnan(u.vx) || std::isnan(u.vy) || std::isinf(u.vx) || std::isinf(u.vy)) {
    if (node) {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *clock_, 500,
        "MPC returned NaN/Inf — stopping");
    }
    last_ux = 0.0;
    last_uy = 0.0;
    mpc_->setControlAnchorU(0.0, 0.0);
    mpc_->resetWarmStart();
    cmd_out = geometry_msgs::msg::TwistStamped();
    cmd_out.header = cmd_header;
    return true;
  }

  if (!solve.success) {
    double min_limit = 0.0;
    double max_limit = 0.0;
    if (!horizon_speed_limits_.empty()) {
      auto minmax = std::minmax_element(horizon_speed_limits_.begin(), horizon_speed_limits_.end());
      min_limit = *minmax.first;
      max_limit = *minmax.second;
    }
    if (node) {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *clock_, 500,
        "MPC solver failed — stopping (status=%d, active_obs=%d, speed_limit=[%.2f, %.2f])",
        static_cast<int>(solve.status), active_count, min_limit, max_limit);
    }
    publishLocalPath(base_to_odom_tf);
    last_ux = 0.0;
    last_uy = 0.0;
    mpc_->setControlAnchorU(0.0, 0.0);
    mpc_->resetWarmStart();
    cmd_out = geometry_msgs::msg::TwistStamped();
    cmd_out.header = cmd_header;
    return true;
  }

  return false;
}

bool MpcController::checkPredictedCollision(
  double r_x,
  double r_y,
  geometry_msgs::msg::TwistStamped &)
{
  // 用 MPC 最新预测轨迹检查代价地图碰撞，持续碰撞时触发上层恢复。
  auto node = node_.lock();
  if (holePassModeActive()) {
    collision_stop_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    if (node) {
      RCLCPP_INFO_THROTTLE(
        node->get_logger(), *clock_, 1000,
        "MPC predicted collision check skipped in hole_pass mode");
    }
    return false;
  }

  auto costmap_ptr = costmap_ros_->getCostmap();
  double px = r_x;
  double py = r_y;
  bool collision = false;
  bool out_of_map = false;
  unsigned char hit_cost = 0;
  int hit_step = -1;
  double hit_x = px;
  double hit_y = py;
  const auto & last_U = mpc_->getLastU();
  const int check_steps = std::min(static_cast<int>(last_U.size() / 2), horizon_);
  const int hard_stop_steps = std::max(1, static_cast<int>(std::ceil(0.5 / control_dt_)));
  for (int ci = 0; ci < check_steps; ++ci) {
    px += last_U(2 * ci) * control_dt_;
    py += last_U(2 * ci + 1) * control_dt_;
    unsigned int mx, my;
    if (!costmap_ptr->worldToMap(px, py, mx, my)) {
      collision = true;
      out_of_map = true;
      hit_step = ci;
      hit_x = px;
      hit_y = py;
      break;
    }
    const auto cost = costmap_ptr->getCost(mx, my);
    if (cost >= 253) {
      collision = true;
      hit_cost = cost;
      hit_step = ci;
      hit_x = px;
      hit_y = py;
      break;
    }
  }

  if (!collision) {
    collision_stop_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    return false;
  }

  if (out_of_map && hit_step >= hard_stop_steps) {
    if (node) {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *clock_, 500,
        "Predicted trajectory leaves costmap late — resetting warm start only "
        "(step=%d x=%.2f y=%.2f)",
        hit_step, hit_x, hit_y);
    }
    mpc_->resetWarmStart();
    collision_stop_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    return false;
  }

  if (node) {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *clock_, 500,
      "Predicted trajectory collision — stopping (step=%d x=%.2f y=%.2f cost=%u out_of_map=%d)",
      hit_step, hit_x, hit_y, static_cast<unsigned int>(hit_cost), out_of_map ? 1 : 0);
  }
  last_ux = 0.0;
  last_uy = 0.0;
  mpc_->setControlAnchorU(0.0, 0.0);
  mpc_->resetWarmStart();
  const rclcpp::Time now = clock_->now();
  if (collision_stop_since_.nanoseconds() == 0) {
    collision_stop_since_ = now;
  }
  double stop_sec = 0.0;
  try {
    stop_sec = (now - collision_stop_since_).seconds();
  } catch (const std::runtime_error &) {
    collision_stop_since_ = now;
  }
  if (stop_sec >= collision_stop_failure_sec_) {
    collision_stop_since_ = now;
    throw nav2_core::PlannerException("MPC predicted trajectory remains in collision");
  }
  return true;
}

bool MpcController::holePassModeActive() const
{
  std::lock_guard<std::mutex> lk(navigation_mode_mutex_);
  return hole_pass_mode_active_;
}

void MpcController::applyGoalStopProtection(
  geometry_msgs::msg::TwistStamped & cmd,
  double r_x,
  double r_y)
{
  // 在接近终点时根据剩余距离限制速度，并在真正到达终端参考后强制停稳。
  auto node = node_.lock();
  const double reference_remaining =
    reference_distance_valid_ ?
    (reference_total_dist_ - reference_current_s_) :
    (path_total_dist_ - current_s_);
  double dist_to_goal_protect = std::max(0.0, reference_remaining);
  const double ref0_dist_for_goal = ref.empty() ?
    std::numeric_limits<double>::infinity() :
    std::hypot(ref.front().x - r_x, ref.front().y - r_y);
  const bool robot_reached_terminal_ref =
    ref0_dist_for_goal <= std::max(goal_stop_distance_ * 2.0, 0.30);
  const bool invalid_remote_terminal_hold =
    reference_distance_valid_ &&
    reference_total_dist_ <= std::max(goal_stop_distance_ * 0.5, 0.05) &&
    !robot_reached_terminal_ref;
  if (invalid_remote_terminal_hold) {
    if (node) {
      RCLCPP_ERROR_THROTTLE(
        node->get_logger(), *clock_, 500,
        "MPC received invalid terminal hold far from robot; zeroing cmd and forcing replan "
        "(ref0_dist=%.2f ref_total=%.3f ref_s=%.3f path_s=%.2f path_total=%.2f)",
        ref0_dist_for_goal, reference_total_dist_, reference_current_s_,
        current_s_, path_total_dist_);
    }
    cmd.twist.linear.x = 0.0;
    cmd.twist.linear.y = 0.0;
    last_ux = 0.0;
    last_uy = 0.0;
    if (mpc_) {
      mpc_->setControlAnchorU(0.0, 0.0);
      mpc_->resetWarmStart();
    }
    throw nav2_core::PlannerException("MPC received invalid terminal hold far from robot");
  }
  if (dist_to_goal_protect < goal_stop_distance_ && robot_reached_terminal_ref) {
    if (node) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *clock_, 500,
        "Goal stop protection zeroed cmd: dist_to_goal=%.3f threshold=%.3f ref0_dist=%.2f "
        "ref_s=%.2f ref_total=%.2f path_s=%.2f path_total=%.2f",
        dist_to_goal_protect, goal_stop_distance_, ref0_dist_for_goal,
        reference_current_s_, reference_total_dist_, current_s_, path_total_dist_);
    }
    cmd.twist.linear.x = 0.0;
    cmd.twist.linear.y = 0.0;
  } else if (dist_to_goal_protect < goal_stop_distance_) {
    if (node) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *clock_, 500,
        "Goal stop protection suppressed: remaining=%.3f threshold=%.3f but ref0_dist=%.2f",
        dist_to_goal_protect, goal_stop_distance_, ref0_dist_for_goal);
    }
  } else {
    double a_safe = std::max(0.5, ax_max_ * brake_safety_factor_);
    double v_max_for_distance = std::sqrt(2.0 * a_safe * dist_to_goal_protect);
    double cmd_v = std::hypot(cmd.twist.linear.x, cmd.twist.linear.y);
    if (cmd_v > v_max_for_distance && cmd_v > 1e-3) {
      double scale = v_max_for_distance / cmd_v;
      cmd.twist.linear.x *= scale;
      cmd.twist.linear.y *= scale;
    }
  }
}

void MpcController::finalizeCommand(
  geometry_msgs::msg::TwistStamped & cmd,
  double cp,
  double sp)
{
  // 将最终输出记录回 odom 坐标系，供下一周期 warm start、限加速度和反向保护使用。
  last_ux = cp * cmd.twist.linear.x - sp * cmd.twist.linear.y;
  last_uy = sp * cmd.twist.linear.x + cp * cmd.twist.linear.y;
  mpc_->setControlAnchorU(last_ux, last_uy);
  if (auto node = node_.lock()) {
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *clock_, 1000,
      "computeVelocityCommands: cmd_base=(%.3f, %.3f) cmd_odom=(%.3f, %.3f) ref=%zu",
      cmd.twist.linear.x, cmd.twist.linear.y, last_ux, last_uy, ref.size());
  }
}

geometry_msgs::msg::TwistStamped MpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker *)
{
  // Nav2 每个控制周期的主入口：更新状态、生成参考、求解 MPC 并返回底盘速度。
  const auto t_start = SteadyClock::now();
  auto t_last = t_start;
  const auto mark_ms = [&t_last]() {
    const auto now = SteadyClock::now();
    const double ms = std::chrono::duration<double, std::milli>(now - t_last).count();
    t_last = now;
    return ms;
  };
  const double compute_interval_ms = has_last_compute_start_wall_ ?
    std::chrono::duration<double, std::milli>(t_start - last_compute_start_wall_).count() : 0.0;
  last_compute_start_wall_ = t_start;
  has_last_compute_start_wall_ = true;

  auto node = node_.lock();
  pose_ = pose;
  if (!node) {
    throw nav2_core::PlannerException("MPC controller node expired");
  }
  if (global_plan_odom_.poses.empty()) {
    throw nav2_core::PlannerException("MPC has no global path");
  }

  tf2::Transform state_to_odom_tf;
  tf2::Transform command_to_odom_tf;
  rclcpp::Time state_tf_stamp = clock_->now();
  if (!lookupControlTransform(state_to_odom_tf, command_to_odom_tf, state_tf_stamp)) {
    return geometry_msgs::msg::TwistStamped();
  }
  const double tf_ms = mark_ms();

  double state_time_sec = clock_->now().seconds();
  const bool using_odom_state = getOdomControlState(state_to_odom_tf, state_time_sec);
  if (using_odom_state) {
    if (command_frame_ == state_frame_) {
      command_to_odom_tf = state_to_odom_tf;
    } else {
      try {
        auto command_ts = tf_->lookupTransform(
          costmap_ros_->getGlobalFrameID(), command_frame_, tf2::TimePointZero);
        tf2::fromMsg(command_ts.transform, command_to_odom_tf);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          node->get_logger(), *clock_, 500,
          "computeVelocityCommands: command_frame TF failed after odom state update '%s': %s",
          command_frame_.c_str(), ex.what());
        return geometry_msgs::msg::TwistStamped();
      }
    }
    RCLCPP_DEBUG_THROTTLE(node->get_logger(), *clock_, 1000,
      "computeVelocityCommands: using odometry state");
  }

  pose_.header = pose.header;
  pose_.header.frame_id = costmap_ros_->getGlobalFrameID();
  // Keep the controller state aligned with the TF/odometry pose used for control.
  pose_.pose.position.x = state_to_odom_tf.getOrigin().x();
  pose_.pose.position.y = state_to_odom_tf.getOrigin().y();
  pose_.pose.orientation = tf2::toMsg(state_to_odom_tf.getRotation());

  double pose_jump_speed_scale = applyPoseJumpDamping(
    state_to_odom_tf, state_tf_stamp, using_odom_state, state_time_sec);
  const double state_ms = mark_ms();

  const double r_x = state_to_odom_tf.getOrigin().x();
  const double r_y = state_to_odom_tf.getOrigin().y();

  updateTargetIndex();
  updateEffectiveReferenceSpeed(r_x, r_y, pose_jump_speed_scale);
  generateReferenceTrajectory(state_to_odom_tf);
  const double ref_ms = mark_ms();

  if (reference_waiting_for_minco_) {
    publishLocalPath(state_to_odom_tf);
    const double wait_publish_ms = mark_ms();
    const double total_ms = std::chrono::duration<double, std::milli>(
      SteadyClock::now() - t_start).count();
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *clock_, 1000,
      "MPC timing(waiting_minco): total=%.1fms interval=%.1fms tf=%.1f state=%.1f ref=%.1f publish=%.1f",
      total_ms, compute_interval_ms, tf_ms, state_ms, ref_ms, wait_publish_ms);
    const rclcpp::Time now = clock_->now();
    if (minco_unavailable_since_.nanoseconds() == 0) {
      minco_unavailable_since_ = now;
    }
    double wait_sec = 0.0;
    try {
      wait_sec = (now - minco_unavailable_since_).seconds();
    } catch (const std::runtime_error &) {
      minco_unavailable_since_ = now;
    }
    if (wait_sec >= minco_unavailable_failure_sec_) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *clock_, 500,
        "MPC no executable MINCO for %.2fs — failing FollowPath to trigger recovery",
        wait_sec);
      minco_unavailable_since_ = now;
      throw nav2_core::PlannerException("MPC waiting for executable MINCO trajectory");
    }
    geometry_msgs::msg::TwistStamped stop_cmd;
    stop_cmd.header = pose.header;
    stop_cmd.header.frame_id = command_frame_;
    return stop_cmd;
  }
  minco_unavailable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  double yaw = tf2::getYaw(command_to_odom_tf.getRotation());
  double cp = std::cos(yaw), sp = std::sin(yaw);

  const auto p_prev = buildPreviousPrediction(r_x, r_y);
  const double prev_ms = mark_ms();
  int active_count = applyObstacleAndEsdfConstraints(r_x, r_y, p_prev);
  const double obs_ms = mark_ms();
  updateMpcVelocityAnchor(r_x, r_y);
  applyHorizonSpeedLimitFloor();
  updateReferenceDirectionConstraints(r_x, r_y);
  const double limit_ms = mark_ms();
  SolveResult solve = solveMpcWithFallbacks(r_x, r_y, active_count);
  const double solve_ms = mark_ms();

  const Control u = solve.control;
  double min_limit_diag = 0.0;
  double max_limit_diag = 0.0;
  if (!horizon_speed_limits_.empty()) {
    auto minmax = std::minmax_element(horizon_speed_limits_.begin(), horizon_speed_limits_.end());
    min_limit_diag = *minmax.first;
    max_limit_diag = *minmax.second;
  }
  const double ref0_speed_diag = v_ref_.empty() ? 0.0 : std::hypot(v_ref_.front().x, v_ref_.front().y);
  const double ref0_dist_diag = ref.empty() ? 0.0 : std::hypot(ref.front().x - r_x, ref.front().y - r_y);
  if (debug_logging_) {
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *clock_, 1000,
      "MPC solve diag: success=%d status=%d u=(%.3f, %.3f) ref0_dist=%.2f ref0_v=%.2f "
      "speed_limit=[%.2f, %.2f] curv_min=%.2f time_scale=%.2f "
      "cmd_last=(%.2f, %.2f) odom_vel=(%.2f, %.2f) anchor=(%.2f, %.2f) anchor_valid=%d "
      "frames state='%s' cmd='%s' ref_s=%.2f ref_total=%.2f ref_valid=%d "
      "path_s=%.2f path_total=%.2f active_obs=%d",
      solve.success ? 1 : 0, static_cast<int>(solve.status), u.vx, u.vy,
      ref0_dist_diag, ref0_speed_diag, min_limit_diag, max_limit_diag,
      last_min_curvature_speed_limit_, last_ref_time_scale_,
      last_ux, last_uy, measured_velocity_raw_x_, measured_velocity_raw_y_,
      velocity_anchor_x_, velocity_anchor_y_, current_velocity_anchor_valid_ ? 1 : 0,
      state_frame_.c_str(), command_frame_.c_str(),
      reference_current_s_, reference_total_dist_, reference_distance_valid_ ? 1 : 0,
      current_s_, path_total_dist_, active_count);
  }

  geometry_msgs::msg::TwistStamped cmd;
  auto cmd_header = pose.header;
  cmd_header.frame_id = command_frame_;
  if (handleInvalidOrFailedSolve(solve, active_count, state_to_odom_tf, cmd_header, cmd)) {
    const double failed_ms = mark_ms();
    const double total_ms = std::chrono::duration<double, std::milli>(
      SteadyClock::now() - t_start).count();
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *clock_, 1000,
      "MPC timing(failed_solve): total=%.1fms interval=%.1fms tf=%.1f state=%.1f ref=%.1f "
      "prev=%.1f obs=%.1f limit=%.1f solve=%.1f fail=%.1f active_obs=%d",
      total_ms, compute_interval_ms, tf_ms, state_ms, ref_ms, prev_ms, obs_ms,
      limit_ms, solve_ms, failed_ms, active_count);
    return cmd;
  }

  publishLocalPath(state_to_odom_tf);
  const double publish_ms = mark_ms();

  cmd.header = cmd_header;

  if (checkPredictedCollision(r_x, r_y, cmd)) {
    const double collision_ms = mark_ms();
    const double total_ms = std::chrono::duration<double, std::milli>(
      SteadyClock::now() - t_start).count();
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *clock_, 1000,
      "MPC timing(collision_stop): total=%.1fms interval=%.1fms tf=%.1f state=%.1f ref=%.1f "
      "prev=%.1f obs=%.1f limit=%.1f solve=%.1f publish=%.1f collision=%.1f active_obs=%d",
      total_ms, compute_interval_ms, tf_ms, state_ms, ref_ms, prev_ms, obs_ms,
      limit_ms, solve_ms, publish_ms, collision_ms, active_count);
    return cmd;
  }

  // Convert MPC output from odom/global frame to the configured command frame.
  cmd.twist.linear.x =  cp * u.vx + sp * u.vy;
  cmd.twist.linear.y = -sp * u.vx + cp * u.vy;
  enforceNoReverseTrackingCommand(cmd, r_x, r_y, cp, sp);
  applyGoalStopProtection(cmd, r_x, r_y);
  finalizeCommand(cmd, cp, sp);
  const double final_ms = mark_ms();
  const double total_ms = std::chrono::duration<double, std::milli>(
    SteadyClock::now() - t_start).count();
  const double expected_ms = 1000.0 / std::max(control_frequency_, 1.0);
  if (total_ms > expected_ms * 0.8) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *clock_, 1000,
      "MPC timing(ok): total=%.1fms interval=%.1fms expected=%.1fms tf=%.1f state=%.1f ref=%.1f "
      "prev=%.1f obs=%.1f limit=%.1f solve=%.1f publish=%.1f final=%.1f active_obs=%d",
      total_ms, compute_interval_ms, expected_ms, tf_ms, state_ms, ref_ms, prev_ms,
      obs_ms, limit_ms, solve_ms, publish_ms, final_ms, active_count);
  }

  return cmd;
}

}  // namespace f_mpc_controller

PLUGINLIB_EXPORT_CLASS(
  f_mpc_controller::MpcController,
  nav2_core::Controller)
