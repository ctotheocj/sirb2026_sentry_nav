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

#include "pb_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "gcopter/minco.hpp"
#include "gcopter/trajectory.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;
using namespace nav2_costmap_2d;  // NOLINT
using rcl_interfaces::msg::ParameterType;

namespace pb_omni_pid_pursuit_controller
{

void OmniPidPursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  double transform_tolerance = 1.0;
  double control_frequency = 20.0;
  max_robot_pose_search_dist_ = getCostmapMaxExtent();

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_kp", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_ki", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_kd", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_rotation", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_kp", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_ki", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_kd", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_max_sum_error", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_velocity_scaled_lookahead_dist", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_interpolation", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading_treshold", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_velocity_scaling_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_linear_min", rclcpp::ParameterValue(-3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_linear_max", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_angular_min", rclcpp::ParameterValue(-3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_angular_max", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_robot_pose_search_dist",
    rclcpp::ParameterValue(getCostmapMaxExtent()));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_min", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_max", rclcpp::ParameterValue(0.7));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".reduction_ratio_at_high_curvature", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_forward_dist", rclcpp::ParameterValue(0.7));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_backward_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_velocity_scaling_factor_rate", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_minco_tracking_path", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".minco_traj_topic",
    rclcpp::ParameterValue("trajectory_manager/trajectory_for_mpc"));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".minco_tracking_timeout", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".minco_tracking_sample_dt", rclcpp::ParameterValue(0.08));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".minco_tracking_min_duration", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".minco_tracking_max_duration", rclcpp::ParameterValue(4.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".minco_projection_search_ahead_sec", rclcpp::ParameterValue(0.30));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".minco_projection_max_advance_sec", rclcpp::ParameterValue(0.12));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".minco_projection_max_lag_sec", rclcpp::ParameterValue(0.80));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".skip_collision_check_in_hole_pass", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".navigation_mode_topic",
    rclcpp::ParameterValue("navigation_mode_manager/mode"));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".hole_pass_mode_name", rclcpp::ParameterValue("hole_pass"));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".navigation_mode_timeout", rclcpp::ParameterValue(0.5));

  node->get_parameter(plugin_name_ + ".translation_kp", translation_kp_);
  node->get_parameter(plugin_name_ + ".translation_ki", translation_ki_);
  node->get_parameter(plugin_name_ + ".translation_kd", translation_kd_);
  node->get_parameter(plugin_name_ + ".enable_rotation", enable_rotation_);
  node->get_parameter(plugin_name_ + ".rotation_kp", rotation_kp_);
  node->get_parameter(plugin_name_ + ".rotation_ki", rotation_ki_);
  node->get_parameter(plugin_name_ + ".rotation_kd", rotation_kd_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(plugin_name_ + ".min_max_sum_error", min_max_sum_error_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(
    plugin_name_ + ".use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(plugin_name_ + ".use_interpolation", use_interpolation_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
  node->get_parameter(
    plugin_name_ + ".use_rotate_to_heading_treshold", use_rotate_to_heading_treshold_);
  node->get_parameter(
    plugin_name_ + ".min_approach_linear_velocity", min_approach_linear_velocity_);
  node->get_parameter(
    plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
  if (approach_velocity_scaling_dist_ > costmap_->getSizeInMetersX() / 2.0) {
    RCLCPP_WARN(
      logger_,
      "approach_velocity_scaling_dist is larger than forward costmap extent, "
      "leading to permanent slowdown");
  }
  node->get_parameter(plugin_name_ + ".v_linear_max", v_linear_max_);
  node->get_parameter(plugin_name_ + ".v_linear_min", v_linear_min_);
  node->get_parameter(plugin_name_ + ".v_angular_max", v_angular_max_);
  node->get_parameter(plugin_name_ + ".v_angular_min", v_angular_min_);
  node->get_parameter(plugin_name_ + ".max_robot_pose_search_dist", max_robot_pose_search_dist_);
  node->get_parameter(plugin_name_ + ".curvature_min", curvature_min_);
  node->get_parameter(plugin_name_ + ".curvature_max", curvature_max_);
  node->get_parameter(
    plugin_name_ + ".reduction_ratio_at_high_curvature", reduction_ratio_at_high_curvature_);
  node->get_parameter(plugin_name_ + ".curvature_forward_dist", curvature_forward_dist_);
  node->get_parameter(plugin_name_ + ".curvature_backward_dist", curvature_backward_dist_);
  node->get_parameter(
    plugin_name_ + ".max_velocity_scaling_factor_rate", max_velocity_scaling_factor_rate_);
  node->get_parameter(plugin_name_ + ".use_minco_tracking_path", use_minco_tracking_path_);
  node->get_parameter(plugin_name_ + ".minco_traj_topic", minco_traj_topic_);
  node->get_parameter(plugin_name_ + ".minco_tracking_timeout", minco_tracking_timeout_);
  node->get_parameter(plugin_name_ + ".minco_tracking_sample_dt", minco_tracking_sample_dt_);
  node->get_parameter(plugin_name_ + ".minco_tracking_min_duration", minco_tracking_min_duration_);
  node->get_parameter(plugin_name_ + ".minco_tracking_max_duration", minco_tracking_max_duration_);
  node->get_parameter(
    plugin_name_ + ".minco_projection_search_ahead_sec", minco_projection_search_ahead_sec_);
  node->get_parameter(
    plugin_name_ + ".minco_projection_max_advance_sec", minco_projection_max_advance_sec_);
  node->get_parameter(
    plugin_name_ + ".minco_projection_max_lag_sec", minco_projection_max_lag_sec_);
  node->get_parameter(
    plugin_name_ + ".skip_collision_check_in_hole_pass", skip_collision_check_in_hole_pass_);
  node->get_parameter(plugin_name_ + ".navigation_mode_topic", navigation_mode_topic_);
  node->get_parameter(plugin_name_ + ".hole_pass_mode_name", hole_pass_mode_name_);
  node->get_parameter(plugin_name_ + ".navigation_mode_timeout", navigation_mode_timeout_);

  node->get_parameter("controller_frequency", control_frequency);

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  control_duration_ = 1.0 / control_frequency;
  last_velocity_scaling_factor_ = v_linear_max_;
  minco_tracking_timeout_ = std::max(0.05, minco_tracking_timeout_);
  minco_tracking_sample_dt_ = std::clamp(minco_tracking_sample_dt_, 0.02, 0.50);
  minco_tracking_min_duration_ = std::max(0.05, minco_tracking_min_duration_);
  minco_tracking_max_duration_ =
    std::max(minco_tracking_min_duration_, minco_tracking_max_duration_);
  minco_projection_search_ahead_sec_ = std::clamp(minco_projection_search_ahead_sec_, 0.02, 2.0);
  minco_projection_max_advance_sec_ = std::clamp(minco_projection_max_advance_sec_, 0.0, 1.0);
  minco_projection_max_lag_sec_ = std::clamp(minco_projection_max_lag_sec_, 0.0, 5.0);
  navigation_mode_timeout_ = std::max(0.05, navigation_mode_timeout_);

  local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
  curvature_points_pub_ =
    node_.lock()
      ->create_publisher<visualization_msgs::msg::MarkerArray>(  // 初始化 MarkerArray Publisher
        "curvature_points_marker_array", rclcpp::QoS(10));
  minco_traj_sub_ = node->create_subscription<MincoTrajectoryMsg>(
    minco_traj_topic_, rclcpp::QoS(1),
    std::bind(&OmniPidPursuitController::mincoTrajectoryCallback, this, std::placeholders::_1));
  navigation_mode_sub_ = node->create_subscription<std_msgs::msg::String>(
    navigation_mode_topic_, rclcpp::QoS(10),
    std::bind(&OmniPidPursuitController::navigationModeCallback, this, std::placeholders::_1));

  move_pid_ = std::make_shared<PID>(
    control_duration_, v_linear_max_, v_linear_min_, translation_kp_, translation_kd_,
    translation_ki_);
  heading_pid_ = std::make_shared<PID>(
    control_duration_, v_angular_max_, v_angular_min_, rotation_kp_, rotation_kd_, rotation_ki_);
}

void OmniPidPursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " pb_omni_pid_pursuit_controller::OmniPidPursuitController",
    plugin_name_.c_str());
  local_path_pub_.reset();
  carrot_pub_.reset();
  curvature_points_pub_.reset();
  minco_traj_sub_.reset();
  navigation_mode_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(minco_mutex_);
    has_latest_minco_traj_ = false;
    latest_minco_trajectory_id_ = 0;
    last_projected_minco_trajectory_id_ = 0;
    last_minco_projected_time_ = 0.0;
  }
  {
    std::lock_guard<std::mutex> lock(navigation_mode_mutex_);
    latest_navigation_mode_ = "normal";
    latest_navigation_mode_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  }
}

void OmniPidPursuitController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "regulated_pure_pursuit_controller::OmniPidPursuitController",
    plugin_name_.c_str());
  local_path_pub_->on_activate();
  carrot_pub_->on_activate();
  curvature_points_pub_->on_activate();
  // Add callback for dynamic parameters
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&OmniPidPursuitController::dynamicParametersCallback, this, std::placeholders::_1));
}

void OmniPidPursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "regulated_pure_pursuit_controller::OmniPidPursuitController",
    plugin_name_.c_str());
  local_path_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
  curvature_points_pub_->on_deactivate();
  dyn_params_handler_.reset();
}

geometry_msgs::msg::TwistStamped OmniPidPursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  bool using_minco_tracking_path = false;
  bool minco_terminal_stop = true;
  global_plan_ = getControllerPlan(pose, using_minco_tracking_path, minco_terminal_stop);

  // Transform path to robot base frame
  auto transformed_plan = transformGlobalPlan(pose);

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(velocity);

  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  double lin_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  double theta_dist = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  double angle_to_goal = tf2::getYaw(carrot_pose.pose.orientation);

  if (use_rotate_to_heading_) {
    angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    if (fabs(angle_to_goal) > use_rotate_to_heading_treshold_) {
      lin_dist = 0;
    }
  }

  auto lin_vel = move_pid_->calculate(lin_dist, 0);
  auto angular_vel = enable_rotation_ ? heading_pid_->calculate(angle_to_goal, 0) : 0.0;

  applyCurvatureLimitation(transformed_plan, carrot_pose, lin_vel);

  if (!using_minco_tracking_path || minco_terminal_stop) {
    applyApproachVelocityScaling(transformed_plan, lin_vel);
  }

  // Transform local frame to global frame to use in collision checking
  nav_msgs::msg::Path costmap_frame_local_plan;

  int sample_points = 10;
  int plan_size = transformed_plan.poses.size();
  for (int i = 0; i < sample_points; ++i) {
    int index = std::min((i * plan_size) / sample_points, plan_size - 1);
    geometry_msgs::msg::PoseStamped map_pose;
    transformPose(costmap_ros_->getGlobalFrameID(), transformed_plan.poses[index], map_pose);
    costmap_frame_local_plan.poses.push_back(map_pose);
  }

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  const bool skip_collision_check = shouldSkipCollisionCheck();
  if (skip_collision_check || !isCollisionDetected(costmap_frame_local_plan)) {
    if (skip_collision_check) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "PID collision check skipped while navigation mode is '%s'",
        hole_pass_mode_name_.c_str());
    }
    cmd_vel.twist.linear.x = lin_vel * cos(theta_dist);
    cmd_vel.twist.linear.y = lin_vel * sin(theta_dist);
    cmd_vel.twist.angular.z = angular_vel;
  } else {
    throw nav2_core::PlannerException("Collision detected in the trajectory. Stopping the robot!");
  }

  return cmd_vel;
}

void OmniPidPursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  bt_global_plan_ = path;
  global_plan_ = path;
}

void OmniPidPursuitController::setSpeedLimit(
  const double & /*speed_limit*/, const bool & /*percentage*/)
{
  RCLCPP_WARN(logger_, "Speed limit is not implemented in this controller.");
}

void OmniPidPursuitController::mincoTrajectoryCallback(const MincoTrajectoryMsg::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  Trajectory<5> traj;
  if (!buildTrajectoryFromMsg(*msg, traj)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "PID MINCO tracking rejected invalid trajectory: id=%lu waypoints=%zu segments=%zu",
      static_cast<unsigned long>(msg->trajectory_id), msg->waypoints.size(),
      msg->segment_times.size());
    return;
  }

  std::lock_guard<std::mutex> lock(minco_mutex_);
  const bool new_trajectory =
    msg->trajectory_id != 0 && msg->trajectory_id != latest_minco_trajectory_id_;
  const rclcpp::Time receive_time = clock_->now();
  latest_minco_msg_ = *msg;
  latest_minco_traj_ = std::make_shared<Trajectory<5>>(traj);
  latest_minco_receive_time_ = receive_time;
  if (new_trajectory || latest_minco_first_receive_time_.nanoseconds() == 0) {
    latest_minco_first_receive_time_ = receive_time;
  }
  if (new_trajectory || latest_minco_trajectory_id_ == 0) {
    try {
      const rclcpp::Time msg_stamp(msg->header.stamp);
      const rclcpp::Time start_time(msg->start_time);
      latest_minco_time_base_offset_sec_ =
        start_time.nanoseconds() == 0 ? 0.0 : (msg_stamp - start_time).seconds();
    } catch (const std::runtime_error &) {
      latest_minco_time_base_offset_sec_ = 0.0;
    }
  }
  latest_minco_trajectory_id_ = msg->trajectory_id;
  has_latest_minco_traj_ = true;
}

void OmniPidPursuitController::navigationModeCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(navigation_mode_mutex_);
  latest_navigation_mode_ = msg->data;
  latest_navigation_mode_time_ = clock_->now();
}

bool OmniPidPursuitController::shouldSkipCollisionCheck() const
{
  if (!skip_collision_check_in_hole_pass_) {
    return false;
  }

  std::string mode;
  rclcpp::Time mode_time{0, 0, clock_->get_clock_type()};
  {
    std::lock_guard<std::mutex> lock(navigation_mode_mutex_);
    mode = latest_navigation_mode_;
    mode_time = latest_navigation_mode_time_;
  }

  if (mode != hole_pass_mode_name_ || mode_time.nanoseconds() == 0) {
    return false;
  }

  double age = std::numeric_limits<double>::infinity();
  try {
    age = (clock_->now() - mode_time).seconds();
  } catch (const std::runtime_error &) {
    return false;
  }

  if (!std::isfinite(age) || age > navigation_mode_timeout_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "PID collision check using normal mode because navigation mode status is stale: age %.3fs > %.3fs",
      age, navigation_mode_timeout_);
    return false;
  }

  return true;
}

bool OmniPidPursuitController::buildTrajectoryFromMsg(
  const MincoTrajectoryMsg & msg, Trajectory<5> & traj) const
{
  const size_t n = msg.waypoints.size();
  if (n < 2 || msg.segment_times.size() + 1 != n) {
    return false;
  }

  std::vector<Eigen::Vector3d> pts;
  pts.reserve(n);
  for (const auto & p : msg.waypoints) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      return false;
    }
    pts.emplace_back(p.x, p.y, p.z);
  }

  Eigen::VectorXd times(static_cast<Eigen::Index>(n - 1));
  for (size_t i = 0; i + 1 < n; ++i) {
    const double dt = msg.segment_times[i];
    if (!std::isfinite(dt) || dt <= 1.0e-4) {
      return false;
    }
    times(static_cast<Eigen::Index>(i)) = dt;
  }

  const int pieces = static_cast<int>(n) - 1;
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

  Eigen::Matrix3Xd inner(3, std::max(0, pieces - 1));
  for (int i = 1; i < pieces; ++i) {
    inner.col(i - 1) = pts[static_cast<size_t>(i)];
  }

  minco::MINCO_S3NU solver;
  solver.setConditions(head, tail, pieces);
  solver.setParameters(inner, times);
  solver.getTrajectory(traj);
  return traj.getTotalDuration() > 1.0e-4;
}

nav_msgs::msg::Path OmniPidPursuitController::sampleMincoTrackingPath(
  const MincoTrajectoryMsg & msg, const Trajectory<5> & traj, double start_time) const
{
  nav_msgs::msg::Path path;
  path.header = msg.header;
  path.header.stamp = clock_->now();

  const double duration = traj.getTotalDuration();
  if (!std::isfinite(duration) || duration <= 1.0e-4) {
    return path;
  }

  if (!std::isfinite(start_time)) {
    start_time = 0.0;
  }
  start_time = std::clamp(start_time, 0.0, duration);
  const double remaining = duration - start_time;
  if (remaining <= 1.0e-3) {
    return path;
  }

  const double desired_duration = std::clamp(
    remaining, minco_tracking_min_duration_, minco_tracking_max_duration_);
  const double sample_duration = std::min(remaining, desired_duration);
  const double end_time = std::min(duration, start_time + sample_duration);
  const double sample_dt = std::clamp(minco_tracking_sample_dt_, 0.02, 0.50);

  auto append_pose = [&](double t) {
      const Eigen::Vector3d p = traj.getPos(std::clamp(t, 0.0, duration));
      if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) {
        return;
      }
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.position.z = p.z();
      pose.pose.orientation.w = 1.0;
      if (!path.poses.empty()) {
        const auto & last = path.poses.back().pose.position;
        if (std::hypot(p.x() - last.x, p.y() - last.y) < 0.02) {
          return;
        }
      }
      path.poses.push_back(pose);
    };

  for (double t = start_time; t < end_time; t += sample_dt) {
    append_pose(t);
  }
  append_pose(end_time);

  if (path.poses.size() < 2 && end_time < duration) {
    append_pose(std::min(duration, end_time + sample_dt));
  }
  updatePathOrientations(path);
  return path;
}

nav_msgs::msg::Path OmniPidPursuitController::getControllerPlan(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  bool & using_minco, bool & minco_terminal_stop)
{
  using_minco = false;
  minco_terminal_stop = true;

  if (!use_minco_tracking_path_) {
    return global_plan_;
  }

  MincoTrajectoryMsg msg;
  std::shared_ptr<Trajectory<5>> traj;
  rclcpp::Time receive_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time first_receive_time{0, 0, RCL_ROS_TIME};
  double time_base_offset = 0.0;
  {
    std::lock_guard<std::mutex> lock(minco_mutex_);
    if (!has_latest_minco_traj_ || !latest_minco_traj_) {
      return bt_global_plan_.poses.empty() ? global_plan_ : bt_global_plan_;
    }
    msg = latest_minco_msg_;
    traj = latest_minco_traj_;
    receive_time = latest_minco_receive_time_;
    first_receive_time = latest_minco_first_receive_time_;
    time_base_offset = latest_minco_time_base_offset_sec_;
  }

  const rclcpp::Time now = clock_->now();
  double age = std::numeric_limits<double>::infinity();
  try {
    age = (now - receive_time).seconds();
  } catch (const std::runtime_error &) {
    age = std::numeric_limits<double>::infinity();
  }
  if (!std::isfinite(age) || age > minco_tracking_timeout_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "PID MINCO tracking fallback to BT path: trajectory age %.3fs > %.3fs",
      age, minco_tracking_timeout_);
    return bt_global_plan_.poses.empty() ? global_plan_ : bt_global_plan_;
  }

  double msg_age = 0.0;
  double trajectory_time = 0.0;
  try {
    msg_age = (now - first_receive_time).seconds();
    trajectory_time = msg_age + time_base_offset;
  } catch (const std::runtime_error &) {
    trajectory_time = 0.0;
  }
  trajectory_time = std::clamp(trajectory_time, 0.0, traj->getTotalDuration());

  double projected_time = trajectory_time;
  if (!projectRobotOntoMincoPath(msg, *traj, robot_pose, trajectory_time, projected_time)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "PID MINCO tracking fallback to BT path: robot projection failed");
    return bt_global_plan_.poses.empty() ? global_plan_ : bt_global_plan_;
  }

  auto minco_path = sampleMincoTrackingPath(msg, *traj, projected_time);
  if (minco_path.poses.size() >= 2) {
    using_minco = true;
    minco_terminal_stop = msg.terminal_stop;
    active_controller_plan_ = minco_path;
    return minco_path;
  }

  RCLCPP_WARN_THROTTLE(
    logger_, *clock_, 1000,
    "PID MINCO tracking fallback to BT path: sampled path has %zu pose(s)",
    minco_path.poses.size());
  return bt_global_plan_.poses.empty() ? global_plan_ : bt_global_plan_;
}

bool OmniPidPursuitController::transformPoseLatest(
  const std::string & frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    const auto transform = tf_->lookupTransform(frame, in_pose.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(in_pose, out_pose, transform);
    out_pose.header.stamp = clock_->now();
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "PID MINCO tracking latest TF failed: %s", ex.what());
  }
  return false;
}

bool OmniPidPursuitController::projectRobotOntoMincoPath(
  const MincoTrajectoryMsg & msg,
  const Trajectory<5> & traj,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double time_seed,
  double & projected_time)
{
  const double duration = traj.getTotalDuration();
  if (!std::isfinite(duration) || duration <= 1.0e-4) {
    return false;
  }

  geometry_msgs::msg::PoseStamped robot_in_traj_frame;
  if (!transformPoseLatest(msg.header.frame_id, robot_pose, robot_in_traj_frame)) {
    return false;
  }

  time_seed = std::clamp(time_seed, 0.0, duration);
  const rclcpp::Time now = clock_->now();
  const bool new_traj =
    msg.trajectory_id != 0 && msg.trajectory_id != last_projected_minco_trajectory_id_;
  if (new_traj) {
    last_projected_minco_trajectory_id_ = msg.trajectory_id;
    last_minco_projected_time_ = 0.0;
    last_minco_projection_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  double elapsed = control_duration_;
  if (!new_traj && last_minco_projection_update_time_.nanoseconds() != 0) {
    try {
      elapsed = std::clamp((now - last_minco_projection_update_time_).seconds(), 0.0, 0.5);
    } catch (const std::runtime_error &) {
      elapsed = control_duration_;
    }
  }

  const double max_advance = std::max(
    minco_projection_max_advance_sec_, elapsed + 2.0 * control_duration_);
  const double min_time_from_exec =
    std::clamp(time_seed - std::max(0.0, minco_projection_max_lag_sec_), 0.0, duration);
  const double seed_center = std::clamp(
    std::max(last_minco_projected_time_, min_time_from_exec), 0.0, duration);
  const double search_lo = std::clamp(
    std::max(
      seed_center - (new_traj ? control_duration_ : 0.0),
      min_time_from_exec),
    0.0, duration);
  const double search_hi = std::min(
    duration,
    std::max(
      time_seed + minco_projection_search_ahead_sec_,
      seed_center + std::max(minco_projection_search_ahead_sec_, max_advance)));
  if (search_hi < search_lo || !std::isfinite(search_lo) || !std::isfinite(search_hi)) {
    return false;
  }

  const double rx = robot_in_traj_frame.pose.position.x;
  const double ry = robot_in_traj_frame.pose.position.y;
  auto dist2_at = [&](double t) {
      const Eigen::Vector3d p = traj.getPos(std::clamp(t, 0.0, duration));
      const double dx = p.x() - rx;
      const double dy = p.y() - ry;
      return dx * dx + dy * dy;
    };

  constexpr int coarse_samples = 10;
  double best_dist2 = std::numeric_limits<double>::max();
  double best_t = search_lo;
  for (int s = 0; s <= coarse_samples; ++s) {
    const double ratio = static_cast<double>(s) / static_cast<double>(coarse_samples);
    const double t = search_lo + ratio * (search_hi - search_lo);
    const double d2 = dist2_at(t);
    if (d2 < best_dist2) {
      best_dist2 = d2;
      best_t = t;
    }
  }

  double lo = std::max(search_lo, best_t - 0.1);
  double hi = std::min(search_hi, best_t + 0.1);
  for (int i = 0; i < 20; ++i) {
    const double m1 = lo + (hi - lo) * 0.382;
    const double m2 = lo + (hi - lo) * 0.618;
    if (dist2_at(m1) < dist2_at(m2)) {
      hi = m2;
    } else {
      lo = m1;
    }
  }

  const double t_proj = std::clamp((lo + hi) * 0.5, search_lo, search_hi);
  const double lower_bound = new_traj ?
    search_lo : std::max(last_minco_projected_time_, min_time_from_exec);
  const double upper_bound = new_traj ?
    search_hi : std::min(duration, std::max(time_seed, last_minco_projected_time_ + max_advance));
  last_minco_projected_time_ = std::clamp(t_proj, lower_bound, upper_bound);
  last_minco_projection_update_time_ = now;
  projected_time = last_minco_projected_time_;

  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 1000,
    "PID MINCO tracking projection id=%lu t=%.2f seed=%.2f dist=%.2f search=[%.2f, %.2f]",
    static_cast<unsigned long>(msg.trajectory_id), projected_time, time_seed, std::sqrt(best_dist2),
    search_lo, search_hi);
  return true;
}

void OmniPidPursuitController::updatePathOrientations(nav_msgs::msg::Path & path) const
{
  if (path.poses.size() < 2) {
    return;
  }
  for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const auto & p0 = path.poses[i].pose.position;
    const auto & p1 = path.poses[i + 1].pose.position;
    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;
    if (std::hypot(dx, dy) < 1.0e-6) {
      continue;
    }
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, std::atan2(dy, dx));
    path.poses[i].pose.orientation = tf2::toMsg(q);
  }
  path.poses.back().pose.orientation = path.poses[path.poses.size() - 2].pose.orientation;
}

nav_msgs::msg::Path OmniPidPursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  double max_costmap_extent = getCostmapMaxExtent();

  auto closest_pose_upper_bound = nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto transformation_begin = nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // Find points up to max_transform_dist so we only transform them.
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) { return euclidean_distance(pose, robot_pose) > max_costmap_extent; });

  // Lambda to transform a PoseStamped from global frame to local
  auto transform_global_pose_to_local = [&](const auto & global_plan_pose) {
    geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
    stamped_pose.header.frame_id = global_plan_.header.frame_id;
    stamped_pose.header.stamp = robot_pose.header.stamp;
    stamped_pose.pose = global_plan_pose.pose;
    transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
    transformed_pose.pose.position.z = 0.0;
    return transformed_pose;
  };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end, std::back_inserter(transformed_plan.poses),
    transform_global_pose_to_local);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  local_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

std::unique_ptr<geometry_msgs::msg::PointStamped> OmniPidPursuitController::createCarrotMsg(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header = carrot_pose.header;
  carrot_msg->point.x = carrot_pose.pose.position.x;
  carrot_msg->point.y = carrot_pose.pose.position.y;
  carrot_msg->point.z = 0.01;  // publish right over map to stand out
  return carrot_msg;
}

geometry_msgs::msg::PoseStamped OmniPidPursuitController::getLookAheadPoint(
  const double & lookahead_dist, const nav_msgs::msg::Path & transformed_plan)
{
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  } else if (use_interpolation_ && goal_pose_it != transformed_plan.poses.begin()) {
    // Find the point on the line segment between the two poses
    // that is exactly the lookahead distance away from the robot pose (the origin)
    // This can be found with a closed form for the intersection of a segment and a circle
    // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
    // and goal_pose is guaranteed to be outside the circle.
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = circleSegmentIntersection(
      prev_pose_it->pose.position, goal_pose_it->pose.position, lookahead_dist);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    return pose;
  }

  return *goal_pose_it;
}

geometry_msgs::msg::Point OmniPidPursuitController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r)
{
  // Formula for intersection of a line with a circle centered at the origin,
  // modified to always return the point that is on the segment between the two points.
  // https://mathworld.wolfram.com/Circle-LineIntersection.html
  // This works because the poses are transformed into the robot frame.
  // This can be derived from solving the system of equations of a line and a circle
  // which results in something that is just a reformulation of the quadratic formula.
  // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
  // https://www.desmos.com/calculator/td5cwbuocd
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double d = x1 * y2 - x2 * y1;

  // Augmentation to only return point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  geometry_msgs::msg::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - d * d);
  p.x = (d * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-d * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

double OmniPidPursuitController::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters =
    std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}
bool OmniPidPursuitController::transformPose(
  const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

bool OmniPidPursuitController::isCollisionDetected(const nav_msgs::msg::Path & path)
{
  auto costmap = costmap_ros_->getCostmap();
  for (const auto & pose_stamped : path.poses) {
    const auto & pose = pose_stamped.pose;
    unsigned int mx, my;
    if (costmap->worldToMap(pose.position.x, pose.position.y, mx, my)) {
      if (costmap->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return true;
      }
    } else {
      // RCLCPP_WARN(
      //   logger_,
      //   "The Local path is not in the costmap. Cannot check for collisions. "
      //   "Proceed at your own risk, slow the robot, or increase your costmap size.");
      return false;
    }
  }
  return false;
}

double OmniPidPursuitController::getLookAheadDistance(const geometry_msgs::msg::Twist & speed)
{
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance
  double lookahead_dist = lookahead_dist_;

  if (use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = hypot(speed.linear.x, speed.linear.y) * lookahead_time_;
    lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  }

  return lookahead_dist;
}

double OmniPidPursuitController::approachVelocityScalingFactor(
  const nav_msgs::msg::Path & transformed_path) const
{
  // Waiting to apply the threshold based on integrated distance ensures we don't
  // erroneously apply approach scaling on curvy paths that are contained in a large local costmap.
  double remaining_distance = nav2_util::geometry_utils::calculate_path_length(transformed_path);
  if (remaining_distance < approach_velocity_scaling_dist_) {
    auto & last = transformed_path.poses.back();
    // Here we will use a regular euclidean distance from the robot frame (origin)
    // to get smooth scaling, regardless of path density.
    double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
    return distance_to_last_pose / approach_velocity_scaling_dist_;
  } else {
    return 1.0;
  }
}

void OmniPidPursuitController::applyApproachVelocityScaling(
  const nav_msgs::msg::Path & path, double & linear_vel) const
{
  double approach_vel = linear_vel;
  double velocity_scaling = approachVelocityScalingFactor(path);
  double unbounded_vel = approach_vel * velocity_scaling;
  if (unbounded_vel < min_approach_linear_velocity_) {
    approach_vel = min_approach_linear_velocity_;
  } else {
    approach_vel *= velocity_scaling;
  }

  // Use the lowest velocity between approach and other constraints, if all overlapping
  linear_vel = std::min(linear_vel, approach_vel);
}

void OmniPidPursuitController::applyCurvatureLimitation(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
  double & linear_vel)
{
  double curvature =
    calculateCurvature(path, lookahead_pose, curvature_forward_dist_, curvature_backward_dist_);

  double scaled_linear_vel = linear_vel;
  if (curvature > curvature_min_) {
    double reduction_ratio = 1.0;
    if (curvature > curvature_max_) {
      reduction_ratio = reduction_ratio_at_high_curvature_;
    } else {
      reduction_ratio = 1.0 - (curvature - curvature_min_) / (curvature_max_ - curvature_min_) *
                                (1.0 - reduction_ratio_at_high_curvature_);
    }

    double target_scaled_vel = linear_vel * reduction_ratio;
    scaled_linear_vel =
      last_velocity_scaling_factor_ + std::clamp(
                                        target_scaled_vel - last_velocity_scaling_factor_,
                                        -max_velocity_scaling_factor_rate_ * control_duration_,
                                        max_velocity_scaling_factor_rate_ * control_duration_);
  }
  scaled_linear_vel = std::max(scaled_linear_vel, 2.0 * min_approach_linear_velocity_);

  linear_vel = std::min(linear_vel, scaled_linear_vel);
  last_velocity_scaling_factor_ = linear_vel;
}

double OmniPidPursuitController::calculateCurvature(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
  double forward_dist, double backward_dist) const
{
  geometry_msgs::msg::PoseStamped backward_pose, forward_pose;
  std::vector<double> cumulative_distances = calculateCumulativeDistances(path);

  double lookahead_pose_cumulative_distance = 0.0;
  geometry_msgs::msg::PoseStamped robot_base_frame_pose;
  robot_base_frame_pose.pose = geometry_msgs::msg::Pose();
  lookahead_pose_cumulative_distance =
    nav2_util::geometry_utils::euclidean_distance(robot_base_frame_pose, lookahead_pose);

  backward_pose = findPoseAtDistance(
    path, cumulative_distances, lookahead_pose_cumulative_distance - backward_dist);

  forward_pose = findPoseAtDistance(
    path, cumulative_distances, lookahead_pose_cumulative_distance + forward_dist);

  double curvature_radius = calculateCurvatureRadius(
    backward_pose.pose.position, lookahead_pose.pose.position, forward_pose.pose.position);
  double curvature = 1.0 / curvature_radius;
  visualizeCurvaturePoints(backward_pose, forward_pose);
  return curvature;
}

double OmniPidPursuitController::calculateCurvatureRadius(
  const geometry_msgs::msg::Point & near_point, const geometry_msgs::msg::Point & current_point,
  const geometry_msgs::msg::Point & far_point) const
{
  double x1 = near_point.x, y1 = near_point.y;
  double x2 = current_point.x, y2 = current_point.y;
  double x3 = far_point.x, y3 = far_point.y;

  double center_x = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) +
                     (x3 * x3 + y3 * y3) * (y1 - y2)) /
                    (2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)));
  double center_y = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) +
                     (x3 * x3 + y3 * y3) * (x2 - x1)) /
                    (2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)));
  double radius = std::hypot(x2 - center_x, y2 - center_y);
  if (std::isnan(radius) || std::isinf(radius) || radius < 1e-9) {
    return 1e9;
  }
  return radius;
}

void OmniPidPursuitController::visualizeCurvaturePoints(
  const geometry_msgs::msg::PoseStamped & backward_pose,
  const geometry_msgs::msg::PoseStamped & forward_pose) const
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker near_marker;
  near_marker.header = backward_pose.header;
  near_marker.ns = "curvature_points";
  near_marker.id = 0;
  near_marker.type = visualization_msgs::msg::Marker::SPHERE;
  near_marker.action = visualization_msgs::msg::Marker::ADD;
  near_marker.pose = backward_pose.pose;
  near_marker.scale.x = near_marker.scale.y = near_marker.scale.z = 0.1;
  near_marker.color.g = 1.0;
  near_marker.color.a = 1.0;

  visualization_msgs::msg::Marker far_marker;
  far_marker.header = forward_pose.header;
  far_marker.ns = "curvature_points";
  far_marker.id = 1;
  far_marker.type = visualization_msgs::msg::Marker::SPHERE;
  far_marker.action = visualization_msgs::msg::Marker::ADD;
  far_marker.pose = forward_pose.pose;
  far_marker.scale.x = far_marker.scale.y = far_marker.scale.z = 0.1;
  far_marker.color.r = 1.0;
  far_marker.color.a = 1.0;

  marker_array.markers.push_back(near_marker);
  marker_array.markers.push_back(far_marker);

  curvature_points_pub_->publish(marker_array);
}

std::vector<double> OmniPidPursuitController::calculateCumulativeDistances(
  const nav_msgs::msg::Path & path) const
{
  std::vector<double> cumulative_distances;
  cumulative_distances.push_back(0.0);

  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & prev_pose = path.poses[i - 1].pose.position;
    const auto & curr_pose = path.poses[i].pose.position;
    double distance = hypot(curr_pose.x - prev_pose.x, curr_pose.y - prev_pose.y);
    cumulative_distances.push_back(cumulative_distances.back() + distance);
  }
  return cumulative_distances;
}

geometry_msgs::msg::PoseStamped OmniPidPursuitController::findPoseAtDistance(
  const nav_msgs::msg::Path & path, const std::vector<double> & cumulative_distances,
  double target_distance) const
{
  if (path.poses.empty() || cumulative_distances.empty()) {
    return geometry_msgs::msg::PoseStamped();
  }
  if (target_distance <= 0.0) {
    return path.poses.front();
  }
  if (target_distance >= cumulative_distances.back()) {
    return path.poses.back();
  }
  auto it =
    std::lower_bound(cumulative_distances.begin(), cumulative_distances.end(), target_distance);
  size_t index = std::distance(cumulative_distances.begin(), it);

  if (index == 0) {
    return path.poses.front();
  }

  double ratio = (target_distance - cumulative_distances[index - 1]) /
                 (cumulative_distances[index] - cumulative_distances[index - 1]);
  geometry_msgs::msg::PoseStamped pose1 = path.poses[index - 1];
  geometry_msgs::msg::PoseStamped pose2 = path.poses[index];

  geometry_msgs::msg::PoseStamped interpolated_pose;
  interpolated_pose.header = pose2.header;
  interpolated_pose.pose.position.x =
    pose1.pose.position.x + ratio * (pose2.pose.position.x - pose1.pose.position.x);
  interpolated_pose.pose.position.y =
    pose1.pose.position.y + ratio * (pose2.pose.position.y - pose1.pose.position.y);
  interpolated_pose.pose.position.z =
    pose1.pose.position.z + ratio * (pose2.pose.position.z - pose1.pose.position.z);
  interpolated_pose.pose.orientation = pose2.pose.orientation;

  return interpolated_pose;
}

rcl_interfaces::msg::SetParametersResult OmniPidPursuitController::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (const auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".translation_kp") {
        translation_kp_ = parameter.as_double();
      } else if (name == plugin_name_ + ".translation_ki") {
        translation_ki_ = parameter.as_double();
      } else if (name == plugin_name_ + ".translation_kd") {
        translation_kd_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_kp") {
        rotation_kp_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_ki") {
        rotation_ki_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_kd") {
        rotation_kd_ = parameter.as_double();
      } else if (name == plugin_name_ + ".transform_tolerance") {
        double transform_tolerance = parameter.as_double();
        transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
      } else if (name == plugin_name_ + ".min_max_sum_error") {
        min_max_sum_error_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_dist") {
        lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_lookahead_dist") {
        min_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_lookahead_dist") {
        max_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_time") {
        lookahead_time_ = parameter.as_double();
      } else if (name == plugin_name_ + ".use_rotate_to_heading_treshold") {
        use_rotate_to_heading_treshold_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_approach_linear_velocity") {
        min_approach_linear_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".approach_velocity_scaling_dist") {
        approach_velocity_scaling_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_linear_max") {
        v_linear_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_linear_min") {
        v_linear_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_angular_max") {
        v_angular_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_angular_min") {
        v_angular_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_min") {
        curvature_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_max") {
        curvature_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".reduction_ratio_at_high_curvature") {
        reduction_ratio_at_high_curvature_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_forward_dist") {
        curvature_forward_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_backward_dist") {
        curvature_backward_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_velocity_scaling_factor_rate") {
        max_velocity_scaling_factor_rate_ = parameter.as_double();
      } else if (name == plugin_name_ + ".minco_tracking_timeout") {
        minco_tracking_timeout_ = std::max(0.05, parameter.as_double());
      } else if (name == plugin_name_ + ".minco_tracking_sample_dt") {
        minco_tracking_sample_dt_ = std::clamp(parameter.as_double(), 0.02, 0.50);
      } else if (name == plugin_name_ + ".minco_tracking_min_duration") {
        minco_tracking_min_duration_ = std::max(0.05, parameter.as_double());
        minco_tracking_max_duration_ =
          std::max(minco_tracking_min_duration_, minco_tracking_max_duration_);
      } else if (name == plugin_name_ + ".minco_tracking_max_duration") {
        minco_tracking_max_duration_ =
          std::max(minco_tracking_min_duration_, parameter.as_double());
      } else if (name == plugin_name_ + ".minco_projection_search_ahead_sec") {
        minco_projection_search_ahead_sec_ = std::clamp(parameter.as_double(), 0.02, 2.0);
      } else if (name == plugin_name_ + ".minco_projection_max_advance_sec") {
        minco_projection_max_advance_sec_ = std::clamp(parameter.as_double(), 0.0, 1.0);
      } else if (name == plugin_name_ + ".minco_projection_max_lag_sec") {
        minco_projection_max_lag_sec_ = std::clamp(parameter.as_double(), 0.0, 5.0);
      } else if (name == plugin_name_ + ".navigation_mode_timeout") {
        navigation_mode_timeout_ = std::max(0.05, parameter.as_double());
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".use_velocity_scaled_lookahead_dist") {
        use_velocity_scaled_lookahead_dist_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_interpolation") {
        use_interpolation_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_rotate_to_heading") {
        use_rotate_to_heading_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_minco_tracking_path") {
        use_minco_tracking_path_ = parameter.as_bool();
        if (!use_minco_tracking_path_) {
          global_plan_ = bt_global_plan_;
        }
      } else if (name == plugin_name_ + ".skip_collision_check_in_hole_pass") {
        skip_collision_check_in_hole_pass_ = parameter.as_bool();
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == plugin_name_ + ".hole_pass_mode_name") {
        hole_pass_mode_name_ = parameter.as_string();
      }
    }
  }
  result.successful = true;
  return result;
}

};  // namespace pb_omni_pid_pursuit_controller
// Register this controller as a nav2_core plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pb_omni_pid_pursuit_controller::OmniPidPursuitController, nav2_core::Controller)
