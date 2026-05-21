// Copyright 2025 Pan
// Licensed under the Apache License, Version 2.0
//
// ReplanCondition — BT condition node implementation
//
// Returns SUCCESS when a replan is needed, FAILURE otherwise.
// Checks: (1) time-based rate, (2) path deviation, (3) forward collision via costmap.

#include "sirb_nav2_plugins/bt_nodes/corridor_replan_condition.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2_ros/transform_listener.h"

namespace sirb_nav2_plugins
{
namespace
{
double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

ReplanCondition::ReplanCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  callback_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());
  last_replan_time_ = std::chrono::steady_clock::now();
}

BT::NodeStatus ReplanCondition::tick()
{
  // ---- Read port values ----
  double deviation_threshold = 0.5;
  int collision_cost_threshold = 100;
  double lookahead_distance = 1.5;
  double goal_update_distance = 0.5;
  double min_replan_period = 0.4;
  double max_replan_period = 5.0;
  std::string costmap_topic = "local_costmap/costmap_raw";
  std::string global_frame = "map";
  std::string robot_frame = "base_footprint";
  std::string dyn_obs_topic = "dynamic_obstacles";
  std::string ts_path_topic = "GridBased/timestamped_path";
  std::string trajectory_status_topic = "trajectory_manager/status";
  double trajectory_status_max_age = 1.0;
  double traj_collision_radius = 0.5;
  double traj_lookahead_time = 2.0;
  bool use_dynamic_obstacles = node_->has_parameter("use_dynamic_obstacles") &&
    node_->get_parameter("use_dynamic_obstacles").as_bool();

  getInput("deviation_threshold", deviation_threshold);
  getInput("collision_cost_threshold", collision_cost_threshold);
  getInput("lookahead_distance", lookahead_distance);
  getInput("goal_update_distance", goal_update_distance);
  getInput("min_replan_period", min_replan_period);
  getInput("max_replan_period", max_replan_period);
  getInput("costmap_topic", costmap_topic);
  getInput("global_frame", global_frame);
  getInput("robot_frame", robot_frame);
  getInput("dyn_obs_topic", dyn_obs_topic);
  getInput("use_dynamic_obstacles", use_dynamic_obstacles);
  getInput("timestamped_path_topic", ts_path_topic);
  getInput("trajectory_status_topic", trajectory_status_topic);
  getInput("trajectory_status_max_age", trajectory_status_max_age);
  getInput("traj_collision_radius", traj_collision_radius);
  getInput("traj_lookahead_time", traj_lookahead_time);
  max_replan_period = std::max(max_replan_period, min_replan_period);

  // ---- Lazy costmap subscription ----
  if (!costmap_sub_ || costmap_topic_ != costmap_topic) {
    auto qos = rclcpp::QoS(1);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
      costmap_topic, qos,
      [this](const nav2_msgs::msg::Costmap::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(costmap_mutex_);
        last_costmap_ = msg;
      }, sub_options);
    costmap_topic_ = costmap_topic;
  }

  // ---- Lazy dynamic obstacle subscription ----
  using ObsMsg = sentry_nav_interfaces::msg::TrackedObstacleArray;
  if (use_dynamic_obstacles && (!dyn_obs_sub_ || dyn_obs_topic_ != dyn_obs_topic)) {
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    dyn_obs_sub_ = node_->create_subscription<ObsMsg>(
      dyn_obs_topic, rclcpp::SensorDataQoS(),
      [this](const ObsMsg::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(dyn_obs_mutex_);
        latest_dyn_obs_ = msg;
      }, sub_options);
    dyn_obs_topic_ = dyn_obs_topic;
  } else if (!use_dynamic_obstacles && dyn_obs_sub_) {
    dyn_obs_sub_.reset();
    latest_dyn_obs_.reset();
    dyn_obs_topic_.clear();
  }

  // ---- Lazy timestamped path subscription ----
  using TsPathMsg = sentry_nav_interfaces::msg::TimestampedPath;
  if (use_dynamic_obstacles && (!ts_path_sub_ || ts_path_topic_ != ts_path_topic)) {
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    ts_path_sub_ = node_->create_subscription<TsPathMsg>(
      ts_path_topic, rclcpp::QoS(10),
      [this](const TsPathMsg::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(ts_path_mutex_);
        latest_ts_path_ = msg;
      }, sub_options);
    ts_path_topic_ = ts_path_topic;
  } else if (!use_dynamic_obstacles && ts_path_sub_) {
    ts_path_sub_.reset();
    latest_ts_path_.reset();
    ts_path_topic_.clear();
  }

  // ---- Lazy trajectory status subscription ----
  if (!trajectory_status_sub_ || trajectory_status_topic_ != trajectory_status_topic) {
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    trajectory_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      trajectory_status_topic, rclcpp::QoS(1),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(trajectory_status_mutex_);
        latest_trajectory_status_ = msg->data;
        latest_trajectory_status_time_ = std::chrono::steady_clock::now();
        have_trajectory_status_ = true;
      }, sub_options);
    trajectory_status_topic_ = trajectory_status_topic;
  }

  // Spin once to receive subscription updates
  {
    std::lock_guard<std::mutex> spin_lock(spin_mutex_);
    callback_group_executor_.spin_some();
  }

  // ---- Get the current path from blackboard ----
  nav_msgs::msg::Path path;
  if (!getInput("path", path) || path.poses.empty()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "[ReplanCondition] No tracking path, requesting replan");
    first_tick_ = false;
    last_replan_time_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped goal;
  if (getInput("goal", goal)) {
    const double goal_shift = distanceFromPathEndToGoal(path, goal);
    if (goal_shift >= goal_update_distance) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[ReplanCondition] Goal/path endpoint changed %.3f m > %.3f m",
        goal_shift, goal_update_distance);
      last_replan_time_ = std::chrono::steady_clock::now();
      first_tick_ = false;
      return BT::NodeStatus::SUCCESS;
    }
  }

  // ---- Get robot position via TF ----
  double robot_x, robot_y;
  if (!getRobotPose(robot_x, robot_y, global_frame, robot_frame)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[ReplanCondition] Failed to get robot pose from TF");
    return BT::NodeStatus::FAILURE;
  }

  // ---- Check 1: Path deviation (with cooldown) ----
  double deviation = distanceToPath(robot_x, robot_y, path);
  auto now = std::chrono::steady_clock::now();
  double elapsed_s = std::chrono::duration<double>(now - last_replan_time_).count();

  if (elapsed_s >= min_replan_period) {
    std::string trajectory_status;
    bool status_fresh = false;
    {
      std::lock_guard<std::mutex> lock(trajectory_status_mutex_);
      if (have_trajectory_status_) {
        trajectory_status = latest_trajectory_status_;
        const double status_age =
          std::chrono::duration<double>(now - latest_trajectory_status_time_).count();
        status_fresh = status_age <= std::max(trajectory_status_max_age, 0.0);
      }
    }
    if (status_fresh &&
      trajectory_status.find("active=0") != std::string::npos)
    {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "[ReplanCondition] Tracking path has no executable MINCO trajectory, requesting replan: %s",
        trajectory_status.c_str());
      last_replan_time_ = now;
      first_tick_ = false;
      return BT::NodeStatus::SUCCESS;
    }
  }

  if (deviation > deviation_threshold && elapsed_s >= min_replan_period) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[ReplanCondition] Path deviation %.3f m > %.3f m threshold",
      deviation, deviation_threshold);
    last_replan_time_ = now;
    first_tick_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  // ---- Check 2: Fallback time-based replan (5s) ----
  // Ensures new goals always get a path even if robot is near old path.
  if (elapsed_s >= max_replan_period) {
    RCLCPP_DEBUG(node_->get_logger(),
      "[ReplanCondition] Fallback time-based replan: %.1f s elapsed", elapsed_s);
    last_replan_time_ = now;
    first_tick_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  // ---- Check 3: Forward collision via costmap ----
  if (elapsed_s >= min_replan_period &&
      checkPathForwardCollision(robot_x, robot_y, path, lookahead_distance, collision_cost_threshold))
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "[ReplanCondition] Forward collision detected on path ahead");
    last_replan_time_ = now;
    first_tick_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  // ---- Check 4: Spatio-temporal dynamic obstacle intersection ----
  if (use_dynamic_obstacles && elapsed_s >= min_replan_period &&
      checkDynamicObstacleIntersection(robot_x, robot_y, traj_collision_radius, traj_lookahead_time))
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "[ReplanCondition] Dynamic obstacle intersection detected on trajectory");
    last_replan_time_ = now;
    first_tick_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  // ---- No replan needed ----
  first_tick_ = false;
  return BT::NodeStatus::FAILURE;
}

bool ReplanCondition::getRobotPose(
  double & x, double & y,
  const std::string & global_frame, const std::string & robot_frame) const
{
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_->lookupTransform(
      global_frame, robot_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return false;
  }

  x = tf_stamped.transform.translation.x;
  y = tf_stamped.transform.translation.y;
  return true;
}

double ReplanCondition::distanceToPath(
  double x, double y, const nav_msgs::msg::Path & path) const
{
  double min_dist_sq = std::numeric_limits<double>::max();

  for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
    // Point-to-segment distance
    double ax = path.poses[i].pose.position.x;
    double ay = path.poses[i].pose.position.y;
    double bx = path.poses[i + 1].pose.position.x;
    double by = path.poses[i + 1].pose.position.y;

    double dx = bx - ax;
    double dy = by - ay;
    double len_sq = dx * dx + dy * dy;

    double t;
    if (len_sq < 1e-12) {
      t = 0.0;
    } else {
      t = ((x - ax) * dx + (y - ay) * dy) / len_sq;
      t = std::clamp(t, 0.0, 1.0);
    }

    double px = ax + t * dx;
    double py = ay + t * dy;
    double dist_sq = (x - px) * (x - px) + (y - py) * (y - py);
    min_dist_sq = std::min(min_dist_sq, dist_sq);
  }

  // Handle single-point path
  if (path.poses.size() == 1) {
    double dx = x - path.poses[0].pose.position.x;
    double dy = y - path.poses[0].pose.position.y;
    min_dist_sq = dx * dx + dy * dy;
  }

  return std::sqrt(min_dist_sq);
}

double ReplanCondition::distanceFromPathEndToGoal(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  if (path.poses.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const auto & end = path.poses.back().pose.position;
  const auto & g = goal.pose.position;
  return std::hypot(end.x - g.x, end.y - g.y);
}

bool ReplanCondition::checkPathForwardCollision(
  double robot_x, double robot_y,
  const nav_msgs::msg::Path & path,
  double lookahead_dist,
  int cost_threshold) const
{
  if (path.poses.size() < 2) {
    return false;
  }

  // Get latest costmap
  nav2_msgs::msg::Costmap::SharedPtr costmap_ptr;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap_ptr = last_costmap_;
  }

  if (!costmap_ptr || costmap_ptr->data.empty()) {
    return false;  // No costmap data — cannot check
  }

  const auto & costmap = *costmap_ptr;

  // Find closest pose index on path
  size_t closest_idx = 0;
  double min_dist_sq = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dx = robot_x - path.poses[i].pose.position.x;
    double dy = robot_y - path.poses[i].pose.position.y;
    double d = dx * dx + dy * dy;
    if (d < min_dist_sq) {
      min_dist_sq = d;
      closest_idx = i;
    }
  }

  // Walk forward along the path, checking costmap costs
  double arc_length = 0.0;
  for (size_t i = closest_idx; i + 1 < path.poses.size(); ++i) {
    double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
    double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
    arc_length += std::sqrt(dx * dx + dy * dy);

    // Check this path pose in the costmap BEFORE deciding to break,
    // so poses exactly at the lookahead boundary are not skipped.
    double wx = path.poses[i + 1].pose.position.x;
    double wy = path.poses[i + 1].pose.position.y;
    unsigned int mx, my;
    if (worldToMap(costmap, wx, wy, mx, my)) {
      size_t index = static_cast<size_t>(my) * costmap.metadata.size_x + mx;
      if (index < costmap.data.size()) {
        int cost = static_cast<int>(costmap.data[index]);
        if (cost >= cost_threshold) {
          return true;  // Collision detected
        }
      }
    }

    if (arc_length > lookahead_dist) {
      break;
    }
  }

  return false;
}

bool ReplanCondition::worldToMap(
  const nav2_msgs::msg::Costmap & costmap,
  double wx, double wy,
  unsigned int & mx, unsigned int & my) const
{
  const auto & meta = costmap.metadata;
  if (meta.resolution <= 0.0 || meta.size_x == 0 || meta.size_y == 0) {
    return false;
  }

  const double origin_x = meta.origin.position.x;
  const double origin_y = meta.origin.position.y;
  const double dx = wx - origin_x;
  const double dy = wy - origin_y;
  const double yaw = yawFromQuaternion(meta.origin.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double local_x = cos_yaw * dx + sin_yaw * dy;
  const double local_y = -sin_yaw * dx + cos_yaw * dy;
  double mx_d = local_x / meta.resolution;
  double my_d = local_y / meta.resolution;

  if (mx_d < 0.0 || my_d < 0.0) {
    return false;
  }

  mx = static_cast<unsigned int>(mx_d);
  my = static_cast<unsigned int>(my_d);

  return (mx < meta.size_x && my < meta.size_y);
}

bool ReplanCondition::checkDynamicObstacleIntersection(
  double robot_x, double robot_y,
  double traj_collision_radius,
  double traj_lookahead_time) const
{
  // 护卫: 系统刚启动时可能尚未收到任何消息
  sentry_nav_interfaces::msg::TimestampedPath::SharedPtr ts_path;
  sentry_nav_interfaces::msg::TrackedObstacleArray::SharedPtr dyn_obs;
  {
    std::scoped_lock lk(ts_path_mutex_, dyn_obs_mutex_);
    ts_path = latest_ts_path_;
    dyn_obs = latest_dyn_obs_;
  }

  if (!ts_path || !dyn_obs) { return false; }
  if (ts_path->path.poses.empty() || ts_path->timestamps.empty()) { return false; }
  if (dyn_obs->obstacles.empty()) { return false; }

  const size_t n_pts = ts_path->path.poses.size();
  if (ts_path->timestamps.size() != n_pts) { return false; }

  // 障碍物消息时延补偿: 用 ROS 时钟 (兼容仿真)
  const double obs_age = (node_->now() - dyn_obs->header.stamp).seconds();
  const double obs_age_clamped = std::max(0.0, std::min(obs_age, 1.0));  // 限制在 1s 内

  // 找到路径中离机器人最近的点作为起始索引
  size_t i_start = 0;
  double min_dist_sq = std::numeric_limits<double>::max();
  for (size_t i = 0; i < n_pts; ++i) {
    const auto & pose = ts_path->path.poses[i].pose.position;
    double dx = robot_x - pose.x;
    double dy = robot_y - pose.y;
    double d2 = dx * dx + dy * dy;
    if (d2 < min_dist_sq) {
      min_dist_sq = d2;
      i_start = i;
    }
  }

  const double t0 = ts_path->timestamps[i_start];

  for (size_t i = i_start + 1; i < n_pts; ++i) {
    if (ts_path->timestamps[i] < ts_path->timestamps[i - 1]) {
      return false;
    }
  }

  // 遍历 i_start 之后的路径点
  for (size_t i = i_start; i < n_pts; ++i) {
    const double t_i = ts_path->timestamps[i] - t0;
    if (t_i > traj_lookahead_time) { break; }

    const double px = ts_path->path.poses[i].pose.position.x;
    const double py = ts_path->path.poses[i].pose.position.y;

    for (const auto & obs : dyn_obs->obstacles) {
      // 时延补偿: 估计障碍物当前位置
      const double cx = obs.x + obs.vx * obs_age_clamped;
      const double cy = obs.y + obs.vy * obs_age_clamped;

      // 时空预测: 障碍物在机器人到达 path[i] 时刻的位置
      const double p_obs_x = cx + obs.vx * t_i;
      const double p_obs_y = cy + obs.vy * t_i;

      double dx = px - p_obs_x;
      double dy = py - p_obs_y;
      double dist = std::sqrt(dx * dx + dy * dy);

      if (dist < traj_collision_radius + obs.radius) {
        return true;
      }
    }
  }

  return false;
}

}  // namespace sirb_nav2_plugins
