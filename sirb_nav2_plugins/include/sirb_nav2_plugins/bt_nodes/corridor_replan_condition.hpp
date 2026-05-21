// Copyright 2025 Pan
// Licensed under the Apache License, Version 2.0
//
// ReplanCondition — BT condition node for intelligent replan triggering
//
// Returns SUCCESS (= "should replan") when ANY of the following conditions hold:
//   1. Path deviation:      robot drifted > deviation_threshold from current path
//   2. Forward collision:   costmap cost along lookahead > collision_cost_threshold
//   3. Minimum rate:        at least min_replan_period seconds since last replan
//
// Returns FAILURE (= "no replan needed") otherwise.
//
// Design: The condition node queries the planner state via a shared blackboard
// variable "path" and TF transforms, and subscribes to the global costmap for
// forward collision checking.  This is a lightweight check that does not
// require direct access to the planner plugin instance.

#ifndef SIRB_NAV2_PLUGINS__BT_NODES__CORRIDOR_REPLAN_CONDITION_HPP_
#define SIRB_NAV2_PLUGINS__BT_NODES__CORRIDOR_REPLAN_CONDITION_HPP_

#include <string>
#include <chrono>
#include <cmath>
#include <mutex>

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"

#include "sentry_nav_interfaces/msg/tracked_obstacle_array.hpp"
#include "sentry_nav_interfaces/msg/timestamped_path.hpp"

namespace sirb_nav2_plugins
{

/**
 * @brief BT condition node that triggers global replan based on:
 *        path deviation, forward collision risk (via costmap), or time-based minimum rate.
 *
 * BT Ports:
 *   Input "path":                    current planned path (from blackboard)
 *   Input "deviation_threshold":     max allowed deviation (m), default 0.5
 *   Input "collision_cost_threshold":costmap cost >= this triggers replan, default 100
 *   Input "lookahead_distance":      how far ahead to check collision (m), default 1.5
 *   Input "min_replan_period":       minimum replan interval (s), default 0.4
 *   Input "max_replan_period":       maximum replan interval (s), default 5.0
 *   Input "global_frame":            TF frame for robot pose, default "map"
 *   Input "robot_frame":             TF robot frame, default "base_footprint"
 *   Input "costmap_topic":           costmap topic for collision check
 *   Input "use_dynamic_obstacles":   enable dynamic obstacle intersection check
 *   Input "dyn_obs_topic":           dynamic obstacles topic, default "/dynamic_obstacles"
 *   Input "timestamped_path_topic":  atomic path+timestamps topic, default "/GridBased/timestamped_path"
 *   Input "trajectory_status_topic": TrajectoryManager status topic, default "trajectory_manager/status"
 *   Input "traj_collision_radius":   collision check radius (m), default 0.5
 *   Input "traj_lookahead_time":     max look-ahead time (s), default 2.0
 */
class ReplanCondition : public BT::ConditionNode
{
public:
  ReplanCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  ~ReplanCondition() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Current planned path"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Current navigation goal"),
      BT::InputPort<double>("goal_update_distance", 0.5, "Replan if current path endpoint is this far from goal"),
      BT::InputPort<double>("deviation_threshold", 0.5, "Path deviation threshold (m)"),
      BT::InputPort<int>("collision_cost_threshold",
        100, "Costmap cost threshold for collision (0-254)"),
      BT::InputPort<double>("lookahead_distance", 1.5, "Forward collision check distance (m)"),
      BT::InputPort<double>("min_replan_period", 0.4, "Min replan interval (s)"),
      BT::InputPort<double>("max_replan_period", 5.0, "Max replan interval (s), forces replan even without triggers"),
      BT::InputPort<std::string>("global_frame", "map", "Global TF frame"),
      BT::InputPort<std::string>("robot_frame", "base_footprint", "Robot TF frame"),
      BT::InputPort<std::string>("costmap_topic", "local_costmap/costmap_raw",
        "Costmap topic for forward collision check (use local for dynamic obstacles)"),
      BT::InputPort<bool>("use_dynamic_obstacles",
        "Enable dynamic obstacle trajectory intersection check"),
      BT::InputPort<std::string>("dyn_obs_topic", "dynamic_obstacles",
        "Dynamic obstacles topic (TrackedObstacleArray)"),
      BT::InputPort<std::string>("timestamped_path_topic", "GridBased/timestamped_path",
        "Atomic path+timestamps topic (TimestampedPath)"),
      BT::InputPort<std::string>("trajectory_status_topic", "trajectory_manager/status",
        "TrajectoryManager status topic"),
      BT::InputPort<double>("trajectory_status_max_age", 1.0,
        "Ignore trajectory status older than this many seconds"),
      BT::InputPort<double>("traj_collision_radius", 0.5,
        "Robot+safety radius for dynamic obstacle intersection check (m)"),
      BT::InputPort<double>("traj_lookahead_time", 2.0,
        "How far ahead in time to check obstacle intersection (s)"),
    };
  }

  BT::NodeStatus tick() override;

private:
  /// Get current robot position from TF
  bool getRobotPose(
    double & x, double & y,
    const std::string & global_frame, const std::string & robot_frame) const;

  /// Compute minimum distance from point to the path
  double distanceToPath(double x, double y, const nav_msgs::msg::Path & path) const;

  /// Compute path endpoint distance to current goal. Returns infinity if unavailable.
  double distanceFromPathEndToGoal(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & goal) const;

  /// Check if path poses ahead of closest index have high costmap cost
  /// @return true if any path pose within lookahead_dist has cost >= threshold
  bool checkPathForwardCollision(
    double robot_x, double robot_y,
    const nav_msgs::msg::Path & path,
    double lookahead_dist,
    int cost_threshold) const;

  /// Convert world coordinates to costmap cell indices
  bool worldToMap(
    const nav2_msgs::msg::Costmap & costmap,
    double wx, double wy,
    unsigned int & mx, unsigned int & my) const;

  /// Spatio-temporal intersection check: returns true if path will collide with
  /// any dynamic obstacle within traj_lookahead_time, accounting for obs_age delay.
  bool checkDynamicObstacleIntersection(
    double robot_x, double robot_y,
    double traj_collision_radius,
    double traj_lookahead_time) const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // Costmap subscription for forward collision check
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  nav2_msgs::msg::Costmap::SharedPtr last_costmap_;
  mutable std::mutex costmap_mutex_;
  std::string costmap_topic_;

  // Dynamic obstacle subscription for spatio-temporal intersection check
  rclcpp::Subscription<sentry_nav_interfaces::msg::TrackedObstacleArray>::SharedPtr dyn_obs_sub_;
  sentry_nav_interfaces::msg::TrackedObstacleArray::SharedPtr latest_dyn_obs_;
  mutable std::mutex dyn_obs_mutex_;
  std::string dyn_obs_topic_;

  // Timestamped path subscription (atomic: path + MINCO timestamps)
  rclcpp::Subscription<sentry_nav_interfaces::msg::TimestampedPath>::SharedPtr ts_path_sub_;
  sentry_nav_interfaces::msg::TimestampedPath::SharedPtr latest_ts_path_;
  mutable std::mutex ts_path_mutex_;
  std::string ts_path_topic_;

  // TrajectoryManager status. This distinguishes a blackboard path from an executable MINCO trajectory.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trajectory_status_sub_;
  std::string latest_trajectory_status_;
  std::chrono::steady_clock::time_point latest_trajectory_status_time_;
  bool have_trajectory_status_{false};
  mutable std::mutex trajectory_status_mutex_;
  std::string trajectory_status_topic_;

  std::chrono::steady_clock::time_point last_replan_time_;
  bool first_tick_{true};
  mutable std::mutex spin_mutex_;
};

}  // namespace sirb_nav2_plugins

#endif  // SIRB_NAV2_PLUGINS__BT_NODES__CORRIDOR_REPLAN_CONDITION_HPP_
