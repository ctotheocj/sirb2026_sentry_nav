#ifndef SIRB_NAV2_PLUGINS__BT_NODES__PATH_GATE_HPP_
#define SIRB_NAV2_PLUGINS__BT_NODES__PATH_GATE_HPP_

#include <chrono>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sirb_nav2_plugins
{

class PathGate : public BT::SyncActionNode
{
public:
  PathGate(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("candidate_path", "Newly planned/smoothed path"),
      BT::InputPort<nav_msgs::msg::Path>("tracking_path", "Stable path used by FollowPath"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "Updated stable tracking path"),
      BT::InputPort<double>("goal_update_distance", 0.5, "Accept new path if goal moved this far"),
      BT::InputPort<double>("path_update_distance", 0.25, "Accept new path if geometry differs this far"),
      BT::InputPort<double>("forward_compare_distance", 3.0, "Forward arc length window for near-term path comparison"),
      BT::InputPort<double>("global_path_update_distance", 0.45, "Accept new path if full-route geometry differs this far"),
      BT::InputPort<double>("length_update_ratio", 0.10, "Accept new path if length changes by this ratio"),
      BT::InputPort<double>("min_accept_interval_sec", 2.5, "Minimum interval between same-goal path switches"),
      BT::InputPort<double>("immediate_update_multiplier", 2.0, "Bypass cooldown if geometry change exceeds this multiplier"),
      BT::InputPort<bool>("accept_empty_tracking_path", true, "Seed tracking path if it is empty"),
      BT::InputPort<bool>("fail_on_reject", false, "Return FAILURE when candidate is retained so downstream actions can be skipped"),
    };
  }

  BT::NodeStatus tick() override;

private:
  double goalDistance(
    const nav_msgs::msg::Path & a,
    const nav_msgs::msg::Path & b) const;
  double pathLength(const nav_msgs::msg::Path & path) const;
  double pathDeviation(
    const nav_msgs::msg::Path & source,
    const nav_msgs::msg::Path & target,
    double max_source_arc,
    size_t max_samples) const;
  double pointToPathDistance(
    double x,
    double y,
    const nav_msgs::msg::Path & path) const;

  rclcpp::Node::SharedPtr node_;
  std::chrono::steady_clock::time_point last_accept_time_{};
  bool has_last_accept_{false};
};

}  // namespace sirb_nav2_plugins

#endif  // SIRB_NAV2_PLUGINS__BT_NODES__PATH_GATE_HPP_
