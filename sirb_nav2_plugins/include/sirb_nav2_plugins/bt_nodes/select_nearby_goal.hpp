#ifndef SIRB_NAV2_PLUGINS__BT_NODES__SELECT_NEARBY_GOAL_HPP_
#define SIRB_NAV2_PLUGINS__BT_NODES__SELECT_NEARBY_GOAL_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sirb_nav2_plugins
{

// Select the input goal when it is free, otherwise select a nearby free fallback.
class SelectNearbyGoal : public BT::StatefulActionNode
{
public:
  SelectNearbyGoal(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  BT::NodeStatus tickImpl();

  bool worldToMap(
    const nav2_msgs::msg::Costmap & map,
    double wx, double wy,
    unsigned int & mx, unsigned int & my) const;

  bool getCostAtWorld(
    const nav2_msgs::msg::Costmap & map,
    double wx, double wy,
    int & cost) const;

  bool isCostAllowed(
    int cost,
    int cost_threshold,
    bool allow_unknown) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  nav2_msgs::msg::Costmap::SharedPtr last_costmap_;
  std::mutex costmap_mutex_;
  std::string topic_name_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};

}  // namespace sirb_nav2_plugins

#endif  // SIRB_NAV2_PLUGINS__BT_NODES__SELECT_NEARBY_GOAL_HPP_
