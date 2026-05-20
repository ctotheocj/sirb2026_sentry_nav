#ifndef SIRB_NAV2_PLUGINS__BT_NODES__HOLE_PASS_SCOPE_HPP_
#define SIRB_NAV2_PLUGINS__BT_NODES__HOLE_PASS_SCOPE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentry_nav_interfaces/srv/set_navigation_mode.hpp"

namespace sirb_nav2_plugins
{

class HolePassScope : public BT::DecoratorNode
{
public:
  HolePassScope(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("blackboard_key", "hole_mode_active", "Blackboard flag key"),
      BT::InputPort<std::string>(
        "mode_service", "navigation_mode_manager/set_navigation_mode",
        "SetNavigationMode service"),
      BT::InputPort<std::string>("hole_id", "", "Hole id"),
      BT::InputPort<geometry_msgs::msg::PolygonStamped>("corridor", "Hole corridor"),
      BT::InputPort<double>("watchdog_timeout_sec", 8.0, "Mode watchdog timeout"),
      BT::InputPort<double>("service_timeout", 0.5, "Service wait/call timeout seconds"),
    };
  }

  BT::NodeStatus tick() override;
  void halt() override;

private:
  bool enterHoleMode();
  void exitHoleMode();
  bool callSetNavigationMode(
    const std::string & service,
    const std::string & mode,
    const std::string & owner_id,
    const std::string & hole_id,
    const geometry_msgs::msg::PolygonStamped & corridor,
    double watchdog_timeout_sec,
    double timeout_sec);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  bool active_{false};
  std::string owner_id_;
};

}  // namespace sirb_nav2_plugins

#endif  // SIRB_NAV2_PLUGINS__BT_NODES__HOLE_PASS_SCOPE_HPP_
