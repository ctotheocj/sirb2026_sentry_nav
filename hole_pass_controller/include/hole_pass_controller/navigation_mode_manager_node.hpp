#ifndef HOLE_PASS_CONTROLLER__NAVIGATION_MODE_MANAGER_NODE_HPP_
#define HOLE_PASS_CONTROLLER__NAVIGATION_MODE_MANAGER_NODE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sentry_nav_interfaces/msg/hole_pass_cmd.hpp"
#include "sentry_nav_interfaces/srv/set_navigation_mode.hpp"
#include "sentry_nav_interfaces/srv/set_semantic_layer_mode.hpp"

namespace hole_pass_controller
{

class NavigationModeManagerNode : public rclcpp::Node
{
public:
  explicit NavigationModeManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using SetNavigationMode = sentry_nav_interfaces::srv::SetNavigationMode;
  using SetSemanticLayerMode = sentry_nav_interfaces::srv::SetSemanticLayerMode;

  void handleSetMode(
    const std::shared_ptr<SetNavigationMode::Request> request,
    std::shared_ptr<SetNavigationMode::Response> response);
  void watchdogCallback();
  void holeCommandTimerCallback();
  void publishHoleCommand();
  bool enterHolePass(const SetNavigationMode::Request & request, std::string & message);
  bool restoreNormal(const std::string & owner_id, std::string & message);
  bool applyNormalModeServices();
  bool callSemanticLayer(
    const std::string & service,
    const std::string & mode);
  std::vector<std::string> parseList(const std::string & text) const;

  rclcpp::Service<SetNavigationMode>::SharedPtr service_;
  rclcpp::Publisher<sentry_nav_interfaces::msg::HolePassCmd>::SharedPtr hole_cmd_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr hole_cmd_timer_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::executors::SingleThreadedExecutor client_executor_;

  std::vector<std::string> semantic_layer_services_;
  double service_timeout_sec_{0.2};
  double default_watchdog_timeout_sec_{8.0};
  double hole_command_publish_period_sec_{0.05};
  std::string hole_pass_cmd_topic_;

  bool hole_mode_active_{false};
  uint8_t active_hole_cmd_{sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE};
  float active_v_yaw_{0.0F};
  std::string active_owner_id_;
  std::string active_hole_id_;
  rclcpp::Time active_deadline_{0, 0, RCL_ROS_TIME};
};

}  // namespace hole_pass_controller

#endif  // HOLE_PASS_CONTROLLER__NAVIGATION_MODE_MANAGER_NODE_HPP_
