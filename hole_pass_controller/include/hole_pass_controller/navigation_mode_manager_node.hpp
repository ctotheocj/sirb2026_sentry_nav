#ifndef HOLE_PASS_CONTROLLER__NAVIGATION_MODE_MANAGER_NODE_HPP_
#define HOLE_PASS_CONTROLLER__NAVIGATION_MODE_MANAGER_NODE_HPP_

#include <map>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sentry_nav_interfaces/srv/set_navigation_mode.hpp"
#include "sentry_nav_interfaces/srv/set_semantic_layer_mode.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

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
  bool enterHolePass(const SetNavigationMode::Request & request, std::string & message);
  bool restoreNormal(const std::string & owner_id, std::string & message);
  bool applyNormalModeServices();
  bool callSemanticLayer(
    const std::string & service,
    const std::string & mode,
    const std::string & frame_id,
    const geometry_msgs::msg::Polygon & corridor);
  bool callSetBool(const std::string & service, bool value);
  bool setInflationRadius(const std::string & node_name, double radius);
  std::vector<std::string> parseList(const std::string & text) const;
  void setHoleModeBlackboardHint(bool active);

  rclcpp::Service<SetNavigationMode>::SharedPtr service_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hole_mode_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::executors::SingleThreadedExecutor client_executor_;

  std::vector<std::string> semantic_layer_services_;
  std::vector<std::string> legacy_layer_services_;
  std::string replan_suppression_service_;
  std::string trajectory_policy_service_;
  std::string global_costmap_node_;
  std::string local_costmap_node_;
  double normal_global_inflation_radius_{0.55};
  double normal_local_inflation_radius_{0.8};
  double hole_global_inflation_radius_{0.10};
  double hole_local_inflation_radius_{0.10};
  double service_timeout_sec_{0.2};
  double default_watchdog_timeout_sec_{8.0};
  bool use_inflation_radius_switch_{true};
  bool use_semantic_layer_mode_{true};
  bool use_trajectory_policy_switch_{true};

  bool hole_mode_active_{false};
  std::string active_owner_id_;
  std::string active_hole_id_;
  rclcpp::Time active_deadline_{0, 0, RCL_ROS_TIME};
};

}  // namespace hole_pass_controller

#endif  // HOLE_PASS_CONTROLLER__NAVIGATION_MODE_MANAGER_NODE_HPP_
