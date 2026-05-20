#include "sirb_nav2_plugins/bt_nodes/hole_pass_scope.hpp"

#include <chrono>

namespace sirb_nav2_plugins
{

HolePassScope::HolePassScope(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
}

BT::NodeStatus HolePassScope::tick()
{
  if (!active_) {
    if (!enterHoleMode()) {
      return BT::NodeStatus::FAILURE;
    }
  }

  const BT::NodeStatus child_status = child_node_->executeTick();
  if (child_status != BT::NodeStatus::RUNNING) {
    exitHoleMode();
  }
  return child_status;
}

void HolePassScope::halt()
{
  exitHoleMode();
  haltChild();
  resetStatus();
}

bool HolePassScope::enterHoleMode()
{
  std::string key = "hole_mode_active";
  std::string service = "navigation_mode_manager/set_navigation_mode";
  std::string hole_id;
  geometry_msgs::msg::PolygonStamped corridor;
  double watchdog_timeout_sec = 8.0;
  double timeout_sec = 0.5;
  getInput("blackboard_key", key);
  getInput("mode_service", service);
  getInput("hole_id", hole_id);
  getInput("corridor", corridor);
  getInput("watchdog_timeout_sec", watchdog_timeout_sec);
  getInput("service_timeout", timeout_sec);

  owner_id_ = name() + ":" + std::to_string(reinterpret_cast<std::uintptr_t>(this));
  const bool entered = callSetNavigationMode(
    service, "hole_pass", owner_id_, hole_id, corridor, watchdog_timeout_sec, timeout_sec);
  if (!entered) {
    owner_id_.clear();
    config().blackboard->set(key, false);
    active_ = false;
    RCLCPP_WARN(node_->get_logger(), "HolePassScope: failed to enter hole_pass mode");
    return false;
  }

  config().blackboard->set(key, true);
  active_ = true;
  RCLCPP_INFO(node_->get_logger(), "HolePassScope: entered hole_pass mode");
  return true;
}

void HolePassScope::exitHoleMode()
{
  if (!active_) {
    return;
  }

  std::string key = "hole_mode_active";
  std::string service = "navigation_mode_manager/set_navigation_mode";
  geometry_msgs::msg::PolygonStamped corridor;
  double timeout_sec = 0.5;
  getInput("blackboard_key", key);
  getInput("mode_service", service);
  getInput("service_timeout", timeout_sec);

  callSetNavigationMode(
    service, "normal", owner_id_, "", corridor, 0.0, timeout_sec);
  config().blackboard->set(key, false);
  active_ = false;
  RCLCPP_INFO(node_->get_logger(), "HolePassScope: restored normal mode");
}

bool HolePassScope::callSetNavigationMode(
  const std::string & service,
  const std::string & mode,
  const std::string & owner_id,
  const std::string & hole_id,
  const geometry_msgs::msg::PolygonStamped & corridor,
  double watchdog_timeout_sec,
  double timeout_sec)
{
  if (service.empty()) {
    return false;
  }

  auto client = node_->create_client<sentry_nav_interfaces::srv::SetNavigationMode>(
    service, rmw_qos_profile_services_default, callback_group_);
  const auto timeout = std::chrono::duration<double>(std::max(0.01, timeout_sec));
  if (!client->wait_for_service(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout))) {
    RCLCPP_WARN(
      node_->get_logger(), "HolePassScope: service '%s' unavailable", service.c_str());
    return false;
  }

  auto request = std::make_shared<sentry_nav_interfaces::srv::SetNavigationMode::Request>();
  request->mode = mode;
  request->owner_id = owner_id;
  request->hole_id = hole_id;
  request->frame_id = corridor.header.frame_id.empty() ? "map" : corridor.header.frame_id;
  request->corridor = corridor.polygon;
  request->watchdog_timeout_sec = static_cast<float>(watchdog_timeout_sec);
  auto future = client->async_send_request(request);
  const auto result = callback_group_executor_.spin_until_future_complete(future, timeout);
  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_WARN(node_->get_logger(), "HolePassScope: service '%s' timed out", service.c_str());
    return false;
  }
  const auto response = future.get();
  if (!response->success) {
    RCLCPP_WARN(
      node_->get_logger(), "HolePassScope: service '%s' rejected mode '%s': %s",
      service.c_str(), mode.c_str(), response->message.c_str());
    return false;
  }
  return true;
}

}  // namespace sirb_nav2_plugins
