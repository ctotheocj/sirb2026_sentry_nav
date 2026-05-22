#include "hole_pass_controller/navigation_mode_manager_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

#include "rclcpp_components/register_node_macro.hpp"

namespace hole_pass_controller
{
namespace
{
bool validHoleCommand(uint8_t cmd)
{
  return cmd == sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE ||
    cmd == sentry_nav_interfaces::msg::HolePassCmd::HOLE_LOWER;
}
}  // namespace

NavigationModeManagerNode::NavigationModeManagerNode(const rclcpp::NodeOptions & options)
: Node("navigation_mode_manager", options)
{
  semantic_layer_services_ = parseList(declare_parameter<std::string>(
    "semantic_layer_services",
    "global_costmap/occupancy_grid_layer/set_semantic_layer_mode"));
  service_timeout_sec_ = declare_parameter<double>("service_timeout_sec", 0.2);
  default_watchdog_timeout_sec_ = declare_parameter<double>("watchdog_timeout_sec", 8.0);
  hole_pass_cmd_topic_ = declare_parameter<std::string>("hole_pass_cmd_topic", "mpc/hole_pass_cmd");
  hole_command_publish_period_sec_ =
    declare_parameter<double>("hole_command_publish_period_sec", 0.05);
  watchdog_restore_normal_ = declare_parameter<bool>("watchdog_restore_normal", false);

  client_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_executor_.add_callback_group(client_callback_group_, get_node_base_interface());

  service_ = create_service<SetNavigationMode>(
    "navigation_mode_manager/set_navigation_mode",
    std::bind(
      &NavigationModeManagerNode::handleSetMode, this,
      std::placeholders::_1, std::placeholders::_2));
  status_service_ = create_service<GetNavigationMode>(
    "navigation_mode_manager/get_navigation_mode",
    std::bind(
      &NavigationModeManagerNode::handleGetMode, this,
      std::placeholders::_1, std::placeholders::_2));
  hole_cmd_pub_ =
    create_publisher<sentry_nav_interfaces::msg::HolePassCmd>(hole_pass_cmd_topic_, 10);
  mode_pub_ = create_publisher<std_msgs::msg::String>("navigation_mode_manager/mode", 10);
  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&NavigationModeManagerNode::watchdogCallback, this));
  hole_cmd_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(std::max(0.005, hole_command_publish_period_sec_))),
    std::bind(&NavigationModeManagerNode::holeCommandTimerCallback, this));

  RCLCPP_INFO(
    get_logger(), "NavigationModeManager ready semantic_services=%zu cmd_topic='%s' "
    "cmd_period=%.3fs watchdog_restore_normal=%d",
    semantic_layer_services_.size(), hole_pass_cmd_topic_.c_str(),
    hole_command_publish_period_sec_, watchdog_restore_normal_ ? 1 : 0);
}

void NavigationModeManagerNode::handleSetMode(
  const std::shared_ptr<SetNavigationMode::Request> request,
  std::shared_ptr<SetNavigationMode::Response> response)
{
  std::string message;
  bool ok = false;
  if (request->mode == "hole_pass") {
    ok = enterHolePass(*request, message);
  } else if (request->mode == "normal") {
    ok = restoreNormal(request->owner_id, message);
  } else {
    message = "unsupported navigation mode '" + request->mode + "'";
  }
  response->success = ok;
  response->message = message;
}

void NavigationModeManagerNode::handleGetMode(
  const std::shared_ptr<GetNavigationMode::Request>/*request*/,
  std::shared_ptr<GetNavigationMode::Response> response)
{
  response->active = hole_mode_active_;
  response->mode = hole_mode_active_ ? "hole_pass" : "normal";
  response->owner_id = active_owner_id_;
  response->hole_id = active_hole_id_;
  response->hole_cmd = hole_mode_active_ ?
    active_hole_cmd_ : sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE;
  response->v_yaw = hole_mode_active_ ? active_v_yaw_ : 0.0F;
}

bool NavigationModeManagerNode::enterHolePass(
  const SetNavigationMode::Request & request, std::string & message)
{
  if (request.owner_id.empty()) {
    message = "owner_id is empty";
    return false;
  }
  if (hole_mode_active_) {
    if (request.owner_id != active_owner_id_) {
      if (request.hole_id.empty() || request.hole_id != active_hole_id_) {
        message = "owner mismatch: active='" + active_owner_id_ + "' request='" +
          request.owner_id + "'";
        RCLCPP_WARN(get_logger(), "%s", message.c_str());
        return false;
      }
      RCLCPP_WARN(
        get_logger(),
        "NavigationModeManager transferring active hole_pass owner '%s' -> '%s' for hole='%s'",
        active_owner_id_.c_str(), request.owner_id.c_str(), active_hole_id_.c_str());
      active_owner_id_ = request.owner_id;
    }
    if (!request.hole_id.empty()) {
      active_hole_id_ = request.hole_id;
    }
    if (validHoleCommand(request.hole_cmd)) {
      active_hole_cmd_ = request.hole_cmd;
    }
    active_v_yaw_ = std::isfinite(request.v_yaw) ? request.v_yaw : 0.0F;
    const double timeout = request.watchdog_timeout_sec > 0.0F ?
      static_cast<double>(request.watchdog_timeout_sec) : default_watchdog_timeout_sec_;
    active_deadline_ = now() + rclcpp::Duration::from_seconds(std::max(0.5, timeout));
    watchdog_reported_ = false;
    message = "refreshed hole_pass mode";
    publishHoleCommand();
    return true;
  }

  bool ok = true;
  for (const auto & service : semantic_layer_services_) {
    ok = callSemanticLayer(service, "hole_pass") && ok;
  }

  if (!ok) {
    const bool rollback_ok = applyNormalModeServices();
    hole_mode_active_ = false;
    active_owner_id_.clear();
    active_hole_id_.clear();
    active_deadline_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    message = rollback_ok ?
      "failed to enter hole_pass mode; rolled back to normal" :
      "failed to enter hole_pass mode; rollback to normal was degraded";
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    return false;
  }

  hole_mode_active_ = true;
  active_owner_id_ = request.owner_id;
  active_hole_id_ = request.hole_id;
  active_hole_cmd_ = validHoleCommand(request.hole_cmd) ?
    request.hole_cmd : sentry_nav_interfaces::msg::HolePassCmd::HOLE_LOWER;
  active_v_yaw_ = std::isfinite(request.v_yaw) ? request.v_yaw : 0.0F;
  const double timeout = request.watchdog_timeout_sec > 0.0F ?
    static_cast<double>(request.watchdog_timeout_sec) : default_watchdog_timeout_sec_;
  active_deadline_ = now() + rclcpp::Duration::from_seconds(std::max(0.5, timeout));
  watchdog_reported_ = false;
  message = "entered hole_pass mode";
  publishHoleCommand();
  return ok;
}

bool NavigationModeManagerNode::restoreNormal(const std::string & owner_id, std::string & message)
{
  if (!hole_mode_active_) {
    message = "already normal";
    return true;
  }
  if (!owner_id.empty() && owner_id != active_owner_id_) {
    message = "owner mismatch: active='" + active_owner_id_ + "' request='" + owner_id + "'";
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
    return false;
  }

  const bool ok = applyNormalModeServices();
  if (!ok) {
    message = "failed to restore normal mode";
    RCLCPP_WARN(get_logger(), "NavigationModeManager normal-mode restore failed; keeping active state");
    return false;
  }

  hole_mode_active_ = false;
  active_hole_cmd_ = sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE;
  active_v_yaw_ = 0.0F;
  active_owner_id_.clear();
  active_hole_id_.clear();
  active_deadline_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  watchdog_reported_ = false;
  publishHoleCommand();
  message = "restored normal mode";
  return true;
}

bool NavigationModeManagerNode::applyNormalModeServices()
{
  bool ok = true;
  for (const auto & service : semantic_layer_services_) {
    ok = callSemanticLayer(service, "normal") && ok;
  }
  return ok;
}

void NavigationModeManagerNode::watchdogCallback()
{
  if (!hole_mode_active_ || active_deadline_.nanoseconds() == 0) {
    return;
  }
  if (now() <= active_deadline_) {
    return;
  }
  if (watchdog_restore_normal_) {
    RCLCPP_ERROR(
      get_logger(),
      "NavigationModeManager watchdog restoring normal mode owner='%s' hole='%s'",
      active_owner_id_.c_str(), active_hole_id_.c_str());
    std::string message;
    restoreNormal(active_owner_id_, message);
    return;
  }

  if (!watchdog_reported_) {
    RCLCPP_ERROR(
      get_logger(),
      "NavigationModeManager watchdog expired, keeping last hole command owner='%s' hole='%s' "
      "cmd=%u v_yaw=%.3f",
      active_owner_id_.c_str(), active_hole_id_.c_str(), active_hole_cmd_, active_v_yaw_);
    watchdog_reported_ = true;
  }
  active_deadline_ = now() +
    rclcpp::Duration::from_seconds(std::max(0.5, default_watchdog_timeout_sec_));
}

void NavigationModeManagerNode::holeCommandTimerCallback()
{
  publishHoleCommand();
  publishModeStatus();
}

void NavigationModeManagerNode::publishHoleCommand()
{
  if (!hole_cmd_pub_) {
    return;
  }

  sentry_nav_interfaces::msg::HolePassCmd msg;
  msg.hole_cmd = hole_mode_active_ ?
    active_hole_cmd_ : sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE;
  msg.v_yaw = hole_mode_active_ ? active_v_yaw_ : 0.0F;
  hole_cmd_pub_->publish(msg);
}

void NavigationModeManagerNode::publishModeStatus()
{
  if (!mode_pub_) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = hole_mode_active_ ? "hole_pass" : "normal";
  mode_pub_->publish(msg);
}

bool NavigationModeManagerNode::callSemanticLayer(
  const std::string & service,
  const std::string & mode)
{
  if (service.empty()) {
    return true;
  }
  auto client = create_client<SetSemanticLayerMode>(
    service, rmw_qos_profile_services_default, client_callback_group_);
  const auto timeout = std::chrono::duration<double>(std::max(0.01, service_timeout_sec_));
  if (!client->wait_for_service(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout))) {
    RCLCPP_WARN(get_logger(), "semantic layer service unavailable: %s", service.c_str());
    return false;
  }
  auto request = std::make_shared<SetSemanticLayerMode::Request>();
  request->mode = mode;
  auto future = client->async_send_request(request);
  if (client_executor_.spin_until_future_complete(future, timeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(get_logger(), "semantic layer service timed out: %s", service.c_str());
    return false;
  }
  const auto response = future.get();
  if (!response->success) {
    RCLCPP_WARN(
      get_logger(), "semantic layer service rejected %s mode '%s': %s",
      service.c_str(), mode.c_str(), response->message.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(), "semantic layer service accepted %s mode '%s': %s",
      service.c_str(), mode.c_str(), response->message.c_str());
  }
  return response->success;
}

std::vector<std::string> NavigationModeManagerNode::parseList(const std::string & text) const
{
  std::vector<std::string> out;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, ';')) {
    const auto begin = item.find_first_not_of(" \t\n\r");
    if (begin == std::string::npos) {
      continue;
    }
    const auto end = item.find_last_not_of(" \t\n\r");
    out.push_back(item.substr(begin, end - begin + 1));
  }
  return out;
}

}  // namespace hole_pass_controller

RCLCPP_COMPONENTS_REGISTER_NODE(hole_pass_controller::NavigationModeManagerNode)
