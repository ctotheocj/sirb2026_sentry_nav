#include "hole_pass_controller/navigation_mode_manager_node.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>

#include "rclcpp_components/register_node_macro.hpp"

namespace hole_pass_controller
{

NavigationModeManagerNode::NavigationModeManagerNode(const rclcpp::NodeOptions & options)
: Node("navigation_mode_manager", options)
{
  semantic_layer_services_ = parseList(declare_parameter<std::string>(
    "semantic_layer_services",
    "global_costmap/occupancy_grid_layer/set_semantic_layer_mode;"
    "local_costmap/occupancy_grid_layer/set_semantic_layer_mode"));
  legacy_layer_services_ = parseList(declare_parameter<std::string>(
    "legacy_layer_services",
    "global_costmap/occupancy_grid_layer/set_enabled;"
    "local_costmap/occupancy_grid_layer/set_enabled"));
  replan_suppression_service_ = declare_parameter<std::string>(
    "replan_suppression_service", "");
  trajectory_policy_service_ = declare_parameter<std::string>(
    "trajectory_policy_service", "trajectory_manager/set_hole_collision_policy");
  global_costmap_node_ = declare_parameter<std::string>(
    "global_costmap_node", "global_costmap/global_costmap");
  local_costmap_node_ = declare_parameter<std::string>(
    "local_costmap_node", "local_costmap/local_costmap");
  normal_global_inflation_radius_ = declare_parameter<double>(
    "normal_global_inflation_radius", 0.55);
  normal_local_inflation_radius_ = declare_parameter<double>(
    "normal_local_inflation_radius", 0.8);
  hole_global_inflation_radius_ = declare_parameter<double>(
    "hole_global_inflation_radius", 0.10);
  hole_local_inflation_radius_ = declare_parameter<double>(
    "hole_local_inflation_radius", 0.10);
  service_timeout_sec_ = declare_parameter<double>("service_timeout_sec", 0.2);
  default_watchdog_timeout_sec_ = declare_parameter<double>("watchdog_timeout_sec", 8.0);
  use_inflation_radius_switch_ = declare_parameter<bool>("use_inflation_radius_switch", true);
  use_semantic_layer_mode_ = declare_parameter<bool>("use_semantic_layer_mode", true);
  use_trajectory_policy_switch_ = declare_parameter<bool>("use_trajectory_policy_switch", true);

  client_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_executor_.add_callback_group(client_callback_group_, get_node_base_interface());

  service_ = create_service<SetNavigationMode>(
    "navigation_mode_manager/set_navigation_mode",
    std::bind(
      &NavigationModeManagerNode::handleSetMode, this,
      std::placeholders::_1, std::placeholders::_2));
  hole_mode_pub_ =
    create_publisher<std_msgs::msg::Bool>("navigation_mode_manager/hole_mode_active", 1);
  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&NavigationModeManagerNode::watchdogCallback, this));

  RCLCPP_INFO(get_logger(), "NavigationModeManager ready");
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

bool NavigationModeManagerNode::enterHolePass(
  const SetNavigationMode::Request & request, std::string & message)
{
  if (request.owner_id.empty()) {
    message = "owner_id is empty";
    return false;
  }

  bool ok = true;
  if (use_semantic_layer_mode_) {
    for (const auto & service : semantic_layer_services_) {
      ok = callSemanticLayer(service, "hole_pass", request.frame_id, request.corridor) && ok;
    }
  } else {
    for (const auto & service : legacy_layer_services_) {
      ok = callSetBool(service, false) && ok;
    }
  }
  if (use_inflation_radius_switch_) {
    ok = setInflationRadius(global_costmap_node_, hole_global_inflation_radius_) && ok;
    ok = setInflationRadius(local_costmap_node_, hole_local_inflation_radius_) && ok;
  }
  if (use_trajectory_policy_switch_ && !trajectory_policy_service_.empty()) {
    ok = callSetBool(trajectory_policy_service_, true) && ok;
  }
  if (!replan_suppression_service_.empty()) {
    ok = callSetBool(replan_suppression_service_, true) && ok;
  }

  hole_mode_active_ = true;
  active_owner_id_ = request.owner_id;
  active_hole_id_ = request.hole_id;
  const double timeout = request.watchdog_timeout_sec > 0.0F ?
    static_cast<double>(request.watchdog_timeout_sec) : default_watchdog_timeout_sec_;
  active_deadline_ = now() + rclcpp::Duration::from_seconds(std::max(0.5, timeout));
  setHoleModeBlackboardHint(true);
  message = ok ? "entered hole_pass mode" : "entered hole_pass mode with degraded services";
  RCLCPP_WARN_EXPRESSION(
    get_logger(), !ok, "NavigationModeManager entered degraded hole_pass mode");
  return ok;
}

bool NavigationModeManagerNode::restoreNormal(const std::string & owner_id, std::string & message)
{
  if (!hole_mode_active_) {
    message = "already normal";
    setHoleModeBlackboardHint(false);
    return true;
  }
  if (!owner_id.empty() && owner_id != active_owner_id_) {
    message = "owner mismatch: active='" + active_owner_id_ + "' request='" + owner_id + "'";
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
    return false;
  }

  bool ok = true;
  if (use_semantic_layer_mode_) {
    geometry_msgs::msg::Polygon empty;
    for (const auto & service : semantic_layer_services_) {
      ok = callSemanticLayer(service, "normal", "map", empty) && ok;
    }
  } else {
    for (const auto & service : legacy_layer_services_) {
      ok = callSetBool(service, true) && ok;
    }
  }
  if (use_inflation_radius_switch_) {
    ok = setInflationRadius(global_costmap_node_, normal_global_inflation_radius_) && ok;
    ok = setInflationRadius(local_costmap_node_, normal_local_inflation_radius_) && ok;
  }
  if (use_trajectory_policy_switch_ && !trajectory_policy_service_.empty()) {
    ok = callSetBool(trajectory_policy_service_, false) && ok;
  }
  if (!replan_suppression_service_.empty()) {
    ok = callSetBool(replan_suppression_service_, false) && ok;
  }

  hole_mode_active_ = false;
  active_owner_id_.clear();
  active_hole_id_.clear();
  active_deadline_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  setHoleModeBlackboardHint(false);
  message = ok ? "restored normal mode" : "restored normal mode with degraded services";
  RCLCPP_WARN_EXPRESSION(
    get_logger(), !ok, "NavigationModeManager restored degraded normal mode");
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
  RCLCPP_ERROR(
    get_logger(),
    "NavigationModeManager watchdog restoring normal mode owner='%s' hole='%s'",
    active_owner_id_.c_str(), active_hole_id_.c_str());
  std::string message;
  restoreNormal(active_owner_id_, message);
}

bool NavigationModeManagerNode::callSemanticLayer(
  const std::string & service,
  const std::string & mode,
  const std::string & frame_id,
  const geometry_msgs::msg::Polygon & corridor)
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
  request->frame_id = frame_id.empty() ? "map" : frame_id;
  request->corridor = corridor;
  auto future = client->async_send_request(request);
  if (client_executor_.spin_until_future_complete(future, timeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(get_logger(), "semantic layer service timed out: %s", service.c_str());
    return false;
  }
  return future.get()->success;
}

bool NavigationModeManagerNode::callSetBool(const std::string & service, bool value)
{
  if (service.empty()) {
    return true;
  }
  auto client = create_client<std_srvs::srv::SetBool>(
    service, rmw_qos_profile_services_default, client_callback_group_);
  const auto timeout = std::chrono::duration<double>(std::max(0.01, service_timeout_sec_));
  if (!client->wait_for_service(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout))) {
    RCLCPP_WARN(get_logger(), "SetBool service unavailable: %s", service.c_str());
    return false;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = value;
  auto future = client->async_send_request(request);
  if (client_executor_.spin_until_future_complete(future, timeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(get_logger(), "SetBool service timed out: %s", service.c_str());
    return false;
  }
  return future.get()->success;
}

bool NavigationModeManagerNode::setInflationRadius(const std::string & node_name, double radius)
{
  if (node_name.empty()) {
    return true;
  }
  const std::string param_name = "inflation_layer.inflation_radius";
  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
    get_node_base_interface(),
    get_node_topics_interface(),
    get_node_graph_interface(),
    get_node_services_interface(),
    node_name,
    rmw_qos_profile_parameters,
    client_callback_group_);
  const auto timeout = std::chrono::duration<double>(std::max(0.01, service_timeout_sec_));
  if (!parameters_client->wait_for_service(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout))) {
    RCLCPP_WARN(get_logger(), "parameter service unavailable: %s", node_name.c_str());
    return false;
  }
  auto future = parameters_client->set_parameters({rclcpp::Parameter(param_name, radius)});
  if (client_executor_.spin_until_future_complete(future, timeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(get_logger(), "parameter set timed out: %s.%s", node_name.c_str(), param_name.c_str());
    return false;
  }
  const auto results = future.get();
  return !results.empty() && results.front().successful;
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

void NavigationModeManagerNode::setHoleModeBlackboardHint(bool active)
{
  // The authoritative state is in this manager. The BT blackboard flag is still
  // set by HolePassScope because it is local to bt_navigator.
  if (!hole_mode_pub_) {
    return;
  }
  std_msgs::msg::Bool msg;
  msg.data = active;
  hole_mode_pub_->publish(msg);
}

}  // namespace hole_pass_controller

RCLCPP_COMPONENTS_REGISTER_NODE(hole_pass_controller::NavigationModeManagerNode)
