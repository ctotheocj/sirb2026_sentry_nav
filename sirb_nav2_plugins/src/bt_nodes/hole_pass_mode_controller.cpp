#include "sirb_nav2_plugins/bt_nodes/hole_pass_mode_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <numeric>

#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sirb_nav2_plugins
{
namespace
{
constexpr double kEps = 1.0e-9;
}

HolePassModeController::HolePassModeController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  last_refresh_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  raise_start_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  owner_id_ = name + ":" + std::to_string(reinterpret_cast<std::uintptr_t>(this));
}

HolePassModeController::~HolePassModeController()
{
  exitHoleMode("destruct");
}

BT::NodeStatus HolePassModeController::tick()
{
  callback_group_executor_.spin_some();

  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_yaw = 0.0;
  if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
    if (state_ == ModeState::LOWERING || state_ == ModeState::RAISING) {
      refreshHoleMode(robot_yaw, false);
    }
    return BT::NodeStatus::SUCCESS;
  }

  if (state_ == ModeState::LOWERING) {
    if (pointInPolygon(robot_x, robot_y, active_exit_polygon_)) {
      if (!startRaise(robot_yaw)) {
        return BT::NodeStatus::FAILURE;
      }
    } else {
      refreshHoleMode(robot_yaw, true);
    }
    return BT::NodeStatus::SUCCESS;
  }

  if (state_ == ModeState::RAISING) {
    refreshHoleMode(robot_yaw, true);
    const double raise_duration_sec = parameterOrInput("raise_duration_sec", 1.0);
    if (raise_start_time_.nanoseconds() != 0 &&
      (node_->now() - raise_start_time_).seconds() >= std::max(0.0, raise_duration_sec))
    {
      exitHoleMode("raise complete");
    }
    return BT::NodeStatus::SUCCESS;
  }

  if (state_ == ModeState::WAIT_CLEAR) {
    if (lockedHoleCleared(robot_x, robot_y)) {
      state_ = ModeState::IDLE;
      locked_hole_id_.clear();
    }
    return BT::NodeStatus::SUCCESS;
  }

  const auto trigger = findTrigger(loadHoles(), robot_x, robot_y);
  if (trigger.valid) {
    if (!enterHoleMode(trigger, robot_yaw)) {
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::SUCCESS;
}

void HolePassModeController::halt()
{
  exitHoleMode("halt");
  resetStatus();
}

std::vector<HolePassModeController::Hole> HolePassModeController::loadHoles()
{
  const std::string prefix = paramPrefix();
  if (holes_loaded_ && loaded_param_prefix_ == prefix) {
    return holes_;
  }

  loaded_param_prefix_ = prefix;
  holes_.clear();
  holes_loaded_ = true;

  const std::string ids_param = prefix + ".hole_ids";
  if (!node_->has_parameter(ids_param)) {
    node_->declare_parameter(ids_param, std::vector<std::string>{});
  }
  const auto ids = node_->get_parameter(ids_param).as_string_array();
  for (const auto & id : ids) {
    const std::string a_param = prefix + ".holes." + id + ".port_a_polygon";
    const std::string b_param = prefix + ".holes." + id + ".port_b_polygon";
    if (!node_->has_parameter(a_param)) {
      node_->declare_parameter(a_param, std::vector<double>{});
    }
    if (!node_->has_parameter(b_param)) {
      node_->declare_parameter(b_param, std::vector<double>{});
    }

    Hole hole;
    hole.id = id;
    hole.a = orderedPolygon(node_->get_parameter(a_param).as_double_array());
    hole.b = orderedPolygon(node_->get_parameter(b_param).as_double_array());
    if (!validPolygon(hole.a) || !validPolygon(hole.b)) {
      RCLCPP_WARN_ONCE(
        node_->get_logger(),
        "HolePassModeController: hole '%s' is invalid in bt_navigator params",
        id.c_str());
      continue;
    }
    holes_.push_back(hole);
  }
  return holes_;
}

HolePassModeController::Trigger HolePassModeController::findTrigger(
  const std::vector<Hole> & holes,
  double robot_x,
  double robot_y)
{
  Trigger trigger;
  if (holes.empty()) {
    return trigger;
  }

  for (const auto & hole : holes) {
    const bool in_a = pointInPolygon(robot_x, robot_y, hole.a);
    const bool in_b = pointInPolygon(robot_x, robot_y, hole.b);
    if (!in_a && !in_b) {
      continue;
    }

    const auto & entry = in_a ? hole.a : hole.b;
    const auto & exit = in_a ? hole.b : hole.a;
    trigger.valid = true;
    trigger.hole_id = hole.id;
    trigger.entry_port = in_a ? "A" : "B";
    trigger.entry_polygon = entry;
    trigger.exit_polygon = exit;
    trigger.path_yaw = std::atan2(
      polygonCenterY(exit) - polygonCenterY(entry),
      polygonCenterX(exit) - polygonCenterX(entry));
    return trigger;
  }

  return trigger;
}

bool HolePassModeController::getRobotPose(double & x, double & y, double & yaw)
{
  std::string global_frame = "map";
  std::string robot_frame = "gimbal_yaw_fake";
  getInput("global_frame", global_frame);
  getInput("robot_frame", robot_frame);
  try {
    const auto tf = tf_buffer_->lookupTransform(global_frame, robot_frame, tf2::TimePointZero);
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    yaw = tf2::getYaw(tf.transform.rotation);
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

bool HolePassModeController::enterHoleMode(const Trigger & trigger, double robot_yaw)
{
  double watchdog_timeout_sec = 30.0;
  getInput("watchdog_timeout_sec", watchdog_timeout_sec);
  const double yaw_offset_deg = parameterOrInput("yaw_offset_deg", 0.0);
  active_target_yaw_ = normalizeAngle(trigger.path_yaw + yaw_offset_deg * M_PI / 180.0);
  active_v_yaw_ = activeVYaw(robot_yaw);
  if (!callSetNavigationMode(
      "hole_pass", trigger.hole_id, sentry_nav_interfaces::msg::HolePassCmd::HOLE_LOWER,
      active_v_yaw_, watchdog_timeout_sec))
  {
    return false;
  }

  state_ = ModeState::LOWERING;
  active_hole_id_ = trigger.hole_id;
  locked_hole_id_.clear();
  active_exit_polygon_ = trigger.exit_polygon;
  last_refresh_time_ = node_->now();
  raise_start_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  RCLCPP_INFO(
    node_->get_logger(), "HolePassModeController: entered hole_pass hole='%s' entry='%s' "
    "target_yaw=%.3f v_yaw=%.3f",
    trigger.hole_id.c_str(), trigger.entry_port.c_str(), active_target_yaw_, active_v_yaw_);
  return true;
}

bool HolePassModeController::startRaise(double robot_yaw)
{
  double watchdog_timeout_sec = 30.0;
  getInput("watchdog_timeout_sec", watchdog_timeout_sec);
  active_v_yaw_ = activeVYaw(robot_yaw);
  if (!callSetNavigationMode(
      "hole_pass", active_hole_id_, sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE,
      active_v_yaw_, watchdog_timeout_sec))
  {
    return false;
  }

  state_ = ModeState::RAISING;
  raise_start_time_ = node_->now();
  last_refresh_time_ = raise_start_time_;
  RCLCPP_INFO(
    node_->get_logger(), "HolePassModeController: exit region reached, raising hole='%s'",
    active_hole_id_.c_str());
  return true;
}

void HolePassModeController::refreshHoleMode(double robot_yaw, bool update_yaw_command)
{
  double refresh_period_sec = 0.05;
  double watchdog_timeout_sec = 30.0;
  const uint8_t hole_cmd = state_ == ModeState::RAISING ?
    sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE :
    sentry_nav_interfaces::msg::HolePassCmd::HOLE_LOWER;
  getInput("refresh_period_sec", refresh_period_sec);
  getInput("watchdog_timeout_sec", watchdog_timeout_sec);
  if (last_refresh_time_.nanoseconds() != 0 &&
    (node_->now() - last_refresh_time_).seconds() < std::max(0.01, refresh_period_sec))
  {
    return;
  }
  if (update_yaw_command) {
    active_v_yaw_ = activeVYaw(robot_yaw);
  }
  if (callSetNavigationMode("hole_pass", active_hole_id_, hole_cmd, active_v_yaw_, watchdog_timeout_sec)) {
    last_refresh_time_ = node_->now();
  }
}

void HolePassModeController::exitHoleMode(const char * reason)
{
  if (state_ == ModeState::IDLE) {
    return;
  }
  const bool wait_until_clear = std::string(reason) == "raise complete";
  const bool restored = callSetNavigationMode(
    "normal", "", sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE, 0.0, 0.0);
  if (!restored && wait_until_clear) {
    RCLCPP_WARN(
      node_->get_logger(),
      "HolePassModeController: failed to restore normal mode after raise; will retry");
    return;
  }
  RCLCPP_INFO(
    node_->get_logger(), "HolePassModeController: restored normal mode: %s", reason);
  state_ = wait_until_clear ? ModeState::WAIT_CLEAR : ModeState::IDLE;
  locked_hole_id_ = wait_until_clear ? active_hole_id_ : "";
  active_hole_id_.clear();
  active_target_yaw_ = 0.0;
  active_v_yaw_ = 0.0;
  active_exit_polygon_.clear();
  raise_start_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
}

bool HolePassModeController::lockedHoleCleared(double robot_x, double robot_y) const
{
  if (locked_hole_id_.empty()) {
    return true;
  }
  for (const auto & hole : holes_) {
    if (hole.id != locked_hole_id_) {
      continue;
    }
    return !pointInPolygon(robot_x, robot_y, hole.a) && !pointInPolygon(robot_x, robot_y, hole.b);
  }
  return true;
}

double HolePassModeController::activeVYaw(double robot_yaw)
{
  const double yaw_kp = parameterOrInput("yaw_kp", 2.5);
  const double max_v_yaw = std::max(0.0, parameterOrInput("max_v_yaw", 1.8));
  const double yaw_error = normalizeAngle(active_target_yaw_ - robot_yaw);
  return std::clamp(yaw_kp * yaw_error, -max_v_yaw, max_v_yaw);
}

std::string HolePassModeController::paramPrefix() const
{
  std::string prefix = "hole_pass";
  getInput("param_prefix", prefix);
  return prefix;
}

double HolePassModeController::parameterOrInput(const std::string & key, double default_value)
{
  double value = default_value;
  getInput(key, value);

  const std::string parameter_name = paramPrefix() + "." + key;
  if (!node_->has_parameter(parameter_name)) {
    node_->declare_parameter(parameter_name, value);
  }

  return node_->get_parameter(parameter_name).as_double();
}

bool HolePassModeController::callSetNavigationMode(
  const std::string & mode,
  const std::string & hole_id,
  uint8_t hole_cmd,
  double v_yaw,
  double watchdog_timeout_sec)
{
  std::string service = "navigation_mode_manager/set_navigation_mode";
  double timeout_sec = 0.5;
  getInput("mode_service", service);
  getInput("service_timeout", timeout_sec);
  if (service.empty()) {
    return false;
  }

  auto client = node_->create_client<sentry_nav_interfaces::srv::SetNavigationMode>(
    service, rmw_qos_profile_services_default, callback_group_);
  const auto timeout = std::chrono::duration<double>(std::max(0.01, timeout_sec));
  if (!client->wait_for_service(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout))) {
    RCLCPP_WARN(
      node_->get_logger(), "HolePassModeController: service '%s' unavailable", service.c_str());
    return false;
  }

  auto request = std::make_shared<sentry_nav_interfaces::srv::SetNavigationMode::Request>();
  request->mode = mode;
  request->owner_id = owner_id_;
  request->hole_id = hole_id;
  request->hole_cmd = hole_cmd;
  request->v_yaw = static_cast<float>(v_yaw);
  request->watchdog_timeout_sec = static_cast<float>(watchdog_timeout_sec);
  auto future = client->async_send_request(request);
  const auto result = callback_group_executor_.spin_until_future_complete(future, timeout);
  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_WARN(node_->get_logger(), "HolePassModeController: service '%s' timed out", service.c_str());
    return false;
  }
  const auto response = future.get();
  if (!response->success) {
    RCLCPP_WARN(
      node_->get_logger(), "HolePassModeController: service '%s' rejected mode '%s': %s",
      service.c_str(), mode.c_str(), response->message.c_str());
    return false;
  }
  return true;
}

std::vector<double> HolePassModeController::orderedPolygon(const std::vector<double> & polygon) const
{
  if (!validPolygon(polygon)) {
    return polygon;
  }

  const size_t n = polygon.size() / 2;
  double cx = 0.0;
  double cy = 0.0;
  for (size_t i = 0; i < n; ++i) {
    cx += polygon[2 * i];
    cy += polygon[2 * i + 1];
  }
  cx /= static_cast<double>(n);
  cy /= static_cast<double>(n);

  std::vector<size_t> indices(n);
  std::iota(indices.begin(), indices.end(), 0);
  std::sort(indices.begin(), indices.end(), [&](size_t lhs, size_t rhs) {
    const double la = std::atan2(polygon[2 * lhs + 1] - cy, polygon[2 * lhs] - cx);
    const double ra = std::atan2(polygon[2 * rhs + 1] - cy, polygon[2 * rhs] - cx);
    return la < ra;
  });

  std::vector<double> ordered;
  ordered.reserve(polygon.size());
  for (const size_t index : indices) {
    ordered.push_back(polygon[2 * index]);
    ordered.push_back(polygon[2 * index + 1]);
  }
  return ordered;
}

bool HolePassModeController::validPolygon(const std::vector<double> & polygon) const
{
  return polygon.size() >= 6 && polygon.size() % 2 == 0;
}

bool HolePassModeController::pointInPolygon(
  double x, double y, const std::vector<double> & polygon) const
{
  if (!validPolygon(polygon)) {
    return false;
  }
  bool inside = false;
  const size_t n = polygon.size() / 2;
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = polygon[2 * i];
    const double yi = polygon[2 * i + 1];
    const double xj = polygon[2 * j];
    const double yj = polygon[2 * j + 1];
    if (distancePointToSegment(x, y, xj, yj, xi, yi) < 1.0e-6) {
      return true;
    }
    const bool crosses = ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / (yj - yi + kEps) + xi);
    if (crosses) {
      inside = !inside;
    }
  }
  return inside;
}

double HolePassModeController::distancePointToSegment(
  double px, double py, double ax, double ay, double bx, double by) const
{
  const double vx = bx - ax;
  const double vy = by - ay;
  const double wx = px - ax;
  const double wy = py - ay;
  const double length_sq = vx * vx + vy * vy;
  if (length_sq < kEps) {
    return std::hypot(px - ax, py - ay);
  }
  const double t = std::clamp((wx * vx + wy * vy) / length_sq, 0.0, 1.0);
  return std::hypot(px - (ax + t * vx), py - (ay + t * vy));
}

double HolePassModeController::polygonCenterX(const std::vector<double> & polygon) const
{
  double x = 0.0;
  const size_t n = polygon.size() / 2;
  for (size_t i = 0; i < n; ++i) {
    x += polygon[2 * i];
  }
  return n == 0 ? 0.0 : x / static_cast<double>(n);
}

double HolePassModeController::polygonCenterY(const std::vector<double> & polygon) const
{
  double y = 0.0;
  const size_t n = polygon.size() / 2;
  for (size_t i = 0; i < n; ++i) {
    y += polygon[2 * i + 1];
  }
  return n == 0 ? 0.0 : y / static_cast<double>(n);
}

double HolePassModeController::normalizeAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace sirb_nav2_plugins
