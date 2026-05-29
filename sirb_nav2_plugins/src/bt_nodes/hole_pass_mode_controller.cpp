#include "sirb_nav2_plugins/bt_nodes/hole_pass_mode_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
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
  last_status_sync_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  raise_start_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  last_port_touch_.stamp = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  owner_id_ = name + ":" + std::to_string(reinterpret_cast<std::uintptr_t>(this));
}

HolePassModeController::~HolePassModeController()
{
}

BT::NodeStatus HolePassModeController::tick()
{
  callback_group_executor_.spin_some();
  syncActiveModeFromManager();
  publishEffectiveTargets();

  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_yaw = 0.0;
  if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
    if (state_ == ModeState::LOWERING || state_ == ModeState::RAISING ||
      state_ == ModeState::ROLLING_BACK || state_ == ModeState::RAISING_AT_ENTRY)
    {
      refreshHoleMode(robot_yaw, false, active_pass_progress_);
      return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::SUCCESS;
  }

  const auto holes = loadHoles();
  updatePortTouch(holes, robot_x, robot_y);

  if (state_ == ModeState::LOWERING) {
    const bool have_active_geometry =
      validPolygon(active_entry_polygon_) && validPolygon(active_exit_polygon_);
    if (have_active_geometry) {
      active_pass_progress_ = passProgress(robot_x, robot_y);
    }
    if (have_active_geometry && exitReached(robot_x, robot_y)) {
      if (!startRaise(robot_yaw)) {
        return BT::NodeStatus::FAILURE;
      }
      publishEffectiveTargets();
      return BT::NodeStatus::SUCCESS;
    }

    double target_x = 0.0;
    double target_y = 0.0;
    const bool have_target = getNavigationTarget(target_x, target_y);
    if (have_active_geometry) {
      const auto intent = classifyGoalIntent(have_target, target_x, target_y);
      if (intent == GoalIntent::RETURN_TO_ENTRY) {
        state_ = ModeState::ROLLING_BACK;
        hold_active_target_at_exit_ = false;
        setRollbackTargets();
        refreshHoleMode(robot_yaw, true, active_pass_progress_);
        publishEffectiveTargets();
        RCLCPP_INFO(
          node_->get_logger(),
          "HolePassModeController: goal intent RETURN_TO_ENTRY, rolling back hole='%s' "
          "entry='%s' progress=%.2f",
          active_hole_id_.c_str(), active_entry_port_.c_str(), active_pass_progress_);
        return BT::NodeStatus::SUCCESS;
      }
      hold_active_target_at_exit_ = intent == GoalIntent::AMBIGUOUS &&
        boolParameterOrInput("hold_active_pass_on_ambiguous_goal", true);
      if (intent == GoalIntent::AMBIGUOUS) {
        RCLCPP_DEBUG_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "HolePassModeController: holding active pass on ambiguous goal hole='%s' progress=%.2f",
          active_hole_id_.c_str(), active_pass_progress_);
      }
    }
    refreshHoleMode(robot_yaw, have_active_geometry, active_pass_progress_);
    publishEffectiveTargets();
    return BT::NodeStatus::SUCCESS;
  }

  if (state_ == ModeState::ROLLING_BACK) {
    const bool have_active_geometry =
      validPolygon(active_entry_polygon_) && validPolygon(active_exit_polygon_);
    if (have_active_geometry) {
      active_pass_progress_ = passProgress(robot_x, robot_y);
    }
    setRollbackTargets();
    if (have_active_geometry && activeEntryReached(robot_x, robot_y)) {
      if (!startRaiseAtEntry(robot_yaw)) {
        return BT::NodeStatus::FAILURE;
      }
    } else {
      refreshHoleMode(robot_yaw, have_active_geometry, active_pass_progress_);
    }
    publishEffectiveTargets();
    return BT::NodeStatus::SUCCESS;
  }

  if (state_ == ModeState::RAISING) {
    const bool have_active_geometry =
      validPolygon(active_entry_polygon_) && validPolygon(active_exit_polygon_);
    if (have_active_geometry) {
      active_pass_progress_ = passProgress(robot_x, robot_y);
    }
    refreshHoleMode(robot_yaw, have_active_geometry, active_pass_progress_);
    const double raise_duration_sec = parameterOrInput("raise_duration_sec", 1.0);
    if (raise_start_time_.nanoseconds() != 0 &&
      (node_->now() - raise_start_time_).seconds() >= std::max(0.0, raise_duration_sec))
    {
      exitHoleMode("raise complete");
    }
    publishEffectiveTargets();
    return BT::NodeStatus::SUCCESS;
  }

  if (state_ == ModeState::RAISING_AT_ENTRY) {
    const bool have_active_geometry =
      validPolygon(active_entry_polygon_) && validPolygon(active_exit_polygon_);
    if (have_active_geometry) {
      active_pass_progress_ = passProgress(robot_x, robot_y);
    }
    if (have_active_geometry && !activeEntryReached(robot_x, robot_y)) {
      state_ = ModeState::ROLLING_BACK;
      raise_start_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
      setRollbackTargets();
      refreshHoleMode(robot_yaw, have_active_geometry, active_pass_progress_);
      publishEffectiveTargets();
      return BT::NodeStatus::SUCCESS;
    }
    refreshHoleMode(robot_yaw, have_active_geometry, active_pass_progress_);
    const double raise_duration_sec = parameterOrInput("raise_duration_sec", 1.0);
    if (raise_start_time_.nanoseconds() != 0 &&
      (node_->now() - raise_start_time_).seconds() >= std::max(0.0, raise_duration_sec))
    {
      exitHoleMode("rollback raise complete");
    }
    publishEffectiveTargets();
    return BT::NodeStatus::SUCCESS;
  }

  if (state_ == ModeState::WAIT_CLEAR) {
    if (lockedHoleCleared(robot_x, robot_y)) {
      state_ = ModeState::IDLE;
      locked_hole_id_.clear();
    }
    return BT::NodeStatus::SUCCESS;
  }

  double target_x = 0.0;
  double target_y = 0.0;
  const bool have_target = getNavigationTarget(target_x, target_y);
  const auto trigger = findTrigger(holes, robot_x, robot_y, have_target, target_x, target_y);
  if (trigger.valid) {
    if (!enterHoleMode(trigger, robot_yaw, robot_x, robot_y)) {
      return BT::NodeStatus::FAILURE;
    }
  }
  publishEffectiveTargets();
  return BT::NodeStatus::SUCCESS;
}

void HolePassModeController::halt()
{
  if (state_ == ModeState::LOWERING || state_ == ModeState::ROLLING_BACK) {
    double robot_yaw = 0.0;
    double robot_x = 0.0;
    double robot_y = 0.0;
    getRobotPose(robot_x, robot_y, robot_yaw);
    state_ = ModeState::ROLLING_BACK;
    hold_active_target_at_exit_ = false;
    active_pass_progress_ = passProgress(robot_x, robot_y);
    setRollbackTargets();
    refreshHoleMode(robot_yaw, validPolygon(active_entry_polygon_), active_pass_progress_);
    publishEffectiveTargets();
    resetStatus();
    return;
  }
  if (state_ == ModeState::RAISING || state_ == ModeState::RAISING_AT_ENTRY) {
    double robot_yaw = 0.0;
    double robot_x = 0.0;
    double robot_y = 0.0;
    getRobotPose(robot_x, robot_y, robot_yaw);
    active_pass_progress_ = passProgress(robot_x, robot_y);
    refreshHoleMode(robot_yaw, validPolygon(active_entry_polygon_), active_pass_progress_);
    publishEffectiveTargets();
    resetStatus();
    return;
  }
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
  double robot_y,
  bool have_target,
  double target_x,
  double target_y)
{
  auto trigger = findDirectTrigger(holes, robot_x, robot_y, have_target, target_x, target_y);
  if (trigger.valid) {
    return trigger;
  }

  trigger = findSplitTriggerFromTouch(holes, robot_x, robot_y, have_target, target_x, target_y);
  if (trigger.valid) {
    return trigger;
  }

  return findCorridorTrigger(holes, robot_x, robot_y, have_target, target_x, target_y);
}

HolePassModeController::Trigger HolePassModeController::findDirectTrigger(
  const std::vector<Hole> & holes,
  double robot_x,
  double robot_y,
  bool have_target,
  double target_x,
  double target_y)
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
    if (boolParameterOrInput("require_navigation_intent", true) &&
      (!have_target || !triggerMatchesNavigationIntent(entry, exit, target_x, target_y)))
    {
      continue;
    }

    trigger.valid = true;
    trigger.hole_id = hole.id;
    trigger.entry_port = in_a ? "A" : "B";
    trigger.exit_port = in_a ? "B" : "A";
    trigger.entry_polygon = entry;
    trigger.exit_polygon = exit;
    trigger.path_yaw = std::atan2(
      polygonCenterY(exit) - polygonCenterY(entry),
      polygonCenterX(exit) - polygonCenterX(entry));
    RCLCPP_INFO(
      node_->get_logger(),
      "HolePassModeController: direct hole trigger hole='%s' entry='%s' exit='%s'",
      trigger.hole_id.c_str(), trigger.entry_port.c_str(), trigger.exit_port.c_str());
    return trigger;
  }

  return trigger;
}

HolePassModeController::Trigger HolePassModeController::findSplitTriggerFromTouch(
  const std::vector<Hole> & holes,
  double robot_x,
  double robot_y,
  bool have_target,
  double target_x,
  double target_y)
{
  Trigger trigger;
  if (!boolParameterOrInput("allow_split_hole_pass", true) || !last_port_touch_.valid ||
    !have_target)
  {
    return trigger;
  }

  const double grace_sec = std::max(0.0, parameterOrInput("port_touch_grace_sec", 3.0));
  const double grace_dist = std::max(0.0, parameterOrInput("port_touch_grace_dist", 1.2));
  if (last_port_touch_.stamp.nanoseconds() == 0 ||
    (node_->now() - last_port_touch_.stamp).seconds() > grace_sec ||
    std::hypot(robot_x - last_port_touch_.x, robot_y - last_port_touch_.y) > grace_dist)
  {
    return trigger;
  }

  const std::string exit_port = oppositePort(last_port_touch_.port);
  if (exit_port.empty() ||
    !buildTriggerFromPorts(last_port_touch_.hole_id, last_port_touch_.port, exit_port, trigger))
  {
    return Trigger{};
  }

  if (boolParameterOrInput("require_navigation_intent", true) &&
    !triggerMatchesNavigationIntent(trigger.entry_polygon, trigger.exit_polygon, target_x, target_y))
  {
    return Trigger{};
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "HolePassModeController: split hole trigger hole='%s' entry='%s' exit='%s'",
    trigger.hole_id.c_str(), trigger.entry_port.c_str(), trigger.exit_port.c_str());
  (void)holes;
  return trigger;
}

HolePassModeController::Trigger HolePassModeController::findCorridorTrigger(
  const std::vector<Hole> & holes,
  double robot_x,
  double robot_y,
  bool have_target,
  double target_x,
  double target_y)
{
  Trigger trigger;
  if (!have_target || !boolParameterOrInput("allow_split_hole_pass", true)) {
    return trigger;
  }

  const double lateral_margin = std::max(0.0, parameterOrInput("corridor_lateral_margin", 0.6));
  for (const auto & hole : holes) {
    const double ax = polygonCenterX(hole.a);
    const double ay = polygonCenterY(hole.a);
    const double bx = polygonCenterX(hole.b);
    const double by = polygonCenterY(hole.b);
    const double ab_x = bx - ax;
    const double ab_y = by - ay;
    const double ab_len_sq = ab_x * ab_x + ab_y * ab_y;
    if (ab_len_sq < kEps) {
      continue;
    }
    const double projection =
      ((robot_x - ax) * ab_x + (robot_y - ay) * ab_y) / ab_len_sq;
    if (projection <= 0.0 || projection >= 1.0) {
      continue;
    }
    const double lateral = distancePointToSegment(robot_x, robot_y, ax, ay, bx, by);
    if (lateral > lateral_margin) {
      continue;
    }

    if (triggerMatchesNavigationIntent(hole.a, hole.b, target_x, target_y)) {
      buildTriggerFromPorts(hole.id, "A", "B", trigger);
    } else if (triggerMatchesNavigationIntent(hole.b, hole.a, target_x, target_y)) {
      buildTriggerFromPorts(hole.id, "B", "A", trigger);
    }
    if (trigger.valid) {
      RCLCPP_INFO(
        node_->get_logger(),
        "HolePassModeController: corridor hole trigger hole='%s' entry='%s' exit='%s'",
        trigger.hole_id.c_str(), trigger.entry_port.c_str(), trigger.exit_port.c_str());
      return trigger;
    }
  }

  return trigger;
}

void HolePassModeController::updatePortTouch(
  const std::vector<Hole> & holes, double robot_x, double robot_y)
{
  if (state_ != ModeState::IDLE && state_ != ModeState::WAIT_CLEAR) {
    return;
  }
  for (const auto & hole : holes) {
    const bool in_a = pointInPolygon(robot_x, robot_y, hole.a);
    const bool in_b = pointInPolygon(robot_x, robot_y, hole.b);
    if (!in_a && !in_b) {
      continue;
    }

    const std::string port = in_a ? "A" : "B";
    if (!last_port_touch_.valid || last_port_touch_.hole_id != hole.id ||
      last_port_touch_.port != port)
    {
      RCLCPP_INFO(
        node_->get_logger(),
        "HolePassModeController: touched hole port hole='%s' port='%s'",
        hole.id.c_str(), port.c_str());
    }
    last_port_touch_.valid = true;
    last_port_touch_.hole_id = hole.id;
    last_port_touch_.port = port;
    last_port_touch_.stamp = node_->now();
    last_port_touch_.x = robot_x;
    last_port_touch_.y = robot_y;
    return;
  }
}

bool HolePassModeController::buildTriggerFromPorts(
  const std::string & hole_id,
  const std::string & entry_port,
  const std::string & exit_port,
  Trigger & trigger)
{
  if (entry_port == exit_port || entry_port.empty() || exit_port.empty()) {
    return false;
  }
  for (const auto & hole : loadHoles()) {
    if (hole.id != hole_id) {
      continue;
    }
    const auto * entry = portPolygon(hole, entry_port);
    const auto * exit = portPolygon(hole, exit_port);
    if (entry == nullptr || exit == nullptr || !validPolygon(*entry) || !validPolygon(*exit)) {
      return false;
    }
    trigger.valid = true;
    trigger.hole_id = hole.id;
    trigger.entry_port = entry_port;
    trigger.exit_port = exit_port;
    trigger.entry_polygon = *entry;
    trigger.exit_polygon = *exit;
    trigger.path_yaw = std::atan2(
      polygonCenterY(trigger.exit_polygon) - polygonCenterY(trigger.entry_polygon),
      polygonCenterX(trigger.exit_polygon) - polygonCenterX(trigger.entry_polygon));
    return true;
  }
  return false;
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

bool HolePassModeController::getNavigationTarget(double & x, double & y)
{
  geometry_msgs::msg::PoseStamped goal;
  if (getInput("goal", goal) && !goal.header.frame_id.empty()) {
    x = goal.pose.position.x;
    y = goal.pose.position.y;
    return true;
  }

  std::vector<geometry_msgs::msg::PoseStamped> goals;
  if (getInput("goals", goals) && !goals.empty()) {
    for (auto it = goals.rbegin(); it != goals.rend(); ++it) {
      if (!it->header.frame_id.empty()) {
        x = it->pose.position.x;
        y = it->pose.position.y;
        return true;
      }
    }
    x = goals.back().pose.position.x;
    y = goals.back().pose.position.y;
    return true;
  }

  return false;
}

bool HolePassModeController::syncActiveModeFromManager()
{
  double status_refresh_period_sec = 0.25;
  getInput("status_refresh_period_sec", status_refresh_period_sec);
  if (last_status_sync_time_.nanoseconds() != 0 &&
    (node_->now() - last_status_sync_time_).seconds() <
    std::max(0.05, status_refresh_period_sec))
  {
    return true;
  }
  last_status_sync_time_ = node_->now();

  sentry_nav_interfaces::srv::GetNavigationMode::Response status;
  if (!callGetNavigationMode(status)) {
    return false;
  }
  if (!status.active || status.mode != "hole_pass" || status.hole_id.empty()) {
    if (state_ == ModeState::LOWERING || state_ == ModeState::RAISING ||
      state_ == ModeState::ROLLING_BACK || state_ == ModeState::RAISING_AT_ENTRY)
    {
      state_ = ModeState::IDLE;
      active_hole_id_.clear();
      active_entry_port_.clear();
      active_exit_port_.clear();
      active_entry_polygon_.clear();
      active_exit_polygon_.clear();
      active_target_yaw_ = 0.0;
      active_v_yaw_ = 0.0;
      active_pass_progress_ = 0.0;
      hold_active_target_at_exit_ = false;
      have_original_goal_ = false;
      have_original_goals_ = false;
      raise_start_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    }
    return true;
  }
  const bool same_owner = !status.owner_id.empty() && status.owner_id == owner_id_;
  const bool was_idle = state_ == ModeState::IDLE;
  const std::string previous_hole_id = active_hole_id_;
  active_hole_id_ = status.hole_id;
  active_v_yaw_ = status.v_yaw;
  active_pass_progress_ = std::clamp(static_cast<double>(status.pass_progress), 0.0, 1.0);
  if (state_ != ModeState::ROLLING_BACK && state_ != ModeState::RAISING_AT_ENTRY) {
    state_ = status.hole_cmd == sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE ?
      ModeState::RAISING : ModeState::LOWERING;
  }
  Trigger trigger;
  if (!status.entry_port.empty() && !status.exit_port.empty()) {
    if (!buildTriggerFromPorts(status.hole_id, status.entry_port, status.exit_port, trigger)) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "HolePassModeController: manager has invalid hole geometry hole='%s' entry='%s' exit='%s'",
        status.hole_id.c_str(), status.entry_port.c_str(), status.exit_port.c_str());
      return false;
    }
  } else if (previous_hole_id == status.hole_id &&
    validPolygon(active_entry_polygon_) && validPolygon(active_exit_polygon_) &&
    !active_entry_port_.empty() && !active_exit_port_.empty())
  {
    trigger.valid = true;
    trigger.hole_id = active_hole_id_;
    trigger.entry_port = active_entry_port_;
    trigger.exit_port = active_exit_port_;
    trigger.entry_polygon = active_entry_polygon_;
    trigger.exit_polygon = active_exit_polygon_;
    trigger.path_yaw = std::atan2(
      polygonCenterY(active_exit_polygon_) - polygonCenterY(active_entry_polygon_),
      polygonCenterX(active_exit_polygon_) - polygonCenterX(active_entry_polygon_));
  } else {
    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_yaw = 0.0;
    if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "HolePassModeController: manager is active for hole='%s' but TF is unavailable",
        status.hole_id.c_str());
      return false;
    }

    double target_x = 0.0;
    double target_y = 0.0;
    const bool have_target = getNavigationTarget(target_x, target_y);
    if (!inferActiveTriggerFromManager(
        status.hole_id, status.hole_cmd, robot_x, robot_y, have_target, target_x, target_y,
        trigger))
    {
      return false;
    }
  }

  active_entry_port_ = trigger.entry_port;
  active_exit_port_ = trigger.exit_port;
  active_entry_polygon_ = trigger.entry_polygon;
  active_exit_polygon_ = trigger.exit_polygon;
  active_target_yaw_ = targetYawFromParameter();
  if (!same_owner || was_idle) {
    last_refresh_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  }
  if ((state_ == ModeState::RAISING || state_ == ModeState::RAISING_AT_ENTRY) &&
    raise_start_time_.nanoseconds() == 0)
  {
    raise_start_time_ = node_->now();
  } else if (state_ == ModeState::LOWERING || state_ == ModeState::ROLLING_BACK) {
    raise_start_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  }
  if (same_owner) {
    return true;
  }

  if (callSetNavigationMode(
      "hole_pass", active_hole_id_, active_entry_port_, active_exit_port_, status.hole_cmd,
      active_v_yaw_, active_pass_progress_))
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "HolePassModeController: resumed active hole_pass hole='%s' entry='%s' exit='%s' cmd=%u",
      active_hole_id_.c_str(), active_entry_port_.c_str(), active_exit_port_.c_str(),
      static_cast<unsigned>(status.hole_cmd));
    last_refresh_time_ = node_->now();
    return true;
  }

  return false;
}

bool HolePassModeController::inferActiveTriggerFromManager(
  const std::string & hole_id,
  uint8_t hole_cmd,
  double robot_x,
  double robot_y,
  bool have_target,
  double target_x,
  double target_y,
  Trigger & trigger)
{
  const auto fill_trigger = [&](const Hole & hole, bool a_to_b) {
      trigger.valid = true;
      trigger.hole_id = hole.id;
      trigger.entry_port = a_to_b ? "A" : "B";
      trigger.exit_port = a_to_b ? "B" : "A";
      trigger.entry_polygon = a_to_b ? hole.a : hole.b;
      trigger.exit_polygon = a_to_b ? hole.b : hole.a;
      trigger.path_yaw = std::atan2(
        polygonCenterY(trigger.exit_polygon) - polygonCenterY(trigger.entry_polygon),
        polygonCenterX(trigger.exit_polygon) - polygonCenterX(trigger.entry_polygon));
    };

  for (const auto & hole : loadHoles()) {
    if (hole.id != hole_id) {
      continue;
    }

    if (have_target) {
      if (triggerMatchesNavigationIntent(hole.a, hole.b, target_x, target_y)) {
        fill_trigger(hole, true);
        return true;
      }
      if (triggerMatchesNavigationIntent(hole.b, hole.a, target_x, target_y)) {
        fill_trigger(hole, false);
        return true;
      }
    }

    const bool in_a = pointInPolygon(robot_x, robot_y, hole.a);
    const bool in_b = pointInPolygon(robot_x, robot_y, hole.b);
    if (in_a && !in_b) {
      fill_trigger(hole, hole_cmd != sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE);
      return true;
    }
    if (in_b && !in_a) {
      fill_trigger(hole, hole_cmd == sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE);
      return true;
    }

    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "HolePassModeController: manager is active for hole='%s' but entry/exit direction "
      "cannot be inferred from current pose and navigation target",
      hole_id.c_str());
    return false;
  }

  RCLCPP_WARN_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "HolePassModeController: manager is active for unknown hole='%s'",
    hole_id.c_str());
  return false;
}

bool HolePassModeController::enterHoleMode(
  const Trigger & trigger, double robot_yaw, double robot_x, double robot_y)
{
  active_target_yaw_ = targetYawFromParameter();
  active_v_yaw_ = activeVYaw(robot_yaw);
  active_entry_polygon_ = trigger.entry_polygon;
  active_exit_polygon_ = trigger.exit_polygon;
  active_pass_progress_ = passProgress(robot_x, robot_y);
  have_original_goal_ = getInput("goal", original_goal_) && !original_goal_.header.frame_id.empty();
  have_original_goals_ = getInput("goals", original_goals_) && !original_goals_.empty();
  if (!callSetNavigationMode(
      "hole_pass", trigger.hole_id, trigger.entry_port, trigger.exit_port,
      sentry_nav_interfaces::msg::HolePassCmd::HOLE_LOWER, active_v_yaw_, active_pass_progress_))
  {
    active_entry_polygon_.clear();
    active_exit_polygon_.clear();
    return false;
  }

  state_ = ModeState::LOWERING;
  active_hole_id_ = trigger.hole_id;
  active_entry_port_ = trigger.entry_port;
  active_exit_port_ = trigger.exit_port;
  locked_hole_id_.clear();
  hold_active_target_at_exit_ = false;
  setRollbackTargets();
  last_refresh_time_ = node_->now();
  raise_start_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  RCLCPP_INFO(
    node_->get_logger(), "HolePassModeController: entered hole_pass hole='%s' entry='%s' "
    "exit='%s' progress=%.2f target_yaw=%.3f v_yaw=%.3f",
    trigger.hole_id.c_str(), trigger.entry_port.c_str(), trigger.exit_port.c_str(),
    active_pass_progress_, active_target_yaw_, active_v_yaw_);
  return true;
}

bool HolePassModeController::startRaise(double robot_yaw)
{
  double robot_x = 0.0;
  double robot_y = 0.0;
  double ignored_yaw = 0.0;
  if (!getRobotPose(robot_x, robot_y, ignored_yaw) ||
    !pointInPolygon(robot_x, robot_y, active_exit_polygon_))
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "HolePassModeController: refusing to raise outside exit polygon for hole='%s'",
      active_hole_id_.c_str());
    return false;
  }

  active_v_yaw_ = activeVYaw(robot_yaw);
  if (!callSetNavigationMode(
      "hole_pass", active_hole_id_, active_entry_port_, active_exit_port_,
      sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE, active_v_yaw_, 1.0))
  {
    return false;
  }

  state_ = ModeState::RAISING;
  hold_active_target_at_exit_ = false;
  active_pass_progress_ = 1.0;
  raise_start_time_ = node_->now();
  last_refresh_time_ = raise_start_time_;
  RCLCPP_INFO(
    node_->get_logger(), "HolePassModeController: exit region reached, raising hole='%s'",
    active_hole_id_.c_str());
  return true;
}

bool HolePassModeController::startRaiseAtEntry(double robot_yaw)
{
  double robot_x = 0.0;
  double robot_y = 0.0;
  double ignored_yaw = 0.0;
  if (!getRobotPose(robot_x, robot_y, ignored_yaw) ||
    !pointInPolygon(robot_x, robot_y, active_entry_polygon_))
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "HolePassModeController: refusing rollback raise outside entry polygon for hole='%s'",
      active_hole_id_.c_str());
    return false;
  }

  active_v_yaw_ = activeVYaw(robot_yaw);
  if (!callSetNavigationMode(
      "hole_pass", active_hole_id_, active_entry_port_, active_exit_port_,
      sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE, active_v_yaw_, 0.0))
  {
    return false;
  }

  state_ = ModeState::RAISING_AT_ENTRY;
  hold_active_target_at_exit_ = false;
  active_pass_progress_ = 0.0;
  raise_start_time_ = node_->now();
  last_refresh_time_ = raise_start_time_;
  RCLCPP_INFO(
    node_->get_logger(), "HolePassModeController: rollback entry reached, raising hole='%s'",
    active_hole_id_.c_str());
  return true;
}

void HolePassModeController::refreshHoleMode(
  double robot_yaw, bool update_yaw_command, double progress)
{
  active_pass_progress_ = std::clamp(progress, 0.0, 1.0);
  double refresh_period_sec = 0.05;
  const uint8_t hole_cmd =
    (state_ == ModeState::RAISING || state_ == ModeState::RAISING_AT_ENTRY) ?
    sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE :
    sentry_nav_interfaces::msg::HolePassCmd::HOLE_LOWER;
  getInput("refresh_period_sec", refresh_period_sec);
  if (last_refresh_time_.nanoseconds() != 0 &&
    (node_->now() - last_refresh_time_).seconds() < std::max(0.01, refresh_period_sec))
  {
    return;
  }
  if (update_yaw_command) {
    active_v_yaw_ = activeVYaw(robot_yaw);
  }
  if (callSetNavigationMode(
      "hole_pass", active_hole_id_, active_entry_port_, active_exit_port_, hole_cmd, active_v_yaw_,
      active_pass_progress_))
  {
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
    "normal", "", "", "", sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE, 0.0, 0.0);
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
  active_entry_port_.clear();
  active_exit_port_.clear();
  active_target_yaw_ = 0.0;
  active_v_yaw_ = 0.0;
  active_pass_progress_ = 0.0;
  hold_active_target_at_exit_ = false;
  have_original_goal_ = false;
  have_original_goals_ = false;
  active_entry_polygon_.clear();
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

void HolePassModeController::publishEffectiveTargets()
{
  if (state_ == ModeState::ROLLING_BACK || state_ == ModeState::RAISING_AT_ENTRY) {
    setRollbackTargets();
    setOutput("effective_goal", rollback_goal_);
    std::vector<geometry_msgs::msg::PoseStamped> rollback_goals;
    rollback_goals.reserve(2);
    rollback_goals.push_back(rollback_goal_);
    rollback_goals.push_back(rollback_goal_);
    setOutput("effective_goals", rollback_goals);
    return;
  }

  if (state_ == ModeState::LOWERING && hold_active_target_at_exit_ &&
    validPolygon(active_exit_polygon_))
  {
    geometry_msgs::msg::PoseStamped exit_goal;
    setPortCenterTarget(active_exit_polygon_, exit_goal);
    setOutput("effective_goal", exit_goal);
    std::vector<geometry_msgs::msg::PoseStamped> exit_goals;
    exit_goals.reserve(2);
    exit_goals.push_back(exit_goal);
    exit_goals.push_back(exit_goal);
    setOutput("effective_goals", exit_goals);
    return;
  }

  geometry_msgs::msg::PoseStamped goal;
  if (getInput("goal", goal) && !goal.header.frame_id.empty()) {
    setOutput("effective_goal", goal);
  } else if (have_original_goal_) {
    setOutput("effective_goal", original_goal_);
  }

  std::vector<geometry_msgs::msg::PoseStamped> goals;
  if (getInput("goals", goals) && !goals.empty()) {
    setOutput("effective_goals", goals);
  } else if (have_original_goals_) {
    setOutput("effective_goals", original_goals_);
  }
}

void HolePassModeController::setRollbackTargets()
{
  setPortCenterTarget(active_entry_polygon_, rollback_goal_);
}

void HolePassModeController::setPortCenterTarget(
  const std::vector<double> & polygon,
  geometry_msgs::msg::PoseStamped & goal)
{
  std::string global_frame = "map";
  getInput("global_frame", global_frame);
  goal.header.frame_id = global_frame;
  goal.header.stamp = node_->now();
  goal.pose.position.x = polygonCenterX(polygon);
  goal.pose.position.y = polygonCenterY(polygon);
  goal.pose.position.z = 0.0;
  goal.pose.orientation.w = 1.0;
  goal.pose.orientation.x = 0.0;
  goal.pose.orientation.y = 0.0;
  goal.pose.orientation.z = 0.0;
}

HolePassModeController::GoalIntent HolePassModeController::classifyGoalIntent(
  bool have_target,
  double target_x,
  double target_y)
{
  if (!validPolygon(active_entry_polygon_) || !validPolygon(active_exit_polygon_)) {
    return GoalIntent::AMBIGUOUS;
  }
  if (!boolParameterOrInput("require_navigation_intent", true)) {
    return GoalIntent::SAME_PASS;
  }
  if (!have_target) {
    return GoalIntent::AMBIGUOUS;
  }

  const double hysteresis = std::max(0.0, parameterOrInput("goal_side_hysteresis", 0.2));
  if (targetMatchesDirection(
      active_entry_polygon_, active_exit_polygon_, target_x, target_y, hysteresis))
  {
    return GoalIntent::SAME_PASS;
  }
  if (targetMatchesDirection(
      active_exit_polygon_, active_entry_polygon_, target_x, target_y, hysteresis))
  {
    return GoalIntent::RETURN_TO_ENTRY;
  }
  return GoalIntent::AMBIGUOUS;
}

bool HolePassModeController::activeEntryReached(double robot_x, double robot_y) const
{
  return pointInPolygon(robot_x, robot_y, active_entry_polygon_);
}

bool HolePassModeController::exitReached(double robot_x, double robot_y)
{
  return pointInPolygon(robot_x, robot_y, active_exit_polygon_);
}

bool HolePassModeController::triggerMatchesNavigationIntent(
  const std::vector<double> & entry,
  const std::vector<double> & exit,
  double target_x,
  double target_y)
{
  const double min_goal_exit_margin = std::max(0.0, parameterOrInput("min_goal_exit_margin", 0.2));
  return targetMatchesDirection(entry, exit, target_x, target_y, min_goal_exit_margin);
}

bool HolePassModeController::targetMatchesDirection(
  const std::vector<double> & entry,
  const std::vector<double> & exit,
  double target_x,
  double target_y,
  double exit_margin)
{
  const double entry_x = polygonCenterX(entry);
  const double entry_y = polygonCenterY(entry);
  const double exit_x = polygonCenterX(exit);
  const double exit_y = polygonCenterY(exit);
  const double path_x = exit_x - entry_x;
  const double path_y = exit_y - entry_y;
  const double target_vec_x = target_x - entry_x;
  const double target_vec_y = target_y - entry_y;
  const double path_norm = std::hypot(path_x, path_y);
  const double target_norm = std::hypot(target_vec_x, target_vec_y);
  if (path_norm < kEps || target_norm < kEps) {
    return false;
  }

  const double cos_angle =
    (path_x * target_vec_x + path_y * target_vec_y) / (path_norm * target_norm);
  const double min_goal_direction_cos = parameterOrInput("min_goal_direction_cos", 0.2);
  if (cos_angle < min_goal_direction_cos) {
    return false;
  }

  const double entry_dist = std::hypot(target_x - entry_x, target_y - entry_y);
  const double exit_dist = std::hypot(target_x - exit_x, target_y - exit_y);
  return exit_dist + std::max(0.0, exit_margin) < entry_dist;
}

double HolePassModeController::passProgress(double robot_x, double robot_y) const
{
  if (!validPolygon(active_entry_polygon_) || !validPolygon(active_exit_polygon_)) {
    return active_pass_progress_;
  }
  return std::clamp(pointProjectionAlongPass(robot_x, robot_y), 0.0, 1.0);
}

double HolePassModeController::pointProjectionAlongPass(double x, double y) const
{
  const double entry_x = polygonCenterX(active_entry_polygon_);
  const double entry_y = polygonCenterY(active_entry_polygon_);
  const double exit_x = polygonCenterX(active_exit_polygon_);
  const double exit_y = polygonCenterY(active_exit_polygon_);
  const double path_x = exit_x - entry_x;
  const double path_y = exit_y - entry_y;
  const double path_len_sq = path_x * path_x + path_y * path_y;
  if (path_len_sq < kEps) {
    return 0.0;
  }
  return ((x - entry_x) * path_x + (y - entry_y) * path_y) / path_len_sq;
}

double HolePassModeController::pointLateralDistanceToActivePass(double x, double y) const
{
  if (!validPolygon(active_entry_polygon_) || !validPolygon(active_exit_polygon_)) {
    return std::numeric_limits<double>::infinity();
  }
  return distancePointToSegment(
    x, y,
    polygonCenterX(active_entry_polygon_), polygonCenterY(active_entry_polygon_),
    polygonCenterX(active_exit_polygon_), polygonCenterY(active_exit_polygon_));
}

const std::vector<double> * HolePassModeController::portPolygon(
  const Hole & hole, const std::string & port) const
{
  if (port == "A") {
    return &hole.a;
  }
  if (port == "B") {
    return &hole.b;
  }
  return nullptr;
}

std::string HolePassModeController::oppositePort(const std::string & port) const
{
  if (port == "A") {
    return "B";
  }
  if (port == "B") {
    return "A";
  }
  return "";
}

const char * HolePassModeController::goalIntentName(GoalIntent intent) const
{
  switch (intent) {
    case GoalIntent::SAME_PASS:
      return "SAME_PASS";
    case GoalIntent::RETURN_TO_ENTRY:
      return "RETURN_TO_ENTRY";
    case GoalIntent::AMBIGUOUS:
      return "AMBIGUOUS";
  }
  return "UNKNOWN";
}

double HolePassModeController::activeVYaw(double robot_yaw)
{
  const double yaw_kp = parameterOrInput("yaw_kp", 2.5);
  const double max_v_yaw = std::max(0.0, parameterOrInput("max_v_yaw", 1.8));
  const double yaw_error = normalizeAngle(active_target_yaw_ - robot_yaw);
  const double yaw_error_deg = yaw_error * 180.0 / M_PI;
  return std::clamp(yaw_kp * yaw_error_deg, -max_v_yaw, max_v_yaw);
}

double HolePassModeController::targetYawFromParameter()
{
  const double target_yaw_deg = parameterOrInput("target_yaw_deg", 0.0);
  return normalizeAngle(target_yaw_deg * M_PI / 180.0);
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

bool HolePassModeController::boolParameterOrInput(const std::string & key, bool default_value)
{
  bool value = default_value;
  getInput(key, value);

  const std::string parameter_name = paramPrefix() + "." + key;
  if (!node_->has_parameter(parameter_name)) {
    node_->declare_parameter(parameter_name, value);
  }

  return node_->get_parameter(parameter_name).as_bool();
}

bool HolePassModeController::callSetNavigationMode(
  const std::string & mode,
  const std::string & hole_id,
  const std::string & entry_port,
  const std::string & exit_port,
  uint8_t hole_cmd,
  double v_yaw,
  double pass_progress)
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
  request->entry_port = entry_port;
  request->exit_port = exit_port;
  request->hole_cmd = hole_cmd;
  request->v_yaw = static_cast<float>(v_yaw);
  request->pass_progress = static_cast<float>(std::clamp(pass_progress, 0.0, 1.0));
  request->watchdog_timeout_sec = 0.0F;
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

bool HolePassModeController::callGetNavigationMode(
  sentry_nav_interfaces::srv::GetNavigationMode::Response & response)
{
  std::string service = "navigation_mode_manager/get_navigation_mode";
  double timeout_sec = 0.5;
  getInput("mode_status_service", service);
  getInput("service_timeout", timeout_sec);
  if (service.empty()) {
    return false;
  }

  auto client = node_->create_client<sentry_nav_interfaces::srv::GetNavigationMode>(
    service, rmw_qos_profile_services_default, callback_group_);
  const auto timeout = std::chrono::duration<double>(std::max(0.01, timeout_sec));
  if (!client->wait_for_service(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout))) {
    return false;
  }

  auto request = std::make_shared<sentry_nav_interfaces::srv::GetNavigationMode::Request>();
  auto future = client->async_send_request(request);
  const auto result = callback_group_executor_.spin_until_future_complete(future, timeout);
  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_WARN(
      node_->get_logger(), "HolePassModeController: service '%s' timed out", service.c_str());
    return false;
  }
  response = *future.get();
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
