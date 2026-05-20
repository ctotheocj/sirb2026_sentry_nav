#include "hole_pass_controller/hole_pass_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <thread>

#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace hole_pass_controller
{

namespace
{
constexpr double kEpsilon = 1e-9;

bool hasParameterOverride(const rclcpp::NodeOptions & options, const std::string & name)
{
  const auto & overrides = options.parameter_overrides();
  return std::any_of(
    overrides.begin(), overrides.end(),
    [&name](const rclcpp::Parameter & parameter) {
      return parameter.get_name() == name;
    });
}
}

HolePassController::HolePassController(const rclcpp::NodeOptions & options)
: Node("hole_pass_controller", options)
{
  declare_parameter("global_frame", "map");
  declare_parameter("robot_frame", "gimbal_yaw_fake");
  declare_parameter("output_cmd_vel_topic", "cmd_vel_hole_controller");
  declare_parameter("output_cmd_vel_stamped_topic", "cmd_vel_hole_controller_stamped");
  declare_parameter("cmd_frame_id", "gimbal_yaw_fake");
  declare_parameter("trajectory_topic", "trajectory_manager/trajectory_for_mpc");
  declare_parameter("hole_pass_cmd_topic", "mpc/hole_pass_cmd");
  declare_parameter("hole_pass_state_topic", "serial/hole_pass_state");
  declare_parameter("nav_yaw_topic", "/Nav_yaw");
  declare_parameter("yaw_target_marker_topic", "hole_pass/yaw_target_marker");
  declare_parameter("pass_hole_action_name", "pass_hole");
  declare_parameter("hole_ids", std::vector<std::string>{});
  declare_parameter("prepare_distance", 1.5);
  declare_parameter("stop_distance", 0.4);
  declare_parameter("slow_speed", 0.25);
  declare_parameter("pass_speed", 0.45);
  declare_parameter("yaw_offset", 0.0);
  declare_parameter("yaw_offset_deg", 0.0);
  declare_parameter("yaw_kp", 2.5);
  declare_parameter("max_v_yaw", 1.8);
  declare_parameter("exit_raise_distance", 0.2);
  declare_parameter("exit_timeout_sec", 3.0);
  declare_parameter("lowering_timeout_sec", 2.0);
  declare_parameter("height_state_timeout_sec", 0.3);
  declare_parameter("nav_yaw_timeout_sec", 0.2);
  declare_parameter("trajectory_timeout_sec", 0.5);
  declare_parameter("tf_lookup_timeout_sec", 0.05);
  declare_parameter("min_pass_time_sec", 0.4);
  declare_parameter("min_pass_distance", 0.3);
  declare_parameter("command_publish_period_sec", 0.02);
  declare_parameter("command_hold_timeout_sec", 0.3);
  declare_parameter("allow_reverse_tangent", true);
  declare_parameter("stop_angular_while_waiting", true);
  declare_parameter("yaw_marker_length", 0.7);
  declare_parameter("yaw_marker_z", 0.25);

  get_parameter("global_frame", global_frame_);
  get_parameter("robot_frame", robot_frame_);
  get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  get_parameter("output_cmd_vel_stamped_topic", output_cmd_vel_stamped_topic_);
  get_parameter("cmd_frame_id", cmd_frame_id_);
  get_parameter("trajectory_topic", trajectory_topic_);
  get_parameter("hole_pass_cmd_topic", hole_pass_cmd_topic_);
  get_parameter("hole_pass_state_topic", hole_pass_state_topic_);
  get_parameter("nav_yaw_topic", nav_yaw_topic_);
  get_parameter("yaw_target_marker_topic", yaw_target_marker_topic_);
  get_parameter("pass_hole_action_name", pass_hole_action_name_);
  get_parameter("hole_ids", hole_ids_);
  get_parameter("prepare_distance", prepare_distance_);
  get_parameter("stop_distance", stop_distance_);
  get_parameter("slow_speed", slow_speed_);
  get_parameter("pass_speed", pass_speed_);
  double yaw_offset_rad = 0.0;
  double yaw_offset_deg = 0.0;
  get_parameter("yaw_offset", yaw_offset_rad);
  get_parameter("yaw_offset_deg", yaw_offset_deg);
  if (hasParameterOverride(options, "yaw_offset_deg")) {
    yaw_offset_ = yaw_offset_deg * M_PI / 180.0;
    if (hasParameterOverride(options, "yaw_offset") &&
      std::abs(yaw_offset_rad - yaw_offset_) > 1.0e-6)
    {
      RCLCPP_WARN(
        get_logger(),
        "both yaw_offset(rad)=%.6f and yaw_offset_deg=%.3f are set; using yaw_offset_deg",
        yaw_offset_rad, yaw_offset_deg);
    }
  } else {
    yaw_offset_ = yaw_offset_rad;
    yaw_offset_deg = yaw_offset_ * 180.0 / M_PI;
  }
  get_parameter("yaw_kp", yaw_kp_);
  get_parameter("max_v_yaw", max_v_yaw_);
  get_parameter("exit_raise_distance", exit_raise_distance_);
  get_parameter("exit_timeout_sec", exit_timeout_sec_);
  get_parameter("lowering_timeout_sec", lowering_timeout_sec_);
  get_parameter("height_state_timeout_sec", height_state_timeout_sec_);
  get_parameter("nav_yaw_timeout_sec", nav_yaw_timeout_sec_);
  get_parameter("trajectory_timeout_sec", trajectory_timeout_sec_);
  get_parameter("tf_lookup_timeout_sec", tf_lookup_timeout_sec_);
  get_parameter("min_pass_time_sec", min_pass_time_sec_);
  get_parameter("min_pass_distance", min_pass_distance_);
  get_parameter("command_publish_period_sec", command_publish_period_sec_);
  get_parameter("command_hold_timeout_sec", command_hold_timeout_sec_);
  get_parameter("allow_reverse_tangent", allow_reverse_tangent_);
  get_parameter("stop_angular_while_waiting", stop_angular_while_waiting_);
  get_parameter("yaw_marker_length", yaw_marker_length_);
  get_parameter("yaw_marker_z", yaw_marker_z_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);

  for (const auto & id : hole_ids_) {
    HoleDefinition hole;
    hole.id = id;
    hole.port_a_polygon = declare_parameter(
      "holes." + id + ".port_a_polygon", std::vector<double>{});
    hole.port_b_polygon = declare_parameter(
      "holes." + id + ".port_b_polygon", std::vector<double>{});
    if (!validPolygon(hole.port_a_polygon) || !validPolygon(hole.port_b_polygon)) {
      RCLCPP_WARN(
        get_logger(),
        "hole '%s' is invalid; each hole needs valid port_a_polygon and port_b_polygon",
        id.c_str());
      continue;
    }
    holes_.push_back(hole);
  }

  if (holes_.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "no valid bidirectional holes configured; hole pass trigger will stay disabled");
  }

  latest_output_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  latest_height_state_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  latest_nav_yaw_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  latest_trajectory_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  stage_enter_time_ = get_clock()->now();
  latest_hole_cmd_.hole_cmd = sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE;
  latest_hole_cmd_.v_yaw = 0.0F;

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 10);
  cmd_vel_stamped_pub_ =
    create_publisher<geometry_msgs::msg::TwistStamped>(output_cmd_vel_stamped_topic_, 10);
  hole_cmd_pub_ =
    create_publisher<sentry_nav_interfaces::msg::HolePassCmd>(hole_pass_cmd_topic_, 10);
  yaw_target_marker_pub_ =
    create_publisher<visualization_msgs::msg::Marker>(yaw_target_marker_topic_, 1);

  trajectory_sub_ = create_subscription<sentry_nav_interfaces::msg::MincoTrajectory>(
    trajectory_topic_, rclcpp::QoS(1),
    std::bind(&HolePassController::trajectoryCallback, this, std::placeholders::_1));
  hole_state_sub_ = create_subscription<sentry_nav_interfaces::msg::HolePassState>(
    hole_pass_state_topic_, 10,
    std::bind(&HolePassController::holePassStateCallback, this, std::placeholders::_1));
  nav_yaw_sub_ = create_subscription<std_msgs::msg::Float64>(
    nav_yaw_topic_, rclcpp::QoS(1).best_effort(),
    std::bind(&HolePassController::navYawCallback, this, std::placeholders::_1));
  pass_hole_action_server_ = rclcpp_action::create_server<PassHole>(
    this,
    pass_hole_action_name_,
    std::bind(
      &HolePassController::handlePassHoleGoal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &HolePassController::handlePassHoleCancel, this,
      std::placeholders::_1),
    std::bind(
      &HolePassController::handlePassHoleAccepted, this,
      std::placeholders::_1));

  const auto period = std::chrono::duration<double>(std::max(command_publish_period_sec_, 0.005));
  command_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&HolePassController::commandTimerCallback, this));

  RCLCPP_INFO(
    get_logger(),
    "HolePassController action='%s' cmd_out='%s' stamped_out='%s' frame='%s' "
    "robot='%s' global='%s' trajectory='%s' cmd_topic='%s' state_topic='%s' "
    "yaw_offset=%.2fdeg holes=%zu",
    pass_hole_action_name_.c_str(), output_cmd_vel_topic_.c_str(),
    output_cmd_vel_stamped_topic_.c_str(), cmd_frame_id_.c_str(), robot_frame_.c_str(),
    global_frame_.c_str(), trajectory_topic_.c_str(), hole_pass_cmd_topic_.c_str(),
    hole_pass_state_topic_.c_str(), yaw_offset_deg, holes_.size());
}

rclcpp_action::GoalResponse HolePassController::handlePassHoleGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const PassHole::Goal> goal)
{
  if (action_active_) {
    RCLCPP_WARN(get_logger(), "[HolePass] rejecting goal while another pass is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->hole_id.empty()) {
    RCLCPP_WARN(get_logger(), "[HolePass] rejecting goal with empty hole_id");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->corridor.polygon.points.size() < 3) {
    RCLCPP_WARN(get_logger(), "[HolePass] rejecting goal because corridor has fewer than 3 points");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse HolePassController::handlePassHoleCancel(
  const std::shared_ptr<GoalHandlePassHole>)
{
  cancel_requested_ = true;
  publishZeroVelocity();
  publishHoleCommand(sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE, 0.0);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void HolePassController::handlePassHoleAccepted(
  const std::shared_ptr<GoalHandlePassHole> goal_handle)
{
  std::thread{std::bind(&HolePassController::executePassHole, this, goal_handle)}.detach();
}

void HolePassController::executePassHole(
  const std::shared_ptr<GoalHandlePassHole> goal_handle)
{
  action_active_ = true;
  cancel_requested_ = false;
  active_hole_index_ = -1;
  pass_start_path_distance_ = std::numeric_limits<double>::quiet_NaN();
  const auto goal = goal_handle->get_goal();
  HoleDefinition action_hole;
  action_hole.id = goal->hole_id;
  action_hole.port_a_polygon = squareAroundPose(goal->entry_pose, 0.18);
  action_hole.port_b_polygon = squareAroundPose(goal->exit_pose, 0.18);
  holes_.clear();
  holes_.push_back(action_hole);
  setStage(Stage::NORMAL, "pass hole action start");

  const double timeout_sec = goal->timeout_sec > 0.0F ? goal->timeout_sec : exit_timeout_sec_ + lowering_timeout_sec_ + 5.0;
  const auto start_time = get_clock()->now();
  auto feedback = std::make_shared<PassHole::Feedback>();
  auto result = std::make_shared<PassHole::Result>();

  rclcpp::Rate rate(1.0 / std::max(command_publish_period_sec_, 0.02));
  bool success = false;
  std::string reason = "timeout";
  while (rclcpp::ok()) {
    if (goal_handle->is_canceling() || cancel_requested_) {
      reason = "canceled";
      break;
    }

    geometry_msgs::msg::PoseStamped robot_pose;
    if (inputsReady() && getRobotPose(&robot_pose)) {
      const auto & position = robot_pose.pose.position;
      const PathProjection projection = analyzePath(position.x, position.y);
      updateStage(projection);
      geometry_msgs::msg::Twist base_cmd;
      if (projection.valid) {
        const double yaw = projection.tangent_yaw;
        base_cmd.linear.x = std::cos(yaw) * pass_speed_;
        base_cmd.linear.y = std::sin(yaw) * pass_speed_;
      }
      const auto output = applyHoleControl(base_cmd, projection);
      latest_output_cmd_ = output;
      latest_output_time_ = get_clock()->now();
      publishVelocity(output);
      publishYawTargetMarker(
        robot_pose, projection, stage_ == Stage::APPROACHING || stage_ == Stage::PASSING);

      feedback->stage = stageId(stage_);
      feedback->stage_name = stageName(stage_);
      feedback->distance_to_entry = static_cast<float>(projection.distance_to_entry);
      feedback->traveled_distance =
        std::isfinite(pass_start_path_distance_) ?
        static_cast<float>(projection.path_distance_from_start - pass_start_path_distance_) : 0.0F;
      goal_handle->publish_feedback(feedback);
    } else {
      publishZeroVelocity();
      publishHoleCommand(
        stage_ == Stage::EXITING ? sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE :
        sentry_nav_interfaces::msg::HolePassCmd::HOLE_LOWER, 0.0);
    }

    if (stage_ == Stage::NORMAL && stage_enter_time_ > start_time) {
      success = true;
      reason = "hole pass completed";
      break;
    }

    if ((get_clock()->now() - start_time).seconds() > timeout_sec) {
      reason = "timeout";
      break;
    }
    rate.sleep();
  }

  publishZeroVelocity();
  publishHoleCommand(sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE, 0.0);
  setStage(Stage::NORMAL, success ? "pass hole action completed" : reason.c_str());
  action_active_ = false;
  cancel_requested_ = false;
  holes_.clear();

  result->success = success;
  result->reason = reason;
  result->final_stage = stageName(stage_);
  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
  } else if (success) {
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
  }
}

void HolePassController::trajectoryCallback(
  const sentry_nav_interfaces::msg::MincoTrajectory::SharedPtr msg)
{
  latest_trajectory_ = msg;
  latest_trajectory_time_ = get_clock()->now();
}

void HolePassController::holePassStateCallback(
  const sentry_nav_interfaces::msg::HolePassState::SharedPtr msg)
{
  latest_height_state_ = msg->height_state;
  if (msg->stamp.sec != 0 || msg->stamp.nanosec != 0U) {
    latest_height_state_time_ = rclcpp::Time(msg->stamp, get_clock()->get_clock_type());
  } else {
    latest_height_state_time_ = get_clock()->now();
  }
}

void HolePassController::navYawCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  latest_nav_yaw_ = msg->data;
  latest_nav_yaw_time_ = get_clock()->now();
  nav_yaw_received_ = true;
}

void HolePassController::commandTimerCallback()
{
  publishLatestCommand();
}

bool HolePassController::getRobotPose(geometry_msgs::msg::PoseStamped * pose)
{
  if (!pose) {
    return false;
  }

  try {
    const auto tf = tf_buffer_->lookupTransform(
      global_frame_, robot_frame_, tf2::TimePointZero,
      tf2::durationFromSec(std::max(0.0, tf_lookup_timeout_sec_)));
    pose->header = tf.header;
    pose->pose.position.x = tf.transform.translation.x;
    pose->pose.position.y = tf.transform.translation.y;
    pose->pose.position.z = tf.transform.translation.z;
    pose->pose.orientation = tf.transform.rotation;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[HolePass] TF %s -> %s unavailable: %s",
      global_frame_.c_str(), robot_frame_.c_str(), ex.what());
    return false;
  }
}

bool HolePassController::inputsReady()
{
  return action_active_ && latest_trajectory_ && latest_trajectory_->waypoints.size() >= 2 &&
         latest_trajectory_->segment_times.size() + 1 == latest_trajectory_->waypoints.size() &&
         trajectoryFresh() &&
         nav_yaw_received_ && navYawFresh() && !holes_.empty();
}

bool HolePassController::heightStateFresh()
{
  return latest_height_state_time_.nanoseconds() != 0 &&
         (get_clock()->now() - latest_height_state_time_).seconds() <= height_state_timeout_sec_;
}

bool HolePassController::navYawFresh()
{
  return latest_nav_yaw_time_.nanoseconds() != 0 &&
         (get_clock()->now() - latest_nav_yaw_time_).seconds() <= nav_yaw_timeout_sec_;
}

bool HolePassController::trajectoryFresh()
{
  return latest_trajectory_time_.nanoseconds() != 0 &&
         (get_clock()->now() - latest_trajectory_time_).seconds() <= trajectory_timeout_sec_;
}

bool HolePassController::isLowered()
{
  return heightStateFresh() &&
         latest_height_state_ == sentry_nav_interfaces::msg::HolePassState::HEIGHT_LOWERED;
}

bool HolePassController::isRaised()
{
  return heightStateFresh() &&
         latest_height_state_ == sentry_nav_interfaces::msg::HolePassState::HEIGHT_NORMAL;
}

void HolePassController::updateStage(const PathProjection & projection)
{
  const bool lowered = isLowered();
  const bool raised = isRaised();
  const auto now = get_clock()->now();

  switch (stage_) {
    case Stage::NORMAL:
      if (
        projection.valid && projection.has_hole && projection.ahead_intersects_entry &&
        projection.distance_to_entry <= prepare_distance_) {
        active_hole_index_ = projection.hole_index;
        active_entry_is_port_a_ = projection.entry_is_port_a;
        pass_start_path_distance_ = std::numeric_limits<double>::quiet_NaN();
        setStage(Stage::APPROACHING, "path will enter hole");
      }
      break;

    case Stage::APPROACHING:
      if (lowered) {
        pass_start_path_distance_ = projection.path_distance_from_start;
        setStage(Stage::PASSING, "height lowered");
      } else if ((now - stage_enter_time_).seconds() > lowering_timeout_sec_) {
        setStage(Stage::WAIT_LOWERED, "lowering timeout");
      } else if (projection.distance_to_entry <= stop_distance_) {
        setStage(Stage::WAIT_LOWERED, "too close before lowered");
      }
      break;

    case Stage::WAIT_LOWERED:
      if (lowered) {
        pass_start_path_distance_ = projection.path_distance_from_start;
        setStage(Stage::PASSING, "height lowered after wait");
      }
      break;

    case Stage::PASSING: {
      const double pass_time = (now - stage_enter_time_).seconds();
      const bool enough_time = pass_time >= min_pass_time_sec_;
      const bool enough_distance =
        std::isfinite(pass_start_path_distance_) &&
        projection.path_distance_from_start - pass_start_path_distance_ >= min_pass_distance_;
      const bool at_exit =
        projection.in_exit_polygon ||
        (projection.ahead_intersects_exit &&
        projection.distance_to_exit_end <= exit_raise_distance_);
      if ((enough_time || enough_distance) &&
        at_exit) {
        setStage(Stage::EXITING, "hole exit reached");
      }
      break;
    }

    case Stage::EXITING:
      if (raised || (now - stage_enter_time_).seconds() > exit_timeout_sec_) {
        setStage(Stage::NORMAL, raised ? "height normal" : "raise timeout");
        pass_start_path_distance_ = std::numeric_limits<double>::quiet_NaN();
        active_hole_index_ = -1;
      }
      break;
  }
}

HolePassController::PathProjection HolePassController::analyzePath(
  double robot_x, double robot_y) const
{
  PathProjection result;
  if (!latest_trajectory_ || latest_trajectory_->waypoints.size() < 2) {
    return result;
  }

  const auto & waypoints = latest_trajectory_->waypoints;
  double best_distance = std::numeric_limits<double>::infinity();
  size_t best_index = 0;
  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const auto & a = waypoints[i];
    const auto & b = waypoints[i + 1];
    const double d = distancePointToSegment(robot_x, robot_y, a.x, a.y, b.x, b.y);
    if (d < best_distance) {
      best_distance = d;
      best_index = i;
    }
  }

  const auto & p0 = waypoints[best_index];
  const auto & p1 = waypoints[best_index + 1];
  result.valid = true;
  result.segment_index = best_index;
  result.tangent_yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);

  double distance_from_start = 0.0;
  double best_segment_t = 0.0;
  for (size_t i = 0; i < best_index; ++i) {
    const auto & a = waypoints[i];
    const auto & b = waypoints[i + 1];
    distance_from_start += std::hypot(b.x - a.x, b.y - a.y);
  }
  const double seg_dx = p1.x - p0.x;
  const double seg_dy = p1.y - p0.y;
  const double seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;
  if (seg_len_sq > kEpsilon) {
    const double t = std::clamp(
      ((robot_x - p0.x) * seg_dx + (robot_y - p0.y) * seg_dy) / seg_len_sq, 0.0, 1.0);
    best_segment_t = t;
    distance_from_start += std::sqrt(seg_len_sq) * t;
  }
  result.path_distance_from_start = distance_from_start;
  result.segment_start_t = best_segment_t;

  const std::vector<double> * entry_polygon = nullptr;
  const std::vector<double> * exit_polygon = nullptr;
  int selected_hole_index = -1;
  bool selected_entry_is_port_a = true;
  double selected_entry_distance = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < holes_.size(); ++i) {
    double dist_a = std::numeric_limits<double>::infinity();
    double dist_b = std::numeric_limits<double>::infinity();
    const bool hit_a = pathIntersectsPolygonFrom(
      best_index, best_segment_t, holes_[i].port_a_polygon, prepare_distance_, &dist_a);
    const bool hit_b = pathIntersectsPolygonFrom(
      best_index, best_segment_t, holes_[i].port_b_polygon, prepare_distance_, &dist_b);
    if (hit_a && dist_a < selected_entry_distance) {
      selected_entry_distance = dist_a;
      selected_hole_index = static_cast<int>(i);
      selected_entry_is_port_a = true;
    }
    if (hit_b && dist_b < selected_entry_distance) {
      selected_entry_distance = dist_b;
      selected_hole_index = static_cast<int>(i);
      selected_entry_is_port_a = false;
    }
  }

  if (active_hole_index_ >= 0 && active_hole_index_ < static_cast<int>(holes_.size())) {
    selected_hole_index = active_hole_index_;
    selected_entry_is_port_a = active_entry_is_port_a_;
  }

  if (selected_hole_index < 0) {
    return result;
  }

  const auto & hole = holes_[static_cast<size_t>(selected_hole_index)];
  entry_polygon = selected_entry_is_port_a ? &hole.port_a_polygon : &hole.port_b_polygon;
  exit_polygon = selected_entry_is_port_a ? &hole.port_b_polygon : &hole.port_a_polygon;
  result.has_hole = true;
  result.hole_index = selected_hole_index;
  result.entry_is_port_a = selected_entry_is_port_a;

  double entry_distance = std::numeric_limits<double>::infinity();
  result.ahead_intersects_entry =
    pathIntersectsPolygonFrom(
      best_index, best_segment_t, *entry_polygon, prepare_distance_, &entry_distance);
  result.distance_to_entry = entry_distance;
  result.in_entry_polygon = pointInPolygon(robot_x, robot_y, *entry_polygon);
  result.beyond_entry = !result.ahead_intersects_entry && !result.in_entry_polygon;

  if (validPolygon(*exit_polygon)) {
    double exit_distance = std::numeric_limits<double>::infinity();
    result.ahead_intersects_exit = pathIntersectsPolygonFrom(
      best_index, best_segment_t, *exit_polygon, std::numeric_limits<double>::infinity(),
      &exit_distance);
    result.in_exit_polygon = pointInPolygon(robot_x, robot_y, *exit_polygon);
    result.distance_to_exit_end =
      result.in_exit_polygon ? 0.0 : exit_distance;
    result.beyond_exit = !result.ahead_intersects_exit && !result.in_exit_polygon;
  }

  return result;
}

geometry_msgs::msg::Twist HolePassController::applyHoleControl(
  const geometry_msgs::msg::Twist & input_cmd, const PathProjection & projection)
{
  geometry_msgs::msg::Twist output = input_cmd;
  uint8_t hole_cmd = sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE;
  double v_yaw = 0.0;

  if (stage_ == Stage::WAIT_LOWERED) {
    stopPlanarVelocity(&output);
    publishHoleCommand(sentry_nav_interfaces::msg::HolePassCmd::HOLE_LOWER, 0.0);
    return output;
  }

  if (stage_ == Stage::APPROACHING || stage_ == Stage::PASSING) {
    const double target_yaw = targetYawFromProjection(projection);
    const double yaw_error = normalizeAngle(target_yaw - latest_nav_yaw_);
    v_yaw = std::clamp(yaw_kp_ * yaw_error, -max_v_yaw_, max_v_yaw_);
    hole_cmd = sentry_nav_interfaces::msg::HolePassCmd::HOLE_LOWER;
  } else if (stage_ == Stage::EXITING) {
    hole_cmd = sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE;
    v_yaw = 0.0;
  }

  if (stage_ == Stage::APPROACHING) {
    const double limited = limitPlanarSpeed(output.linear.x, output.linear.y, slow_speed_);
    if (limited > kEpsilon) {
      const double scale = limited / std::hypot(output.linear.x, output.linear.y);
      output.linear.x *= scale;
      output.linear.y *= scale;
    }
  } else if (stage_ == Stage::PASSING) {
    const double limited = limitPlanarSpeed(output.linear.x, output.linear.y, pass_speed_);
    if (limited > kEpsilon) {
      const double scale = limited / std::hypot(output.linear.x, output.linear.y);
      output.linear.x *= scale;
      output.linear.y *= scale;
    }
  }

  publishHoleCommand(hole_cmd, v_yaw);
  return output;
}

double HolePassController::targetYawFromProjection(const PathProjection & projection) const
{
  const double target_yaw_forward = normalizeAngle(projection.tangent_yaw + yaw_offset_);
  if (!allow_reverse_tangent_) {
    return target_yaw_forward;
  }

  const double target_yaw_reverse = normalizeAngle(target_yaw_forward + M_PI);
  if (
    std::abs(normalizeAngle(target_yaw_reverse - latest_nav_yaw_)) <
    std::abs(normalizeAngle(target_yaw_forward - latest_nav_yaw_))) {
    return target_yaw_reverse;
  }
  return target_yaw_forward;
}

void HolePassController::publishYawTargetMarker(
  const geometry_msgs::msg::PoseStamped & robot_pose, const PathProjection & projection,
  bool visible)
{
  if (!yaw_target_marker_pub_) {
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = global_frame_;
  marker.header.stamp = get_clock()->now();
  marker.ns = "hole_pass_yaw_target";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visible ? visualization_msgs::msg::Marker::ADD :
    visualization_msgs::msg::Marker::DELETE;
  if (!visible || !projection.valid || !projection.has_hole) {
    yaw_target_marker_pub_->publish(marker);
    return;
  }

  const double target_yaw = targetYawFromProjection(projection);
  marker.pose.position = robot_pose.pose.position;
  marker.pose.position.z += yaw_marker_z_;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, target_yaw);
  marker.pose.orientation = tf2::toMsg(q);
  marker.scale.x = yaw_marker_length_;
  marker.scale.y = 0.08;
  marker.scale.z = 0.08;
  marker.color.r = 0.05F;
  marker.color.g = 0.85F;
  marker.color.b = 1.0F;
  marker.color.a = 0.95F;
  marker.lifetime = rclcpp::Duration::from_seconds(0.25);
  yaw_target_marker_pub_->publish(marker);
}

void HolePassController::publishVelocity(const geometry_msgs::msg::Twist & output)
{
  cmd_vel_pub_->publish(output);

  geometry_msgs::msg::TwistStamped stamped;
  stamped.header.stamp = get_clock()->now();
  stamped.header.frame_id = cmd_frame_id_;
  stamped.twist = output;
  cmd_vel_stamped_pub_->publish(stamped);
}

void HolePassController::publishZeroVelocity()
{
  geometry_msgs::msg::Twist zero;
  latest_output_cmd_ = zero;
  latest_output_time_ = get_clock()->now();
  publishVelocity(zero);
}

void HolePassController::publishHoleCommand(uint8_t hole_cmd, double v_yaw)
{
  latest_hole_cmd_.hole_cmd = hole_cmd;
  latest_hole_cmd_.v_yaw = static_cast<float>(v_yaw);
  hole_cmd_pub_->publish(latest_hole_cmd_);
}

void HolePassController::publishLatestCommand()
{
  if (stage_ == Stage::NORMAL) {
    return;
  }
  hole_cmd_pub_->publish(latest_hole_cmd_);

  if (latest_output_time_.nanoseconds() == 0) {
    return;
  }
  const double age = (get_clock()->now() - latest_output_time_).seconds();
  if (age < command_hold_timeout_sec_) {
    publishVelocity(latest_output_cmd_);
  }
}

bool HolePassController::pathIntersectsPolygonFrom(
  size_t start_index, double start_t, const std::vector<double> & polygon, double max_distance,
  double * first_distance) const
{
  if (!latest_trajectory_ || latest_trajectory_->waypoints.size() < 2 || !validPolygon(polygon)) {
    return false;
  }

  const auto & waypoints = latest_trajectory_->waypoints;
  double accum = 0.0;
  const size_t begin = std::min(start_index, waypoints.size() - 2);
  const double clamped_start_t = std::clamp(start_t, 0.0, 1.0);
  for (size_t i = begin; i + 1 < waypoints.size(); ++i) {
    const auto & a = waypoints[i];
    const auto & b = waypoints[i + 1];
    double ax = a.x;
    double ay = a.y;
    if (i == begin) {
      ax = a.x + (b.x - a.x) * clamped_start_t;
      ay = a.y + (b.y - a.y) * clamped_start_t;
    }
    double segment_hit_distance = 0.0;
    const bool hit = segmentPolygonFirstIntersectionDistance(
      ax, ay, b.x, b.y, polygon, &segment_hit_distance);
    if (hit) {
      if (first_distance) {
        *first_distance = accum + segment_hit_distance;
      }
      return accum + segment_hit_distance <= max_distance;
    }
    accum += std::hypot(b.x - ax, b.y - ay);
    if (accum > max_distance) {
      return false;
    }
  }
  return false;
}

bool HolePassController::segmentPolygonFirstIntersectionDistance(
  double ax, double ay, double bx, double by, const std::vector<double> & polygon,
  double * distance) const
{
  if (!validPolygon(polygon)) {
    return false;
  }

  const double segment_length = std::hypot(bx - ax, by - ay);
  if (segment_length < kEpsilon) {
    if (pointInPolygon(ax, ay, polygon)) {
      if (distance) {
        *distance = 0.0;
      }
      return true;
    }
    return false;
  }

  double best_t = std::numeric_limits<double>::infinity();
  if (pointInPolygon(ax, ay, polygon)) {
    best_t = 0.0;
  }

  const size_t n = polygon.size() / 2;
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    double t = std::numeric_limits<double>::infinity();
    if (segmentIntersectionParameter(
        ax, ay, bx, by, polygon[2 * j], polygon[2 * j + 1],
        polygon[2 * i], polygon[2 * i + 1], &t)) {
      best_t = std::min(best_t, t);
    }
  }

  if (!std::isfinite(best_t)) {
    return false;
  }
  if (distance) {
    *distance = std::clamp(best_t, 0.0, 1.0) * segment_length;
  }
  return true;
}

bool HolePassController::segmentIntersectionParameter(
  double ax, double ay, double bx, double by, double cx, double cy, double dx,
  double dy, double * t) const
{
  const double rx = bx - ax;
  const double ry = by - ay;
  const double sx = dx - cx;
  const double sy = dy - cy;
  const double denom = rx * sy - ry * sx;
  const double qpx = cx - ax;
  const double qpy = cy - ay;

  if (std::abs(denom) < kEpsilon) {
    if (std::abs(qpx * ry - qpy * rx) > 1e-6) {
      return false;
    }
    const double rr = rx * rx + ry * ry;
    if (rr < kEpsilon) {
      return false;
    }
    const double t0 = ((cx - ax) * rx + (cy - ay) * ry) / rr;
    const double t1 = ((dx - ax) * rx + (dy - ay) * ry) / rr;
    const double lo = std::max(0.0, std::min(t0, t1));
    const double hi = std::min(1.0, std::max(t0, t1));
    if (lo > hi) {
      return false;
    }
    if (t) {
      *t = lo;
    }
    return true;
  }

  const double u = (qpx * ry - qpy * rx) / denom;
  const double ti = (qpx * sy - qpy * sx) / denom;
  if (ti < -1e-9 || ti > 1.0 + 1e-9 || u < -1e-9 || u > 1.0 + 1e-9) {
    return false;
  }
  if (t) {
    *t = std::clamp(ti, 0.0, 1.0);
  }
  return true;
}

bool HolePassController::pointInPolygon(
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
    if (distancePointToSegment(x, y, xi, yi, xj, yj) < 1e-6) {
      return true;
    }
    const bool crosses = ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / (yj - yi + kEpsilon) + xi);
    if (crosses) {
      inside = !inside;
    }
  }
  return inside;
}

bool HolePassController::segmentIntersectsPolygon(
  double ax, double ay, double bx, double by, const std::vector<double> & polygon) const
{
  const size_t n = polygon.size() / 2;
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    if (segmentsIntersect(
        ax, ay, bx, by, polygon[2 * j], polygon[2 * j + 1], polygon[2 * i],
        polygon[2 * i + 1]))
    {
      return true;
    }
  }
  return false;
}

bool HolePassController::segmentsIntersect(
  double ax, double ay, double bx, double by, double cx, double cy, double dx,
  double dy) const
{
  const double d1 = signed2dCross(ax, ay, bx, by, cx, cy);
  const double d2 = signed2dCross(ax, ay, bx, by, dx, dy);
  const double d3 = signed2dCross(cx, cy, dx, dy, ax, ay);
  const double d4 = signed2dCross(cx, cy, dx, dy, bx, by);

  if (((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) &&
    ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0)))
  {
    return true;
  }

  return distancePointToSegment(cx, cy, ax, ay, bx, by) < 1e-6 ||
         distancePointToSegment(dx, dy, ax, ay, bx, by) < 1e-6 ||
         distancePointToSegment(ax, ay, cx, cy, dx, dy) < 1e-6 ||
         distancePointToSegment(bx, by, cx, cy, dx, dy) < 1e-6;
}

double HolePassController::distanceToPolygon(
  double x, double y, const std::vector<double> & polygon) const
{
  if (!validPolygon(polygon)) {
    return std::numeric_limits<double>::infinity();
  }
  if (pointInPolygon(x, y, polygon)) {
    return 0.0;
  }

  double min_distance = std::numeric_limits<double>::infinity();
  const size_t n = polygon.size() / 2;
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    min_distance = std::min(
      min_distance,
      distancePointToSegment(x, y, polygon[2 * j], polygon[2 * j + 1], polygon[2 * i],
      polygon[2 * i + 1]));
  }
  return min_distance;
}

double HolePassController::distancePointToSegment(
  double px, double py, double ax, double ay, double bx, double by) const
{
  const double vx = bx - ax;
  const double vy = by - ay;
  const double wx = px - ax;
  const double wy = py - ay;
  const double len_sq = vx * vx + vy * vy;
  if (len_sq < kEpsilon) {
    return std::hypot(px - ax, py - ay);
  }
  const double t = std::clamp((wx * vx + wy * vy) / len_sq, 0.0, 1.0);
  const double proj_x = ax + t * vx;
  const double proj_y = ay + t * vy;
  return std::hypot(px - proj_x, py - proj_y);
}

double HolePassController::signed2dCross(
  double ax, double ay, double bx, double by, double cx, double cy) const
{
  return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

double HolePassController::normalizeAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double HolePassController::limitPlanarSpeed(double x, double y, double max_speed) const
{
  const double speed = std::hypot(x, y);
  if (speed < kEpsilon) {
    return 0.0;
  }
  return std::min(speed, std::max(0.0, max_speed));
}

void HolePassController::stopPlanarVelocity(geometry_msgs::msg::Twist * output) const
{
  if (!output) {
    return;
  }
  output->linear.x = 0.0;
  output->linear.y = 0.0;
  output->linear.z = 0.0;
  if (stop_angular_while_waiting_) {
    output->angular.x = 0.0;
    output->angular.y = 0.0;
    output->angular.z = 0.0;
  }
}

bool HolePassController::validPolygon(const std::vector<double> & polygon) const
{
  return polygon.size() >= 6 && polygon.size() % 2 == 0;
}

std::vector<double> HolePassController::squareAroundPose(
  const geometry_msgs::msg::PoseStamped & pose, double half_size) const
{
  const double x = pose.pose.position.x;
  const double y = pose.pose.position.y;
  return {
    x - half_size, y - half_size,
    x + half_size, y - half_size,
    x + half_size, y + half_size,
    x - half_size, y + half_size};
}

const char * HolePassController::stageName(Stage stage) const
{
  switch (stage) {
    case Stage::NORMAL:
      return "NORMAL";
    case Stage::APPROACHING:
      return "APPROACHING";
    case Stage::WAIT_LOWERED:
      return "WAIT_LOWERED";
    case Stage::PASSING:
      return "PASSING";
    case Stage::EXITING:
      return "EXITING";
  }
  return "UNKNOWN";
}

uint8_t HolePassController::stageId(Stage stage) const
{
  return static_cast<uint8_t>(stage);
}

void HolePassController::setStage(Stage stage, const char * reason)
{
  if (stage_ == stage) {
    return;
  }
  RCLCPP_INFO(
    get_logger(), "[HolePass] %s -> %s: %s", stageName(stage_), stageName(stage), reason);
  stage_ = stage;
  stage_enter_time_ = get_clock()->now();
  if (stage_ == Stage::NORMAL) {
    publishHoleCommand(sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE, 0.0);
  } else if (stage_ == Stage::EXITING) {
    publishHoleCommand(sentry_nav_interfaces::msg::HolePassCmd::HOLE_RAISE, 0.0);
  }
}

}  // namespace hole_pass_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hole_pass_controller::HolePassController)
