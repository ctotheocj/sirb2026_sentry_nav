#include "sirb_smoother/trajectory_manager_node.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <numeric>
#include <sstream>
#include <thread>

#include "gcopter/minco.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace sirb_smoother
{

namespace
{

class AtomicFlagGuard
{
public:
  explicit AtomicFlagGuard(std::atomic_bool & flag)
  : flag_(flag)
  {}

  ~AtomicFlagGuard()
  {
    flag_.store(false);
  }

  AtomicFlagGuard(const AtomicFlagGuard &) = delete;
  AtomicFlagGuard & operator=(const AtomicFlagGuard &) = delete;

private:
  std::atomic_bool & flag_;
};

}  // namespace

TrajectoryManagerNode::TrajectoryManagerNode(const rclcpp::NodeOptions & options)
: Node("trajectory_manager", options)
{
  input_topic_ = declare_parameter<std::string>(
    "input_topic", "safe_geometric_smoother/trajectory_for_mpc");
  output_topic_ = declare_parameter<std::string>(
    "output_topic", "trajectory_manager/trajectory_for_mpc");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "odometry");
  costmap_topic_ = declare_parameter<std::string>("costmap_topic", "local_costmap/costmap_raw");
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
  active_timeout_sec_ = declare_parameter<double>("active_timeout_sec", 6.0);
  min_remaining_time_sec_ = declare_parameter<double>("min_remaining_time_sec", 0.15);
  crop_epsilon_sec_ = declare_parameter<double>("crop_epsilon_sec", 0.02);
  projection_search_window_sec_ = declare_parameter<double>("projection_search_window_sec", 0.8);
  projection_max_tracking_error_ = declare_parameter<double>("projection_max_tracking_error", 0.80);
  candidate_projection_window_sec_ = declare_parameter<double>("candidate_projection_window_sec", 0.35);
  switch_goal_change_distance_ = declare_parameter<double>("switch_goal_change_distance", 0.5);
  switch_max_position_error_ = declare_parameter<double>("switch_max_position_error", 0.45);
  switch_max_velocity_error_ = declare_parameter<double>("switch_max_velocity_error", 1.8);
  switch_max_acceleration_error_ = declare_parameter<double>("switch_max_acceleration_error", 4.0);
  switch_min_direction_dot_ = declare_parameter<double>("switch_min_direction_dot", -0.2);
  switch_min_candidate_duration_sec_ = declare_parameter<double>(
    "switch_min_candidate_duration_sec", 0.2);
  switch_min_interval_sec_ = declare_parameter<double>("switch_min_interval_sec", 3.0);
  switch_immediate_deviation_ = declare_parameter<double>("switch_immediate_deviation", 1.2);
  switch_immediate_length_ratio_ = declare_parameter<double>("switch_immediate_length_ratio", 0.25);
  switch_allow_when_remaining_sec_ = declare_parameter<double>("switch_allow_when_remaining_sec", 1.0);
  emergency_stop_remaining_time_sec_ = declare_parameter<double>(
    "emergency_stop_remaining_time_sec", 0.10);
  emergency_stop_duration_sec_ = declare_parameter<double>("emergency_stop_duration_sec", 0.20);
  enable_forward_collision_check_ = declare_parameter<bool>("enable_forward_collision_check", true);
  allow_unknown_costmap_ = declare_parameter<bool>("allow_unknown_costmap", true);
  collision_cost_threshold_ = declare_parameter<int>("collision_cost_threshold", 253);
  collision_check_horizon_sec_ = declare_parameter<double>("collision_check_horizon_sec", 1.2);
  collision_check_step_sec_ = declare_parameter<double>("collision_check_step_sec", 0.10);
  collision_check_radius_ = declare_parameter<double>("collision_check_radius", 0.20);
  collision_tf_failure_is_safe_ = declare_parameter<bool>("collision_tf_failure_is_safe", false);
  publish_full_active_trajectory_ = declare_parameter<bool>("publish_full_active_trajectory", true);
  require_odom_ = declare_parameter<bool>("require_odom", true);
  tracking_error_replan_frames_ = declare_parameter<int>("tracking_error_replan_frames", 3);
  tracking_error_replan_frames_ = std::max(1, tracking_error_replan_frames_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  traj_pub_ = create_publisher<sentry_nav_interfaces::msg::MincoTrajectory>(
    output_topic_, rclcpp::QoS(1));
  status_pub_ = create_publisher<std_msgs::msg::String>("trajectory_manager/status", rclcpp::QoS(1));
  traj_sub_ = create_subscription<sentry_nav_interfaces::msg::MincoTrajectory>(
    input_topic_, rclcpp::QoS(1),
    std::bind(&TrajectoryManagerNode::trajectoryCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS(1),
    std::bind(&TrajectoryManagerNode::odomCallback, this, std::placeholders::_1));
  costmap_sub_ = create_subscription<nav2_msgs::msg::Costmap>(
    costmap_topic_, rclcpp::QoS(1),
    std::bind(&TrajectoryManagerNode::costmapCallback, this, std::placeholders::_1));
  commit_action_server_ = rclcpp_action::create_server<CommitTrajectory>(
    this,
    "trajectory_manager/commit_trajectory",
    std::bind(
      &TrajectoryManagerNode::handleCommitGoal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &TrajectoryManagerNode::handleCommitCancel, this,
      std::placeholders::_1),
      std::bind(
      &TrajectoryManagerNode::handleCommitAccepted, this,
      std::placeholders::_1));
  hole_collision_policy_srv_ = create_service<std_srvs::srv::SetBool>(
    "trajectory_manager/set_hole_collision_policy",
    std::bind(
      &TrajectoryManagerNode::setHoleCollisionPolicyService, this,
      std::placeholders::_1, std::placeholders::_2));

  if (!publish_full_active_trajectory_) {
    RCLCPP_WARN(
      get_logger(),
      "TrajectoryManager publish_full_active_trajectory is false; MPC time-axis tracking requires full active trajectories");
  }
  if (enable_forward_collision_check_ && collision_cost_threshold_ >
    static_cast<int>(nav2_costmap_2d::NO_INFORMATION))
  {
    RCLCPP_WARN(
      get_logger(),
      "TrajectoryManager collision_cost_threshold=%d exceeds NO_INFORMATION=%d; collision checks may never trigger",
      collision_cost_threshold_, static_cast<int>(nav2_costmap_2d::NO_INFORMATION));
  }
  if (candidate_projection_window_sec_ > projection_search_window_sec_) {
    RCLCPP_WARN(
      get_logger(),
      "TrajectoryManager candidate_projection_window_sec %.2f > projection_search_window_sec %.2f",
      candidate_projection_window_sec_, projection_search_window_sec_);
  }
  if (switch_max_acceleration_error_ <= 0.0 || switch_max_velocity_error_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "TrajectoryManager switch continuity limits are non-positive: vel=%.2f acc=%.2f",
      switch_max_velocity_error_, switch_max_acceleration_error_);
  }

  const double rate = std::max(1.0, publish_rate_hz_);
  publish_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate),
    std::bind(&TrajectoryManagerNode::publishTimerCallback, this));

  RCLCPP_INFO(
    get_logger(),
    "TrajectoryManager configured input='%s' output='%s' odom='%s' costmap='%s' rate=%.1fHz",
    input_topic_.c_str(), output_topic_.c_str(), odom_topic_.c_str(),
    costmap_topic_.c_str(), rate);
}

void TrajectoryManagerNode::trajectoryCallback(
  const sentry_nav_interfaces::msg::MincoTrajectory::SharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "TrajectoryManager ignored legacy candidate topic: goal=%lu waypoints=%zu times=%zu",
    static_cast<unsigned long>(msg->goal_id), msg->waypoints.size(), msg->segment_times.size());
}

rclcpp_action::GoalResponse TrajectoryManagerNode::handleCommitGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const CommitTrajectory::Goal>)
{
  bool expected = false;
  if (!commit_in_flight_.compare_exchange_strong(expected, true)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "TrajectoryManager rejected CommitTrajectory while previous commit is still running");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrajectoryManagerNode::handleCommitCancel(
  const std::shared_ptr<GoalHandleCommitTrajectory>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TrajectoryManagerNode::handleCommitAccepted(
  const std::shared_ptr<GoalHandleCommitTrajectory> goal_handle)
{
  std::thread{std::bind(&TrajectoryManagerNode::executeCommit, this, goal_handle)}.detach();
}

void TrajectoryManagerNode::executeCommit(
  const std::shared_ptr<GoalHandleCommitTrajectory> goal_handle)
{
  AtomicFlagGuard in_flight(commit_in_flight_);
  auto result = std::make_shared<CommitTrajectory::Result>();
  const auto goal = goal_handle->get_goal();
  const auto candidate = goal->candidate_minco;

  if (goal_handle->is_canceling()) {
    std::lock_guard<std::mutex> lk(mutex_);
    result->accepted = false;
    result->active_valid = has_active_traj_;
    result->trajectory_id = active_traj_.trajectory_id;
    result->reason = "commit canceled";
    goal_handle->canceled(result);
    return;
  }

  if (!validateTrajectory(candidate)) {
    std::lock_guard<std::mutex> lk(mutex_);
    result->accepted = false;
    result->active_valid = has_active_traj_;
    result->trajectory_id = active_traj_.trajectory_id;
    result->reason = has_active_traj_ ? "invalid candidate, active kept" : "invalid candidate";
    if (has_active_traj_ && goal->allow_keep_active_on_reject) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
    return;
  }

  double candidate_projected_time = 0.0;
  bool accepted = false;
  std::string reason;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    const bool had_active = has_active_traj_;
    if (!had_active) {
      if (require_odom_ && !has_odom_) {
        transitionTo(State::REPLAN_PENDING, "initial trajectory waiting for odom");
        reason = "initial trajectory waiting for odom";
      } else {
        Trajectory<5> candidate_traj;
        if (!buildTrajectory(candidate, candidate_traj)) {
          reason = "candidate build failed";
        } else {
          if (has_odom_) {
            double projected = 0.0;
            const double saved_window = projection_search_window_sec_;
            projection_search_window_sec_ = std::max(candidate_projection_window_sec_, 1.5);
            if (projectOdomToTrajectory(candidate, candidate_traj, 0.0, projected)) {
              candidate_projected_time = projected;
            } else if (require_odom_) {
              reason = "odom projection unavailable";
            }
            projection_search_window_sec_ = saved_window;
          }
          if (reason.empty()) {
            double collision_time = 0.0;
            int collision_cost = 0;
            if (!trajectoryIsCollisionFree(
                candidate, candidate_traj, candidate_projected_time, collision_check_horizon_sec_,
                collision_time, collision_cost))
            {
              std::ostringstream out;
              out << "candidate collision at t=" << collision_time << " cost=" << collision_cost;
              reason = out.str();
            } else {
              acceptTrajectory(candidate, candidate_projected_time, State::TRACKING, "new trajectory");
              accepted = true;
              reason = "accepted initial trajectory";
            }
          }
        }
      }
    } else {
      const bool new_goal = isNewGoal(candidate);
      bool connect_from_active = true;
      if (shouldSwitchToCandidate(candidate, candidate_projected_time, connect_from_active)) {
        acceptTrajectory(
          candidate, candidate_projected_time, new_goal ? State::TRACKING : State::SWITCHING,
          "new trajectory", connect_from_active);
        accepted = true;
        reason = "accepted candidate";
      } else {
        transitionTo(State::REPLAN_PENDING, "candidate rejected, keep active trajectory");
        reason = "candidate rejected, active kept";
      }
    }
    result->active_valid = has_active_traj_;
    result->trajectory_id = active_traj_.trajectory_id;
  }

  result->accepted = accepted;
  result->reason = reason;
  if (accepted || (result->active_valid && goal->allow_keep_active_on_reject)) {
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
  }
}

void TrajectoryManagerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mutex_);
  latest_odom_ = *msg;
  last_odom_stamp_ = rclcpp::Time(msg->header.stamp);
  has_odom_ = true;
}

void TrajectoryManagerNode::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mutex_);
  latest_costmap_ = *msg;
  has_costmap_ = true;
}

void TrajectoryManagerNode::setHoleCollisionPolicyService(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  hole_collision_policy_ = request->data;
  response->success = true;
  response->message = hole_collision_policy_ ?
    "trajectory manager hole collision policy enabled" :
    "trajectory manager normal collision policy restored";
  RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
}

void TrajectoryManagerNode::publishTimerCallback()
{
  bool should_publish = false;
  rclcpp::Time publish_stamp = get_clock()->now();
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!has_active_traj_) {
      if (state_ == State::EMERGENCY_STOP && has_emergency_stop_) {
        publishLastEmergencyStop(publish_stamp);
        publishStatus(publish_stamp, "emergency stop active");
        return;
      }
      transitionTo(State::IDLE, "no active trajectory");
      publishStatus(publish_stamp, "no active trajectory");
      return;
    }

    const rclcpp::Time now = get_clock()->now();
    double age = 0.0;
    try {
      age = (now - active_stamp_).seconds();
    } catch (const std::runtime_error &) {
      transitionTo(State::IDLE, "time source mismatch");
      has_active_traj_ = false;
      return;
    }

    const double duration = trajectoryDuration(active_traj_);
    if (age > duration + active_timeout_sec_) {
      publishEmergencyStop(now, "active trajectory exhausted");
      transitionTo(State::EMERGENCY_STOP, "active trajectory exhausted");
      has_active_traj_ = false;
      publishStatus(now, "active trajectory exhausted");
      return;
    }

    if (require_odom_ && !has_odom_) {
      transitionTo(State::REPLAN_PENDING, "waiting for odom");
      return;
    }

    if (state_ == State::SWITCHING) {
      transitionTo(State::TRACKING, "switch published");
    } else if (state_ == State::REPLAN_PENDING) {
      // Keep publishing the active trajectory while waiting for an acceptable replan.
    } else if (state_ != State::TRACKING) {
      transitionTo(State::TRACKING, "active trajectory available");
    }

    double projected_time = age;
    if (has_odom_) {
      Trajectory<5> traj;
      if (!buildTrajectory(active_traj_, traj)) {
        publishEmergencyStop(now, "active trajectory rebuild failed");
        transitionTo(State::EMERGENCY_STOP, "active trajectory rebuild failed");
        has_active_traj_ = false;
        publishStatus(now, "active trajectory rebuild failed");
        return;
      }
      if (!projectOdomToTrajectory(active_traj_, traj, last_projected_time_, projected_time)) {
        if (require_odom_) {
          transitionTo(State::REPLAN_PENDING, "odom projection unavailable");
          return;
        }
        projected_time = last_projected_time_;
      }
      projected_time = std::max(projected_time, last_projected_time_);
      projected_time = std::min(projected_time, duration);
      last_projected_time_ = projected_time;
      double rx = 0.0;
      double ry = 0.0;
      if (getOdomPositionInTrajectoryFrame(active_traj_, rx, ry)) {
        const auto ref_pos = traj.getPos(last_projected_time_);
        const double tracking_error = std::hypot(ref_pos.x() - rx, ref_pos.y() - ry);
        const double max_error =
          std::max(projection_max_tracking_error_, switch_max_position_error_);
        if (tracking_error > max_error) {
          ++tracking_error_replan_count_;
          if (tracking_error_replan_count_ >= tracking_error_replan_frames_) {
            transitionTo(State::REPLAN_PENDING, "tracking error exceeds replan threshold");
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "TrajectoryManager tracking error %.2fm > %.2fm for %d frame(s); replan pending",
              tracking_error, max_error, tracking_error_replan_count_);
          }
        } else {
          tracking_error_replan_count_ = 0;
        }
      }
      double collision_time = 0.0;
      int collision_cost = 0;
      if (!trajectoryIsCollisionFree(
          active_traj_, traj, last_projected_time_, collision_check_horizon_sec_,
          collision_time, collision_cost))
      {
        publishEmergencyStop(now, "active trajectory collision");
        transitionTo(State::EMERGENCY_STOP, "active trajectory collision");
        has_active_traj_ = false;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 500,
          "TrajectoryManager emergency stop: active collision at t=%.2fs cost=%d",
          collision_time, collision_cost);
        publishStatus(now, "active trajectory collision");
        return;
      }
    } else if (require_odom_) {
      transitionTo(State::REPLAN_PENDING, "waiting for odom");
      return;
    }

    const double remaining = duration - last_projected_time_;
    if (remaining < emergency_stop_remaining_time_sec_) {
      publishEmergencyStop(now, "remaining trajectory too short");
      transitionTo(State::EMERGENCY_STOP, "remaining trajectory too short");
      has_active_traj_ = false;
      publishStatus(now, "remaining trajectory too short");
      return;
    }
    should_publish = publish_full_active_trajectory_;
    publish_stamp = now;
    publishStatus(now, "active");
  }

  if (should_publish) {
    publishActiveTrajectory(publish_stamp);
  }
}

void TrajectoryManagerNode::acceptTrajectory(
  const sentry_nav_interfaces::msg::MincoTrajectory & msg,
  double projected_time,
  State next_state,
  const char * reason,
  bool connect_from_active_override)
{
  const rclcpp::Time accept_stamp = get_clock()->now();
  const bool connect_to_current_state = has_active_traj_;
  const bool connect_from_active = has_active_traj_ && connect_from_active_override;
  Eigen::Vector3d connect_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d connect_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d connect_acc = Eigen::Vector3d::Zero();
  if (connect_from_active) {
    Trajectory<5> active_traj;
    if (buildTrajectory(active_traj_, active_traj)) {
      double active_projected_time = last_projected_time_;
      if (has_odom_) {
        (void)projectOdomToTrajectory(
          active_traj_, active_traj, last_projected_time_, active_projected_time);
      }
      active_projected_time = std::clamp(
        std::max(active_projected_time, last_projected_time_),
        0.0, trajectoryDuration(active_traj_));
      if (has_odom_ && getOdomStateInTrajectoryFrame(msg, connect_pos, connect_vel)) {
        connect_acc = Eigen::Vector3d::Zero();
        const auto active_pos = active_traj.getPos(active_projected_time);
        const double odom_to_active =
          (connect_pos - active_pos).head<2>().norm();
        if (odom_to_active > projection_max_tracking_error_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "TrajectoryManager connecting candidate from odom, not old active projection "
            "(tracking_error=%.2fm)", odom_to_active);
        }
      } else {
        connect_pos = active_traj.getPos(active_projected_time);
        connect_vel = active_traj.getVel(active_projected_time);
        connect_acc = active_traj.getAcc(active_projected_time);
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "TrajectoryManager rejected candidate: cannot rebuild active trajectory for connection");
      transitionTo(State::REPLAN_PENDING, "active connection rebuild failed");
      return;
    }
  } else if (connect_to_current_state) {
    if (has_odom_ && getOdomStateInTrajectoryFrame(msg, connect_pos, connect_vel)) {
      connect_acc = Eigen::Vector3d::Zero();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "TrajectoryManager rebasing candidate from odom, bypassing stale active continuity");
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "TrajectoryManager rejected candidate: odom state unavailable for rebase");
      transitionTo(State::REPLAN_PENDING, "candidate odom rebase failed");
      return;
    }
  }

  sentry_nav_interfaces::msg::MincoTrajectory rebased;
  if (projected_time > 1.0e-3) {
    if (!cropTrajectory(msg, projected_time, rebased)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "TrajectoryManager rejected candidate: cannot rebase at t=%.2fs", projected_time);
      transitionTo(State::REPLAN_PENDING, "candidate rebase failed");
      return;
    }
  } else {
    rebased = msg;
  }

  if (connect_to_current_state) {
    if (rebased.waypoints.empty()) {
      transitionTo(State::REPLAN_PENDING, "candidate rebase produced empty trajectory");
      return;
    }
    rebased.waypoints.front().x = connect_pos.x();
    rebased.waypoints.front().y = connect_pos.y();
    rebased.waypoints.front().z = connect_pos.z();
    if (!trimConnectedTrajectoryStart(rebased, connect_pos, connect_vel)) {
      transitionTo(State::REPLAN_PENDING, "connected candidate start trim failed");
      return;
    }
    rebased.initial_velocity.x = connect_vel.x();
    rebased.initial_velocity.y = connect_vel.y();
    rebased.initial_velocity.z = connect_vel.z();
    rebased.initial_acceleration.x = connect_acc.x();
    rebased.initial_acceleration.y = connect_acc.y();
    rebased.initial_acceleration.z = connect_acc.z();
    if (!validateTrajectory(rebased)) {
      transitionTo(State::REPLAN_PENDING, "connected candidate invalid");
      return;
    }
    Trajectory<5> connected_traj;
    if (!buildTrajectory(rebased, connected_traj)) {
      transitionTo(State::REPLAN_PENDING, "connected candidate rebuild failed");
      return;
    }
    double collision_time = 0.0;
    int collision_cost = 0;
    if (!trajectoryIsCollisionFree(
        rebased, connected_traj, 0.0, collision_check_horizon_sec_, collision_time, collision_cost))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "TrajectoryManager rejected connected candidate: collision at t=%.2fs cost=%d",
        collision_time, collision_cost);
      transitionTo(State::REPLAN_PENDING, "connected candidate collision");
      return;
    }
  }

  const double duration = trajectoryDuration(rebased);
  active_stamp_ = accept_stamp;
  active_traj_ = rebased;
  active_goal_id_ = msg.goal_id;
  has_active_traj_ = true;
  has_emergency_stop_ = false;
  last_projected_time_ = 0.0;
  tracking_error_replan_count_ = 0;
  ++active_seq_;
  active_traj_.trajectory_id = static_cast<uint64_t>(active_seq_);
  stampActiveTrajectoryTimeOrigin(active_stamp_, last_projected_time_);
  active_traj_.header.stamp = active_stamp_;
  transitionTo(next_state, reason);
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "TrajectoryManager accepted trajectory #%ld: waypoints=%zu dur=%.2fs rebased_from=%.2fs",
    static_cast<long>(active_seq_), active_traj_.waypoints.size(),
    duration, projected_time);
  traj_pub_->publish(active_traj_);
}

void TrajectoryManagerNode::stampActiveTrajectoryTimeOrigin(
  const rclcpp::Time & now, double projected_time)
{
  const int64_t start_ns =
    now.nanoseconds() - static_cast<int64_t>(projected_time * 1000000000.0);
  active_traj_.start_time.sec = static_cast<int32_t>(start_ns / 1000000000LL);
  active_traj_.start_time.nanosec = static_cast<uint32_t>(start_ns % 1000000000LL);
}

bool TrajectoryManagerNode::shouldSwitchToCandidate(
  const sentry_nav_interfaces::msg::MincoTrajectory & candidate_msg,
  double & candidate_projected_time,
  bool & connect_from_active)
{
  connect_from_active = true;
  const double candidate_duration = trajectoryDuration(candidate_msg);
  if (candidate_duration < switch_min_candidate_duration_sec_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "TrajectoryManager rejected candidate: duration %.2fs < %.2fs",
      candidate_duration, switch_min_candidate_duration_sec_);
    return false;
  }

  if (require_odom_ && !has_odom_) {
    return false;
  }

  Trajectory<5> active_traj;
  Trajectory<5> candidate_traj;
  if (!buildTrajectory(active_traj_, active_traj) ||
    !buildTrajectory(candidate_msg, candidate_traj))
  {
    return false;
  }

  double active_projected_time = last_projected_time_;
  if (has_odom_ &&
    !projectOdomToTrajectory(active_traj_, active_traj, last_projected_time_, active_projected_time))
  {
    return !require_odom_;
  }
  active_projected_time = std::clamp(
    std::max(active_projected_time, last_projected_time_),
    0.0, trajectoryDuration(active_traj_));
  const double active_duration = trajectoryDuration(active_traj_);
  const double remaining = active_duration - active_projected_time;
  const bool active_expiring = remaining <= switch_allow_when_remaining_sec_;
  const bool new_goal = isNewGoal(candidate_msg);

  if (has_odom_) {
    const double saved_window = projection_search_window_sec_;
    projection_search_window_sec_ = candidate_projection_window_sec_;
    if (!projectOdomToTrajectory(
        candidate_msg, candidate_traj, 0.0, candidate_projected_time))
    {
      projection_search_window_sec_ = saved_window;
      return !require_odom_;
    }
    projection_search_window_sec_ = saved_window;
  } else {
    candidate_projected_time = 0.0;
  }
  candidate_projected_time = std::clamp(candidate_projected_time, 0.0, candidate_duration);

  double collision_time = 0.0;
  int collision_cost = 0;
  if (!trajectoryIsCollisionFree(
      candidate_msg, candidate_traj, candidate_projected_time, collision_check_horizon_sec_,
      collision_time, collision_cost))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "TrajectoryManager rejected candidate: collision at t=%.2fs cost=%d",
      collision_time, collision_cost);
    return false;
  }

  const auto active_pos = active_traj.getPos(active_projected_time);
  const auto candidate_pos = candidate_traj.getPos(candidate_projected_time);
  const auto active_vel = active_traj.getVel(active_projected_time);
  const auto candidate_vel = candidate_traj.getVel(candidate_projected_time);
  const auto active_acc = active_traj.getAcc(active_projected_time);
  const auto candidate_acc = candidate_traj.getAcc(candidate_projected_time);
  const double pos_error = (active_pos - candidate_pos).head<2>().norm();
  const double vel_error = (active_vel - candidate_vel).head<2>().norm();
  const double acc_error = (active_acc - candidate_acc).head<2>().norm();
  if (new_goal || pos_error > projection_max_tracking_error_ * 2.0) {
    connect_from_active = false;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "TrajectoryManager accepting candidate from odom rebase: new_goal=%d pos_error=%.2fm "
      "candidate_t=%.2fs",
      new_goal, pos_error, candidate_projected_time);
    return true;
  }
  const double active_speed = active_vel.head<2>().norm();
  const double candidate_speed = candidate_vel.head<2>().norm();
  double direction_dot = 1.0;
  if (active_speed > 0.2 && candidate_speed > 0.2) {
    direction_dot = active_vel.head<2>().dot(candidate_vel.head<2>()) /
      std::max(active_speed * candidate_speed, 1.0e-6);
  }
  const bool velocity_mismatch = vel_error > switch_max_velocity_error_;
  if (pos_error > switch_max_position_error_ ||
    (velocity_mismatch && !active_expiring) ||
    direction_dot < switch_min_direction_dot_)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "TrajectoryManager rejected candidate: continuity pos=%.2fm vel=%.2fm/s "
      "acc(raw)=%.2fm/s^2 dir=%.2f (limits %.2f, %.2f, %.2f, %.2f)",
      pos_error, vel_error, acc_error, direction_dot,
      switch_max_position_error_, switch_max_velocity_error_,
      switch_max_acceleration_error_, switch_min_direction_dot_);
    return false;
  }
  if (velocity_mismatch) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "TrajectoryManager accepting candidate despite velocity mismatch %.2fm/s > %.2fm/s "
      "because active trajectory remaining %.2fs <= %.2fs",
      vel_error, switch_max_velocity_error_, remaining, switch_allow_when_remaining_sec_);
  }
  if (acc_error > switch_max_acceleration_error_ && !active_expiring) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "TrajectoryManager rejected candidate: acceleration mismatch %.2fm/s^2 > %.2fm/s^2",
      acc_error, switch_max_acceleration_error_);
    return false;
  }
  if (acc_error > switch_max_acceleration_error_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "TrajectoryManager accepting acceleration mismatch %.2fm/s^2 > %.2fm/s^2 "
      "because active trajectory remaining %.2fs <= %.2fs",
      acc_error, switch_max_acceleration_error_, remaining, switch_allow_when_remaining_sec_);
  }

  bool cooldown_bypass = new_goal || active_expiring;
  if (!cooldown_bypass) {
    const double active_remaining_len = std::max(remaining, 0.0);
    const double candidate_remaining_len =
      std::max(candidate_duration - candidate_projected_time, 0.0);
    const double length_ratio =
      std::abs(candidate_remaining_len - active_remaining_len) /
      std::max({active_remaining_len, candidate_remaining_len, 1.0});
    double path_deviation = 0.0;
    const int future_samples = 8;
    for (int i = 0; i <= future_samples; ++i) {
      const double horizon_t = 0.5 * static_cast<double>(i);
      const double active_t = std::min(active_duration, active_projected_time + horizon_t);
      const double candidate_t = std::min(candidate_duration, candidate_projected_time + horizon_t);
      const auto future_active_pos = active_traj.getPos(active_t);
      const auto future_candidate_pos = candidate_traj.getPos(candidate_t);
      path_deviation = std::max(
        path_deviation, (future_active_pos - future_candidate_pos).head<2>().norm());
    }
    cooldown_bypass =
      remaining <= switch_allow_when_remaining_sec_ ||
      path_deviation >= switch_immediate_deviation_ ||
      length_ratio >= switch_immediate_length_ratio_;

    const rclcpp::Time now = get_clock()->now();
    double active_age = std::numeric_limits<double>::infinity();
    try {
      active_age = (now - active_stamp_).seconds();
    } catch (const std::runtime_error &) {
      active_age = std::numeric_limits<double>::infinity();
    }
    if (!cooldown_bypass && active_age < switch_min_interval_sec_) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "TrajectoryManager retained active trajectory: switch cooldown age=%.2fs < %.2fs "
        "dev=%.2fm len_ratio=%.2f remaining=%.2fs",
        active_age, switch_min_interval_sec_, path_deviation, length_ratio, remaining);
      return false;
    }
  }

  return true;
}

void TrajectoryManagerNode::transitionTo(State next, const char * reason)
{
  if (state_ == next) {
    return;
  }
  RCLCPP_INFO(
    get_logger(), "TrajectoryManager %s -> %s (%s)",
    stateName(state_), stateName(next), reason);
  state_ = next;
  last_transition_reason_ = reason;
}

const char * TrajectoryManagerNode::stateName(State state) const
{
  switch (state) {
    case State::IDLE:
      return "IDLE";
    case State::TRACKING:
      return "TRACKING";
    case State::REPLAN_PENDING:
      return "REPLAN_PENDING";
    case State::SWITCHING:
      return "SWITCHING";
    case State::FINISHING:
      return "FINISHING";
    case State::EMERGENCY_STOP:
      return "EMERGENCY_STOP";
  }
  return "UNKNOWN";
}

void TrajectoryManagerNode::publishActiveTrajectory(const rclcpp::Time & stamp)
{
  sentry_nav_interfaces::msg::MincoTrajectory msg;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!has_active_traj_) {
      return;
    }
    msg = active_traj_;
    msg.header.stamp = stamp;
  }
  traj_pub_->publish(msg);
}

void TrajectoryManagerNode::publishStatus(const rclcpp::Time & stamp, const char * reason)
{
  if (!status_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  std::ostringstream out;
  out << "stamp=" << stamp.seconds()
      << " state=" << stateName(state_)
      << " active=" << (has_active_traj_ ? 1 : 0)
      << " trajectory_id=" << static_cast<unsigned long>(active_traj_.trajectory_id)
      << " goal_id=" << static_cast<unsigned long>(active_goal_id_)
      << " projected_time=" << last_projected_time_
      << " duration=" << (has_active_traj_ ? trajectoryDuration(active_traj_) : 0.0)
      << " reason=" << reason
      << " last_transition=" << last_transition_reason_;
  msg.data = out.str();
  status_pub_->publish(msg);
}

void TrajectoryManagerNode::publishEmergencyStop(const rclcpp::Time & stamp, const char * reason)
{
  sentry_nav_interfaces::msg::MincoTrajectory stop;
  stop.header.frame_id = active_traj_.header.frame_id.empty() ? "map" : active_traj_.header.frame_id;
  stop.header.stamp = stamp;
  stop.trajectory_id = static_cast<uint64_t>(++active_seq_);
  stop.goal_id = active_goal_id_;
  const int64_t start_ns = stamp.nanoseconds();
  stop.start_time.sec = static_cast<int32_t>(start_ns / 1000000000LL);
  stop.start_time.nanosec = static_cast<uint32_t>(start_ns % 1000000000LL);
  stop.segment_times.push_back(std::max(0.05, emergency_stop_duration_sec_));

  geometry_msgs::msg::Point p;
  bool have_stop_point = false;
  Trajectory<5> traj;
  if (has_active_traj_ && buildTrajectory(active_traj_, traj)) {
    const double t = std::clamp(last_projected_time_, 0.0, trajectoryDuration(active_traj_));
    const auto pos = traj.getPos(t);
    p.x = pos.x();
    p.y = pos.y();
    p.z = pos.z();
    have_stop_point = true;
  } else if (!active_traj_.waypoints.empty()) {
    p = active_traj_.waypoints.front();
    have_stop_point = true;
  }
  if (!have_stop_point) {
    RCLCPP_WARN(get_logger(), "TrajectoryManager emergency stop skipped: no stop point (%s)", reason);
    return;
  }

  stop.waypoints.push_back(p);
  stop.waypoints.push_back(p);
  emergency_stop_traj_ = stop;
  has_emergency_stop_ = true;
  traj_pub_->publish(stop);
  RCLCPP_WARN(get_logger(), "TrajectoryManager published emergency stop: %s", reason);
}

void TrajectoryManagerNode::publishLastEmergencyStop(const rclcpp::Time & stamp)
{
  if (!has_emergency_stop_) {
    return;
  }
  const int64_t start_ns = stamp.nanoseconds();
  emergency_stop_traj_.header.stamp = stamp;
  emergency_stop_traj_.start_time.sec = static_cast<int32_t>(start_ns / 1000000000LL);
  emergency_stop_traj_.start_time.nanosec = static_cast<uint32_t>(start_ns % 1000000000LL);
  traj_pub_->publish(emergency_stop_traj_);
}

void TrajectoryManagerNode::clearActiveTrajectory(const char * reason)
{
  has_active_traj_ = false;
  last_projected_time_ = 0.0;
  active_goal_id_ = 0;
  transitionTo(State::IDLE, reason);
}

bool TrajectoryManagerNode::validateTrajectory(
  const sentry_nav_interfaces::msg::MincoTrajectory & msg) const
{
  if (msg.waypoints.size() < 2 || msg.segment_times.size() + 1 != msg.waypoints.size()) {
    return false;
  }
  for (const auto t : msg.segment_times) {
    if (!std::isfinite(t) || t <= 1.0e-4) {
      return false;
    }
  }
  return true;
}

double TrajectoryManagerNode::trajectoryDuration(
  const sentry_nav_interfaces::msg::MincoTrajectory & msg) const
{
  return std::accumulate(msg.segment_times.begin(), msg.segment_times.end(), 0.0);
}

double TrajectoryManagerNode::endpointDistance(
  const sentry_nav_interfaces::msg::MincoTrajectory & a,
  const sentry_nav_interfaces::msg::MincoTrajectory & b) const
{
  if (a.waypoints.empty() || b.waypoints.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const auto & pa = a.waypoints.back();
  const auto & pb = b.waypoints.back();
  return std::hypot(pa.x - pb.x, pa.y - pb.y);
}

bool TrajectoryManagerNode::isNewGoal(
  const sentry_nav_interfaces::msg::MincoTrajectory & msg) const
{
  if (!has_active_traj_) {
    return false;
  }
  if (active_goal_id_ != 0 && msg.goal_id != 0 && msg.goal_id != active_goal_id_) {
    return true;
  }
  return endpointDistance(active_traj_, msg) > switch_goal_change_distance_;
}

bool TrajectoryManagerNode::cropTrajectory(
  const sentry_nav_interfaces::msg::MincoTrajectory & src,
  double crop_time,
  sentry_nav_interfaces::msg::MincoTrajectory & dst) const
{
  if (!validateTrajectory(src)) {
    return false;
  }

  Trajectory<5> traj;
  if (!buildTrajectory(src, traj)) {
    return false;
  }

  crop_time = std::max(0.0, crop_time - std::max(0.0, crop_epsilon_sec_));
  const double duration = trajectoryDuration(src);
  if (duration - crop_time < min_remaining_time_sec_) {
    return false;
  }

  size_t seg_idx = 0;
  double t_into_seg = crop_time;
  while (seg_idx < src.segment_times.size() && t_into_seg >= src.segment_times[seg_idx]) {
    t_into_seg -= src.segment_times[seg_idx];
    ++seg_idx;
  }

  if (seg_idx >= src.segment_times.size()) {
    return false;
  }

  dst = sentry_nav_interfaces::msg::MincoTrajectory();
  dst.header = src.header;

  dst.waypoints.reserve(src.waypoints.size() - seg_idx);
  dst.segment_times.reserve(src.segment_times.size() - seg_idx);

  const auto pos = traj.getPos(std::min(crop_time, duration));
  geometry_msgs::msg::Point start;
  start.x = pos.x();
  start.y = pos.y();
  start.z = pos.z();
  dst.waypoints.push_back(start);

  const double first_remaining = src.segment_times[seg_idx] - t_into_seg;
  if (first_remaining < min_remaining_time_sec_) {
    if (seg_idx + 1 >= src.segment_times.size()) {
      return false;
    }
    dst.waypoints.clear();
    dst.waypoints.push_back(src.waypoints[seg_idx + 1]);
    dst.segment_times.push_back(src.segment_times[seg_idx + 1]);
    for (size_t i = seg_idx + 2; i < src.waypoints.size(); ++i) {
      dst.waypoints.push_back(src.waypoints[i]);
    }
    for (size_t i = seg_idx + 2; i < src.segment_times.size(); ++i) {
      dst.segment_times.push_back(src.segment_times[i]);
    }
  } else {
    dst.segment_times.push_back(first_remaining);
    for (size_t i = seg_idx + 1; i < src.waypoints.size(); ++i) {
      dst.waypoints.push_back(src.waypoints[i]);
    }
    for (size_t i = seg_idx + 1; i < src.segment_times.size(); ++i) {
      dst.segment_times.push_back(src.segment_times[i]);
    }
  }

  if (!validateTrajectory(dst)) {
    return false;
  }

  const double velocity_t = first_remaining < min_remaining_time_sec_ ?
    std::min(duration, crop_time + first_remaining) : std::min(crop_time, duration);
  const auto vel = traj.getVel(velocity_t);
  const auto acc = traj.getAcc(velocity_t);
  dst.initial_velocity.x = vel.x();
  dst.initial_velocity.y = vel.y();
  dst.initial_velocity.z = vel.z();
  dst.initial_acceleration.x = acc.x();
  dst.initial_acceleration.y = acc.y();
  dst.initial_acceleration.z = acc.z();
  return true;
}

bool TrajectoryManagerNode::projectOdomToTrajectory(
  const sentry_nav_interfaces::msg::MincoTrajectory & msg,
  const Trajectory<5> & traj,
  double seed_time,
  double & projected_time)
{
  const double duration = trajectoryDuration(msg);
  if (duration <= 1.0e-3) {
    projected_time = 0.0;
    return true;
  }

  double rx = 0.0;
  double ry = 0.0;
  if (!getOdomPositionInTrajectoryFrame(msg, rx, ry)) {
    return false;
  }

  const double window = std::clamp(projection_search_window_sec_, 0.1, duration);
  double lo = seed_time <= 1.0e-3 ? 0.0 : std::max(0.0, seed_time - 0.10);
  double hi = std::min(duration, seed_time + window);
  if (hi <= lo + 1.0e-3) {
    lo = std::clamp(seed_time, 0.0, duration);
    hi = std::min(duration, lo + 0.10);
  }

  auto dist2_at = [&](double t) {
      const auto p = traj.getPos(std::clamp(t, 0.0, duration));
      const double dx = p.x() - rx;
      const double dy = p.y() - ry;
      return dx * dx + dy * dy;
    };

  const double seed_d2 = dist2_at(std::clamp(seed_time, 0.0, duration));
  const double max_error = std::max(projection_max_tracking_error_, switch_max_position_error_);
  if (seed_time > 1.0e-3 && seed_d2 > max_error * max_error) {
    projected_time = std::clamp(seed_time, 0.0, duration);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "TrajectoryManager projection held: tracking error %.2fm > %.2fm at t=%.2fs",
      std::sqrt(seed_d2), max_error, projected_time);
    return true;
  }

  double best_t = lo;
  double best_d2 = std::numeric_limits<double>::max();
  const int coarse_samples = 16;
  for (int i = 0; i <= coarse_samples; ++i) {
    const double t = lo + (hi - lo) * static_cast<double>(i) / coarse_samples;
    const double d2 = dist2_at(t);
    if (d2 < best_d2) {
      best_d2 = d2;
      best_t = t;
    }
  }

  lo = std::max(seed_time <= 1.0e-3 ? 0.0 : seed_time, best_t - 0.15);
  hi = std::min(duration, best_t + 0.15);
  for (int i = 0; i < 20; ++i) {
    const double m1 = lo + (hi - lo) * 0.382;
    const double m2 = lo + (hi - lo) * 0.618;
    if (dist2_at(m1) < dist2_at(m2)) {
      hi = m2;
    } else {
      lo = m1;
    }
  }

  projected_time = 0.5 * (lo + hi);
  return true;
}

bool TrajectoryManagerNode::getOdomPositionInTrajectoryFrame(
  const sentry_nav_interfaces::msg::MincoTrajectory & msg,
  double & x,
  double & y)
{
  const std::string & traj_frame = msg.header.frame_id;
  const std::string & odom_frame = latest_odom_.header.frame_id;
  if (traj_frame.empty() || odom_frame.empty() || traj_frame == odom_frame) {
    x = latest_odom_.pose.pose.position.x;
    y = latest_odom_.pose.pose.position.y;
    return true;
  }

  geometry_msgs::msg::PoseStamped odom_pose;
  odom_pose.header = latest_odom_.header;
  odom_pose.pose = latest_odom_.pose.pose;

  try {
    const auto tf_msg = tf_buffer_->lookupTransform(
      traj_frame, odom_frame, tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped traj_pose;
    tf2::doTransform(odom_pose, traj_pose, tf_msg);
    x = traj_pose.pose.position.x;
    y = traj_pose.pose.position.y;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "TrajectoryManager cannot transform odom from '%s' to trajectory frame '%s': %s",
      odom_frame.c_str(), traj_frame.c_str(), ex.what());
    return false;
  }
}

bool TrajectoryManagerNode::getOdomStateInTrajectoryFrame(
  const sentry_nav_interfaces::msg::MincoTrajectory & msg,
  Eigen::Vector3d & position,
  Eigen::Vector3d & velocity) const
{
  const std::string & traj_frame = msg.header.frame_id;
  const std::string & odom_frame = latest_odom_.header.frame_id;
  position = Eigen::Vector3d(
    latest_odom_.pose.pose.position.x,
    latest_odom_.pose.pose.position.y,
    latest_odom_.pose.pose.position.z);
  velocity = Eigen::Vector3d(
    latest_odom_.twist.twist.linear.x,
    latest_odom_.twist.twist.linear.y,
    latest_odom_.twist.twist.linear.z);

  if (traj_frame.empty() || odom_frame.empty() || traj_frame == odom_frame) {
    return true;
  }

  geometry_msgs::msg::PoseStamped odom_pose;
  odom_pose.header = latest_odom_.header;
  odom_pose.pose = latest_odom_.pose.pose;

  try {
    const auto tf_msg = tf_buffer_->lookupTransform(
      traj_frame, odom_frame, tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped traj_pose;
    tf2::doTransform(odom_pose, traj_pose, tf_msg);
    position = Eigen::Vector3d(
      traj_pose.pose.position.x,
      traj_pose.pose.position.y,
      traj_pose.pose.position.z);

    const auto & r = tf_msg.transform.rotation;
    tf2::Quaternion q(r.x, r.y, r.z, r.w);
    tf2::Matrix3x3 rot(q);
    const tf2::Vector3 v_odom(
      latest_odom_.twist.twist.linear.x,
      latest_odom_.twist.twist.linear.y,
      latest_odom_.twist.twist.linear.z);
    const tf2::Vector3 v_traj = rot * v_odom;
    velocity = Eigen::Vector3d(v_traj.x(), v_traj.y(), v_traj.z());
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      get_logger(),
      "TrajectoryManager cannot transform odom state from '%s' to trajectory frame '%s': %s",
      odom_frame.c_str(), traj_frame.c_str(), ex.what());
    return false;
  }
}

bool TrajectoryManagerNode::trimConnectedTrajectoryStart(
  sentry_nav_interfaces::msg::MincoTrajectory & msg,
  const Eigen::Vector3d & start,
  const Eigen::Vector3d & velocity) const
{
  if (!validateTrajectory(msg)) {
    return false;
  }

  Eigen::Vector2d forward(velocity.x(), velocity.y());
  if (forward.norm() < 0.10 && msg.waypoints.size() >= 2) {
    const auto & p1 = msg.waypoints[1];
    forward = Eigen::Vector2d(p1.x - start.x(), p1.y - start.y());
  }
  if (forward.norm() > 1.0e-3) {
    forward.normalize();
  }

  while (msg.waypoints.size() >= 3 && !msg.segment_times.empty()) {
    const auto & p1 = msg.waypoints[1];
    const Eigen::Vector2d to_next(p1.x - start.x(), p1.y - start.y());
    const double dist = to_next.norm();
    const double along = forward.norm() > 1.0e-3 ? to_next.dot(forward) : dist;
    if (dist >= 0.05 && along >= -0.03) {
      break;
    }
    msg.waypoints.erase(msg.waypoints.begin() + 1);
    msg.segment_times.erase(msg.segment_times.begin());
  }

  if (!validateTrajectory(msg)) {
    return false;
  }

  if (msg.waypoints.size() >= 2) {
    const auto & p1 = msg.waypoints[1];
    const double dist = std::hypot(p1.x - start.x(), p1.y - start.y());
    if (dist < 0.03) {
      return false;
    }
    msg.segment_times.front() = std::max(msg.segment_times.front(), 0.10);
  }
  return true;
}

bool TrajectoryManagerNode::buildTrajectory(
  const sentry_nav_interfaces::msg::MincoTrajectory & msg,
  Trajectory<5> & traj) const
{
  if (!validateTrajectory(msg)) {
    return false;
  }

  const size_t n = msg.waypoints.size();
  std::vector<Eigen::Vector3d> pts;
  pts.reserve(n);
  for (const auto & p : msg.waypoints) {
    pts.emplace_back(p.x, p.y, p.z);
  }

  Eigen::VectorXd times(static_cast<Eigen::Index>(n - 1));
  for (size_t i = 0; i < n - 1; ++i) {
    times(static_cast<Eigen::Index>(i)) = msg.segment_times[i];
  }

  const int pieces = static_cast<int>(n) - 1;
  Eigen::Matrix3d head = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d tail = Eigen::Matrix3d::Zero();
  head.col(0) = pts.front();
  head.col(1) = Eigen::Vector3d(
    msg.initial_velocity.x, msg.initial_velocity.y, msg.initial_velocity.z);
  head.col(2) = Eigen::Vector3d(
    msg.initial_acceleration.x, msg.initial_acceleration.y, msg.initial_acceleration.z);
  tail.col(0) = pts.back();

  Eigen::Matrix3Xd inner(3, std::max(0, pieces - 1));
  for (int i = 1; i < pieces; ++i) {
    inner.col(i - 1) = pts[static_cast<size_t>(i)];
  }

  minco::MINCO_S3NU solver;
  solver.setConditions(head, tail, pieces);
  solver.setParameters(inner, times);
  solver.getTrajectory(traj);
  return traj.getPieceNum() > 0;
}

bool TrajectoryManagerNode::worldToCostmap(
  double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  if (!has_costmap_ || latest_costmap_.metadata.resolution <= 0.0) {
    return false;
  }
  const auto & origin = latest_costmap_.metadata.origin;
  const double origin_x = origin.position.x;
  const double origin_y = origin.position.y;
  const double dx = wx - origin_x;
  const double dy = wy - origin_y;
  const double yaw = tf2::getYaw(origin.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double local_x = cos_yaw * dx + sin_yaw * dy;
  const double local_y = -sin_yaw * dx + cos_yaw * dy;
  if (local_x < 0.0 || local_y < 0.0) {
    return false;
  }
  mx = static_cast<unsigned int>(local_x / latest_costmap_.metadata.resolution);
  my = static_cast<unsigned int>(local_y / latest_costmap_.metadata.resolution);
  return mx < latest_costmap_.metadata.size_x && my < latest_costmap_.metadata.size_y;
}

bool TrajectoryManagerNode::costmapPointIsCollisionFree(
  double wx,
  double wy,
  double & collision_x,
  double & collision_y,
  int & collision_cost) const
{
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!worldToCostmap(wx, wy, mx, my)) {
    if (allow_unknown_costmap_) {
      return true;
    }
    collision_x = wx;
    collision_y = wy;
    collision_cost = -1;
    return false;
  }
  const size_t idx = static_cast<size_t>(my) * latest_costmap_.metadata.size_x + mx;
  if (idx >= latest_costmap_.data.size()) {
    return true;
  }
  const int cost = static_cast<int>(latest_costmap_.data[idx]);
  if (cost == static_cast<int>(nav2_costmap_2d::NO_INFORMATION)) {
    if (allow_unknown_costmap_) {
      return true;
    }
    collision_x = wx;
    collision_y = wy;
    collision_cost = cost;
    return false;
  }
  if (cost >= collision_cost_threshold_) {
    collision_x = wx;
    collision_y = wy;
    collision_cost = cost;
    return false;
  }
  return true;
}

bool TrajectoryManagerNode::trajectoryIsCollisionFree(
  const sentry_nav_interfaces::msg::MincoTrajectory & msg,
  const Trajectory<5> & traj,
  double start_time,
  double horizon_sec,
  double & collision_time,
  int & collision_cost) const
{
  if (!enable_forward_collision_check_ || hole_collision_policy_ || !has_costmap_) {
    return true;
  }

  geometry_msgs::msg::TransformStamped tf_msg;
  const std::string & traj_frame = msg.header.frame_id;
  const std::string & costmap_frame = latest_costmap_.header.frame_id;
  const bool need_tf = !traj_frame.empty() && !costmap_frame.empty() && traj_frame != costmap_frame;
  if (need_tf) {
    try {
      tf_msg = tf_buffer_->lookupTransform(costmap_frame, traj_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(),
        "TrajectoryManager collision check skipped: TF '%s' -> '%s' failed: %s",
        traj_frame.c_str(), costmap_frame.c_str(), ex.what());
      if (collision_tf_failure_is_safe_) {
        return true;
      }
      collision_time = start_time;
      collision_cost = -2;
      return false;
    }
  }

  const double duration = trajectoryDuration(msg);
  const double step = std::clamp(collision_check_step_sec_, 0.02, 0.50);
  const double t_end = std::min(duration, start_time + std::max(0.0, horizon_sec));
  for (double t = std::clamp(start_time, 0.0, duration); t <= t_end + 1.0e-6; t += step) {
    const auto p = traj.getPos(t);
    geometry_msgs::msg::PointStamped in;
    in.header.frame_id = traj_frame;
    in.point.x = p.x();
    in.point.y = p.y();
    in.point.z = p.z();
    geometry_msgs::msg::PointStamped out = in;
    if (need_tf) {
      tf2::doTransform(in, out, tf_msg);
    }

    double collision_x = out.point.x;
    double collision_y = out.point.y;
    const double radius = std::max(0.0, collision_check_radius_);
    const double sample_step =
      std::max(static_cast<double>(latest_costmap_.metadata.resolution), 0.05);
    std::vector<std::pair<double, double>> offsets{{0.0, 0.0}};
    if (radius > 1.0e-3) {
      offsets.push_back({radius, 0.0});
      offsets.push_back({-radius, 0.0});
      offsets.push_back({0.0, radius});
      offsets.push_back({0.0, -radius});
      const double diag = radius * 0.70710678118;
      offsets.push_back({diag, diag});
      offsets.push_back({diag, -diag});
      offsets.push_back({-diag, diag});
      offsets.push_back({-diag, -diag});
      for (double r = sample_step; r < radius - 1.0e-6; r += sample_step) {
        offsets.push_back({r, 0.0});
        offsets.push_back({-r, 0.0});
        offsets.push_back({0.0, r});
        offsets.push_back({0.0, -r});
      }
    }

    for (const auto & offset : offsets) {
      if (!costmapPointIsCollisionFree(
          out.point.x + offset.first, out.point.y + offset.second,
          collision_x, collision_y, collision_cost))
      {
        collision_time = t;
        return false;
      }
    }
  }

  return true;
}

}  // namespace sirb_smoother

RCLCPP_COMPONENTS_REGISTER_NODE(sirb_smoother::TrajectoryManagerNode)
