#include "sirb_nav2_plugins/bt_nodes/path_gate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sirb_nav2_plugins
{

PathGate::PathGate(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  try {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  } catch (const std::exception &) {
    node_.reset();
  }
}

BT::NodeStatus PathGate::tick()
{
  nav_msgs::msg::Path candidate_path;
  if (!getInput("candidate_path", candidate_path) || candidate_path.poses.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  nav_msgs::msg::Path tracking_path;
  const bool has_tracking_path =
    getInput("tracking_path", tracking_path) && !tracking_path.poses.empty();

  bool accept_empty_tracking_path = true;
  double goal_update_distance = 0.5;
  double path_update_distance = 0.25;
  double forward_compare_distance = 3.0;
  double global_path_update_distance = 0.45;
  double length_update_ratio = 0.10;
  double min_accept_interval_sec = 2.5;
  double immediate_update_multiplier = 2.0;
  bool fail_on_reject = false;
  getInput("accept_empty_tracking_path", accept_empty_tracking_path);
  getInput("goal_update_distance", goal_update_distance);
  getInput("path_update_distance", path_update_distance);
  getInput("forward_compare_distance", forward_compare_distance);
  getInput("global_path_update_distance", global_path_update_distance);
  getInput("length_update_ratio", length_update_ratio);
  getInput("min_accept_interval_sec", min_accept_interval_sec);
  getInput("immediate_update_multiplier", immediate_update_multiplier);
  getInput("fail_on_reject", fail_on_reject);
  min_accept_interval_sec = std::max(0.0, min_accept_interval_sec);
  immediate_update_multiplier = std::max(1.0, immediate_update_multiplier);

  bool accept_candidate = false;
  const char * reason = "tracking_path_retained";
  bool goal_moved = false;
  bool immediate_geometry_change = false;
  double goal_delta = 0.0;
  double forward_delta = 0.0;
  double global_delta = 0.0;
  double length_ratio = 0.0;
  if (!has_tracking_path) {
    accept_candidate = accept_empty_tracking_path;
    reason = accept_candidate ? "empty_tracking_path" : "empty_tracking_path_rejected";
  } else {
    goal_delta = goalDistance(candidate_path, tracking_path);
    const double candidate_len = pathLength(candidate_path);
    const double tracking_len = pathLength(tracking_path);
    const double max_len = std::max({candidate_len, tracking_len, 1.0});
    length_ratio = std::abs(candidate_len - tracking_len) / max_len;

    forward_delta = std::max(
      pathDeviation(candidate_path, tracking_path, forward_compare_distance, 24),
      pathDeviation(tracking_path, candidate_path, forward_compare_distance, 24));
    global_delta = std::max(
      pathDeviation(candidate_path, tracking_path, std::numeric_limits<double>::infinity(), 48),
      pathDeviation(tracking_path, candidate_path, std::numeric_limits<double>::infinity(), 48));

    if (goal_delta >= goal_update_distance) {
      accept_candidate = true;
      reason = "goal_moved";
      goal_moved = true;
    } else if (forward_delta >= path_update_distance) {
      accept_candidate = true;
      reason = "forward_geometry_changed";
    } else if (global_delta >= global_path_update_distance) {
      accept_candidate = true;
      reason = "global_geometry_changed";
    } else if (length_ratio >= length_update_ratio) {
      accept_candidate = true;
      reason = "length_changed";
    }

    immediate_geometry_change =
      forward_delta >= path_update_distance * immediate_update_multiplier ||
      global_delta >= global_path_update_distance * immediate_update_multiplier ||
      length_ratio >= length_update_ratio * immediate_update_multiplier;
  }

  const auto now = std::chrono::steady_clock::now();
  double accept_interval = std::numeric_limits<double>::infinity();
  if (has_last_accept_) {
    accept_interval = std::chrono::duration<double>(now - last_accept_time_).count();
  }
  if (accept_candidate && has_tracking_path && !goal_moved && !immediate_geometry_change &&
    has_last_accept_ && accept_interval < min_accept_interval_sec)
  {
    accept_candidate = false;
    reason = "same_goal_cooldown";
  }

  if (accept_candidate) {
    setOutput("output_path", candidate_path);
    last_accept_time_ = now;
    has_last_accept_ = true;
  } else {
    setOutput("output_path", tracking_path);
  }

  if (node_) {
    if (accept_candidate) {
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "PathGate accepted candidate: reason=%s goal=%.2f forward=%.2f global=%.2f len_ratio=%.2f interval=%.2f",
        reason, goal_delta, forward_delta, global_delta, length_ratio, accept_interval);
    } else {
      RCLCPP_DEBUG_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "PathGate retained tracking path: reason=%s goal=%.2f forward=%.2f global=%.2f len_ratio=%.2f interval=%.2f",
        reason, goal_delta, forward_delta, global_delta, length_ratio, accept_interval);
    }
  }

  return accept_candidate || !fail_on_reject ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

double PathGate::goalDistance(
  const nav_msgs::msg::Path & a,
  const nav_msgs::msg::Path & b) const
{
  if (a.poses.empty() || b.poses.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const auto & pa = a.poses.back().pose.position;
  const auto & pb = b.poses.back().pose.position;
  return std::hypot(pa.x - pb.x, pa.y - pb.y);
}

double PathGate::pathLength(const nav_msgs::msg::Path & path) const
{
  double length = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    length += std::hypot(b.x - a.x, b.y - a.y);
  }
  return length;
}

double PathGate::pathDeviation(
  const nav_msgs::msg::Path & source,
  const nav_msgs::msg::Path & target,
  double max_source_arc,
  size_t max_samples) const
{
  if (source.poses.empty() || target.poses.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  if (source.poses.size() == 1) {
    const auto & p = source.poses.front().pose.position;
    return pointToPathDistance(p.x, p.y, target);
  }

  const size_t sample_stride = std::max<size_t>(1, source.poses.size() / std::max<size_t>(1, max_samples));
  double max_dev = 0.0;
  double arc = 0.0;
  bool sampled_end = false;
  for (size_t i = 0; i < source.poses.size(); i += sample_stride) {
    if (i > 0) {
      for (size_t j = i - sample_stride + 1; j <= i && j < source.poses.size(); ++j) {
        const auto & a = source.poses[j - 1].pose.position;
        const auto & b = source.poses[j].pose.position;
        arc += std::hypot(b.x - a.x, b.y - a.y);
      }
    }
    if (arc > max_source_arc) {
      break;
    }
    const auto & p = source.poses[i].pose.position;
    max_dev = std::max(max_dev, pointToPathDistance(p.x, p.y, target));
    if (i + sample_stride >= source.poses.size()) {
      sampled_end = true;
    }
  }

  if (!sampled_end && !std::isfinite(max_source_arc)) {
    const auto & p = source.poses.back().pose.position;
    max_dev = std::max(max_dev, pointToPathDistance(p.x, p.y, target));
  }
  return max_dev;
}

double PathGate::pointToPathDistance(
  double x,
  double y,
  const nav_msgs::msg::Path & path) const
{
  if (path.poses.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  if (path.poses.size() == 1) {
    const auto & p = path.poses.front().pose.position;
    return std::hypot(x - p.x, y - p.y);
  }

  double min_dist_sq = std::numeric_limits<double>::max();
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len_sq = dx * dx + dy * dy;
    double t = 0.0;
    if (len_sq > 1.0e-12) {
      t = ((x - a.x) * dx + (y - a.y) * dy) / len_sq;
      t = std::clamp(t, 0.0, 1.0);
    }
    const double px = a.x + t * dx;
    const double py = a.y + t * dy;
    const double ex = x - px;
    const double ey = y - py;
    min_dist_sq = std::min(min_dist_sq, ex * ex + ey * ey);
  }
  return std::sqrt(min_dist_sq);
}

}  // namespace sirb_nav2_plugins
