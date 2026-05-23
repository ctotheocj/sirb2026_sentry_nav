#include "sirb_nav2_plugins/bt_nodes/generate_minco_candidate_action.hpp"

#include <algorithm>
#include <cmath>

namespace sirb_nav2_plugins
{

GenerateMincoCandidateAction::GenerateMincoCandidateAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<sentry_nav_interfaces::action::GenerateMincoCandidate>(
    xml_tag_name, action_name, conf)
{
}

void GenerateMincoCandidateAction::on_tick()
{
  nav_msgs::msg::Path input_path;
  const auto input_path_result = getInput("input_path", input_path);
  if (!input_path_result) {
    RCLCPP_WARN(
      node_->get_logger(),
      "GenerateMincoCandidate: skip tick, missing input_path blackboard value");
    should_send_goal_ = false;
    return;
  }
  if (input_path.poses.empty()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "GenerateMincoCandidate: skip tick, input_path is empty");
    should_send_goal_ = false;
    return;
  }

  double max_smoothing_duration = 0.5;
  getInput("max_smoothing_duration", max_smoothing_duration);
  max_smoothing_duration = std::max(0.01, max_smoothing_duration);

  std::string smoother_id = "safe_geometric_smoother";
  getInput("smoother_id", smoother_id);

  RCLCPP_DEBUG(
    node_->get_logger(),
    "GenerateMincoCandidate: tick path_poses=%zu frame='%s' smoother_id='%s' max_duration=%.3fs",
    input_path.poses.size(), input_path.header.frame_id.c_str(), smoother_id.c_str(),
    max_smoothing_duration);

  goal_.input_path = input_path;
  goal_.smoother_id = smoother_id;
  const int32_t sec = static_cast<int32_t>(std::floor(max_smoothing_duration));
  const double frac = max_smoothing_duration - static_cast<double>(sec);
  goal_.max_smoothing_duration.sec = sec;
  goal_.max_smoothing_duration.nanosec =
    static_cast<uint32_t>(std::llround(frac * 1.0e9));
}

BT::NodeStatus GenerateMincoCandidateAction::on_success()
{
  const auto & result = result_.result;
  if (!result) {
    RCLCPP_WARN(
      node_->get_logger(),
      "GenerateMincoCandidate: action succeeded without a result payload");
    setOutput("reason", std::string("missing result"));
    return BT::NodeStatus::FAILURE;
  }

  setOutput("reason", result->reason);
  setOutput("product_type", result->product_type);
  setOutput("prefer_keep_active", result->prefer_keep_active);
  RCLCPP_DEBUG(
    node_->get_logger(),
    "GenerateMincoCandidate: result success=%d product='%s' prefer_keep_active=%d "
    "path_poses=%zu minco_waypoints=%zu minco_segments=%zu reason='%s'",
    result->success, result->product_type.c_str(), result->prefer_keep_active,
    result->smoothed_path.poses.size(),
    result->candidate_minco.waypoints.size(), result->candidate_minco.segment_times.size(),
    result->reason.c_str());

  if (!result->success) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("smoothed_path", result->smoothed_path);
  setOutput("candidate_minco", result->candidate_minco);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GenerateMincoCandidateAction::on_aborted()
{
  if (result_.result) {
    setOutput("reason", result_.result->reason);
    RCLCPP_WARN(
      node_->get_logger(),
      "GenerateMincoCandidate: aborted reason='%s'",
      result_.result->reason.c_str());
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "GenerateMincoCandidate: aborted without result payload");
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace sirb_nav2_plugins
