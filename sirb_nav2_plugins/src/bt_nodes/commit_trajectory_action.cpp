#include "sirb_nav2_plugins/bt_nodes/commit_trajectory_action.hpp"

namespace sirb_nav2_plugins
{

CommitTrajectoryAction::CommitTrajectoryAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<sentry_nav_interfaces::action::CommitTrajectory>(
    xml_tag_name, action_name, conf)
{
}

void CommitTrajectoryAction::on_tick()
{
  candidate_path_ = nav_msgs::msg::Path();
  const auto candidate_path_result = getInput("candidate_path", candidate_path_);
  if (!candidate_path_result) {
    RCLCPP_WARN(
      node_->get_logger(),
      "CommitTrajectory: skip tick, missing candidate_path blackboard value");
    should_send_goal_ = false;
    return;
  }
  if (candidate_path_.poses.empty()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "CommitTrajectory: skip tick, candidate_path is empty");
    should_send_goal_ = false;
    return;
  }

  const auto candidate_minco_result = getInput("candidate_minco", goal_.candidate_minco);
  if (!candidate_minco_result) {
    RCLCPP_WARN(
      node_->get_logger(),
      "CommitTrajectory: skip tick, missing candidate_minco blackboard value");
    should_send_goal_ = false;
    return;
  }
  if (goal_.candidate_minco.waypoints.size() < 2 || goal_.candidate_minco.segment_times.empty()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "CommitTrajectory: skip tick, invalid candidate_minco waypoints=%zu segments=%zu",
      goal_.candidate_minco.waypoints.size(), goal_.candidate_minco.segment_times.size());
    should_send_goal_ = false;
    return;
  }

  bool allow_keep_active_on_reject = true;
  getInput("allow_keep_active_on_reject", allow_keep_active_on_reject);
  goal_.allow_keep_active_on_reject = allow_keep_active_on_reject;
  bool prefer_keep_active = false;
  getInput("prefer_keep_active", prefer_keep_active);
  goal_.prefer_keep_active = prefer_keep_active;

  RCLCPP_INFO(
    node_->get_logger(),
    "CommitTrajectory: tick path_poses=%zu frame='%s' minco_waypoints=%zu minco_segments=%zu "
    "allow_keep_active_on_reject=%d prefer_keep_active=%d",
    candidate_path_.poses.size(), candidate_path_.header.frame_id.c_str(),
    goal_.candidate_minco.waypoints.size(), goal_.candidate_minco.segment_times.size(),
    goal_.allow_keep_active_on_reject, goal_.prefer_keep_active);
}

BT::NodeStatus CommitTrajectoryAction::on_success()
{
  const auto & result = result_.result;
  if (!result) {
    RCLCPP_WARN(
      node_->get_logger(),
      "CommitTrajectory: action succeeded without a result payload");
    setOutput("accepted", false);
    setOutput("active_valid", false);
    setOutput("reason", std::string("missing result"));
    return BT::NodeStatus::FAILURE;
  }

  setOutput("accepted", result->accepted);
  setOutput("active_valid", result->active_valid);
  setOutput("reason", result->reason);

  RCLCPP_INFO(
    node_->get_logger(),
    "CommitTrajectory: result accepted=%d active_valid=%d trajectory_id=%lu reason='%s'",
    result->accepted, result->active_valid,
    static_cast<unsigned long>(result->trajectory_id), result->reason.c_str());

  if (result->accepted) {
    setOutput("tracking_path", candidate_path_);
  }

  return result->accepted || result->active_valid ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus CommitTrajectoryAction::on_aborted()
{
  if (result_.result) {
    setOutput("accepted", result_.result->accepted);
    setOutput("active_valid", result_.result->active_valid);
    setOutput("reason", result_.result->reason);
    RCLCPP_WARN(
      node_->get_logger(),
      "CommitTrajectory: aborted accepted=%d active_valid=%d trajectory_id=%lu reason='%s'",
      result_.result->accepted, result_.result->active_valid,
      static_cast<unsigned long>(result_.result->trajectory_id),
      result_.result->reason.c_str());
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "CommitTrajectory: aborted without result payload");
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace sirb_nav2_plugins
