#include "sirb_nav2_plugins/bt_nodes/pass_hole_action.hpp"

#include <algorithm>

namespace sirb_nav2_plugins
{

PassHoleAction::PassHoleAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<sentry_nav_interfaces::action::PassHole>(
    xml_tag_name, action_name, conf)
{
}

void PassHoleAction::on_tick()
{
  std::string hole_id;
  if (!getInput("hole_id", hole_id) || hole_id.empty()) {
    should_send_goal_ = false;
    return;
  }
  goal_.hole_id = hole_id;
  getInput("entry_port", goal_.entry_port);
  getInput("entry_pose", goal_.entry_pose);
  getInput("exit_pose", goal_.exit_pose);
  getInput("entry_polygon", goal_.entry_polygon);
  getInput("exit_polygon", goal_.exit_polygon);
  getInput("corridor", goal_.corridor);

  double timeout_sec = 8.0;
  getInput("timeout_sec", timeout_sec);
  goal_.timeout_sec = static_cast<float>(std::max(0.1, timeout_sec));
}

BT::NodeStatus PassHoleAction::on_success()
{
  if (result_.result) {
    setOutput("reason", result_.result->reason);
    setOutput("final_stage", result_.result->final_stage);
    return result_.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus PassHoleAction::on_aborted()
{
  if (result_.result) {
    setOutput("reason", result_.result->reason);
    setOutput("final_stage", result_.result->final_stage);
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus PassHoleAction::on_cancelled()
{
  if (result_.result) {
    setOutput("reason", result_.result->reason);
    setOutput("final_stage", result_.result->final_stage);
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace sirb_nav2_plugins
