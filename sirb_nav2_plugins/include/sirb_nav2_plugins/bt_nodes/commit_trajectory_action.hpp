#ifndef SIRB_NAV2_PLUGINS__BT_NODES__COMMIT_TRAJECTORY_ACTION_HPP_
#define SIRB_NAV2_PLUGINS__BT_NODES__COMMIT_TRAJECTORY_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sentry_nav_interfaces/action/commit_trajectory.hpp"
#include "sentry_nav_interfaces/msg/minco_trajectory.hpp"

namespace sirb_nav2_plugins
{

class CommitTrajectoryAction
  : public nav2_behavior_tree::BtActionNode<sentry_nav_interfaces::action::CommitTrajectory>
{
public:
  CommitTrajectoryAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Path>("candidate_path", "Candidate smoothed path"),
        BT::InputPort<sentry_nav_interfaces::msg::MincoTrajectory>(
          "candidate_minco", "Candidate MINCO trajectory to atomically commit"),
        BT::InputPort<bool>(
          "allow_keep_active_on_reject", true,
          "Return success when commit is rejected but the old active trajectory remains valid"),
        BT::OutputPort<nav_msgs::msg::Path>("tracking_path", "Committed path for FollowPath"),
        BT::OutputPort<bool>("accepted", "True when the candidate trajectory was committed"),
        BT::OutputPort<bool>("active_valid", "True when trajectory manager still has an active trajectory"),
        BT::OutputPort<std::string>("reason", "Commit decision reason"),
      });
  }

private:
  nav_msgs::msg::Path candidate_path_;
};

}  // namespace sirb_nav2_plugins

#endif  // SIRB_NAV2_PLUGINS__BT_NODES__COMMIT_TRAJECTORY_ACTION_HPP_
