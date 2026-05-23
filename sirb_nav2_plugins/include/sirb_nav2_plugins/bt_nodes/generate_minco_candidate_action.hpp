#ifndef SIRB_NAV2_PLUGINS__BT_NODES__GENERATE_MINCO_CANDIDATE_ACTION_HPP_
#define SIRB_NAV2_PLUGINS__BT_NODES__GENERATE_MINCO_CANDIDATE_ACTION_HPP_

#include <string>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sentry_nav_interfaces/action/generate_minco_candidate.hpp"
#include "sentry_nav_interfaces/msg/minco_trajectory.hpp"

namespace sirb_nav2_plugins
{

class GenerateMincoCandidateAction
  : public nav2_behavior_tree::BtActionNode<sentry_nav_interfaces::action::GenerateMincoCandidate>
{
public:
  GenerateMincoCandidateAction(
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
        BT::InputPort<nav_msgs::msg::Path>("input_path", "Raw planner path"),
        BT::InputPort<double>(
          "max_smoothing_duration", 0.5,
          "Maximum candidate generation time in seconds"),
        BT::InputPort<std::string>(
          "smoother_id", "safe_geometric_smoother",
          "Smoother instance id"),
        BT::OutputPort<nav_msgs::msg::Path>("smoothed_path", "Candidate smoothed path"),
        BT::OutputPort<sentry_nav_interfaces::msg::MincoTrajectory>(
          "candidate_minco", "Candidate MINCO trajectory"),
        BT::OutputPort<std::string>("reason", "Candidate generation decision reason"),
        BT::OutputPort<std::string>("product_type", "Generated trajectory product type"),
	        BT::OutputPort<bool>(
	          "prefer_keep_active",
	          "True only for degraded fallback products; normal time_reference may replace active trajectory"),
      });
  }
};

}  // namespace sirb_nav2_plugins

#endif  // SIRB_NAV2_PLUGINS__BT_NODES__GENERATE_MINCO_CANDIDATE_ACTION_HPP_
