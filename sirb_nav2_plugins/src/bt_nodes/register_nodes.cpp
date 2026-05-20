#include "behaviortree_cpp_v3/bt_factory.h"

#include "sirb_nav2_plugins/bt_nodes/enemy_detected_condition.hpp"
#include "sirb_nav2_plugins/bt_nodes/select_nearby_goal.hpp"
#include "sirb_nav2_plugins/bt_nodes/corridor_replan_condition.hpp"
#include "sirb_nav2_plugins/bt_nodes/path_gate.hpp"
#include "sirb_nav2_plugins/bt_nodes/localization_ready_condition.hpp"
#include "sirb_nav2_plugins/bt_nodes/is_hole_mode_active_condition.hpp"
#include "sirb_nav2_plugins/bt_nodes/hole_approach_condition.hpp"
#include "sirb_nav2_plugins/bt_nodes/hole_pass_scope.hpp"
#include "sirb_nav2_plugins/bt_nodes/commit_trajectory_action.hpp"
#include "sirb_nav2_plugins/bt_nodes/generate_minco_candidate_action.hpp"
#include "sirb_nav2_plugins/bt_nodes/pass_hole_action.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sirb_nav2_plugins::EnemyDetectedCondition>("EnemyDetected");
  factory.registerNodeType<sirb_nav2_plugins::SelectNearbyGoal>("SelectNearbyGoal");
  factory.registerNodeType<sirb_nav2_plugins::ReplanCondition>("ReplanCondition");
  factory.registerNodeType<sirb_nav2_plugins::PathGate>("PathGate");
  factory.registerNodeType<sirb_nav2_plugins::LocalizationReadyCondition>("LocalizationReady");
  factory.registerNodeType<sirb_nav2_plugins::IsHoleModeActiveCondition>("IsHoleModeActive");
  factory.registerNodeType<sirb_nav2_plugins::HoleApproachCondition>("HoleApproachCondition");
  factory.registerNodeType<sirb_nav2_plugins::HolePassScope>("HolePassScope");
  BT::NodeBuilder commit_builder =
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<sirb_nav2_plugins::CommitTrajectoryAction>(
        name, "trajectory_manager/commit_trajectory", config);
    };
  factory.registerBuilder<sirb_nav2_plugins::CommitTrajectoryAction>(
    "CommitTrajectory", commit_builder);
  BT::NodeBuilder generate_builder =
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<sirb_nav2_plugins::GenerateMincoCandidateAction>(
        name, "safe_geometric_smoother/generate_minco_candidate", config);
    };
  factory.registerBuilder<sirb_nav2_plugins::GenerateMincoCandidateAction>(
    "GenerateMincoCandidate", generate_builder);
  BT::NodeBuilder pass_hole_builder =
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<sirb_nav2_plugins::PassHoleAction>(
        name, "pass_hole", config);
    };
  factory.registerBuilder<sirb_nav2_plugins::PassHoleAction>(
    "PassHole", pass_hole_builder);
}
