#include "sirb_nav2_plugins/bt_nodes/is_hole_mode_active_condition.hpp"

namespace sirb_nav2_plugins
{

IsHoleModeActiveCondition::IsHoleModeActiveCondition(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
}

BT::NodeStatus IsHoleModeActiveCondition::tick()
{
  std::string key = "hole_mode_active";
  getInput("blackboard_key", key);

  bool active = false;
  if (config().blackboard->get(key, active) && active) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace sirb_nav2_plugins
