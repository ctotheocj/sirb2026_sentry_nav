#ifndef SIRB_NAV2_PLUGINS__BT_NODES__IS_HOLE_MODE_ACTIVE_CONDITION_HPP_
#define SIRB_NAV2_PLUGINS__BT_NODES__IS_HOLE_MODE_ACTIVE_CONDITION_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"

namespace sirb_nav2_plugins
{

class IsHoleModeActiveCondition : public BT::ConditionNode
{
public:
  IsHoleModeActiveCondition(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("blackboard_key", "hole_mode_active", "Blackboard flag key"),
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace sirb_nav2_plugins

#endif  // SIRB_NAV2_PLUGINS__BT_NODES__IS_HOLE_MODE_ACTIVE_CONDITION_HPP_
