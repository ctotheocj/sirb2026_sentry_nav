#ifndef SIRB_NAV2_PLUGINS__BT_NODES__ENEMY_DETECTED_CONDITION_HPP_
#define SIRB_NAV2_PLUGINS__BT_NODES__ENEMY_DETECTED_CONDITION_HPP_

#include <string>
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace sirb_nav2_plugins
{

/**
 * @brief 示例：检测是否发现敌人的 BT 条件节点
 * @details 可以订阅敌方位置topic，返回 SUCCESS/FAILURE
 */
class EnemyDetectedCondition : public BT::ConditionNode
{
public:
  EnemyDetectedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  ~EnemyDetectedCondition() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("timeout", 1.0, "检测超时时间(秒)"),
      BT::InputPort<double>("min_distance", 2.0, "最小敌人距离阈值(米)")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  // TODO: 添加敌人检测订阅器
};

}  // namespace sirb_nav2_plugins

#endif  // SIRB_NAV2_PLUGINS__BT_NODES__ENEMY_DETECTED_CONDITION_HPP_
