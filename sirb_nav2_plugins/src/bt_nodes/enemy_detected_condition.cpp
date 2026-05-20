#include "sirb_nav2_plugins/bt_nodes/enemy_detected_condition.hpp"

namespace sirb_nav2_plugins
{

EnemyDetectedCondition::EnemyDetectedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  
  // TODO: 在此初始化订阅器，订阅敌人位置 topic
  // enemy_sub_ = node_->create_subscription<YourMsgType>(...);
}

BT::NodeStatus EnemyDetectedCondition::tick()
{
  double timeout, min_distance;
  getInput("timeout", timeout);
  getInput("min_distance", min_distance);

  // TODO: 实现敌人检测逻辑
  // 示例：检查是否有敌人在 min_distance 范围内
  bool enemy_detected = false; 

  if (enemy_detected) {
    RCLCPP_INFO(node_->get_logger(), "敌人已检测到，触发战术切换");
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace sirb_nav2_plugins

// BT nodes are registered in register_nodes.cpp
