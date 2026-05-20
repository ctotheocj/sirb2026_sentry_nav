#ifndef SIRB_NAV2_PLUGINS__BT_NODES__LOCALIZATION_READY_CONDITION_HPP_
#define SIRB_NAV2_PLUGINS__BT_NODES__LOCALIZATION_READY_CONDITION_HPP_

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sirb_nav2_plugins
{

class LocalizationReadyCondition : public BT::StatefulActionNode
{
public:
  LocalizationReadyCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  struct StatusSnapshot
  {
    rclcpp::Time received_time{0, 0, RCL_ROS_TIME};
    std::string state{"UNKNOWN"};
    std::string message;
    bool trusted{false};
    bool have_status{false};
  };

  void diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void diagnosticsCallbackForTopic(
    const std::string & topic,
    const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void ensureSubscriptions(const std::vector<std::string> & diagnostics_topics);
  std::vector<std::string> getDiagnosticsTopics();
  BT::NodeStatus checkReady();

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::vector<rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr>
    diagnostics_subs_;

  mutable std::mutex mutex_;
  std::vector<std::string> diagnostics_topics_;
  std::unordered_map<std::string, StatusSnapshot> status_by_topic_;
  int ready_count_{0};
  bool last_ready_{false};
  bool has_reported_state_{false};
  std::string last_reported_topic_;
  std::string last_reported_state_;
  std::string last_reported_message_;
  bool last_reported_trusted_{false};
  bool last_reported_stale_{false};
};

}  // namespace sirb_nav2_plugins

#endif  // SIRB_NAV2_PLUGINS__BT_NODES__LOCALIZATION_READY_CONDITION_HPP_
