#ifndef SIRB_NAV2_PLUGINS__BT_NODES__PASS_HOLE_ACTION_HPP_
#define SIRB_NAV2_PLUGINS__BT_NODES__PASS_HOLE_ACTION_HPP_

#include <string>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "sentry_nav_interfaces/action/pass_hole.hpp"

namespace sirb_nav2_plugins
{

class PassHoleAction
  : public nav2_behavior_tree::BtActionNode<sentry_nav_interfaces::action::PassHole>
{
public:
  PassHoleAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("hole_id", "Hole id"),
        BT::InputPort<std::string>("entry_port", "", "Entry port name"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("entry_pose", "Entry pose"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("exit_pose", "Exit pose"),
        BT::InputPort<geometry_msgs::msg::PolygonStamped>("corridor", "Hole corridor"),
        BT::InputPort<double>("timeout_sec", 8.0, "Pass hole timeout"),
        BT::OutputPort<std::string>("reason", "Result reason"),
        BT::OutputPort<std::string>("final_stage", "Final hole pass stage"),
      });
  }
};

}  // namespace sirb_nav2_plugins

#endif  // SIRB_NAV2_PLUGINS__BT_NODES__PASS_HOLE_ACTION_HPP_
