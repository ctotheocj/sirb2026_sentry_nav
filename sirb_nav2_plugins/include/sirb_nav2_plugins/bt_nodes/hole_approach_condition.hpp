#ifndef SIRB_NAV2_PLUGINS__BT_NODES__HOLE_APPROACH_CONDITION_HPP_
#define SIRB_NAV2_PLUGINS__BT_NODES__HOLE_APPROACH_CONDITION_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentry_nav_interfaces/msg/minco_trajectory.hpp"
#include "tf2_ros/buffer.h"

namespace sirb_nav2_plugins
{

class HoleApproachCondition : public BT::ConditionNode
{
public:
  HoleApproachCondition(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("trajectory_topic", "trajectory_manager/trajectory_for_mpc", "Active trajectory topic"),
      BT::InputPort<std::string>("global_frame", "map", "Global frame"),
      BT::InputPort<std::string>("robot_frame", "gimbal_yaw_fake", "Robot frame"),
      BT::InputPort<double>("prepare_distance", 1.5, "Lookahead distance to hole entry"),
      BT::InputPort<double>("exit_goal_offset", 0.4, "Distance beyond exit port for hole exit goal"),
      BT::InputPort<double>("alignment_tolerance_deg", 25.0, "Max path/port direction error"),
      BT::InputPort<double>("trajectory_max_age", 0.3, "Maximum trajectory age"),
      BT::InputPort<std::string>("param_prefix", "hole_pass", "bt_navigator parameter prefix"),
      BT::InputPort<std::string>("hole_ids", "", "Comma-separated hole ids"),
      BT::InputPort<std::string>("port_a_polygons", "", "Semicolon-separated polygons"),
      BT::InputPort<std::string>("port_b_polygons", "", "Semicolon-separated polygons"),
      BT::OutputPort<std::string>("hole_id", "Selected hole id"),
      BT::OutputPort<std::string>("entry_port", "A or B"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("entry_pose", "Entry pose"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("exit_pose", "Exit pose"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("hole_exit_goal", "Goal past the hole"),
      BT::OutputPort<geometry_msgs::msg::PolygonStamped>("entry_polygon", "Selected entry port polygon"),
      BT::OutputPort<geometry_msgs::msg::PolygonStamped>("exit_polygon", "Selected exit port polygon"),
      BT::OutputPort<geometry_msgs::msg::PolygonStamped>("corridor_polygon", "Hole corridor"),
    };
  }

  BT::NodeStatus tick() override;

private:
  struct Hole
  {
    std::string id;
    std::vector<double> a;
    std::vector<double> b;
  };

  bool getRobotPose(double & x, double & y);
  std::vector<Hole> loadHoles();
  std::vector<Hole> parseHolesFromPorts();
  std::vector<std::string> split(const std::string & text, char delimiter) const;
  std::vector<double> parsePolygon(const std::string & text) const;
  bool validPolygon(const std::vector<double> & polygon) const;
  bool pointInPolygon(double x, double y, const std::vector<double> & polygon) const;
  bool segmentIntersectionDistance(
    double ax, double ay, double bx, double by,
    const std::vector<double> & polygon, double * distance) const;
  bool segmentIntersectionParameter(
    double ax, double ay, double bx, double by,
    double cx, double cy, double dx, double dy, double * t) const;
  double distancePointToSegment(double px, double py, double ax, double ay, double bx, double by) const;
  geometry_msgs::msg::PoseStamped polygonCenterPose(const std::vector<double> & polygon, double yaw) const;
  geometry_msgs::msg::PoseStamped offsetPose(
    const geometry_msgs::msg::PoseStamped & pose, double yaw, double distance) const;
  geometry_msgs::msg::PolygonStamped stampedPolygon(const std::vector<double> & polygon) const;
  geometry_msgs::msg::PolygonStamped corridorPolygon(
    const std::vector<double> & a, const std::vector<double> & b) const;
  double polygonCenterX(const std::vector<double> & polygon) const;
  double polygonCenterY(const std::vector<double> & polygon) const;
  double normalizeAngle(double angle) const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<sentry_nav_interfaces::msg::MincoTrajectory>::SharedPtr traj_sub_;
  sentry_nav_interfaces::msg::MincoTrajectory::SharedPtr latest_traj_;
  rclcpp::Time latest_traj_time_;
  std::string trajectory_topic_;
  std::string loaded_param_prefix_;
  std::vector<Hole> holes_;
  bool holes_loaded_{false};
  std::mutex mutex_;
};

}  // namespace sirb_nav2_plugins

#endif  // SIRB_NAV2_PLUGINS__BT_NODES__HOLE_APPROACH_CONDITION_HPP_
