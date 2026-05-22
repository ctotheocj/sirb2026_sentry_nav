#ifndef SIRB_NAV2_PLUGINS__BT_NODES__HOLE_PASS_MODE_CONTROLLER_HPP_
#define SIRB_NAV2_PLUGINS__BT_NODES__HOLE_PASS_MODE_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sentry_nav_interfaces/msg/hole_pass_cmd.hpp"
#include "sentry_nav_interfaces/srv/set_navigation_mode.hpp"
#include "tf2_ros/buffer.h"

namespace sirb_nav2_plugins
{

class HolePassModeController : public BT::ActionNodeBase
{
public:
  HolePassModeController(
    const std::string & name,
    const BT::NodeConfiguration & conf);
  ~HolePassModeController() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("global_frame", "map", "Global frame"),
      BT::InputPort<std::string>("robot_frame", "gimbal_yaw_fake", "Robot frame"),
      BT::InputPort<std::string>("param_prefix", "hole_pass", "bt_navigator parameter prefix"),
      BT::InputPort<std::string>(
        "mode_service", "navigation_mode_manager/set_navigation_mode", "Mode service"),
      BT::InputPort<double>("service_timeout", 0.5, "Mode service timeout"),
      BT::InputPort<double>("watchdog_timeout_sec", 30.0, "Mode watchdog timeout"),
      BT::InputPort<double>("refresh_period_sec", 0.05, "Active command refresh period"),
      BT::InputPort<double>("raise_duration_sec", 1.0, "Raise command hold time before normal mode"),
      BT::InputPort<double>("yaw_kp", 2.5, "Yaw velocity proportional gain"),
      BT::InputPort<double>("max_v_yaw", 1.8, "Absolute yaw velocity command limit"),
    };
  }

  BT::NodeStatus tick() override;
  void halt() override;

private:
  struct Hole
  {
    std::string id;
    std::vector<double> a;
    std::vector<double> b;
  };

  struct Trigger
  {
    bool valid{false};
    std::string hole_id;
    std::string entry_port;
    std::vector<double> entry_polygon;
    std::vector<double> exit_polygon;
    double path_yaw{0.0};
  };

  enum class ModeState
  {
    IDLE,
    LOWERING,
    RAISING,
    WAIT_CLEAR,
  };

  std::vector<Hole> loadHoles();
  Trigger findTrigger(const std::vector<Hole> & holes, double robot_x, double robot_y);
  bool getRobotPose(double & x, double & y, double & yaw);
  bool enterHoleMode(const Trigger & trigger, double robot_yaw);
  bool startRaise(double robot_yaw);
  void refreshHoleMode(double robot_yaw, bool update_yaw_command);
  void exitHoleMode(const char * reason);
  bool lockedHoleCleared(double robot_x, double robot_y) const;
  double activeVYaw(double robot_yaw);
  std::string paramPrefix() const;
  double parameterOrInput(const std::string & key, double default_value);
  bool callSetNavigationMode(
    const std::string & mode,
    const std::string & hole_id,
    uint8_t hole_cmd,
    double v_yaw,
    double watchdog_timeout_sec);

  std::vector<double> orderedPolygon(const std::vector<double> & polygon) const;
  bool validPolygon(const std::vector<double> & polygon) const;
  bool pointInPolygon(double x, double y, const std::vector<double> & polygon) const;
  double distancePointToSegment(
    double px, double py, double ax, double ay, double bx, double by) const;
  double polygonCenterX(const std::vector<double> & polygon) const;
  double polygonCenterY(const std::vector<double> & polygon) const;
  double normalizeAngle(double angle) const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::string loaded_param_prefix_;
  std::vector<Hole> holes_;
  bool holes_loaded_{false};

  ModeState state_{ModeState::IDLE};
  std::string owner_id_;
  std::string active_hole_id_;
  std::string locked_hole_id_;
  double active_target_yaw_{0.0};
  double active_v_yaw_{0.0};
  std::vector<double> active_exit_polygon_;
  rclcpp::Time last_refresh_time_;
  rclcpp::Time raise_start_time_;
};

}  // namespace sirb_nav2_plugins

#endif  // SIRB_NAV2_PLUGINS__BT_NODES__HOLE_PASS_MODE_CONTROLLER_HPP_
