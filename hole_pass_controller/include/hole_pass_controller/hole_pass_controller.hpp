#ifndef HOLE_PASS_CONTROLLER__HOLE_PASS_CONTROLLER_HPP_
#define HOLE_PASS_CONTROLLER__HOLE_PASS_CONTROLLER_HPP_

#include <cstddef>
#include <atomic>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentry_nav_interfaces/msg/hole_pass_cmd.hpp"
#include "sentry_nav_interfaces/msg/hole_pass_state.hpp"
#include "sentry_nav_interfaces/msg/minco_trajectory.hpp"
#include "sentry_nav_interfaces/action/pass_hole.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

namespace hole_pass_controller
{

class HolePassController : public rclcpp::Node
{
public:
  explicit HolePassController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using PassHole = sentry_nav_interfaces::action::PassHole;
  using GoalHandlePassHole = rclcpp_action::ServerGoalHandle<PassHole>;

  enum class Stage
  {
    NORMAL,
    APPROACHING,
    WAIT_LOWERED,
    PASSING,
    EXITING
  };

  struct PathProjection
  {
    bool valid{false};
    bool has_hole{false};
    int hole_index{-1};
    bool entry_is_port_a{true};
    size_t segment_index{0};
    double tangent_yaw{0.0};
    double distance_to_entry{std::numeric_limits<double>::infinity()};
    double distance_to_exit_end{std::numeric_limits<double>::infinity()};
    double path_distance_from_start{0.0};
    bool ahead_intersects_entry{false};
    bool ahead_intersects_exit{false};
    bool in_entry_polygon{false};
    bool in_exit_polygon{false};
    bool beyond_exit{false};
    bool beyond_entry{false};
    double segment_start_t{0.0};
  };

  struct HoleDefinition
  {
    std::string id;
    std::vector<double> port_a_polygon;
    std::vector<double> port_b_polygon;
  };

  void trajectoryCallback(const sentry_nav_interfaces::msg::MincoTrajectory::SharedPtr msg);
  void holePassStateCallback(
    const sentry_nav_interfaces::msg::HolePassState::SharedPtr msg);
  void navYawCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void commandTimerCallback();
  rclcpp_action::GoalResponse handlePassHoleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PassHole::Goal> goal);
  rclcpp_action::CancelResponse handlePassHoleCancel(
    const std::shared_ptr<GoalHandlePassHole> goal_handle);
  void handlePassHoleAccepted(const std::shared_ptr<GoalHandlePassHole> goal_handle);
  void executePassHole(const std::shared_ptr<GoalHandlePassHole> goal_handle);

  bool getRobotPose(geometry_msgs::msg::PoseStamped * pose);
  bool inputsReady();
  bool heightStateFresh();
  bool navYawFresh();
  bool trajectoryFresh();
  bool isLowered();
  bool isRaised();
  void updateStage(const PathProjection & projection);
  PathProjection analyzePath(double robot_x, double robot_y) const;
  double targetYawFromProjection(const PathProjection & projection) const;
  geometry_msgs::msg::Twist applyHoleControl(
    const geometry_msgs::msg::Twist & input_cmd, const PathProjection & projection);
  void publishVelocity(const geometry_msgs::msg::Twist & output);
  void publishZeroVelocity();
  void publishHoleCommand(uint8_t hole_cmd, double v_yaw);
  void publishLatestCommand();
  void publishYawTargetMarker(
    const geometry_msgs::msg::PoseStamped & robot_pose, const PathProjection & projection,
    bool visible);

  bool pathIntersectsPolygonFrom(
    size_t start_index, double start_t, const std::vector<double> & polygon, double max_distance,
    double * first_distance) const;
  bool pointInPolygon(double x, double y, const std::vector<double> & polygon) const;
  bool segmentIntersectsPolygon(
    double ax, double ay, double bx, double by, const std::vector<double> & polygon) const;
  bool segmentPolygonFirstIntersectionDistance(
    double ax, double ay, double bx, double by, const std::vector<double> & polygon,
    double * distance) const;
  bool segmentIntersectionParameter(
    double ax, double ay, double bx, double by, double cx, double cy, double dx,
    double dy, double * t) const;
  bool segmentsIntersect(
    double ax, double ay, double bx, double by, double cx, double cy, double dx,
    double dy) const;
  double distanceToPolygon(double x, double y, const std::vector<double> & polygon) const;
  double distancePointToSegment(
    double px, double py, double ax, double ay, double bx, double by) const;
  double signed2dCross(double ax, double ay, double bx, double by, double cx, double cy) const;
  double normalizeAngle(double angle) const;
  double limitPlanarSpeed(double x, double y, double max_speed) const;
  void stopPlanarVelocity(geometry_msgs::msg::Twist * output) const;
  bool validPolygon(const std::vector<double> & polygon) const;
  std::vector<double> squareAroundPose(
    const geometry_msgs::msg::PoseStamped & pose, double half_size) const;
  const char * stageName(Stage stage) const;
  uint8_t stageId(Stage stage) const;
  void setStage(Stage stage, const char * reason);

  rclcpp::Subscription<sentry_nav_interfaces::msg::MincoTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<sentry_nav_interfaces::msg::HolePassState>::SharedPtr hole_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr nav_yaw_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_pub_;
  rclcpp::Publisher<sentry_nav_interfaces::msg::HolePassCmd>::SharedPtr hole_cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr yaw_target_marker_pub_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp_action::Server<PassHole>::SharedPtr pass_hole_action_server_;

  sentry_nav_interfaces::msg::MincoTrajectory::SharedPtr latest_trajectory_;
  geometry_msgs::msg::Twist latest_output_cmd_;
  rclcpp::Time latest_output_time_;
  sentry_nav_interfaces::msg::HolePassCmd latest_hole_cmd_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  Stage stage_{Stage::NORMAL};
  rclcpp::Time stage_enter_time_;
  uint8_t latest_height_state_{sentry_nav_interfaces::msg::HolePassState::HEIGHT_NORMAL};
  rclcpp::Time latest_height_state_time_;
  double latest_nav_yaw_{0.0};
  rclcpp::Time latest_nav_yaw_time_;
  rclcpp::Time latest_trajectory_time_;
  bool nav_yaw_received_{false};
  std::atomic_bool action_active_{false};
  std::atomic_bool cancel_requested_{false};
  double pass_start_path_distance_{std::numeric_limits<double>::quiet_NaN()};

  std::string global_frame_;
  std::string robot_frame_;
  std::string output_cmd_vel_topic_;
  std::string output_cmd_vel_stamped_topic_;
  std::string cmd_frame_id_;
  std::string trajectory_topic_;
  std::string hole_pass_cmd_topic_;
  std::string hole_pass_state_topic_;
  std::string nav_yaw_topic_;
  std::string yaw_target_marker_topic_;
  std::string pass_hole_action_name_;
  std::vector<std::string> hole_ids_;
  std::vector<HoleDefinition> holes_;
  int active_hole_index_{-1};
  bool active_entry_is_port_a_{true};
  double prepare_distance_{1.5};
  double stop_distance_{0.4};
  double slow_speed_{0.25};
  double pass_speed_{0.45};
  double yaw_offset_{0.0};
  double yaw_kp_{2.5};
  double max_v_yaw_{1.8};
  double exit_raise_distance_{0.2};
  double exit_timeout_sec_{3.0};
  double lowering_timeout_sec_{2.0};
  double height_state_timeout_sec_{0.3};
  double nav_yaw_timeout_sec_{0.2};
  double trajectory_timeout_sec_{0.5};
  double tf_lookup_timeout_sec_{0.05};
  double min_pass_time_sec_{0.4};
  double min_pass_distance_{0.3};
  double command_publish_period_sec_{0.02};
  double command_hold_timeout_sec_{0.3};
  bool allow_reverse_tangent_{true};
  bool stop_angular_while_waiting_{true};
  double yaw_marker_length_{0.7};
  double yaw_marker_z_{0.25};
};

}  // namespace hole_pass_controller

#endif  // HOLE_PASS_CONTROLLER__HOLE_PASS_CONTROLLER_HPP_
