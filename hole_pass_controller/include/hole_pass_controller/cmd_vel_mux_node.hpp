#ifndef HOLE_PASS_CONTROLLER__CMD_VEL_MUX_NODE_HPP_
#define HOLE_PASS_CONTROLLER__CMD_VEL_MUX_NODE_HPP_

#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace hole_pass_controller
{

class CmdVelMuxNode : public rclcpp::Node
{
public:
  explicit CmdVelMuxNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void navCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void holeCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void holeModeCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void timerCallback();

  enum class SelectedSource
  {
    NONE,
    NAV,
    HOLE
  };

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr hole_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hole_mode_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  geometry_msgs::msg::Twist latest_nav_;
  geometry_msgs::msg::Twist latest_hole_;
  rclcpp::Time latest_nav_time_;
  rclcpp::Time latest_hole_time_;
  SelectedSource last_selected_source_{SelectedSource::NONE};
  std::string output_frame_id_{"gimbal_yaw_fake"};
  double nav_timeout_{0.5};
  double hole_timeout_{0.3};
  int nav_priority_{10};
  int hole_priority_{20};
  bool publish_zero_when_idle_{false};
  bool require_hole_mode_{true};
  bool hole_mode_active_{false};
  bool debug_logging_{false};
};

}  // namespace hole_pass_controller

#endif  // HOLE_PASS_CONTROLLER__CMD_VEL_MUX_NODE_HPP_
