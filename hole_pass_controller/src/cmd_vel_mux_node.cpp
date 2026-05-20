#include "hole_pass_controller/cmd_vel_mux_node.hpp"

#include <algorithm>
#include <chrono>

namespace hole_pass_controller
{

CmdVelMuxNode::CmdVelMuxNode(const rclcpp::NodeOptions & options)
: Node("cmd_vel_mux", options)
{
  const auto nav_topic = declare_parameter<std::string>("nav_topic", "cmd_vel_nav2_result");
  const auto hole_topic =
    declare_parameter<std::string>("hole_topic", "cmd_vel_hole_controller");
  const auto output_topic = declare_parameter<std::string>("output_topic", "cmd_vel_selected");
  const auto output_stamped_topic =
    declare_parameter<std::string>("output_stamped_topic", "cmd_vel_selected_stamped");
  output_frame_id_ = declare_parameter<std::string>("output_frame_id", "gimbal_yaw_fake");
  nav_timeout_ = declare_parameter<double>("nav_timeout", 0.5);
  hole_timeout_ = declare_parameter<double>("hole_timeout", 0.3);
  nav_priority_ = declare_parameter<int>("nav_priority", 10);
  hole_priority_ = declare_parameter<int>("hole_priority", 20);
  publish_zero_when_idle_ = declare_parameter<bool>("publish_zero_when_idle", false);
  require_hole_mode_ = declare_parameter<bool>("require_hole_mode", true);
  debug_logging_ = declare_parameter<bool>("debug_logging", false);
  const auto hole_mode_topic =
    declare_parameter<std::string>("hole_mode_topic", "navigation_mode_manager/hole_mode_active");
  const double publish_period = declare_parameter<double>("publish_period", 0.02);

  latest_nav_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  latest_hole_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

  nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    nav_topic, 10, std::bind(&CmdVelMuxNode::navCallback, this, std::placeholders::_1));
  hole_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    hole_topic, 10, std::bind(&CmdVelMuxNode::holeCallback, this, std::placeholders::_1));
  if (require_hole_mode_) {
    hole_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
      hole_mode_topic, 10,
      std::bind(&CmdVelMuxNode::holeModeCallback, this, std::placeholders::_1));
  } else {
    hole_mode_active_ = true;
  }
  pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, 10);
  stamped_pub_ =
    create_publisher<geometry_msgs::msg::TwistStamped>(output_stamped_topic, 10);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(std::max(0.005, publish_period))),
    std::bind(&CmdVelMuxNode::timerCallback, this));

  RCLCPP_INFO(
    get_logger(),
    "CmdVelMux chain: nav='%s' hole='%s' output='%s' stamped='%s' frame='%s' "
    "timeouts(nav=%.3fs,hole=%.3fs) priority(nav=%d,hole=%d) period=%.3fs idle_zero=%d "
    "require_hole_mode=%d mode_topic='%s'",
    nav_topic.c_str(), hole_topic.c_str(), output_topic.c_str(),
    output_stamped_topic.c_str(), output_frame_id_.c_str(), nav_timeout_, hole_timeout_,
    nav_priority_, hole_priority_, publish_period, publish_zero_when_idle_ ? 1 : 0,
    require_hole_mode_ ? 1 : 0, hole_mode_topic.c_str());
}

void CmdVelMuxNode::navCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_nav_ = *msg;
  latest_nav_time_ = now();
}

void CmdVelMuxNode::holeCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_hole_ = *msg;
  latest_hole_time_ = now();
}

void CmdVelMuxNode::holeModeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  hole_mode_active_ = msg->data;
  if (!hole_mode_active_) {
    latest_hole_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }
}

void CmdVelMuxNode::timerCallback()
{
  const auto t = now();
  geometry_msgs::msg::Twist output;
  SelectedSource selected_source = SelectedSource::NONE;
  double nav_age = -1.0;
  double hole_age = -1.0;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool have_hole = latest_hole_time_.nanoseconds() != 0;
    const bool have_nav = latest_nav_time_.nanoseconds() != 0;
    hole_age = have_hole ? (t - latest_hole_time_).seconds() : -1.0;
    nav_age = have_nav ? (t - latest_nav_time_).seconds() : -1.0;
    const bool hole_allowed = !require_hole_mode_ || hole_mode_active_;
    const bool hole_fresh = hole_allowed && have_hole && hole_age <= hole_timeout_;
    const bool nav_fresh = have_nav && nav_age <= nav_timeout_;
    if (hole_fresh && (!nav_fresh || hole_priority_ >= nav_priority_)) {
      output = latest_hole_;
      selected_source = SelectedSource::HOLE;
    } else if (nav_fresh) {
      output = latest_nav_;
      selected_source = SelectedSource::NAV;
    } else if (hole_fresh) {
      output = latest_hole_;
      selected_source = SelectedSource::HOLE;
    }
    if (debug_logging_ && selected_source != last_selected_source_) {
      const char * source_name = selected_source == SelectedSource::HOLE ? "hole" :
        selected_source == SelectedSource::NAV ? "nav" : "none";
      RCLCPP_INFO(
        get_logger(),
        "CmdVelMux selected source='%s' nav_age=%.3fs hole_age=%.3fs output=(%.3f, %.3f, %.3f)",
        source_name, nav_age, hole_age, output.linear.x, output.linear.y, output.angular.z);
    }
    last_selected_source_ = selected_source;
  }
  if (selected_source == SelectedSource::NONE && !publish_zero_when_idle_) {
    return;
  }

  pub_->publish(output);

  geometry_msgs::msg::TwistStamped stamped;
  stamped.header.stamp = t;
  stamped.header.frame_id = output_frame_id_;
  stamped.twist = output;
  stamped_pub_->publish(stamped);
}

}  // namespace hole_pass_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hole_pass_controller::CmdVelMuxNode)
