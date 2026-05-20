#include "sirb_nav2_plugins/bt_nodes/select_nearby_goal.hpp"

#include <cmath>
#include <limits>
#include <mutex>

#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sirb_nav2_plugins
{

SelectNearbyGoal::SelectNearbyGoal(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());
}

BT::PortsList SelectNearbyGoal::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("input_goal", "Input goal"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_goal", "Output goal"),
    BT::InputPort<double>("radius", 0.5, "Maximum search radius (m)"),
    BT::InputPort<double>("angle_step_deg", 10.0, "Angle step in degrees"),
    BT::InputPort<double>("radius_step", 0.1, "Search radius step (m)"),
    BT::InputPort<int>(
      "cost_threshold",
      static_cast<int>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE),
      "Max allowed cost"),
    BT::InputPort<bool>("allow_unknown", false, "Allow unknown space"),
    BT::InputPort<std::string>("costmap_topic", "global_costmap/costmap_raw", "Costmap topic name")
  };
}

bool SelectNearbyGoal::worldToMap(
  const nav2_msgs::msg::Costmap & map,
  double wx, double wy,
  unsigned int & mx, unsigned int & my) const
{
  const auto & meta = map.metadata;
  if (meta.resolution <= 0.0 || meta.size_x == 0 || meta.size_y == 0) {
    return false;
  }

  const double dx = wx - meta.origin.position.x;
  const double dy = wy - meta.origin.position.y;
  const double yaw = tf2::getYaw(meta.origin.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double local_x = cos_yaw * dx + sin_yaw * dy;
  const double local_y = -sin_yaw * dx + cos_yaw * dy;
  const double mx_d = local_x / meta.resolution;
  const double my_d = local_y / meta.resolution;

  if (mx_d < 0.0 || my_d < 0.0) {
    return false;
  }

  mx = static_cast<unsigned int>(mx_d);
  my = static_cast<unsigned int>(my_d);
  if (mx >= meta.size_x || my >= meta.size_y) {
    return false;
  }

  return true;
}

bool SelectNearbyGoal::getCostAtWorld(
  const nav2_msgs::msg::Costmap & map,
  double wx, double wy,
  int & cost) const
{
  unsigned int mx, my;
  if (!worldToMap(map, wx, wy, mx, my)) {
    return false;
  }

  const size_t index = static_cast<size_t>(my) * map.metadata.size_x + mx;
  if (index >= map.data.size()) {
    return false;
  }

  cost = static_cast<int>(static_cast<unsigned char>(map.data[index]));
  return true;
}

bool SelectNearbyGoal::isCostAllowed(
  int cost,
  int cost_threshold,
  bool allow_unknown) const
{
  if (!allow_unknown && cost == static_cast<int>(nav2_costmap_2d::NO_INFORMATION)) {
    return false;
  }
  return cost < cost_threshold;
}

BT::NodeStatus SelectNearbyGoal::onStart()
{
  return tickImpl();
}

BT::NodeStatus SelectNearbyGoal::onRunning()
{
  return tickImpl();
}

void SelectNearbyGoal::onHalted()
{
}

BT::NodeStatus SelectNearbyGoal::tickImpl()
{
  geometry_msgs::msg::PoseStamped input_goal;
  if (!getInput("input_goal", input_goal)) {
    RCLCPP_WARN(node_->get_logger(), "SelectNearbyGoal: missing input_goal");
    return BT::NodeStatus::FAILURE;
  }

  double radius = 0.5;
  double angle_step_deg = 10.0;
  double radius_step = 0.1;
  int cost_threshold =
    static_cast<int>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  bool allow_unknown = false;
  std::string costmap_topic = "global_costmap/costmap_raw";

  getInput("radius", radius);
  getInput("angle_step_deg", angle_step_deg);
  getInput("radius_step", radius_step);
  getInput("cost_threshold", cost_threshold);
  getInput("allow_unknown", allow_unknown);
  getInput("costmap_topic", costmap_topic);

  if (radius <= 0.0 || angle_step_deg <= 0.0 || radius_step <= 0.0) {
    RCLCPP_WARN(node_->get_logger(), "SelectNearbyGoal: invalid search parameters");
    return BT::NodeStatus::FAILURE;
  }

  //订阅代价地图topic
  if (!costmap_sub_ || topic_name_ != costmap_topic) {
    topic_name_ = costmap_topic;
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
      topic_name_, qos,
      [this](const nav2_msgs::msg::Costmap::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(costmap_mutex_);
        last_costmap_ = msg;
      }, sub_options);
  }

  callback_group_executor_.spin_some();

  nav2_msgs::msg::Costmap::SharedPtr costmap_ptr;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap_ptr = last_costmap_;
  }

  //等待代价地图数据
  if (!costmap_ptr || costmap_ptr->data.empty()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "SelectNearbyGoal: costmap not received yet");
    return BT::NodeStatus::RUNNING;
  }

  const auto & costmap = *costmap_ptr;

  if (!costmap.header.frame_id.empty() &&
    input_goal.header.frame_id != costmap.header.frame_id)
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "SelectNearbyGoal: goal frame '%s' does not match costmap frame '%s'",
      input_goal.header.frame_id.c_str(),
      costmap.header.frame_id.c_str());
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped output_goal = input_goal;

  int input_cost = 0;
  if (getCostAtWorld(
      costmap, input_goal.pose.position.x, input_goal.pose.position.y, input_cost) &&
    isCostAllowed(input_cost, cost_threshold, allow_unknown))
  {
    output_goal.header.stamp = node_->now();
    setOutput("output_goal", output_goal);
    RCLCPP_DEBUG(
      node_->get_logger(), "SelectNearbyGoal: input goal accepted with cost %d", input_cost);
    return BT::NodeStatus::SUCCESS;
  }

  constexpr double kPi = 3.14159265358979323846;
  const double angle_step_rad = angle_step_deg * kPi / 180.0;
  const int n_samples = std::max(1, static_cast<int>(std::ceil(2.0 * kPi / angle_step_rad)));
  const int n_radii = std::max(1, static_cast<int>(std::ceil(radius / radius_step)));

  bool found = false;
  int best_cost = std::numeric_limits<int>::max();
  double best_radius = std::numeric_limits<double>::max();
  geometry_msgs::msg::PoseStamped best_goal = input_goal;

  for (int ri = 1; ri <= n_radii; ++ri) {
    const double sample_radius = std::min(radius, static_cast<double>(ri) * radius_step);
    int ring_best_cost = std::numeric_limits<int>::max();
    geometry_msgs::msg::PoseStamped ring_best_goal = input_goal;
    bool ring_found = false;

    for (int si = 0; si < n_samples; ++si) {
      const double angle = static_cast<double>(si) * angle_step_rad;
      const double x = input_goal.pose.position.x + sample_radius * std::cos(angle);
      const double y = input_goal.pose.position.y + sample_radius * std::sin(angle);

      int cost = 0;
      if (!getCostAtWorld(costmap, x, y, cost) ||
        !isCostAllowed(cost, cost_threshold, allow_unknown))
      {
        continue;
      }

      if (!ring_found || cost < ring_best_cost) {
        ring_found = true;
        ring_best_cost = cost;
        ring_best_goal.pose.position.x = x;
        ring_best_goal.pose.position.y = y;
      }
    }

    if (ring_found) {
      found = true;
      best_cost = ring_best_cost;
      best_radius = sample_radius;
      best_goal = ring_best_goal;
      break;
    }
  }

  if (!found) {
    RCLCPP_WARN(
      node_->get_logger(),
      "SelectNearbyGoal: input goal unavailable and no free fallback found within %.2f m",
      radius);
    return BT::NodeStatus::FAILURE;
  }

  (void)best_cost;
  (void)best_radius;
  output_goal = best_goal;
  output_goal.header.stamp = node_->now();
  setOutput("output_goal", output_goal);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace sirb_nav2_plugins

// BT nodes are registered in register_nodes.cpp
