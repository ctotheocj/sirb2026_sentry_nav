#include "sirb_nav2_plugins/bt_nodes/hole_approach_condition.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sirb_nav2_plugins
{
namespace
{
constexpr double kEps = 1e-9;
}

HoleApproachCondition::HoleApproachCondition(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  latest_traj_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
}

BT::NodeStatus HoleApproachCondition::tick()
{
  std::string param_prefix = "hole_pass";
  getInput("param_prefix", param_prefix);
  const auto holes = loadHoles();
  if (holes.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  std::string topic = "trajectory_manager/trajectory_for_mpc";
  getInput("trajectory_topic", topic);
  if (!traj_sub_ || trajectory_topic_ != topic) {
    trajectory_topic_ = topic;
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    traj_sub_ = node_->create_subscription<sentry_nav_interfaces::msg::MincoTrajectory>(
      trajectory_topic_, rclcpp::QoS(1),
      [this](const sentry_nav_interfaces::msg::MincoTrajectory::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_traj_ = msg;
        latest_traj_time_ = node_->now();
      }, options);
  }
  callback_group_executor_.spin_some();

  sentry_nav_interfaces::msg::MincoTrajectory::SharedPtr traj;
  rclcpp::Time traj_time;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    traj = latest_traj_;
    traj_time = latest_traj_time_;
  }
  double max_age = 0.3;
  const std::string age_param = param_prefix + ".trajectory_max_age";
  if (node_->has_parameter(age_param)) {
    max_age = node_->get_parameter(age_param).as_double();
  }
  getInput("trajectory_max_age", max_age);
  if (!node_->has_parameter(age_param)) {
    node_->declare_parameter(age_param, max_age);
  }
  if (!traj || traj->waypoints.size() < 2 ||
    (node_->now() - traj_time).seconds() > max_age)
  {
    return BT::NodeStatus::FAILURE;
  }

  double robot_x = 0.0;
  double robot_y = 0.0;
  if (!getRobotPose(robot_x, robot_y)) {
    return BT::NodeStatus::FAILURE;
  }

  double prepare_distance = 1.5;
  double alignment_tolerance_deg = 25.0;
  const std::string prepare_param = param_prefix + ".prepare_distance";
  const std::string alignment_param = param_prefix + ".alignment_tolerance_deg";
  if (node_->has_parameter(prepare_param)) {
    prepare_distance = node_->get_parameter(prepare_param).as_double();
  }
  if (node_->has_parameter(alignment_param)) {
    alignment_tolerance_deg = node_->get_parameter(alignment_param).as_double();
  }
  getInput("prepare_distance", prepare_distance);
  getInput("alignment_tolerance_deg", alignment_tolerance_deg);
  if (!node_->has_parameter(prepare_param)) {
    node_->declare_parameter(prepare_param, prepare_distance);
  }
  if (!node_->has_parameter(alignment_param)) {
    node_->declare_parameter(alignment_param, alignment_tolerance_deg);
  }
  const double alignment_tolerance = alignment_tolerance_deg * M_PI / 180.0;
  size_t best_idx = 0;
  double best_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i + 1 < traj->waypoints.size(); ++i) {
    const auto & a = traj->waypoints[i];
    const auto & b = traj->waypoints[i + 1];
    const double d = distancePointToSegment(robot_x, robot_y, a.x, a.y, b.x, b.y);
    if (d < best_dist) {
      best_dist = d;
      best_idx = i;
    }
  }

  double best_t = 0.0;
  const auto & p0 = traj->waypoints[best_idx];
  const auto & p1 = traj->waypoints[best_idx + 1];
  const double seg_dx = p1.x - p0.x;
  const double seg_dy = p1.y - p0.y;
  const double seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;
  if (seg_len_sq > kEps) {
    best_t = std::clamp(((robot_x - p0.x) * seg_dx + (robot_y - p0.y) * seg_dy) / seg_len_sq, 0.0, 1.0);
  }

  for (const auto & hole : holes) {
    for (int port = 0; port < 2; ++port) {
      const auto & entry = port == 0 ? hole.a : hole.b;
      const auto & exit = port == 0 ? hole.b : hole.a;
      if (!validPolygon(entry) || !validPolygon(exit)) {
        continue;
      }
      double accum = 0.0;
      for (size_t i = best_idx; i + 1 < traj->waypoints.size(); ++i) {
        const auto & a = traj->waypoints[i];
        const auto & b = traj->waypoints[i + 1];
        double ax = a.x;
        double ay = a.y;
        if (i == best_idx) {
          ax = a.x + (b.x - a.x) * best_t;
          ay = a.y + (b.y - a.y) * best_t;
        }
        double hit_dist = 0.0;
        if (segmentIntersectionDistance(ax, ay, b.x, b.y, entry, &hit_dist) &&
          accum + hit_dist <= prepare_distance)
        {
          const double yaw = std::atan2(b.y - ay, b.x - ax);
          const double entry_cx = polygonCenterX(entry);
          const double entry_cy = polygonCenterY(entry);
          const double exit_cx = polygonCenterX(exit);
          const double exit_cy = polygonCenterY(exit);
          const double hole_yaw = std::atan2(exit_cy - entry_cy, exit_cx - entry_cx);
          if (std::abs(normalizeAngle(yaw - hole_yaw)) > alignment_tolerance) {
            continue;
          }
          setOutput("hole_id", hole.id);
          setOutput("entry_port", port == 0 ? std::string("A") : std::string("B"));
          setOutput("entry_pose", polygonCenterPose(entry, yaw));
          setOutput("exit_pose", polygonCenterPose(exit, yaw));
          setOutput("hole_exit_goal", polygonCenterPose(exit, yaw));
          setOutput("corridor_polygon", corridorPolygon(entry, exit));
          return BT::NodeStatus::SUCCESS;
        }
        accum += std::hypot(b.x - ax, b.y - ay);
        if (accum > prepare_distance) {
          break;
        }
      }
    }
  }

  return BT::NodeStatus::FAILURE;
}

bool HoleApproachCondition::getRobotPose(double & x, double & y)
{
  std::string global_frame = "map";
  std::string robot_frame = "gimbal_yaw_fake";
  getInput("global_frame", global_frame);
  getInput("robot_frame", robot_frame);
  try {
    const auto tf = tf_buffer_->lookupTransform(global_frame, robot_frame, tf2::TimePointZero);
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

std::vector<HoleApproachCondition::Hole> HoleApproachCondition::loadHoles()
{
  std::string ids_text;
  getInput("hole_ids", ids_text);
  if (!ids_text.empty()) {
    return parseHolesFromPorts();
  }

  std::string prefix = "hole_pass";
  getInput("param_prefix", prefix);
  if (holes_loaded_ && loaded_param_prefix_ == prefix) {
    return holes_;
  }

  loaded_param_prefix_ = prefix;
  holes_.clear();
  holes_loaded_ = true;

  const std::string ids_param = prefix + ".hole_ids";
  if (!node_->has_parameter(ids_param)) {
    node_->declare_parameter(ids_param, std::vector<std::string>{});
  }
  const auto ids = node_->get_parameter(ids_param).as_string_array();
  for (const auto & id : ids) {
    const std::string a_param = prefix + ".holes." + id + ".port_a_polygon";
    const std::string b_param = prefix + ".holes." + id + ".port_b_polygon";
    if (!node_->has_parameter(a_param)) {
      node_->declare_parameter(a_param, std::vector<double>{});
    }
    if (!node_->has_parameter(b_param)) {
      node_->declare_parameter(b_param, std::vector<double>{});
    }
    Hole hole;
    hole.id = id;
    hole.a = node_->get_parameter(a_param).as_double_array();
    hole.b = node_->get_parameter(b_param).as_double_array();
    if (!validPolygon(hole.a) || !validPolygon(hole.b)) {
      RCLCPP_WARN_ONCE(
        node_->get_logger(),
        "HoleApproachCondition: hole '%s' is invalid in bt_navigator params",
        id.c_str());
      continue;
    }
    holes_.push_back(hole);
  }
  return holes_;
}

std::vector<HoleApproachCondition::Hole> HoleApproachCondition::parseHolesFromPorts()
{
  std::string ids_text;
  std::string a_text;
  std::string b_text;
  getInput("hole_ids", ids_text);
  getInput("port_a_polygons", a_text);
  getInput("port_b_polygons", b_text);
  const auto ids = split(ids_text, ',');
  const auto as = split(a_text, ';');
  const auto bs = split(b_text, ';');
  std::vector<Hole> holes;
  const size_t n = std::min({ids.size(), as.size(), bs.size()});
  for (size_t i = 0; i < n; ++i) {
    holes.push_back(Hole{ids[i], parsePolygon(as[i]), parsePolygon(bs[i])});
  }
  return holes;
}

std::vector<std::string> HoleApproachCondition::split(const std::string & text, char delimiter) const
{
  std::vector<std::string> out;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, delimiter)) {
    const auto begin = item.find_first_not_of(" \t\n\r");
    if (begin == std::string::npos) {
      continue;
    }
    const auto end = item.find_last_not_of(" \t\n\r");
    out.push_back(item.substr(begin, end - begin + 1));
  }
  return out;
}

std::vector<double> HoleApproachCondition::parsePolygon(const std::string & text) const
{
  std::vector<double> values;
  std::string normalized = text;
  std::replace(normalized.begin(), normalized.end(), ',', ' ');
  std::stringstream stream(normalized);
  double value = 0.0;
  while (stream >> value) {
    values.push_back(value);
  }
  return values;
}

bool HoleApproachCondition::validPolygon(const std::vector<double> & polygon) const
{
  return polygon.size() >= 6 && polygon.size() % 2 == 0;
}

bool HoleApproachCondition::pointInPolygon(double x, double y, const std::vector<double> & polygon) const
{
  bool inside = false;
  const size_t n = polygon.size() / 2;
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = polygon[2 * i];
    const double yi = polygon[2 * i + 1];
    const double xj = polygon[2 * j];
    const double yj = polygon[2 * j + 1];
    const bool crosses = ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / (yj - yi + kEps) + xi);
    if (crosses) {
      inside = !inside;
    }
  }
  return inside;
}

bool HoleApproachCondition::segmentIntersectionDistance(
  double ax, double ay, double bx, double by,
  const std::vector<double> & polygon, double * distance) const
{
  const double len = std::hypot(bx - ax, by - ay);
  if (len < kEps) {
    if (pointInPolygon(ax, ay, polygon)) {
      if (distance) {
        *distance = 0.0;
      }
      return true;
    }
    return false;
  }
  double best_t = pointInPolygon(ax, ay, polygon) ? 0.0 : std::numeric_limits<double>::infinity();
  const size_t n = polygon.size() / 2;
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    double t = 0.0;
    if (segmentIntersectionParameter(
        ax, ay, bx, by, polygon[2 * j], polygon[2 * j + 1],
        polygon[2 * i], polygon[2 * i + 1], &t))
    {
      best_t = std::min(best_t, t);
    }
  }
  if (!std::isfinite(best_t)) {
    return false;
  }
  if (distance) {
    *distance = std::clamp(best_t, 0.0, 1.0) * len;
  }
  return true;
}

bool HoleApproachCondition::segmentIntersectionParameter(
  double ax, double ay, double bx, double by,
  double cx, double cy, double dx, double dy, double * t) const
{
  const double rx = bx - ax;
  const double ry = by - ay;
  const double sx = dx - cx;
  const double sy = dy - cy;
  const double denom = rx * sy - ry * sx;
  const double qpx = cx - ax;
  const double qpy = cy - ay;
  if (std::abs(denom) < kEps) {
    return false;
  }
  const double u = (qpx * ry - qpy * rx) / denom;
  const double ti = (qpx * sy - qpy * sx) / denom;
  if (ti < -1e-9 || ti > 1.0 + 1e-9 || u < -1e-9 || u > 1.0 + 1e-9) {
    return false;
  }
  if (t) {
    *t = std::clamp(ti, 0.0, 1.0);
  }
  return true;
}

double HoleApproachCondition::distancePointToSegment(
  double px, double py, double ax, double ay, double bx, double by) const
{
  const double vx = bx - ax;
  const double vy = by - ay;
  const double wx = px - ax;
  const double wy = py - ay;
  const double len_sq = vx * vx + vy * vy;
  if (len_sq < kEps) {
    return std::hypot(px - ax, py - ay);
  }
  const double t = std::clamp((wx * vx + wy * vy) / len_sq, 0.0, 1.0);
  return std::hypot(px - (ax + t * vx), py - (ay + t * vy));
}

geometry_msgs::msg::PoseStamped HoleApproachCondition::polygonCenterPose(
  const std::vector<double> & polygon, double yaw) const
{
  geometry_msgs::msg::PoseStamped pose;
  std::string global_frame = "map";
  getInput("global_frame", global_frame);
  pose.header.frame_id = global_frame;
  pose.header.stamp = node_->now();
  const size_t n = polygon.size() / 2;
  for (size_t i = 0; i < n; ++i) {
    pose.pose.position.x += polygon[2 * i];
    pose.pose.position.y += polygon[2 * i + 1];
  }
  pose.pose.position.x /= static_cast<double>(n);
  pose.pose.position.y /= static_cast<double>(n);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(q);
  return pose;
}

geometry_msgs::msg::PolygonStamped HoleApproachCondition::corridorPolygon(
  const std::vector<double> & a, const std::vector<double> & b) const
{
  geometry_msgs::msg::PolygonStamped polygon;
  std::string global_frame = "map";
  getInput("global_frame", global_frame);
  polygon.header.frame_id = global_frame;
  polygon.header.stamp = node_->now();
  for (size_t i = 0; i + 1 < a.size(); i += 2) {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(a[i]);
    p.y = static_cast<float>(a[i + 1]);
    polygon.polygon.points.push_back(p);
  }
  for (size_t i = 0; i + 1 < b.size(); i += 2) {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(b[i]);
    p.y = static_cast<float>(b[i + 1]);
    polygon.polygon.points.push_back(p);
  }
  return polygon;
}

double HoleApproachCondition::polygonCenterX(const std::vector<double> & polygon) const
{
  double x = 0.0;
  const size_t n = polygon.size() / 2;
  for (size_t i = 0; i < n; ++i) {
    x += polygon[2 * i];
  }
  return n == 0 ? 0.0 : x / static_cast<double>(n);
}

double HoleApproachCondition::polygonCenterY(const std::vector<double> & polygon) const
{
  double y = 0.0;
  const size_t n = polygon.size() / 2;
  for (size_t i = 0; i < n; ++i) {
    y += polygon[2 * i + 1];
  }
  return n == 0 ? 0.0 : y / static_cast<double>(n);
}

double HoleApproachCondition::normalizeAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace sirb_nav2_plugins
