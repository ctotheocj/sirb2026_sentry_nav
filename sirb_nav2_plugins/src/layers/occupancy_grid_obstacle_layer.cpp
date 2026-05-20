#include "sirb_nav2_plugins/layers/occupancy_grid_obstacle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pb_nav2_costmap_2d
{

OccupancyGridObstacleLayer::OccupancyGridObstacleLayer()
: enabled_(true),
  map_received_(false),
  occupied_threshold_(65),
  obstacle_keep_time_(0.35),
  stamp_source_cell_area_(true),
  debug_logging_(false),
  last_resolution_(0.0),
  last_origin_x_(0.0),
  last_origin_y_(0.0),
  last_width_(0),
  last_height_(0)
{
}

void OccupancyGridObstacleLayer::onInitialize()
{
  auto node = node_.lock();

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("occupancy_grid")));
  declareParameter("occupied_threshold", rclcpp::ParameterValue(65));
  declareParameter("obstacle_keep_time", rclcpp::ParameterValue(0.35));
  declareParameter("stamp_source_cell_area", rclcpp::ParameterValue(true));
  declareParameter("debug_logging", rclcpp::ParameterValue(false));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".occupied_threshold", occupied_threshold_);
  node->get_parameter(name_ + ".obstacle_keep_time", obstacle_keep_time_);
  node->get_parameter(name_ + ".stamp_source_cell_area", stamp_source_cell_area_);
  node->get_parameter(name_ + ".debug_logging", debug_logging_);

  clock_ = node->get_clock();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node, false);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(3)).transient_local().reliable();
  map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic_, qos,
    std::bind(&OccupancyGridObstacleLayer::incomingMap, this, std::placeholders::_1));
  set_enabled_srv_ = node->create_service<std_srvs::srv::SetBool>(
    name_ + "/set_enabled",
    std::bind(
      &OccupancyGridObstacleLayer::setEnabledService, this,
      std::placeholders::_1, std::placeholders::_2));
  set_semantic_mode_srv_ =
    node->create_service<sentry_nav_interfaces::srv::SetSemanticLayerMode>(
    name_ + "/set_semantic_layer_mode",
    std::bind(
      &OccupancyGridObstacleLayer::setSemanticLayerModeService, this,
      std::placeholders::_1, std::placeholders::_2));

  current_ = true;
}

void OccupancyGridObstacleLayer::setEnabledService(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  enabled_ = request->data;
  if (!enabled_) {
    std::lock_guard<std::mutex> lock(cells_mutex_);
    observed_cells_.clear();
  }
  current_ = true;
  response->success = true;
  response->message = enabled_ ? "occupancy grid obstacle layer enabled" :
    "occupancy grid obstacle layer disabled";
  RCLCPP_INFO(logger_, "[%s] %s", name_.c_str(), response->message.c_str());
}

void OccupancyGridObstacleLayer::setSemanticLayerModeService(
  const std::shared_ptr<sentry_nav_interfaces::srv::SetSemanticLayerMode::Request> request,
  std::shared_ptr<sentry_nav_interfaces::srv::SetSemanticLayerMode::Response> response)
{
  if (request->mode == "normal") {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    hole_pass_mode_ = false;
    corridor_polygon_.clear();
    response->success = true;
    response->message = "semantic layer mode restored to normal";
    current_ = true;
    RCLCPP_INFO(logger_, "[%s] %s", name_.c_str(), response->message.c_str());
    return;
  }

  if (request->mode != "hole_pass") {
    response->success = false;
    response->message = "unsupported semantic layer mode '" + request->mode + "'";
    return;
  }

  std::vector<std::pair<double, double>> transformed;
  if (!transformCorridorToCostmapFrame(request->frame_id, request->corridor, transformed)) {
    response->success = false;
    response->message = "failed to transform corridor to costmap frame";
    return;
  }
  if (transformed.size() < 3) {
    response->success = false;
    response->message = "corridor polygon has fewer than 3 points";
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    corridor_polygon_ = transformed;
    hole_pass_mode_ = true;
  }
  current_ = true;
  response->success = true;
  response->message = "semantic layer mode set to hole_pass corridor mask";
  RCLCPP_INFO(logger_, "[%s] %s", name_.c_str(), response->message.c_str());
}

void OccupancyGridObstacleLayer::incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (!msg || msg->info.resolution <= 0.0f) {
    return;
  }

  const auto & info = msg->info;
  const double resolution = static_cast<double>(info.resolution);
  const double origin_x = info.origin.position.x;
  const double origin_y = info.origin.position.y;
  const rclcpp::Time now = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
    clock_->now() : rclcpp::Time(msg->header.stamp);

  size_t occupied_count = 0;
  size_t cached_count = 0;
  bool geometry_changed = false;
  {
    std::lock_guard<std::mutex> lock(cells_mutex_);
    geometry_changed = mapGeometryChanged(info);
    if (geometry_changed) {
      observed_cells_.clear();
      last_resolution_ = resolution;
      last_origin_x_ = origin_x;
      last_origin_y_ = origin_y;
      last_width_ = info.width;
      last_height_ = info.height;
    }

    for (unsigned int y = 0; y < info.height; ++y) {
      for (unsigned int x = 0; x < info.width; ++x) {
        const size_t index = static_cast<size_t>(y) * info.width + x;
        if (index >= msg->data.size() || msg->data[index] < occupied_threshold_) {
          continue;
        }

        ++occupied_count;
        ObservedCell cell;
        if (stamp_source_cell_area_) {
          cell.min_x = origin_x + static_cast<double>(x) * resolution;
          cell.min_y = origin_y + static_cast<double>(y) * resolution;
          cell.max_x = origin_x + static_cast<double>(x + 1) * resolution;
          cell.max_y = origin_y + static_cast<double>(y + 1) * resolution;
        } else {
          const double wx = origin_x + (static_cast<double>(x) + 0.5) * resolution;
          const double wy = origin_y + (static_cast<double>(y) + 0.5) * resolution;
          cell.min_x = wx;
          cell.min_y = wy;
          cell.max_x = wx;
          cell.max_y = wy;
        }
        cell.stamp = now;
        observed_cells_[cellKey(x, y)] = cell;
      }
    }

    pruneExpiredCells(now);
    cached_count = observed_cells_.size();
  }

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_ = msg;
    map_received_ = true;
  }

  if (debug_logging_) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 1000,
      "[%s] occupancy grid: occupied=%zu cached=%zu geometry_changed=%s",
      name_.c_str(), occupied_count, cached_count, geometry_changed ? "true" : "false");
  }
}

void OccupancyGridObstacleLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  const std::vector<ObservedCell> cells = getValidCells();
  if (cells.empty()) {
    return;
  }

  for (const auto & cell : cells) {
    touch(cell.min_x, cell.min_y, min_x, min_y, max_x, max_y);
    touch(cell.max_x, cell.max_y, min_x, min_y, max_x, max_y);
  }
}

void OccupancyGridObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  const std::vector<ObservedCell> cells = getValidCells();
  if (cells.empty()) {
    return;
  }

  size_t stamped_cells = 0;
  constexpr double kBoundaryEpsilon = 1e-6;
  for (const auto & cell : cells) {
    if (cellMaskedByCorridor(cell)) {
      continue;
    }
    const double max_x = cell.max_x > cell.min_x ? cell.max_x - kBoundaryEpsilon : cell.max_x;
    const double max_y = cell.max_y > cell.min_y ? cell.max_y - kBoundaryEpsilon : cell.max_y;
    int mx0 = 0;
    int my0 = 0;
    int mx1 = 0;
    int my1 = 0;
    master_grid.worldToMapNoBounds(cell.min_x, cell.min_y, mx0, my0);
    master_grid.worldToMapNoBounds(max_x, max_y, mx1, my1);

    const int mx_min = std::max(min_i, std::min(mx0, mx1));
    const int mx_max = std::min(max_i - 1, std::max(mx0, mx1));
    const int my_min = std::max(min_j, std::min(my0, my1));
    const int my_max = std::min(max_j - 1, std::max(my0, my1));

    if (mx_min > mx_max || my_min > my_max) {
      continue;
    }

    for (int mx = mx_min; mx <= mx_max; ++mx) {
      for (int my = my_min; my <= my_max; ++my) {
        master_grid.setCost(
          static_cast<unsigned int>(mx), static_cast<unsigned int>(my),
          nav2_costmap_2d::LETHAL_OBSTACLE);
        ++stamped_cells;
      }
    }
  }

  if (debug_logging_) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 1000,
      "[%s] stamped cached occupancy cells: source=%zu master_cells=%zu",
      name_.c_str(), cells.size(), stamped_cells);
  }
}

bool OccupancyGridObstacleLayer::cellMaskedByCorridor(const ObservedCell & cell) const
{
  std::lock_guard<std::mutex> lock(mode_mutex_);
  if (!hole_pass_mode_ || corridor_polygon_.size() < 3) {
    return false;
  }
  const double cx = 0.5 * (cell.min_x + cell.max_x);
  const double cy = 0.5 * (cell.min_y + cell.max_y);
  return pointInPolygon(cx, cy, corridor_polygon_);
}

bool OccupancyGridObstacleLayer::pointInCorridor(double x, double y) const
{
  std::lock_guard<std::mutex> lock(mode_mutex_);
  return hole_pass_mode_ && corridor_polygon_.size() >= 3 &&
         pointInPolygon(x, y, corridor_polygon_);
}

bool OccupancyGridObstacleLayer::pointInPolygon(
  double x, double y, const std::vector<std::pair<double, double>> & polygon) const
{
  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const double xi = polygon[i].first;
    const double yi = polygon[i].second;
    const double xj = polygon[j].first;
    const double yj = polygon[j].second;
    const bool crosses = ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / (yj - yi + 1.0e-9) + xi);
    if (crosses) {
      inside = !inside;
    }
  }
  return inside;
}

bool OccupancyGridObstacleLayer::transformCorridorToCostmapFrame(
  const std::string & frame_id,
  const geometry_msgs::msg::Polygon & corridor,
  std::vector<std::pair<double, double>> & transformed) const
{
  transformed.clear();
  if (corridor.points.size() < 3) {
    return false;
  }

  const std::string source_frame = frame_id.empty() ? layered_costmap_->getGlobalFrameID() : frame_id;
  const std::string target_frame = layered_costmap_->getGlobalFrameID();
  geometry_msgs::msg::TransformStamped tf;
  const bool same_frame = source_frame == target_frame;
  if (!same_frame) {
    try {
      tf = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        logger_, "[%s] corridor TF %s -> %s unavailable: %s",
        name_.c_str(), source_frame.c_str(), target_frame.c_str(), ex.what());
      return false;
    }
  }

  transformed.reserve(corridor.points.size());
  for (const auto & point : corridor.points) {
    geometry_msgs::msg::PointStamped src;
    geometry_msgs::msg::PointStamped dst;
    src.header.frame_id = source_frame;
    src.header.stamp = clock_->now();
    src.point.x = point.x;
    src.point.y = point.y;
    src.point.z = point.z;
    if (same_frame) {
      dst = src;
    } else {
      tf2::doTransform(src, dst, tf);
    }
    transformed.emplace_back(dst.point.x, dst.point.y);
  }
  return true;
}

void OccupancyGridObstacleLayer::reset()
{
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_.reset();
    map_received_ = false;
  }
  {
    std::lock_guard<std::mutex> lock(cells_mutex_);
    observed_cells_.clear();
  }
  current_ = true;
}

std::vector<OccupancyGridObstacleLayer::ObservedCell> OccupancyGridObstacleLayer::getValidCells()
{
  const rclcpp::Time now = clock_->now();
  std::lock_guard<std::mutex> lock(cells_mutex_);
  pruneExpiredCells(now);

  std::vector<ObservedCell> cells;
  cells.reserve(observed_cells_.size());
  for (const auto & item : observed_cells_) {
    cells.push_back(item.second);
  }
  return cells;
}

void OccupancyGridObstacleLayer::pruneExpiredCells(const rclcpp::Time & now)
{
  for (auto it = observed_cells_.begin(); it != observed_cells_.end(); ) {
    if ((now - it->second.stamp).seconds() > obstacle_keep_time_) {
      it = observed_cells_.erase(it);
    } else {
      ++it;
    }
  }
}

bool OccupancyGridObstacleLayer::mapGeometryChanged(
  const nav_msgs::msg::MapMetaData & info) const
{
  constexpr double kGeometryEpsilon = 1e-9;
  return last_width_ != info.width || last_height_ != info.height ||
         std::abs(last_resolution_ - static_cast<double>(info.resolution)) > kGeometryEpsilon ||
         std::abs(last_origin_x_ - info.origin.position.x) > kGeometryEpsilon ||
         std::abs(last_origin_y_ - info.origin.position.y) > kGeometryEpsilon;
}

int64_t OccupancyGridObstacleLayer::cellKey(unsigned int x, unsigned int y)
{
  return (static_cast<int64_t>(y) << 32) | static_cast<int64_t>(x);
}

void OccupancyGridObstacleLayer::touch(
  double x, double y, double * min_x, double * min_y, double * max_x, double * max_y)
{
  *min_x = std::min(*min_x, x);
  *min_y = std::min(*min_y, y);
  *max_x = std::max(*max_x, x);
  *max_y = std::max(*max_y, y);
}

}  // namespace pb_nav2_costmap_2d

PLUGINLIB_EXPORT_CLASS(pb_nav2_costmap_2d::OccupancyGridObstacleLayer, nav2_costmap_2d::Layer)
