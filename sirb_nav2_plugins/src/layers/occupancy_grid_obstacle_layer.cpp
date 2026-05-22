#include "sirb_nav2_plugins/layers/occupancy_grid_obstacle_layer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
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
  source_timeout_(0.6),
  stamp_source_cell_area_(true),
  debug_logging_(false)
{
}

void OccupancyGridObstacleLayer::onInitialize()
{
  auto node = node_.lock();

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("occupancy_grid")));
  declareParameter("occupied_threshold", rclcpp::ParameterValue(65));
  declareParameter("source_timeout", rclcpp::ParameterValue(0.6));
  declareParameter("stamp_source_cell_area", rclcpp::ParameterValue(true));
  declareParameter("debug_logging", rclcpp::ParameterValue(false));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".occupied_threshold", occupied_threshold_);
  node->get_parameter(name_ + ".source_timeout", source_timeout_);
  node->get_parameter(name_ + ".stamp_source_cell_area", stamp_source_cell_area_);
  node->get_parameter(name_ + ".debug_logging", debug_logging_);

  clock_ = node->get_clock();
  auto qos = rclcpp::QoS(rclcpp::KeepLast(3)).transient_local().reliable();
  map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic_, qos,
    std::bind(&OccupancyGridObstacleLayer::incomingMap, this, std::placeholders::_1));
  set_semantic_mode_srv_ =
    node->create_service<sentry_nav_interfaces::srv::SetSemanticLayerMode>(
    name_ + "/set_semantic_layer_mode",
    std::bind(
      &OccupancyGridObstacleLayer::setSemanticLayerModeService, this,
      std::placeholders::_1, std::placeholders::_2));

  current_ = true;
}

void OccupancyGridObstacleLayer::setSemanticLayerModeService(
  const std::shared_ptr<sentry_nav_interfaces::srv::SetSemanticLayerMode::Request> request,
  std::shared_ptr<sentry_nav_interfaces::srv::SetSemanticLayerMode::Response> response)
{
  if (request->mode == "normal") {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    hole_pass_mode_ = false;
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

  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    hole_pass_mode_ = true;
  }
  current_ = true;
  response->success = true;
  response->message = "semantic layer mode set to hole_pass obstacle-layer suppression";
  RCLCPP_INFO(logger_, "[%s] %s", name_.c_str(), response->message.c_str());
}

void OccupancyGridObstacleLayer::incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (!msg || msg->info.resolution <= 0.0f) {
    return;
  }

  const auto & info = msg->info;

  size_t occupied_count = 0;
  std::vector<CellArea> new_source_cells;
  new_source_cells.reserve(msg->data.size());
  for (unsigned int y = 0; y < info.height; ++y) {
    for (unsigned int x = 0; x < info.width; ++x) {
      const size_t index = static_cast<size_t>(y) * info.width + x;
      if (index >= msg->data.size() || msg->data[index] < occupied_threshold_) {
        continue;
      }

      ++occupied_count;
      new_source_cells.push_back(cellAreaFromGrid(info, x, y));
    }
  }

  {
    std::lock_guard<std::mutex> lock(cells_mutex_);
    source_cells_ = std::move(new_source_cells);
    source_frame_ = msg->header.frame_id;
    map_received_ = true;
    last_map_time_sec_ = clock_->now().seconds();
    needs_clear_previous_ = true;
  }

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_ = msg;
  }

  if (debug_logging_) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 1000,
      "[%s] occupancy grid snapshot: occupied=%zu frame='%s'",
      name_.c_str(), occupied_count, msg->header.frame_id.c_str());
  }
}

void OccupancyGridObstacleLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  bool suppress_obstacles = false;
  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    suppress_obstacles = hole_pass_mode_;
  }

  if (!enabled_) {
    std::vector<CellArea> previous_cells;
    {
      std::lock_guard<std::mutex> lock(cells_mutex_);
      if (!needs_clear_previous_ || last_stamped_target_cells_.empty()) {
        return;
      }
      previous_cells = last_stamped_target_cells_;
      cost_update_cells_.clear();
    }
    touchCells(previous_cells, min_x, min_y, max_x, max_y);
    return;
  }
  if (suppress_obstacles) {
    std::vector<CellArea> previous_cells;
    {
      std::lock_guard<std::mutex> lock(cells_mutex_);
      previous_cells = last_stamped_target_cells_;
      cost_update_cells_.clear();
      needs_clear_previous_ = false;
    }
    current_ = true;
    touchCells(previous_cells, min_x, min_y, max_x, max_y);
    return;
  }

  std::vector<CellArea> source_cells;
  std::vector<CellArea> previous_cells;
  std::string source_frame;
  bool has_map = false;
  double last_map_time_sec = -1.0;
  {
    std::lock_guard<std::mutex> lock(cells_mutex_);
    has_map = map_received_;
    source_cells = source_cells_;
    source_frame = source_frame_;
    previous_cells = last_stamped_target_cells_;
    last_map_time_sec = last_map_time_sec_;
  }

  if (!has_map) {
    current_ = false;
    if (!previous_cells.empty()) {
      std::lock_guard<std::mutex> lock(cells_mutex_);
      cost_update_cells_.clear();
      needs_clear_previous_ = true;
      touchCells(previous_cells, min_x, min_y, max_x, max_y);
    }
    return;
  }

  const double age = clock_->now().seconds() - last_map_time_sec;
  current_ = source_timeout_ <= 0.0 || age <= source_timeout_;
  if (!current_ && debug_logging_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "[%s] occupancy grid source stale: %.3fs > %.3fs",
      name_.c_str(), age, source_timeout_);
  }

  std::vector<CellArea> target_cells;
  if (!transformCellsToCostmapFrame(source_frame, source_cells, target_cells)) {
    current_ = false;
    std::lock_guard<std::mutex> lock(cells_mutex_);
    cost_update_cells_ = last_stamped_target_cells_;
    needs_clear_previous_ = false;
    touchCells(last_stamped_target_cells_, min_x, min_y, max_x, max_y);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(cells_mutex_);
    previous_cells = last_stamped_target_cells_;
    cost_update_cells_ = target_cells;
  }

  touchCells(previous_cells, min_x, min_y, max_x, max_y);
  touchCells(target_cells, min_x, min_y, max_x, max_y);
}

void OccupancyGridObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  std::vector<CellArea> cells;
  bool suppress_obstacles = false;
  {
    std::lock_guard<std::mutex> lock(cells_mutex_);
    cells = enabled_ ? cost_update_cells_ : std::vector<CellArea>{};
  }
  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    suppress_obstacles = hole_pass_mode_;
  }

  if (!enabled_ || suppress_obstacles) {
    std::lock_guard<std::mutex> lock(cells_mutex_);
    last_stamped_target_cells_.clear();
    needs_clear_previous_ = false;
    return;
  }

  if (cells.empty()) {
    std::lock_guard<std::mutex> lock(cells_mutex_);
    last_stamped_target_cells_.clear();
    needs_clear_previous_ = false;
    return;
  }

  size_t stamped_cells = 0;
  constexpr double kBoundaryEpsilon = 1e-6;
  for (const auto & cell : cells) {
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
      "[%s] stamped occupancy snapshot cells: source=%zu master_cells=%zu",
      name_.c_str(), cells.size(), stamped_cells);
  }

  {
    std::lock_guard<std::mutex> lock(cells_mutex_);
    last_stamped_target_cells_ = cost_update_cells_;
    needs_clear_previous_ = false;
  }
}

OccupancyGridObstacleLayer::CellArea OccupancyGridObstacleLayer::cellAreaFromGrid(
  const nav_msgs::msg::MapMetaData & info, unsigned int x, unsigned int y) const
{
  const double resolution = static_cast<double>(info.resolution);
  const double origin_x = info.origin.position.x;
  const double origin_y = info.origin.position.y;
  if (stamp_source_cell_area_) {
    return CellArea{
      origin_x + static_cast<double>(x) * resolution,
      origin_y + static_cast<double>(y) * resolution,
      origin_x + static_cast<double>(x + 1) * resolution,
      origin_y + static_cast<double>(y + 1) * resolution};
  }

  const double wx = origin_x + (static_cast<double>(x) + 0.5) * resolution;
  const double wy = origin_y + (static_cast<double>(y) + 0.5) * resolution;
  return CellArea{wx, wy, wx, wy};
}

bool OccupancyGridObstacleLayer::lookupTransformToCostmapFrame(
  const std::string & source_frame, geometry_msgs::msg::TransformStamped & transform,
  bool & same_frame) const
{
  const std::string target_frame = layered_costmap_->getGlobalFrameID();
  const std::string resolved_source_frame = source_frame.empty() ? target_frame : source_frame;
  same_frame = resolved_source_frame == target_frame;
  if (same_frame) {
    return true;
  }

  try {
    transform = tf_->lookupTransform(target_frame, resolved_source_frame, tf2::TimePointZero);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "[%s] occupancy grid TF %s -> %s unavailable: %s",
      name_.c_str(), resolved_source_frame.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

OccupancyGridObstacleLayer::CellArea OccupancyGridObstacleLayer::transformCellToCostmapFrame(
  const CellArea & cell, const geometry_msgs::msg::TransformStamped & transform,
  bool same_frame) const
{
  if (same_frame) {
    return cell;
  }

  const std::array<std::pair<double, double>, 4> corners{{
    {cell.min_x, cell.min_y},
    {cell.min_x, cell.max_y},
    {cell.max_x, cell.min_y},
    {cell.max_x, cell.max_y}}};

  CellArea result{
    std::numeric_limits<double>::max(),
    std::numeric_limits<double>::max(),
    std::numeric_limits<double>::lowest(),
    std::numeric_limits<double>::lowest()};
  for (const auto & corner : corners) {
    geometry_msgs::msg::PointStamped src;
    geometry_msgs::msg::PointStamped dst;
    src.header.frame_id = transform.child_frame_id;
    src.header.stamp = clock_->now();
    src.point.x = corner.first;
    src.point.y = corner.second;
    src.point.z = 0.0;
    tf2::doTransform(src, dst, transform);
    result.min_x = std::min(result.min_x, dst.point.x);
    result.min_y = std::min(result.min_y, dst.point.y);
    result.max_x = std::max(result.max_x, dst.point.x);
    result.max_y = std::max(result.max_y, dst.point.y);
  }
  return result;
}

bool OccupancyGridObstacleLayer::transformCellsToCostmapFrame(
  const std::string & source_frame, const std::vector<CellArea> & source_cells,
  std::vector<CellArea> & target_cells) const
{
  target_cells.clear();
  if (source_cells.empty()) {
    return true;
  }

  geometry_msgs::msg::TransformStamped transform;
  bool same_frame = false;
  if (!lookupTransformToCostmapFrame(source_frame, transform, same_frame)) {
    return false;
  }

  target_cells.reserve(source_cells.size());
  for (const auto & cell : source_cells) {
    target_cells.push_back(transformCellToCostmapFrame(cell, transform, same_frame));
  }
  return true;
}

void OccupancyGridObstacleLayer::reset()
{
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(cells_mutex_);
    source_cells_.clear();
    source_frame_.clear();
    map_received_ = false;
    cost_update_cells_.clear();
    needs_clear_previous_ = !last_stamped_target_cells_.empty();
  }
  current_ = true;
}

void OccupancyGridObstacleLayer::touchCells(
  const std::vector<CellArea> & cells, double * min_x, double * min_y, double * max_x,
  double * max_y) const
{
  for (const auto & cell : cells) {
    touch(cell.min_x, cell.min_y, min_x, min_y, max_x, max_y);
    touch(cell.max_x, cell.max_y, min_x, min_y, max_x, max_y);
  }
}

void OccupancyGridObstacleLayer::touch(
  double x, double y, double * min_x, double * min_y, double * max_x, double * max_y)
  const
{
  *min_x = std::min(*min_x, x);
  *min_y = std::min(*min_y, y);
  *max_x = std::max(*max_x, x);
  *max_y = std::max(*max_y, y);
}

}  // namespace pb_nav2_costmap_2d

PLUGINLIB_EXPORT_CLASS(pb_nav2_costmap_2d::OccupancyGridObstacleLayer, nav2_costmap_2d::Layer)
