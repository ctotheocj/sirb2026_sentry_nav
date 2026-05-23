#include "sirb_nav2_plugins/layers/occupancy_grid_obstacle_layer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
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
  declareParameter("clear_hole_corridors", rclcpp::ParameterValue(false));
  declareParameter("clear_hole_margin", rclcpp::ParameterValue(0.15));
  declareParameter("clear_hole_frame", rclcpp::ParameterValue(std::string("map")));
  declareParameter("clear_hole_ids", rclcpp::ParameterValue(std::vector<std::string>{}));
  declareParameter("debug_logging", rclcpp::ParameterValue(false));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".occupied_threshold", occupied_threshold_);
  node->get_parameter(name_ + ".source_timeout", source_timeout_);
  node->get_parameter(name_ + ".stamp_source_cell_area", stamp_source_cell_area_);
  node->get_parameter(name_ + ".clear_hole_corridors", clear_hole_corridors_);
  node->get_parameter(name_ + ".clear_hole_margin", clear_hole_margin_);
  node->get_parameter(name_ + ".clear_hole_frame", clear_hole_frame_);
  node->get_parameter(name_ + ".debug_logging", debug_logging_);

  clear_hole_margin_ = std::max(0.0, clear_hole_margin_);
  loadClearZones();

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

  const size_t accepted_count = new_source_cells.size();
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
      "[%s] occupancy grid snapshot: occupied=%zu accepted=%zu frame='%s'",
      name_.c_str(), occupied_count, accepted_count, msg->header.frame_id.c_str());
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

  size_t filtered_cells = 0;
  if (clear_hole_corridors_ && !target_cells.empty() && !clear_zones_.empty()) {
    std::vector<ClearZone> clear_zones = clear_zones_;
    if (transformClearZonesToCostmapFrame(clear_zones)) {
      filtered_cells = filterCellsInClearZones(target_cells, clear_zones);
    } else {
      current_ = false;
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "[%s] skipping occupancy-grid obstacle update because hole clear corridor "
        "TF is unavailable",
        name_.c_str());
      std::lock_guard<std::mutex> lock(cells_mutex_);
      cost_update_cells_ = last_stamped_target_cells_;
      needs_clear_previous_ = false;
      touchCells(last_stamped_target_cells_, min_x, min_y, max_x, max_y);
      return;
    }
  }
  if (debug_logging_ && filtered_cells > 0) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 1000,
      "[%s] filtered %zu occupancy cells inside configured hole clear corridors",
      name_.c_str(), filtered_cells);
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

void OccupancyGridObstacleLayer::loadClearZones()
{
  auto node = node_.lock();
  if (!node || !clear_hole_corridors_) {
    return;
  }

  std::vector<std::string> hole_ids;
  node->get_parameter(name_ + ".clear_hole_ids", hole_ids);
  if (hole_ids.empty()) {
    RCLCPP_WARN(
      logger_, "[%s] clear_hole_corridors is enabled but clear_hole_ids is empty",
      name_.c_str());
    return;
  }

  clear_zones_.clear();
  for (const auto & id : hole_ids) {
    const std::string prefix = "clear_holes." + id;
    declareParameter(prefix + ".port_a_polygon", rclcpp::ParameterValue(std::vector<double>{}));
    declareParameter(prefix + ".port_b_polygon", rclcpp::ParameterValue(std::vector<double>{}));

    std::vector<double> port_a;
    std::vector<double> port_b;
    if (!node->get_parameter(name_ + "." + prefix + ".port_a_polygon", port_a) ||
      !node->get_parameter(name_ + "." + prefix + ".port_b_polygon", port_b))
    {
      RCLCPP_WARN(
        logger_, "[%s] clear hole '%s' is missing port_a_polygon or port_b_polygon",
        name_.c_str(), id.c_str());
      continue;
    }
    if (!validPolygonParameter(port_a) || !validPolygonParameter(port_b)) {
      RCLCPP_WARN(
        logger_, "[%s] clear hole '%s' has invalid port polygons",
        name_.c_str(), id.c_str());
      continue;
    }

    auto corridor = buildCorridorPolygon(port_a, port_b);
    if (corridor.size() < 3) {
      RCLCPP_WARN(
        logger_, "[%s] clear hole '%s' produced invalid corridor polygon",
        name_.c_str(), id.c_str());
      continue;
    }

    ClearZone zone;
    zone.id = id;
    zone.polygon = std::move(corridor);
    zone.bounds = boundsForPolygon(zone.polygon, clear_hole_margin_);
    clear_zones_.push_back(std::move(zone));
  }

  RCLCPP_INFO(
    logger_, "[%s] loaded %zu hole clear corridor(s), frame='%s', margin=%.2f",
    name_.c_str(), clear_zones_.size(), clear_hole_frame_.c_str(), clear_hole_margin_);
}

bool OccupancyGridObstacleLayer::validPolygonParameter(const std::vector<double> & polygon) const
{
  return polygon.size() >= 6 && polygon.size() % 2 == 0;
}

std::vector<OccupancyGridObstacleLayer::Point2D> OccupancyGridObstacleLayer::pointsFromParameter(
  const std::vector<double> & polygon) const
{
  std::vector<Point2D> points;
  points.reserve(polygon.size() / 2);
  for (size_t i = 0; i + 1 < polygon.size(); i += 2) {
    points.push_back(Point2D{polygon[i], polygon[i + 1]});
  }
  return points;
}

std::vector<OccupancyGridObstacleLayer::Point2D>
OccupancyGridObstacleLayer::buildCorridorPolygon(
  const std::vector<double> & port_a,
  const std::vector<double> & port_b) const
{
  auto points = pointsFromParameter(port_a);
  const auto port_b_points = pointsFromParameter(port_b);
  points.insert(points.end(), port_b_points.begin(), port_b_points.end());
  return convexHull(std::move(points));
}

std::vector<OccupancyGridObstacleLayer::Point2D> OccupancyGridObstacleLayer::convexHull(
  std::vector<Point2D> points) const
{
  constexpr double kEpsilon = 1e-9;
  std::sort(
    points.begin(), points.end(),
    [](const Point2D & lhs, const Point2D & rhs) {
      if (lhs.x == rhs.x) {
        return lhs.y < rhs.y;
      }
      return lhs.x < rhs.x;
    });
  points.erase(
    std::unique(
      points.begin(), points.end(),
      [kEpsilon](const Point2D & lhs, const Point2D & rhs) {
        return std::abs(lhs.x - rhs.x) <= kEpsilon && std::abs(lhs.y - rhs.y) <= kEpsilon;
      }),
    points.end());
  if (points.size() <= 2) {
    return points;
  }

  std::vector<Point2D> hull;
  hull.reserve(points.size() * 2);
  for (const auto & point : points) {
    while (hull.size() >= 2 &&
      cross(hull[hull.size() - 2], hull[hull.size() - 1], point) <= kEpsilon)
    {
      hull.pop_back();
    }
    hull.push_back(point);
  }

  const size_t lower_size = hull.size();
  for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
    while (hull.size() > lower_size &&
      cross(hull[hull.size() - 2], hull[hull.size() - 1], *it) <= kEpsilon)
    {
      hull.pop_back();
    }
    hull.push_back(*it);
  }
  if (!hull.empty()) {
    hull.pop_back();
  }
  return hull;
}

OccupancyGridObstacleLayer::CellArea OccupancyGridObstacleLayer::boundsForPolygon(
  const std::vector<Point2D> & polygon, double margin) const
{
  CellArea bounds{
    std::numeric_limits<double>::max(),
    std::numeric_limits<double>::max(),
    std::numeric_limits<double>::lowest(),
    std::numeric_limits<double>::lowest()};
  for (const auto & point : polygon) {
    bounds.min_x = std::min(bounds.min_x, point.x);
    bounds.min_y = std::min(bounds.min_y, point.y);
    bounds.max_x = std::max(bounds.max_x, point.x);
    bounds.max_y = std::max(bounds.max_y, point.y);
  }
  bounds.min_x -= margin;
  bounds.min_y -= margin;
  bounds.max_x += margin;
  bounds.max_y += margin;
  return bounds;
}

bool OccupancyGridObstacleLayer::transformClearZonesToCostmapFrame(
  std::vector<ClearZone> & zones) const
{
  if (zones.empty()) {
    return true;
  }

  geometry_msgs::msg::TransformStamped transform;
  bool same_frame = false;
  if (!lookupTransformToCostmapFrame(clear_hole_frame_, transform, same_frame)) {
    return false;
  }

  if (same_frame) {
    return true;
  }

  for (auto & zone : zones) {
    zone = transformClearZone(zone, transform, same_frame);
  }
  return true;
}

OccupancyGridObstacleLayer::ClearZone OccupancyGridObstacleLayer::transformClearZone(
  const ClearZone & zone,
  const geometry_msgs::msg::TransformStamped & transform,
  bool same_frame) const
{
  if (same_frame) {
    return zone;
  }

  ClearZone result;
  result.id = zone.id;
  result.polygon.reserve(zone.polygon.size());
  for (const auto & point : zone.polygon) {
    geometry_msgs::msg::PointStamped src;
    geometry_msgs::msg::PointStamped dst;
    src.header.frame_id = transform.child_frame_id;
    src.header.stamp = clock_->now();
    src.point.x = point.x;
    src.point.y = point.y;
    src.point.z = 0.0;
    tf2::doTransform(src, dst, transform);
    result.polygon.push_back(Point2D{dst.point.x, dst.point.y});
  }
  result.bounds = boundsForPolygon(result.polygon, clear_hole_margin_);
  return result;
}

size_t OccupancyGridObstacleLayer::filterCellsInClearZones(
  std::vector<CellArea> & cells,
  const std::vector<ClearZone> & zones) const
{
  const size_t before = cells.size();
  cells.erase(
    std::remove_if(
      cells.begin(), cells.end(),
      [this, &zones](const CellArea & cell) {
        for (const auto & zone : zones) {
          if (cellInClearZone(cell, zone)) {
            return true;
          }
        }
        return false;
      }),
    cells.end());
  return before - cells.size();
}

bool OccupancyGridObstacleLayer::cellInClearZone(
  const CellArea & cell, const ClearZone & zone) const
{
  if (cell.max_x < zone.bounds.min_x || cell.min_x > zone.bounds.max_x ||
    cell.max_y < zone.bounds.min_y || cell.min_y > zone.bounds.max_y)
  {
    return false;
  }

  const Point2D center{
    0.5 * (cell.min_x + cell.max_x),
    0.5 * (cell.min_y + cell.max_y)};
  return pointInPolygon(center, zone.polygon) ||
         distanceToPolygon(center, zone.polygon) <= clear_hole_margin_;
}

bool OccupancyGridObstacleLayer::pointInPolygon(
  const Point2D & point, const std::vector<Point2D> & polygon) const
{
  if (polygon.size() < 3) {
    return false;
  }

  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto & pi = polygon[i];
    const auto & pj = polygon[j];
    const bool intersects = ((pi.y > point.y) != (pj.y > point.y)) &&
      (point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x);
    if (intersects) {
      inside = !inside;
    }
  }
  return inside;
}

double OccupancyGridObstacleLayer::distanceToPolygon(
  const Point2D & point, const std::vector<Point2D> & polygon) const
{
  if (polygon.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    best = std::min(best, distanceToSegment(point, a, b));
  }
  return best;
}

double OccupancyGridObstacleLayer::distanceToSegment(
  const Point2D & point, const Point2D & a, const Point2D & b) const
{
  const double vx = b.x - a.x;
  const double vy = b.y - a.y;
  const double wx = point.x - a.x;
  const double wy = point.y - a.y;
  const double len_sq = vx * vx + vy * vy;
  if (len_sq <= 1e-12) {
    return std::hypot(point.x - a.x, point.y - a.y);
  }

  const double t = std::clamp((wx * vx + wy * vy) / len_sq, 0.0, 1.0);
  const double proj_x = a.x + t * vx;
  const double proj_y = a.y + t * vy;
  return std::hypot(point.x - proj_x, point.y - proj_y);
}

double OccupancyGridObstacleLayer::cross(
  const Point2D & origin, const Point2D & a, const Point2D & b) const
{
  return (a.x - origin.x) * (b.y - origin.y) -
         (a.y - origin.y) * (b.x - origin.x);
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
