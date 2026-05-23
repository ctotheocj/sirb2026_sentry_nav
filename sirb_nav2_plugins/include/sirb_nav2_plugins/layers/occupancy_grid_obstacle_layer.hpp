#ifndef SIRB_NAV2_PLUGINS__LAYERS__OCCUPANCY_GRID_OBSTACLE_LAYER_HPP_
#define SIRB_NAV2_PLUGINS__LAYERS__OCCUPANCY_GRID_OBSTACLE_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentry_nav_interfaces/srv/set_semantic_layer_mode.hpp"

namespace pb_nav2_costmap_2d
{

class OccupancyGridObstacleLayer : public nav2_costmap_2d::Layer
{
public:
  OccupancyGridObstacleLayer();
  virtual ~OccupancyGridObstacleLayer() = default;

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void reset();
  virtual bool isClearable() { return false; }

private:
  struct CellArea
  {
    double min_x;
    double min_y;
    double max_x;
    double max_y;
  };

  struct Point2D
  {
    double x;
    double y;
  };

  struct ClearZone
  {
    std::string id;
    std::vector<Point2D> polygon;
    CellArea bounds;
  };

  void incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void setSemanticLayerModeService(
    const std::shared_ptr<sentry_nav_interfaces::srv::SetSemanticLayerMode::Request> request,
    std::shared_ptr<sentry_nav_interfaces::srv::SetSemanticLayerMode::Response> response);
  void loadClearZones();
  bool validPolygonParameter(const std::vector<double> & polygon) const;
  std::vector<Point2D> pointsFromParameter(const std::vector<double> & polygon) const;
  std::vector<Point2D> buildCorridorPolygon(
    const std::vector<double> & port_a,
    const std::vector<double> & port_b) const;
  CellArea boundsForPolygon(const std::vector<Point2D> & polygon, double margin) const;
  bool transformClearZonesToCostmapFrame(std::vector<ClearZone> & zones) const;
  ClearZone transformClearZone(
    const ClearZone & zone,
    const geometry_msgs::msg::TransformStamped & transform,
    bool same_frame) const;
  size_t filterCellsInClearZones(
    std::vector<CellArea> & cells,
    const std::vector<ClearZone> & zones) const;
  bool cellInClearZone(const CellArea & cell, const ClearZone & zone) const;
  bool pointInPolygon(const Point2D & point, const std::vector<Point2D> & polygon) const;
  double distanceToPolygon(const Point2D & point, const std::vector<Point2D> & polygon) const;
  double distanceToSegment(const Point2D & point, const Point2D & a, const Point2D & b) const;
  CellArea cellAreaFromGrid(
    const nav_msgs::msg::MapMetaData & info, unsigned int x, unsigned int y) const;
  bool lookupTransformToCostmapFrame(
    const std::string & source_frame, geometry_msgs::msg::TransformStamped & transform,
    bool & same_frame) const;
  CellArea transformCellToCostmapFrame(
    const CellArea & cell, const geometry_msgs::msg::TransformStamped & transform,
    bool same_frame) const;
  bool transformCellsToCostmapFrame(
    const std::string & source_frame, const std::vector<CellArea> & source_cells,
    std::vector<CellArea> & target_cells) const;
  void touchCells(
    const std::vector<CellArea> & cells, double * min_x, double * min_y, double * max_x,
    double * max_y) const;
  void touch(double x, double y, double * min_x, double * min_y, double * max_x, double * max_y)
    const;

  bool enabled_;
  bool map_received_;
  int occupied_threshold_;
  double source_timeout_;
  bool stamp_source_cell_area_;
  bool clear_hole_corridors_{false};
  double clear_hole_margin_{0.15};
  double clear_hole_max_area_{0.0};
  bool debug_logging_;
  std::string topic_;
  std::string clear_hole_frame_{"map"};
  rclcpp::Service<sentry_nav_interfaces::srv::SetSemanticLayerMode>::SharedPtr
    set_semantic_mode_srv_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Clock::SharedPtr clock_;
  std::mutex map_mutex_;
  std::mutex cells_mutex_;
  mutable std::mutex mode_mutex_;
  std::vector<CellArea> source_cells_;
  std::vector<CellArea> last_stamped_target_cells_;
  std::vector<CellArea> cost_update_cells_;
  std::vector<ClearZone> clear_zones_;
  std::string source_frame_;
  double last_map_time_sec_{-1.0};
  bool needs_clear_previous_{false};
  bool hole_pass_mode_{false};
};

}  // namespace pb_nav2_costmap_2d

#endif  // SIRB_NAV2_PLUGINS__LAYERS__OCCUPANCY_GRID_OBSTACLE_LAYER_HPP_
