#ifndef SIRB_NAV2_PLUGINS__LAYERS__OCCUPANCY_GRID_OBSTACLE_LAYER_HPP_
#define SIRB_NAV2_PLUGINS__LAYERS__OCCUPANCY_GRID_OBSTACLE_LAYER_HPP_

#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentry_nav_interfaces/srv/set_semantic_layer_mode.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

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

  void incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void setEnabledService(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void setSemanticLayerModeService(
    const std::shared_ptr<sentry_nav_interfaces::srv::SetSemanticLayerMode::Request> request,
    std::shared_ptr<sentry_nav_interfaces::srv::SetSemanticLayerMode::Response> response);
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
  bool cellMaskedByCorridor(const CellArea & cell) const;
  bool pointInCorridor(double x, double y) const;
  bool pointInPolygon(
    double x, double y, const std::vector<std::pair<double, double>> & polygon) const;
  bool transformCorridorToCostmapFrame(
    const std::string & frame_id,
    const geometry_msgs::msg::Polygon & corridor,
    std::vector<std::pair<double, double>> & transformed) const;
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
  bool debug_logging_;
  std::string topic_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_enabled_srv_;
  rclcpp::Service<sentry_nav_interfaces::srv::SetSemanticLayerMode>::SharedPtr
    set_semantic_mode_srv_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::mutex map_mutex_;
  std::mutex cells_mutex_;
  mutable std::mutex mode_mutex_;
  std::vector<CellArea> source_cells_;
  std::vector<CellArea> last_stamped_target_cells_;
  std::vector<CellArea> cost_update_cells_;
  std::string source_frame_;
  double last_map_time_sec_{-1.0};
  bool needs_clear_previous_{false};
  bool hole_pass_mode_{false};
  std::vector<std::pair<double, double>> corridor_polygon_;
};

}  // namespace pb_nav2_costmap_2d

#endif  // SIRB_NAV2_PLUGINS__LAYERS__OCCUPANCY_GRID_OBSTACLE_LAYER_HPP_
