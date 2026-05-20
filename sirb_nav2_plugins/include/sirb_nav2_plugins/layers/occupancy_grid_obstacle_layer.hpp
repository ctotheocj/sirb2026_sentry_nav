#ifndef SIRB_NAV2_PLUGINS__LAYERS__OCCUPANCY_GRID_OBSTACLE_LAYER_HPP_
#define SIRB_NAV2_PLUGINS__LAYERS__OCCUPANCY_GRID_OBSTACLE_LAYER_HPP_

#include <mutex>
#include <utility>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/polygon.hpp"
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
  struct ObservedCell
  {
    double min_x;
    double min_y;
    double max_x;
    double max_y;
    rclcpp::Time stamp;
  };

  void incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void setEnabledService(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void setSemanticLayerModeService(
    const std::shared_ptr<sentry_nav_interfaces::srv::SetSemanticLayerMode::Request> request,
    std::shared_ptr<sentry_nav_interfaces::srv::SetSemanticLayerMode::Response> response);
  std::vector<ObservedCell> getValidCells();
  void pruneExpiredCells(const rclcpp::Time & now);
  bool cellMaskedByCorridor(const ObservedCell & cell) const;
  bool pointInCorridor(double x, double y) const;
  bool pointInPolygon(
    double x, double y, const std::vector<std::pair<double, double>> & polygon) const;
  bool transformCorridorToCostmapFrame(
    const std::string & frame_id,
    const geometry_msgs::msg::Polygon & corridor,
    std::vector<std::pair<double, double>> & transformed) const;
  bool mapGeometryChanged(const nav_msgs::msg::MapMetaData & info) const;
  static int64_t cellKey(unsigned int x, unsigned int y);
  void touch(double x, double y, double * min_x, double * min_y, double * max_x, double * max_y);

  bool enabled_;
  bool map_received_;
  int occupied_threshold_;
  double obstacle_keep_time_;
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
  std::unordered_map<int64_t, ObservedCell> observed_cells_;
  bool hole_pass_mode_{false};
  std::vector<std::pair<double, double>> corridor_polygon_;

  double last_resolution_;
  double last_origin_x_;
  double last_origin_y_;
  uint32_t last_width_;
  uint32_t last_height_;
};

}  // namespace pb_nav2_costmap_2d

#endif  // SIRB_NAV2_PLUGINS__LAYERS__OCCUPANCY_GRID_OBSTACLE_LAYER_HPP_
