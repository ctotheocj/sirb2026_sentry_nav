// Copyright 2025 Pan — Apache-2.0
// plan_env: FIESTA-based occupancy + ESDF mapping, ROS2 adaptation
// Reference: FIESTA (tommy 2018, HKUST)

#pragma once
#ifndef PLAN_ENV__GRID_MAP_H_
#define PLAN_ENV__GRID_MAP_H_

#include <Eigen/Eigen>
#include <atomic>
#include <condition_variable>
#include <thread>
#include <vector>
#include <queue>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace plan_env
{

struct MappingParameters
{
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;
  Eigen::Vector3i map_voxel_num_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  double obstacles_inflation_xy_;
  double obstacles_inflation_z_;
  std::string frame_id_;

  // log-odds raycasting
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_;
  double min_ray_length_, max_ray_length_;

  int local_map_margin_;
  double virtual_ceil_height_, ground_height_;
  double unknown_flag_;
  bool enable_esdf_{true};
  bool enable_negative_esdf_{true};
};

struct MappingData
{
  std::vector<double> occupancy_buffer_;
  std::vector<char>   occupancy_buffer_inflate_;

  // FIESTA ESDF incremental: closest obstacle + distance
  std::vector<double>        esdf_buffer_;
  std::vector<double>        esdf_buffer_neg_;     // dist from occ voxel to nearest free voxel
  std::vector<double>        esdf_buffer_signed_;  // signed: positive outside, negative inside
  std::vector<Eigen::Vector3i> closest_obstacle_;  // 3D voxel coord of closest occ voxel, (-1,-1,-1)=none
  std::vector<int>           num_hit_, num_miss_; // per-voxel raycasting counters
  std::vector<double>        last_hit_time_;      // timestamp of last hit (seconds)

  // FIESTA linked-list for O(1) bulk invalidation (head_/prev_/next_ indexed by voxel)
  std::vector<int>           head_, prev_, next_;

  Eigen::Vector3d camera_pos_;
  bool has_odom_{false}, has_cloud_{false};
  bool occ_need_update_{false}, local_updated_{false};

  Eigen::Vector3i local_bound_min_, local_bound_max_;
  Eigen::Vector3i last_local_bound_min_, last_local_bound_max_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap
{
public:
  using Ptr = std::shared_ptr<GridMap>;

  GridMap() = default;
  ~GridMap();

  void initMap(rclcpp::Node::SharedPtr node);

  // Query interface (used by esdf_map wrapper)
  int  getInflateOccupancy(const Eigen::Vector3d & pos) const;
  bool isInMap(const Eigen::Vector3d & pos) const;
  bool isInMap(const Eigen::Vector3i & idx) const;
  double getResolution() const { return mp_.resolution_; }
  Eigen::Vector3d getOrigin() const { return mp_.map_origin_; }

  // FIESTA incremental ESDF query
  double getDistance(const Eigen::Vector3d & pos) const;
  double getDistance2D(double x, double y, double z) const;
  // Signed distance: positive outside obstacles, negative inside.
  // Returns -1e6 for out-of-bounds (distinct from any valid signed distance).
  double getSignedDistance2D(double x, double y, double z) const;
  void   getGradient2D(double x, double y, double z,
                       double & gx, double & gy) const;

  void posToIndex(const Eigen::Vector3d & pos, Eigen::Vector3i & id) const;
  void indexToPos(const Eigen::Vector3i & id, Eigen::Vector3d & pos) const;
  int  toAddress(const Eigen::Vector3i & id) const;
  void boundIndex(Eigen::Vector3i & id) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void updateOccupancy();
  void publishOccupancyGridTimer();
  void updateESDFTimer();
  void esdfWorkerLoop();
  void clearAndInflateLocalMap();
  void updateESDFIncremental();
  void computeNegativeESDF();
  void raycastUpdate(const Eigen::Vector3d & start, const Eigen::Vector3d & end);
  bool lookupTransformToMap(
    const std::string & source_frame,
    const rclcpp::Time & stamp,
    bool allow_latest_fallback,
    Eigen::Affine3d & tf_map_source) const;
  Eigen::Affine3d poseMsgToEigen(const geometry_msgs::msg::Pose & pose) const;
  // FIESTA ESDF linked-list helpers
  void insertIntoList(int obs_idx, int vox_idx);
  void deleteFromList(int obs_idx, int vox_idx);
  void inflatePoint(const Eigen::Vector3i & pt, int step_xy, int step_z,
                    std::vector<Eigen::Vector3i> & pts) const;
  void publishOccupancyGrid();
  void publishMapInflate();
  void publishESDF();

  MappingParameters mp_;
  MappingData       md_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr     occ_grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    map_inf_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    esdf_pub_;
  rclcpp::TimerBase::SharedPtr occ_timer_, occ_grid_pub_timer_, vis_timer_;
  mutable std::mutex map_mutex_;
  mutable std::mutex occ_grid_msg_mutex_;
  std::mutex esdf_worker_mutex_;
  std::condition_variable esdf_worker_cv_;
  std::thread esdf_worker_;
  rclcpp::CallbackGroup::SharedPtr map_update_group_;
  rclcpp::CallbackGroup::SharedPtr occ_grid_pub_group_;
  rclcpp::CallbackGroup::SharedPtr visualization_group_;
  nav_msgs::msg::OccupancyGrid latest_occupancy_grid_;
  bool latest_occupancy_grid_valid_{false};
  double pub_z_slice_{0.3};
  int occupancy_grid_publish_period_ms_{100};
  int esdf_update_period_ms_{50};
  bool occupancy_grid_project_all_z_{true};
  double occupancy_grid_min_z_{0.05};
  double occupancy_grid_max_z_{1.2};
  bool occupancy_grid_hold_enabled_{true};
  double occupancy_grid_hold_time_{0.6};
  bool occupancy_grid_debug_{false};
  bool occupancy_grid_has_snapshot_{false};
  bool occupancy_grid_dirty_{false};
  std::atomic_bool esdf_need_update_{false};
  std::atomic_bool esdf_worker_stop_{false};
  std::vector<double> occupancy_grid_last_occupied_time_;
  double visualization_truncate_height_{1.5};
  double voxel_decay_time_{5.0};
  double cloud_stale_timeout_sec_{0.5};
  double transform_timeout_sec_{0.05};
  bool allow_latest_tf_fallback_{true};
  double latest_tf_fallback_max_age_sec_{0.5};
  std::atomic<double> last_cloud_time_{-1.0};  // seconds; -1 = never received

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace plan_env

// Static registry for intra-process shared_ptr injection (composable node pattern)
namespace plan_env
{
class GridMapRegistry
{
public:
  static void set(GridMap::Ptr ptr) { instance_ = ptr; }
  static GridMap::Ptr get() { return instance_; }
private:
  static GridMap::Ptr instance_;
};
}  // namespace plan_env

#endif  // PLAN_ENV__GRID_MAP_H_
