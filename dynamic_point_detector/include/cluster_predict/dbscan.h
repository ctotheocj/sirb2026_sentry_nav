// Copyright 2025 Pan
// Licensed under the Apache License, Version 2.0
//
// 动态障碍物追踪节点
// 输入: /cloud_registered (PointCloud2, map frame, Point-LIO 直出)
//       可通过参数 pointcloud_topic 改为 /pointcloud（仿真）等
// 输出: /dynamic_obstacles          (TrackedObstacleArray)
//       /dynamic_obstacles_viz      (MarkerArray: 圆柱 + 速度箭头)
//       /dynamic_obstacles_cloud    (PointCloud2: 彩色聚类点云, RViz 可视化)
//
// 流程:
//   1. PointCloud2 → 体素下采样 (voxel_leaf_size)
//   2. 自适应网格地面分割 (ground_filter.hpp) — 替代依赖上游 terrain_map
//   3. 2D DBSCAN + Gap-Split 聚类
//   4. EKF-SORT 追踪
//   5. 发布三路输出

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cstdint>

#include <std_msgs/msg/header.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "dynamic_obstacle_tracker/cluster_split.hpp"
#include "dynamic_obstacle_tracker/dbscan.hpp"
#include "dynamic_obstacle_tracker/ekf_tracker.hpp"
#include "dynamic_obstacle_tracker/ground_filter.hpp"
#include "sentry_nav_interfaces/msg/tracked_obstacle.hpp"
#include "sentry_nav_interfaces/msg/tracked_obstacle_array.hpp"

namespace dynamic_obstacle_tracker
{

// 预定义 7 台机器人对应的簇颜色 (R,G,B)
static const uint8_t kClusterColors[7][3] = {
  {230,  25,  75},   // 红
  { 60, 180,  75},   // 绿
  { 67, 133, 191},   // 蓝
  {245, 130,  48},   // 橙
  {145,  30, 180},   // 紫
  { 70, 240, 240},   // 青
  {210, 245,  60},   // 黄绿
};

class ObstacleTrackerNode : public rclcpp::Node
{
public:
  explicit ObstacleTrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("obstacle_tracker", options)
  {
    declare_parameter("pointcloud_topic", "/cloud_registered");  // 实车 Point-LIO 输出
    declare_parameter("output_topic", "/dynamic_obstacles");
    declare_parameter("viz_topic", "/dynamic_obstacles_viz");
    declare_parameter("cloud_viz_topic", "/dynamic_obstacles_cloud");
    declare_parameter("voxel_leaf_size", 0.10);  // 增大：0.04→0.10，减少噪声点
    declare_parameter("ground_cell_size", 0.4);
    declare_parameter("ground_low_percentile", 0.05);
    declare_parameter("obstacle_min_height", 0.15);  // 提高：0.05→0.15，过滤轮子和底盘
    declare_parameter("obstacle_max_height", 1.80);  // 保持不变
    declare_parameter("dbscan_epsilon", 0.35);  // 增大：0.25→0.35，避免机器人被分裂
    declare_parameter("dbscan_min_pts", 5);  // 增大：3→5，提高聚类稳定性
    declare_parameter("gap_split_max_single_radius", 0.28);
    declare_parameter("gap_split_threshold", 0.06);
    declare_parameter("q_pos", 0.01);
    declare_parameter("q_vel", 2.0);
    declare_parameter("r_pos", 0.05);
    declare_parameter("association_threshold", 2.5);  // 放宽：3.5→2.5，避免过度宽松
    declare_parameter("max_missed_frames", 5);
    declare_parameter("confirm_frames", 3);  // 恢复：2→3，减少误检
    declare_parameter("obstacle_radius_default", 0.35);
    declare_parameter("max_output_obstacles", 7);

    // 读取参数
    voxel_leaf_size_     = get_parameter("voxel_leaf_size").as_double();
    gf_params_.cell_size          = get_parameter("ground_cell_size").as_double();
    gf_params_.low_percentile     = get_parameter("ground_low_percentile").as_double();
    gf_params_.obstacle_min_height= get_parameter("obstacle_min_height").as_double();
    gf_params_.obstacle_max_height= get_parameter("obstacle_max_height").as_double();
    dbscan_epsilon_      = get_parameter("dbscan_epsilon").as_double();
    dbscan_min_pts_      = get_parameter("dbscan_min_pts").as_int();
    gap_split_max_single_radius_ = get_parameter("gap_split_max_single_radius").as_double();
    gap_split_threshold_ = get_parameter("gap_split_threshold").as_double();
    max_output_          = get_parameter("max_output_obstacles").as_int();

    EKFTracker::Params ekf_params;
    ekf_params.q_pos               = get_parameter("q_pos").as_double();
    ekf_params.q_vel               = get_parameter("q_vel").as_double();
    ekf_params.r_pos               = get_parameter("r_pos").as_double();
    ekf_params.association_threshold = get_parameter("association_threshold").as_double();
    ekf_params.max_missed          = get_parameter("max_missed_frames").as_int();
    ekf_params.confirm_frames      = get_parameter("confirm_frames").as_int();
    ekf_params.default_radius      = get_parameter("obstacle_radius_default").as_double();
    tracker_ = std::make_unique<EKFTracker>(ekf_params);

    const auto pc_topic    = get_parameter("pointcloud_topic").as_string();
    const auto out_topic   = get_parameter("output_topic").as_string();
    const auto viz_topic   = get_parameter("viz_topic").as_string();
    const auto cloud_topic = get_parameter("cloud_viz_topic").as_string();

    pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pc_topic, rclcpp::SensorDataQoS(),
      std::bind(&ObstacleTrackerNode::onPointCloud, this, std::placeholders::_1));

    obs_pub_   = create_publisher<sentry_nav_interfaces::msg::TrackedObstacleArray>(out_topic, 10);
    viz_pub_   = create_publisher<visualization_msgs::msg::MarkerArray>(viz_topic, 10);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, 10);

    RCLCPP_INFO(get_logger(),
      "ObstacleTrackerNode started\n"
      "  sub : %s\n"
      "  pub : %s | %s | %s\n"
      "  ground: cell=%.2fm, min_h=%.2fm, max_h=%.2fm\n"
      "  dbscan: eps=%.2f, min_pts=%d",
      pc_topic.c_str(), out_topic.c_str(), viz_topic.c_str(), cloud_topic.c_str(),
      gf_params_.cell_size, gf_params_.obstacle_min_height, gf_params_.obstacle_max_height,
      dbscan_epsilon_, dbscan_min_pts_);
  }

private:
  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // dt
    double dt = 0.1;
    if (last_stamp_.nanoseconds() > 0) {
      dt = (rclcpp::Time(msg->header.stamp) - last_stamp_).seconds();
      if (dt <= 0.0 || dt > 1.0) { dt = 0.1; }
    }
    last_stamp_ = msg->header.stamp;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) { return; }

    {
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(cloud);
      const float ls = static_cast<float>(voxel_leaf_size_);
      vg.setLeafSize(ls, ls, ls);
      pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>);
      vg.filter(*ds);
      cloud = ds;
    }

    std::vector<Eigen::Vector3d> pts3d;
    pts3d.reserve(cloud->size());
    for (const auto & p : cloud->points) {
      pts3d.emplace_back(p.x, p.y, p.z);
    }

    std::vector<Eigen::Vector2d> obs_pts2d;
    std::vector<int> obs_indices = filterGround(pts3d, gf_params_, obs_pts2d);
    // obs_pts2d[i] 对应 pts3d[obs_indices[i]] 的 XY 坐标

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "Ground filter: input=%zu, output=%zu (%.1f%% kept)",
      pts3d.size(), obs_pts2d.size(),
      pts3d.empty() ? 0.0 : 100.0 * obs_pts2d.size() / pts3d.size());

    if (obs_pts2d.empty()) {
      // 无障碍物点：发布空消息清空可视化
      sentry_nav_interfaces::msg::TrackedObstacleArray empty;
      empty.header = msg->header;
      obs_pub_->publish(empty);
      publishViz(empty);
      return;
    }

    auto clusters = dbscan2DWithSplit(
      obs_pts2d, dbscan_epsilon_, dbscan_min_pts_,
      gap_split_max_single_radius_, gap_split_threshold_);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "DBSCAN: input=%zu, clusters=%zu", obs_pts2d.size(), clusters.size());

    // 对过大的簇按其距雷达距离重新自适应分裂
    // gap_threshold = 0.04 + 0.002 * dist: 覆盖 3m(0.046) ~ 20m(0.08)
    // 首轮 dbscan2DWithSplit 使用固定阈值, 此处对仍然过大的簇补充自适应处理
    {
      std::vector<ClusterResult> refined;
      refined.reserve(clusters.size());
      for (const auto & c : clusters) {
        if (c.radius <= gap_split_max_single_radius_) {
          refined.push_back(c);
        } else {
          double dist = c.centroid.norm();
          double adaptive_gap = 0.04 + 0.002 * dist;
          // 仅当自适应阈值大于固定阈值时才重新分裂（远距离才有收益）
          if (adaptive_gap > gap_split_threshold_) {
            std::vector<int> idx(c.point_count);
            // 找出属于该簇的点（按质心距离最近分配）
            std::vector<int> all_idx;
            for (int i = 0; i < static_cast<int>(obs_pts2d.size()); ++i) {
              if ((obs_pts2d[i] - c.centroid).norm() <= c.radius + dbscan_epsilon_) {
                all_idx.push_back(i);
              }
            }
            auto sub = splitOversizedCluster(
              obs_pts2d, all_idx, adaptive_gap,
              gap_split_max_single_radius_, dbscan_min_pts_, 0, 3);
            for (auto & s : sub) { refined.push_back(s); }
          } else {
            refined.push_back(c);
          }
        }
      }
      clusters = std::move(refined);
    }

    if (cloud_pub_->get_subscription_count() > 0 && !clusters.empty()) {
      publishClusterCloud(msg->header, pts3d, obs_indices, obs_pts2d, clusters);
    }

    std::vector<Eigen::Vector2d> centroids;
    std::vector<double> radii;
    centroids.reserve(clusters.size());
    radii.reserve(clusters.size());
    for (const auto & c : clusters) {
      centroids.push_back(c.centroid);
      radii.push_back(std::max(c.radius + 0.1, 0.2));
    }

    const auto & tracks = tracker_->update(centroids, radii, dt);

    // Debug logging for diagnostics
    RCLCPP_DEBUG(get_logger(), "Pipeline: pts=%zu, obs_pts=%zu, clusters=%zu, tracks=%zu",
                 pts3d.size(), obs_pts2d.size(), clusters.size(), tracks.size());

    sentry_nav_interfaces::msg::TrackedObstacleArray out_msg;
    out_msg.header = msg->header;
    int count = 0;
    int total_confirmed = 0;
    for (const auto & t : tracks) {
      if (t.confirmed) { ++total_confirmed; }
      if (!t.confirmed || count >= max_output_) { continue; }
      sentry_nav_interfaces::msg::TrackedObstacle obs;
      obs.id         = t.id;
      obs.x          = t.state[0];
      obs.y          = t.state[1];
      obs.vx         = t.state[2];
      obs.vy         = t.state[3];
      obs.radius     = t.radius;
      obs.confidence = t.confidence;
      out_msg.obstacles.push_back(obs);
      ++count;
    }

    RCLCPP_DEBUG(get_logger(), "Output: confirmed=%d, published=%d", total_confirmed, count);

    obs_pub_->publish(out_msg);

    if (viz_pub_->get_subscription_count() > 0) {
      publishViz(out_msg);
    }
  }

  void publishClusterCloud(
    const std_msgs::msg::Header & header,
    const std::vector<Eigen::Vector3d> & pts3d,
    const std::vector<int> & obs_indices,
    const std::vector<Eigen::Vector2d> & obs_pts2d,
    const std::vector<ClusterResult> & clusters)
  {
    const int K = static_cast<int>(clusters.size());
    const int N = static_cast<int>(obs_pts2d.size());

    // 每个障碍物点分配最近簇 ID
    std::vector<int> assignment(N, -1);
    for (int i = 0; i < N; ++i) {
      double best_d2 = 1e9;
      for (int k = 0; k < K; ++k) {
        double d2 = (obs_pts2d[i] - clusters[k].centroid).squaredNorm();
        if (d2 < best_d2) { best_d2 = d2; assignment[i] = k; }
      }
    }

    // 构建 PointXYZRGB 点云
    pcl::PointCloud<pcl::PointXYZRGB> colored;
    colored.reserve(N);
    for (int i = 0; i < N; ++i) {
      const int cidx = assignment[i];
      if (cidx < 0) { continue; }
      const auto & p3 = pts3d[obs_indices[i]];
      pcl::PointXYZRGB pt;
      pt.x = static_cast<float>(p3.x());
      pt.y = static_cast<float>(p3.y());
      pt.z = static_cast<float>(p3.z());
      const int ci = cidx % 7;
      pt.r = kClusterColors[ci][0];
      pt.g = kClusterColors[ci][1];
      pt.b = kClusterColors[ci][2];
      colored.push_back(pt);
    }

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(colored, out);
    out.header = header;
    cloud_pub_->publish(out);
  }

  void publishViz(const sentry_nav_interfaces::msg::TrackedObstacleArray & msg)
  {
    visualization_msgs::msg::MarkerArray ma;

    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    del.header = msg.header;
    ma.markers.push_back(del);

    int mid = 0;
    for (const auto & obs : msg.obstacles) {
      // 圆柱标记
      visualization_msgs::msg::Marker m;
      m.header = msg.header;
      m.ns     = "obstacles";
      m.id     = mid++;
      m.type   = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x  = obs.x;
      m.pose.position.y  = obs.y;
      m.pose.position.z  = 0.5;
      m.pose.orientation.w = 1.0;
      m.scale.x = obs.radius * 2.0;
      m.scale.y = obs.radius * 2.0;
      m.scale.z = 1.0;
      m.color.r = 1.0f;
      m.color.g = 0.3f * obs.confidence;
      m.color.b = 0.0f;
      m.color.a = 0.5f + 0.5f * obs.confidence;
      m.lifetime = rclcpp::Duration(0, 200'000'000);
      ma.markers.push_back(m);

      // 速度箭头
      const double vlen = std::hypot(obs.vx, obs.vy);
      if (vlen > 0.1) {
        visualization_msgs::msg::Marker arrow;
        arrow.header = msg.header;
        arrow.ns     = "velocities";
        arrow.id     = mid++;
        arrow.type   = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.pose.position.x = obs.x;
        arrow.pose.position.y = obs.y;
        arrow.pose.position.z = 0.8;
        const double yaw = std::atan2(obs.vy, obs.vx);
        arrow.pose.orientation.z = std::sin(yaw / 2.0);
        arrow.pose.orientation.w = std::cos(yaw / 2.0);
        arrow.scale.x = vlen * 0.3;
        arrow.scale.y = 0.05;
        arrow.scale.z = 0.05;
        arrow.color.r = 1.0f;
        arrow.color.g = 1.0f;
        arrow.color.b = 0.0f;
        arrow.color.a = 0.8f;
        arrow.lifetime = rclcpp::Duration(0, 200'000'000);
        ma.markers.push_back(arrow);
      }
    }

    viz_pub_->publish(ma);
  }

  double voxel_leaf_size_{0.07};
  GroundFilterParams gf_params_{};
  double dbscan_epsilon_{0.5};
  int    dbscan_min_pts_{3};
  double gap_split_max_single_radius_{0.28};
  double gap_split_threshold_{0.06};
  int    max_output_{7};

  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<EKFTracker> tracker_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Publisher<sentry_nav_interfaces::msg::TrackedObstacleArray>::SharedPtr obs_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

}  // namespace dynamic_obstacle_tracker

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dynamic_obstacle_tracker::ObstacleTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
