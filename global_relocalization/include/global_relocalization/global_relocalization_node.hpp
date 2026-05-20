// Copyright 2026 Lihan Chen
//
// Licensed under the Apache License, Version 2.0

#ifndef GLOBAL_RELOCALIZATION__GLOBAL_RELOCALIZATION_NODE_HPP_
#define GLOBAL_RELOCALIZATION__GLOBAL_RELOCALIZATION_NODE_HPP_

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "global_relocalization/scan_context.hpp"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

namespace global_relocalization
{

class GlobalRelocalizationNode : public rclcpp::Node
{
public:
  explicit GlobalRelocalizationNode(const rclcpp::NodeOptions & options);
  ~GlobalRelocalizationNode() override;

private:
  struct DatabaseEntry
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    ScanContextDescriptor descriptor;
  };

  struct CandidateScore
  {
    size_t index{0};
    double score{0.0};
    int shift{0};
  };

  struct TimedScan
  {
    rclcpp::Time stamp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  };

  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void searchTimerCallback();
  void buildDatabaseAsync();
  bool loadGlobalMap(const std::string & file_name);
  void buildDatabase();
  bool buildDescriptorAtPose(double x, double y, double yaw, ScanContextDescriptor & descriptor) const;
  bool buildOnlineDescriptor(ScanContextDescriptor & descriptor, size_t & online_points);
  bool updateCandidateConfirmation(
    const std::vector<CandidateScore> & scores,
    const std::vector<DatabaseEntry> & database);
  geometry_msgs::msg::PoseArray makeCandidateMessage(
    const std::vector<CandidateScore> & scores, const std_msgs::msg::Header & header,
    const std::vector<DatabaseEntry> & database) const;
  void publishDiagnostics(
    const std::string & state, size_t online_points, size_t candidate_count,
    double best_score, double search_time_ms);
  bool isTriggered() const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr candidate_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr search_timer_;

  bool enabled_{true};
  std::string scan_topic_{"registered_scan"};
  std::string diagnostics_topic_{"small_gicp_relocalization/diagnostics"};
  std::string candidate_topic_{"global_relocalization/candidates"};
  std::string prior_pcd_file_;
  std::string map_frame_{"map"};
  std::string trigger_state_{"LOST"};
  double accumulation_sec_{1.5};
  double search_period_sec_{1.0};
  double candidate_xy_resolution_{0.5};
  double candidate_yaw_resolution_{0.785398};
  double virtual_scan_range_{12.0};
  double virtual_scan_height_min_{-0.5};
  double virtual_scan_height_max_{3.0};
  int descriptor_rings_{20};
  int descriptor_sectors_{60};
  int descriptor_top_k_{8};
  double min_descriptor_score_{0.35};
  double min_best_score_gap_{0.05};
  int candidate_confirm_frames_{2};
  double candidate_consistency_xy_{0.75};
  double candidate_consistency_yaw_{0.80};
  int min_online_points_{50};
  int max_database_entries_{20000};
  double map_leaf_size_{0.20};
  int max_descriptor_points_{6000};

  mutable std::mutex scan_mutex_;
  std::deque<TimedScan> scan_buffer_;
  size_t accumulated_scan_points_{0};
  rclcpp::Time newest_scan_time_{0, 0, RCL_ROS_TIME};
  std_msgs::msg::Header newest_scan_header_;

  mutable std::mutex state_mutex_;
  std::string localization_state_{"UNKNOWN"};
  bool localization_lost_{false};

  mutable std::mutex candidate_mutex_;
  bool has_pending_candidate_{false};
  double pending_candidate_x_{0.0};
  double pending_candidate_y_{0.0};
  double pending_candidate_yaw_{0.0};
  int pending_candidate_count_{0};

  mutable std::mutex database_mutex_;
  std::thread database_thread_;
  std::atomic_bool database_building_{false};
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr map_tree_;
  std::vector<DatabaseEntry> database_;
  std::atomic_bool database_ready_{false};
};

}  // namespace global_relocalization

#endif  // GLOBAL_RELOCALIZATION__GLOBAL_RELOCALIZATION_NODE_HPP_
