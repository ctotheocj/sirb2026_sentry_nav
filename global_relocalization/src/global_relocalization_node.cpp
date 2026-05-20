// Copyright 2026 Lihan Chen
//
// Licensed under the Apache License, Version 2.0

#include "global_relocalization/global_relocalization_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "pcl/common/transforms.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/io/pcd_io.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp_components/register_node_macro.hpp"

namespace global_relocalization
{

namespace
{

diagnostic_msgs::msg::KeyValue makeKeyValue(
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  return kv;
}

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

GlobalRelocalizationNode::GlobalRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("global_relocalization", options)
{
  this->declare_parameter("enabled", true);
  this->declare_parameter("scan_topic", "registered_scan");
  this->declare_parameter("diagnostics_topic", "small_gicp_relocalization/diagnostics");
  this->declare_parameter("candidate_topic", "global_relocalization/candidates");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("trigger_state", "LOST");
  this->declare_parameter("accumulation_sec", 1.5);
  this->declare_parameter("search_period_sec", 1.0);
  this->declare_parameter("candidate_xy_resolution", 0.5);
  this->declare_parameter("candidate_yaw_resolution", 0.785398);
  this->declare_parameter("virtual_scan_range", 12.0);
  this->declare_parameter("virtual_scan_height_min", -0.5);
  this->declare_parameter("virtual_scan_height_max", 3.0);
  this->declare_parameter("descriptor_rings", 20);
  this->declare_parameter("descriptor_sectors", 60);
  this->declare_parameter("descriptor_top_k", 8);
  this->declare_parameter("min_descriptor_score", 0.35);
  this->declare_parameter("min_best_score_gap", 0.05);
  this->declare_parameter("candidate_confirm_frames", 2);
  this->declare_parameter("candidate_consistency_xy", 0.75);
  this->declare_parameter("candidate_consistency_yaw", 0.80);
  this->declare_parameter("min_online_points", 50);
  this->declare_parameter("max_database_entries", 20000);
  this->declare_parameter("map_leaf_size", 0.20);
  this->declare_parameter("max_descriptor_points", 6000);

  this->get_parameter("enabled", enabled_);
  this->get_parameter("scan_topic", scan_topic_);
  this->get_parameter("diagnostics_topic", diagnostics_topic_);
  this->get_parameter("candidate_topic", candidate_topic_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("trigger_state", trigger_state_);
  this->get_parameter("accumulation_sec", accumulation_sec_);
  this->get_parameter("search_period_sec", search_period_sec_);
  this->get_parameter("candidate_xy_resolution", candidate_xy_resolution_);
  this->get_parameter("candidate_yaw_resolution", candidate_yaw_resolution_);
  this->get_parameter("virtual_scan_range", virtual_scan_range_);
  this->get_parameter("virtual_scan_height_min", virtual_scan_height_min_);
  this->get_parameter("virtual_scan_height_max", virtual_scan_height_max_);
  this->get_parameter("descriptor_rings", descriptor_rings_);
  this->get_parameter("descriptor_sectors", descriptor_sectors_);
  this->get_parameter("descriptor_top_k", descriptor_top_k_);
  this->get_parameter("min_descriptor_score", min_descriptor_score_);
  this->get_parameter("min_best_score_gap", min_best_score_gap_);
  this->get_parameter("candidate_confirm_frames", candidate_confirm_frames_);
  this->get_parameter("candidate_consistency_xy", candidate_consistency_xy_);
  this->get_parameter("candidate_consistency_yaw", candidate_consistency_yaw_);
  this->get_parameter("min_online_points", min_online_points_);
  this->get_parameter("max_database_entries", max_database_entries_);
  this->get_parameter("map_leaf_size", map_leaf_size_);
  this->get_parameter("max_descriptor_points", max_descriptor_points_);

  accumulation_sec_ = std::max(0.1, accumulation_sec_);
  search_period_sec_ = std::max(0.1, search_period_sec_);
  candidate_xy_resolution_ = std::max(0.1, candidate_xy_resolution_);
  candidate_yaw_resolution_ = std::max(0.1, candidate_yaw_resolution_);
  virtual_scan_range_ = std::max(1.0, virtual_scan_range_);
  descriptor_rings_ = std::max(1, descriptor_rings_);
  descriptor_sectors_ = std::max(4, descriptor_sectors_);
  descriptor_top_k_ = std::max(1, descriptor_top_k_);
  min_best_score_gap_ = std::max(0.0, min_best_score_gap_);
  candidate_confirm_frames_ = std::max(1, candidate_confirm_frames_);
  candidate_consistency_xy_ = std::max(0.0, candidate_consistency_xy_);
  candidate_consistency_yaw_ = std::max(0.0, candidate_consistency_yaw_);
  min_online_points_ = std::max(1, min_online_points_);
  max_database_entries_ = std::max(1, max_database_entries_);
  map_leaf_size_ = std::max(0.0, map_leaf_size_);
  max_descriptor_points_ = std::max(100, max_descriptor_points_);

  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  map_tree_ = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();

  if (enabled_) {
    database_building_ = true;
    database_thread_ = std::thread(&GlobalRelocalizationNode::buildDatabaseAsync, this);
  }

  scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&GlobalRelocalizationNode::scanCallback, this, std::placeholders::_1));
  diagnostics_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    diagnostics_topic_, 10,
    std::bind(&GlobalRelocalizationNode::diagnosticsCallback, this, std::placeholders::_1));
  candidate_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(candidate_topic_, 10);
  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "global_relocalization/diagnostics", 10);
  search_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(search_period_sec_)),
    std::bind(&GlobalRelocalizationNode::searchTimerCallback, this));

  RCLCPP_INFO(
    get_logger(),
    "Global relocalization initialized enabled=%d scan_topic='%s' diagnostics='%s'. "
    "Database build runs in background (map_leaf=%.3f max_descriptor_points=%d).",
    enabled_ ? 1 : 0, scan_topic_.c_str(), diagnostics_topic_.c_str(), map_leaf_size_,
    max_descriptor_points_);
}

GlobalRelocalizationNode::~GlobalRelocalizationNode()
{
  if (database_thread_.joinable()) {
    database_thread_.join();
  }
}

void GlobalRelocalizationNode::buildDatabaseAsync()
{
  const auto start = std::chrono::steady_clock::now();
  const bool loaded = loadGlobalMap(prior_pcd_file_);
  if (loaded) {
    buildDatabase();
  }
  database_building_ = false;
  const auto end = std::chrono::steady_clock::now();
  const double elapsed_ms =
    std::chrono::duration<double, std::milli>(end - start).count();
  RCLCPP_INFO(
    get_logger(), "Global relocalization database background build finished: ready=%d time=%.1f ms.",
    database_ready_.load() ? 1 : 0, elapsed_ms);
}

bool GlobalRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (file_name.empty()) {
    RCLCPP_ERROR(get_logger(), "prior_pcd_file is empty; global relocalization disabled.");
    return false;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *raw_map) == -1) {
    RCLCPP_ERROR(get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return false;
  }
  if (raw_map->empty()) {
    RCLCPP_ERROR(get_logger(), "prior PCD is empty.");
    return false;
  }
  if (map_leaf_size_ > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(raw_map);
    const float leaf = static_cast<float>(map_leaf_size_);
    voxel_grid.setLeafSize(leaf, leaf, leaf);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    voxel_grid.filter(*filtered);
    if (!filtered->empty()) {
      global_map_ = filtered;
    } else {
      global_map_ = raw_map;
      RCLCPP_WARN(
        get_logger(),
        "Global relocalization map voxel filter produced an empty cloud; using raw map.");
    }
  } else {
    global_map_ = raw_map;
  }
  map_tree_->setInputCloud(global_map_);
  RCLCPP_INFO(
    get_logger(), "Loaded global relocalization PCD: raw=%zu filtered=%zu leaf=%.3f.",
    raw_map->size(), global_map_->size(), map_leaf_size_);
  return true;
}

void GlobalRelocalizationNode::buildDatabase()
{
  if (!global_map_ || global_map_->empty()) {
    return;
  }
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto & p : global_map_->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      continue;
    }
    min_x = std::min(min_x, static_cast<double>(p.x));
    min_y = std::min(min_y, static_cast<double>(p.y));
    max_x = std::max(max_x, static_cast<double>(p.x));
    max_y = std::max(max_y, static_cast<double>(p.y));
  }
  if (!std::isfinite(min_x) || !std::isfinite(min_y) || !std::isfinite(max_x) || !std::isfinite(max_y)) {
    return;
  }

  const int yaw_steps = std::max(
    1, static_cast<int>(std::ceil(2.0 * M_PI / candidate_yaw_resolution_)));
  const double yaw_step = 2.0 * M_PI / static_cast<double>(yaw_steps);
  struct GridCell
  {
    double x;
    double y;
  };

  std::vector<GridCell> occupied_cells;
  for (double x = min_x; x <= max_x; x += candidate_xy_resolution_) {
    for (double y = min_y; y <= max_y; y += candidate_xy_resolution_) {
      pcl::PointXYZ search;
      search.x = static_cast<float>(x);
      search.y = static_cast<float>(y);
      search.z = 0.0f;
      std::vector<int> indices;
      std::vector<float> distances;
      if (map_tree_->radiusSearch(
          search, static_cast<float>(candidate_xy_resolution_ * 1.5), indices, distances, 1) > 0)
      {
        occupied_cells.push_back(GridCell{x, y});
      }
    }
  }

  std::vector<DatabaseEntry> database;
  const size_t max_entries = static_cast<size_t>(max_database_entries_);
  const size_t max_cells = std::max<size_t>(1, max_entries / static_cast<size_t>(yaw_steps));
  const size_t cell_stride =
    occupied_cells.size() > max_cells ?
    static_cast<size_t>(std::ceil(
      static_cast<double>(occupied_cells.size()) / static_cast<double>(max_cells))) :
    1;

  for (size_t i = 0; i < occupied_cells.size() && database.size() < max_entries; i += cell_stride) {
    const auto & cell = occupied_cells[i];
    for (int iyaw = 0; iyaw < yaw_steps && database.size() < max_entries; ++iyaw) {
      const double yaw = iyaw * yaw_step;
      ScanContextDescriptor descriptor(descriptor_rings_, descriptor_sectors_, virtual_scan_range_);
      if (buildDescriptorAtPose(cell.x, cell.y, yaw, descriptor)) {
        database.push_back(DatabaseEntry{cell.x, cell.y, yaw, descriptor});
      }
    }
  }
  {
    std::lock_guard<std::mutex> lock(database_mutex_);
    database_ = std::move(database);
    database_ready_ = !database_.empty();
  }
  RCLCPP_INFO(
    get_logger(),
    "Global relocalization database built: %zu entries from %zu occupied cells "
    "(stride=%zu), bounds x=[%.2f, %.2f] y=[%.2f, %.2f].",
    database_.size(), occupied_cells.size(), cell_stride, min_x, max_x, min_y, max_y);
}

bool GlobalRelocalizationNode::buildDescriptorAtPose(
  double x, double y, double yaw, ScanContextDescriptor & descriptor) const
{
  pcl::PointCloud<pcl::PointXYZ> local;
  pcl::PointXYZ search;
  search.x = static_cast<float>(x);
  search.y = static_cast<float>(y);
  search.z = 0.0f;
  std::vector<int> indices;
  std::vector<float> distances;
  if (map_tree_->radiusSearch(search, static_cast<float>(virtual_scan_range_), indices, distances) <= 0) {
    return false;
  }
  const size_t stride =
    indices.size() > static_cast<size_t>(max_descriptor_points_) ?
    static_cast<size_t>(std::ceil(
      static_cast<double>(indices.size()) / static_cast<double>(max_descriptor_points_))) :
    1;
  local.reserve(std::min(indices.size(), static_cast<size_t>(max_descriptor_points_)));
  const double c = std::cos(-yaw);
  const double s = std::sin(-yaw);
  for (size_t i = 0; i < indices.size(); i += stride) {
    const int idx = indices[i];
    const auto & p = global_map_->points[static_cast<size_t>(idx)];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    if (p.z < virtual_scan_height_min_ || p.z > virtual_scan_height_max_) {
      continue;
    }
    const double dx = static_cast<double>(p.x) - x;
    const double dy = static_cast<double>(p.y) - y;
    pcl::PointXYZ lp;
    lp.x = static_cast<float>(c * dx - s * dy);
    lp.y = static_cast<float>(s * dx + c * dy);
    lp.z = p.z;
    local.push_back(lp);
  }
  return descriptor.build(local);
}

void GlobalRelocalizationNode::scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  std::lock_guard<std::mutex> lock(scan_mutex_);
  newest_scan_time_ = msg->header.stamp;
  newest_scan_header_ = msg->header;
  scan_buffer_.push_back(TimedScan{newest_scan_time_, scan});
  accumulated_scan_points_ += scan->size();

  const double window_sec = std::max(0.1, accumulation_sec_);
  while (!scan_buffer_.empty() && (newest_scan_time_ - scan_buffer_.front().stamp).seconds() > window_sec) {
    accumulated_scan_points_ -= scan_buffer_.front().cloud->size();
    scan_buffer_.pop_front();
  }
  while (!scan_buffer_.empty() && accumulated_scan_points_ > 200000) {
    accumulated_scan_points_ -= scan_buffer_.front().cloud->size();
    scan_buffer_.pop_front();
  }
}

void GlobalRelocalizationNode::diagnosticsCallback(
  const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  for (const auto & status : msg->status) {
    if (status.name.find("gicp") == std::string::npos &&
      status.name.find("relocalization") == std::string::npos)
    {
      continue;
    }
    for (const auto & kv : status.values) {
      if (kv.key == "localization_state") {
        std::lock_guard<std::mutex> lock(state_mutex_);
        localization_state_ = kv.value;
        localization_lost_ = kv.value == trigger_state_;
        return;
      }
    }
  }
}

bool GlobalRelocalizationNode::isTriggered() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return localization_lost_;
}

bool GlobalRelocalizationNode::buildOnlineDescriptor(
  ScanContextDescriptor & descriptor, size_t & online_points)
{
  pcl::PointCloud<pcl::PointXYZ> local;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    if (accumulated_scan_points_ < static_cast<size_t>(min_online_points_)) {
      online_points = accumulated_scan_points_;
      return false;
    }
    local.reserve(accumulated_scan_points_);
    for (const auto & timed_scan : scan_buffer_) {
      local += *timed_scan.cloud;
    }
  }
  online_points = local.size();
  descriptor.reset(descriptor_rings_, descriptor_sectors_, virtual_scan_range_);
  return descriptor.build(local);
}

bool GlobalRelocalizationNode::updateCandidateConfirmation(
  const std::vector<CandidateScore> & scores,
  const std::vector<DatabaseEntry> & database)
{
  std::lock_guard<std::mutex> lock(candidate_mutex_);
  if (scores.empty()) {
    has_pending_candidate_ = false;
    pending_candidate_count_ = 0;
    return false;
  }

  const auto & best = scores.front();
  const double gap =
    scores.size() > 1 ? best.score - scores[1].score : std::numeric_limits<double>::infinity();
  if (gap < min_best_score_gap_) {
    has_pending_candidate_ = false;
    pending_candidate_count_ = 0;
    return false;
  }

  if (best.index >= database.size()) {
    has_pending_candidate_ = false;
    pending_candidate_count_ = 0;
    return false;
  }

  const auto & entry = database[best.index];
  const double yaw_correction =
    static_cast<double>(best.shift) / static_cast<double>(descriptor_sectors_) * 2.0 * M_PI;
  const double yaw = normalizeAngle(entry.yaw + yaw_correction);

  if (!has_pending_candidate_) {
    pending_candidate_x_ = entry.x;
    pending_candidate_y_ = entry.y;
    pending_candidate_yaw_ = yaw;
    pending_candidate_count_ = 1;
    has_pending_candidate_ = true;
    return pending_candidate_count_ >= candidate_confirm_frames_;
  }

  const double dxy = std::hypot(entry.x - pending_candidate_x_, entry.y - pending_candidate_y_);
  const double dyaw = std::abs(normalizeAngle(yaw - pending_candidate_yaw_));
  if (dxy > candidate_consistency_xy_ || dyaw > candidate_consistency_yaw_) {
    pending_candidate_x_ = entry.x;
    pending_candidate_y_ = entry.y;
    pending_candidate_yaw_ = yaw;
    pending_candidate_count_ = 1;
    return pending_candidate_count_ >= candidate_confirm_frames_;
  }

  pending_candidate_x_ = entry.x;
  pending_candidate_y_ = entry.y;
  pending_candidate_yaw_ = yaw;
  pending_candidate_count_++;
  return pending_candidate_count_ >= candidate_confirm_frames_;
}

void GlobalRelocalizationNode::searchTimerCallback()
{
  if (!enabled_) {
    return;
  }
  if (!database_ready_) {
    publishDiagnostics(database_building_ ? "building_database" : "database_not_ready", 0, 0, 0.0, 0.0);
    return;
  }
  if (!isTriggered()) {
    {
      std::lock_guard<std::mutex> lock(candidate_mutex_);
      has_pending_candidate_ = false;
      pending_candidate_count_ = 0;
    }
    publishDiagnostics("idle", 0, 0, 0.0, 0.0);
    return;
  }

  const auto start = std::chrono::steady_clock::now();
  ScanContextDescriptor online(descriptor_rings_, descriptor_sectors_, virtual_scan_range_);
  size_t online_points = 0;
  if (!buildOnlineDescriptor(online, online_points)) {
    publishDiagnostics("waiting_for_scan", online_points, 0, 0.0, 0.0);
    return;
  }

  std::vector<DatabaseEntry> database;
  {
    std::lock_guard<std::mutex> lock(database_mutex_);
    database = database_;
  }

  std::vector<CandidateScore> scores;
  scores.reserve(database.size());
  for (size_t i = 0; i < database.size(); ++i) {
    int shift = 0;
    const double score = online.similarityTo(database[i].descriptor, &shift);
    if (score >= min_descriptor_score_) {
      scores.push_back(CandidateScore{i, score, shift});
    }
  }
  std::sort(scores.begin(), scores.end(), [](const CandidateScore & a, const CandidateScore & b) {
    return a.score > b.score;
  });
  if (scores.size() > static_cast<size_t>(descriptor_top_k_)) {
    scores.resize(static_cast<size_t>(descriptor_top_k_));
  }

  const auto end = std::chrono::steady_clock::now();
  const double search_time_ms =
    std::chrono::duration<double, std::milli>(end - start).count();
  const double best_score = scores.empty() ? 0.0 : scores.front().score;
  const double score_gap =
    scores.size() > 1 ? scores.front().score - scores[1].score : best_score;
  std_msgs::msg::Header header;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    header = newest_scan_header_;
  }
  const bool candidate_confirmed = updateCandidateConfirmation(scores, database);
  auto msg = candidate_confirmed ? makeCandidateMessage(scores, header, database) : geometry_msgs::msg::PoseArray();
  if (!msg.poses.empty()) {
    candidate_pub_->publish(msg);
    RCLCPP_WARN(
      get_logger(), "Published %zu global relocalization candidates, best_score=%.3f gap=%.3f.",
      msg.poses.size(), best_score, score_gap);
  }
  publishDiagnostics(
    candidate_confirmed ? "search_confirmed" : "search_waiting_confirmation",
    online_points, msg.poses.size(), best_score, search_time_ms);
}

geometry_msgs::msg::PoseArray GlobalRelocalizationNode::makeCandidateMessage(
  const std::vector<CandidateScore> & scores, const std_msgs::msg::Header & header,
  const std::vector<DatabaseEntry> & database) const
{
  geometry_msgs::msg::PoseArray msg;
  msg.header = header;
  msg.header.frame_id = map_frame_;
  msg.header.stamp = this->now();
  for (const auto & score : scores) {
    if (score.index >= database.size()) {
      continue;
    }
    const auto & entry = database[score.index];
    const double yaw_correction =
      static_cast<double>(score.shift) / static_cast<double>(descriptor_sectors_) * 2.0 * M_PI;
    const double yaw = normalizeAngle(entry.yaw + yaw_correction);
    geometry_msgs::msg::Pose pose;
    pose.position.x = entry.x;
    pose.position.y = entry.y;
    pose.position.z = 0.0;
    pose.orientation.z = std::sin(0.5 * yaw);
    pose.orientation.w = std::cos(0.5 * yaw);
    msg.poses.push_back(pose);
  }
  return msg;
}

void GlobalRelocalizationNode::publishDiagnostics(
  const std::string & state, size_t online_points, size_t candidate_count,
  double best_score, double search_time_ms)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "global_relocalization";
  status.hardware_id = "relocalization";
  status.level = candidate_count > 0 ?
    diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = state;
  status.values.push_back(makeKeyValue("enabled", enabled_ ? "true" : "false"));
  status.values.push_back(makeKeyValue("database_ready", database_ready_ ? "true" : "false"));
  status.values.push_back(makeKeyValue("database_building", database_building_ ? "true" : "false"));
  size_t database_size = 0;
  {
    std::lock_guard<std::mutex> lock(database_mutex_);
    database_size = database_.size();
  }
  status.values.push_back(makeKeyValue("database_entries", std::to_string(database_size)));
  status.values.push_back(makeKeyValue("localization_state", localization_state_));
  status.values.push_back(makeKeyValue("online_points", std::to_string(online_points)));
  status.values.push_back(makeKeyValue("candidate_count", std::to_string(candidate_count)));
  status.values.push_back(makeKeyValue("best_score", std::to_string(best_score)));
  status.values.push_back(makeKeyValue(
    "candidate_confirm_frames", std::to_string(candidate_confirm_frames_)));
  int pending_candidate_count = 0;
  {
    std::lock_guard<std::mutex> lock(candidate_mutex_);
    pending_candidate_count = pending_candidate_count_;
  }
  status.values.push_back(makeKeyValue(
    "pending_candidate_count", std::to_string(pending_candidate_count)));
  status.values.push_back(makeKeyValue("min_best_score_gap", std::to_string(min_best_score_gap_)));
  status.values.push_back(makeKeyValue("search_time_ms", std::to_string(search_time_ms)));

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = this->now();
  array.status.push_back(status);
  diagnostics_pub_->publish(array);
}

}  // namespace global_relocalization

RCLCPP_COMPONENTS_REGISTER_NODE(global_relocalization::GlobalRelocalizationNode)
