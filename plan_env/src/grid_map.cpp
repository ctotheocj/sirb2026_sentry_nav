// Copyright 2025 Pan — Apache-2.0
// plan_env: FIESTA-based occupancy mapping, ROS2 adaptation
// Reference: FIESTA (tommy 2018, HKUST)

#include "plan_env/grid_map.h"
#include <cmath>
#include <string>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace plan_env
{

GridMap::Ptr GridMapRegistry::instance_ = nullptr;

GridMap::~GridMap()
{
  esdf_worker_stop_.store(true);
  esdf_worker_cv_.notify_all();
  if (esdf_worker_.joinable()) {
    esdf_worker_.join();
  }
}

void GridMap::initMap(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  map_update_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  occ_grid_pub_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  visualization_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Parameters
  mp_.resolution_     = node->declare_parameter("map.resolution", 0.1);
  mp_.resolution_inv_ = 1.0 / mp_.resolution_;
  mp_.obstacles_inflation_ = node->declare_parameter("map.obstacles_inflation", 0.2);
  mp_.obstacles_inflation_xy_ = node->declare_parameter(
    "map.obstacles_inflation_xy", mp_.obstacles_inflation_);
  mp_.obstacles_inflation_z_ = node->declare_parameter(
    "map.obstacles_inflation_z", mp_.obstacles_inflation_);
  mp_.frame_id_       = node->declare_parameter("map.frame_id", std::string("map"));
  mp_.local_map_margin_ = node->declare_parameter("map.local_map_margin", 10);
  mp_.virtual_ceil_height_ = node->declare_parameter("map.virtual_ceil_height", -1.0);
  mp_.ground_height_  = node->declare_parameter("map.ground_height", 0.0);
  mp_.enable_esdf_ = node->declare_parameter("map.enable_esdf", true);
  mp_.enable_negative_esdf_ = node->declare_parameter("map.enable_negative_esdf", true);
  mp_.unknown_flag_   = 0.01;

  std::vector<double> origin = node->declare_parameter("map.origin", std::vector<double>{-10,-10,-1});
  std::vector<double> size   = node->declare_parameter("map.size",   std::vector<double>{20, 20, 3});
  mp_.map_origin_ = Eigen::Vector3d(origin[0], origin[1], origin[2]);
  mp_.map_size_   = Eigen::Vector3d(size[0],   size[1],   size[2]);
  for (int i = 0; i < 3; ++i) {
    mp_.map_min_boundary_(i) = mp_.map_origin_(i);
    mp_.map_max_boundary_(i) = mp_.map_origin_(i) + mp_.map_size_(i);
    mp_.map_voxel_num_(i) = static_cast<int>(std::ceil(mp_.map_size_(i) * mp_.resolution_inv_));
  }

  // Log-odds parameters
  mp_.prob_hit_log_      = node->declare_parameter("map.prob_hit_log",  0.70);
  mp_.prob_miss_log_     = node->declare_parameter("map.prob_miss_log", -0.40);
  mp_.clamp_min_log_     = node->declare_parameter("map.clamp_min_log", -2.0);
  mp_.clamp_max_log_     = node->declare_parameter("map.clamp_max_log",  3.5);
  mp_.min_occupancy_log_ = node->declare_parameter("map.min_occupancy_log", 0.0);
  mp_.min_ray_length_    = node->declare_parameter("map.min_ray_length", 0.1);
  mp_.max_ray_length_    = node->declare_parameter("map.max_ray_length", 10.0);

  const int total = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
  md_.occupancy_buffer_.assign(total, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_.assign(total, 0);
  md_.num_hit_.assign(total, 0);
  md_.num_miss_.assign(total, 0);
  md_.last_hit_time_.assign(total, 0.0);
  md_.esdf_buffer_.assign(total, 1e3);
  md_.esdf_buffer_neg_.assign(total, 1e3);
  md_.esdf_buffer_signed_.assign(total, 1e3);
  md_.closest_obstacle_.assign(total, Eigen::Vector3i(-1,-1,-1));
  md_.head_.assign(total, -1);
  md_.prev_.assign(total, -1);
  md_.next_.assign(total, -1);

  pub_z_slice_ = node->declare_parameter("map.pub_z_slice", 0.3);
  occupancy_grid_project_all_z_ =
    node->declare_parameter("map.occupancy_grid_project_all_z", true);
  occupancy_grid_min_z_ = node->declare_parameter("map.occupancy_grid_min_z", 0.05);
  occupancy_grid_max_z_ = node->declare_parameter("map.occupancy_grid_max_z", 1.2);
  if (occupancy_grid_min_z_ > occupancy_grid_max_z_) {
    std::swap(occupancy_grid_min_z_, occupancy_grid_max_z_);
  }
  occupancy_grid_hold_enabled_ =
    node->declare_parameter("map.occupancy_grid_hold_enabled", true);
  occupancy_grid_hold_time_ = node->declare_parameter("map.occupancy_grid_hold_time", 0.6);
  occupancy_grid_debug_ = node->declare_parameter("map.occupancy_grid_debug", false);
  occupancy_grid_publish_period_ms_ = std::max<int>(
    20, static_cast<int>(node->declare_parameter<int>("map.occupancy_grid_publish_period_ms", 100)));
  esdf_update_period_ms_ = std::max<int>(
    20, static_cast<int>(node->declare_parameter<int>("map.esdf_update_period_ms", 50)));
  occupancy_grid_last_occupied_time_.assign(
    static_cast<size_t>(mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1)), -1.0);
  voxel_decay_time_ = node->declare_parameter("map.voxel_decay_time", 5.0);
  cloud_stale_timeout_sec_ = node->declare_parameter("map.cloud_stale_timeout_sec", 0.5);
  transform_timeout_sec_ = node->declare_parameter("map.transform_timeout_sec", 0.05);
  allow_latest_tf_fallback_ = node->declare_parameter("map.allow_latest_tf_fallback", true);
  latest_tf_fallback_max_age_sec_ =
    node->declare_parameter("map.latest_tf_fallback_max_age_sec", 0.5);
  const std::string cloud_topic  = node->declare_parameter("map.cloud_topic",  std::string("obstacle_cloud"));
  const std::string odom_topic   = node->declare_parameter("map.odom_topic",   std::string("odometry"));
  occ_grid_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "occupancy_grid", rclcpp::QoS(1).transient_local());
  map_inf_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "grid_map/occupancy_inflate", rclcpp::QoS(1).transient_local());
  esdf_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "grid_map/esdf", rclcpp::QoS(1));

  visualization_truncate_height_ = node->declare_parameter("map.visualization_truncate_height", 1.5);

  rclcpp::SubscriptionOptions map_sub_options;
  map_sub_options.callback_group = map_update_group_;
  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic, rclcpp::SensorDataQoS(),
    std::bind(&GridMap::cloudCallback, this, std::placeholders::_1),
    map_sub_options);

  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::SensorDataQoS(),
    std::bind(&GridMap::odomCallback, this, std::placeholders::_1),
    map_sub_options);

  occ_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&GridMap::updateOccupancy, this),
    map_update_group_);
  occ_grid_pub_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(occupancy_grid_publish_period_ms_),
    std::bind(&GridMap::publishOccupancyGridTimer, this),
    occ_grid_pub_group_);
  vis_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() { publishMapInflate(); publishESDF(); },
    visualization_group_);
  esdf_worker_ = std::thread(&GridMap::esdfWorkerLoop, this);
}

int GridMap::toAddress(const Eigen::Vector3i & id) const
{
  return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)
       + id(1) * mp_.map_voxel_num_(2)
       + id(2);
}

void GridMap::boundIndex(Eigen::Vector3i & id) const
{
  for (int i = 0; i < 3; ++i)
    id(i) = std::max(0, std::min(id(i), mp_.map_voxel_num_(i) - 1));
}

bool GridMap::isInMap(const Eigen::Vector3d & pos) const
{
  for (int i = 0; i < 3; ++i)
    if (pos(i) < mp_.map_min_boundary_(i) + 1e-4 || pos(i) > mp_.map_max_boundary_(i) - 1e-4)
      return false;
  return true;
}

bool GridMap::isInMap(const Eigen::Vector3i & idx) const
{
  for (int i = 0; i < 3; ++i)
    if (idx(i) < 0 || idx(i) >= mp_.map_voxel_num_(i)) return false;
  return true;
}

void GridMap::posToIndex(const Eigen::Vector3d & pos, Eigen::Vector3i & id) const
{
  for (int i = 0; i < 3; ++i)
    id(i) = static_cast<int>(std::floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_));
}

void GridMap::indexToPos(const Eigen::Vector3i & id, Eigen::Vector3d & pos) const
{
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

int GridMap::getInflateOccupancy(const Eigen::Vector3d & pos) const
{
  if (!isInMap(pos)) return -1;
  Eigen::Vector3i id;
  posToIndex(pos, id);
  std::lock_guard<std::mutex> lock(map_mutex_);
  return static_cast<int>(md_.occupancy_buffer_inflate_[toAddress(id)]);
}

void GridMap::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  Eigen::Affine3d tf_map_odom = Eigen::Affine3d::Identity();
  if (!lookupTransformToMap(msg->header.frame_id, msg->header.stamp, true, tf_map_odom)) return;

  const Eigen::Affine3d tf_odom_base = poseMsgToEigen(msg->pose.pose);
  const Eigen::Affine3d tf_map_base = tf_map_odom * tf_odom_base;
  std::lock_guard<std::mutex> lock(map_mutex_);
  md_.camera_pos_ = tf_map_base.translation();
  md_.has_odom_ = true;
}

void GridMap::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  Eigen::Vector3d camera_pos;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!md_.has_odom_) return;
    camera_pos = md_.camera_pos_;
  }

  Eigen::Affine3d tf_map_cloud = Eigen::Affine3d::Identity();
  if (!lookupTransformToMap(msg->header.frame_id, msg->header.stamp, true, tf_map_cloud)) return;

  pcl::PointCloud<pcl::PointXYZ> cloud_in;
  pcl::fromROSMsg(*msg, cloud_in);

  Eigen::Vector3i cam_idx;
  posToIndex(camera_pos, cam_idx);
  const int r = static_cast<int>(mp_.max_ray_length_ * mp_.resolution_inv_);
  std::lock_guard<std::mutex> lock(map_mutex_);
  md_.camera_pos_ = camera_pos;
  md_.last_local_bound_min_ = md_.local_bound_min_;
  md_.last_local_bound_max_ = md_.local_bound_max_;
  md_.local_bound_min_ = cam_idx - Eigen::Vector3i(r, r, r);
  md_.local_bound_max_ = cam_idx + Eigen::Vector3i(r, r, r);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  const double now = node_->now().seconds();
  for (const auto & pt : cloud_in.points) {
    Eigen::Vector3d end = tf_map_cloud * Eigen::Vector3d(pt.x, pt.y, pt.z);
    double ray_len = (end - camera_pos).norm();
    if (ray_len < mp_.min_ray_length_) continue;
    if (ray_len > mp_.max_ray_length_)
      end = camera_pos + (end - camera_pos) * (mp_.max_ray_length_ / ray_len);
    raycastUpdate(camera_pos, end);
    Eigen::Vector3i end_idx;
    posToIndex(end, end_idx);
    if (isInMap(end_idx)) {
      int adr = toAddress(end_idx);
      md_.last_hit_time_[adr] = now;
    }
  }

  md_.occ_need_update_ = true;
  md_.local_updated_   = true;
  last_cloud_time_.store(node_->now().seconds());
}

bool GridMap::lookupTransformToMap(
  const std::string & source_frame,
  const rclcpp::Time & stamp,
  bool allow_latest_fallback,
  Eigen::Affine3d & tf_map_source) const
{
  if (source_frame.empty() || source_frame == mp_.frame_id_) {
    tf_map_source = Eigen::Affine3d::Identity();
    return true;
  }

  try {
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform(
        mp_.frame_id_, source_frame, stamp, rclcpp::Duration::from_seconds(transform_timeout_sec_));
    } catch (const tf2::ExtrapolationException & ex) {
      if (!allow_latest_fallback || !allow_latest_tf_fallback_) {
        throw;
      }
      const rclcpp::Time latest_time(0, 0, node_->get_clock()->get_clock_type());
      tf_msg = tf_buffer_->lookupTransform(
        mp_.frame_id_, source_frame, latest_time,
        rclcpp::Duration::from_seconds(transform_timeout_sec_));
      const double latest_age = (node_->now() - rclcpp::Time(tf_msg.header.stamp)).seconds();
      if (latest_age > latest_tf_fallback_max_age_sec_) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[GridMap] latest TF %s -> %s is stale %.3fs > %.3fs; dropping frame",
          source_frame.c_str(), mp_.frame_id_.c_str(), latest_age, latest_tf_fallback_max_age_sec_);
        return false;
      }
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "[GridMap] TF %s -> %s at stamp %.3f unavailable (%s); using latest transform stamp %.3f",
        source_frame.c_str(), mp_.frame_id_.c_str(),
        rclcpp::Time(stamp).seconds(), ex.what(), rclcpp::Time(tf_msg.header.stamp).seconds());
    }
    tf_map_source = tf2::transformToEigen(tf_msg.transform);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "[GridMap] TF %s -> %s failed: %s",
      source_frame.c_str(), mp_.frame_id_.c_str(), ex.what());
    return false;
  }
}

Eigen::Affine3d GridMap::poseMsgToEigen(const geometry_msgs::msg::Pose & pose) const
{
  Eigen::Translation3d translation(
    pose.position.x,
    pose.position.y,
    pose.position.z);
  Eigen::Quaterniond rotation(
    pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z);
  return translation * rotation;
}

// DDA 3D raycasting: mark miss along ray, hit at endpoint (FIESTA style)
void GridMap::raycastUpdate(const Eigen::Vector3d & start, const Eigen::Vector3d & end)
{
  Eigen::Vector3i start_idx, end_idx;
  posToIndex(start, start_idx);
  posToIndex(end,   end_idx);

  Eigen::Vector3i diff = end_idx - start_idx;
  int step[3], n = 0;
  double tMax[3], tDelta[3];
  for (int i = 0; i < 3; ++i) {
    step[i]   = (diff(i) > 0) ? 1 : (diff(i) < 0 ? -1 : 0);
    n        += std::abs(diff(i));
    tDelta[i] = (step[i] != 0) ? 1.0 / std::abs(diff(i) + 1e-9) : 1e9;
    tMax[i]   = (step[i] != 0) ? tDelta[i] * 0.5 : 1e9;
  }

  Eigen::Vector3i cur = start_idx;
  for (int k = 0; k < n; ++k) {
    if (isInMap(cur)) {
      int adr = toAddress(cur);
      md_.num_miss_[adr]++;
    }
    // advance DDA
    int axis = (tMax[0] < tMax[1]) ? ((tMax[0] < tMax[2]) ? 0 : 2)
                                    : ((tMax[1] < tMax[2]) ? 1 : 2);
    cur(axis) += step[axis];
    tMax[axis] += tDelta[axis];
  }
  // hit at endpoint
  if (isInMap(end_idx)) {
    md_.num_hit_[toAddress(end_idx)]++;
  }
}

void GridMap::inflatePoint(
  const Eigen::Vector3i & pt, int step_xy, int step_z,
  std::vector<Eigen::Vector3i> & pts) const
{
  int num = 0;
  for (int x = -step_xy; x <= step_xy; ++x)
    for (int y = -step_xy; y <= step_xy; ++y)
      for (int z = -step_z; z <= step_z; ++z)
        pts[num++] = Eigen::Vector3i(pt(0)+x, pt(1)+y, pt(2)+z);
}

void GridMap::clearAndInflateLocalMap()
{
  // Clear inflate buffer in local region
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
        md_.occupancy_buffer_inflate_[toAddress(Eigen::Vector3i(x,y,z))] = 0;

  const int inf_step_xy = std::max(0, static_cast<int>(
    std::ceil(mp_.obstacles_inflation_xy_ * mp_.resolution_inv_)));
  const int inf_step_z = std::max(0, static_cast<int>(
    std::ceil(mp_.obstacles_inflation_z_ * mp_.resolution_inv_)));
  const size_t inf_diameter_xy = static_cast<size_t>(2 * inf_step_xy + 1);
  const size_t inf_diameter_z = static_cast<size_t>(2 * inf_step_z + 1);
  const size_t inf_pts_count = inf_diameter_xy * inf_diameter_xy * inf_diameter_z;
  std::vector<Eigen::Vector3i> inf_pts(inf_pts_count);

  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        if (md_.occupancy_buffer_[toAddress(Eigen::Vector3i(x,y,z))] > mp_.min_occupancy_log_) {
          inflatePoint(Eigen::Vector3i(x,y,z), inf_step_xy, inf_step_z, inf_pts);
          for (const auto & ip : inf_pts) {
            if (!isInMap(ip)) continue;
            md_.occupancy_buffer_inflate_[toAddress(ip)] = 1;
          }
        }
      }
}

void GridMap::updateOccupancy()
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  // Warn if obstacle_cloud has not arrived recently — map is silently decaying
  const double last_cloud_time = last_cloud_time_.load();
  if (last_cloud_time > 0.0) {
    const double cloud_age = node_->now().seconds() - last_cloud_time;
    if (cloud_age > cloud_stale_timeout_sec_)
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "[GridMap] obstacle_cloud stale (%.2fs) — occupancy_grid publishing is held", cloud_age);
  }

  if (!md_.occ_need_update_) return;

  // FIESTA: apply num_hit/num_miss to log-odds (majority-vote, matches FIESTA UpdateOccupancy)
  const double current_time = node_->now().seconds();
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        Eigen::Vector3i id(x, y, z);
        int adr = toAddress(id);
        if (md_.num_hit_[adr] == 0 && md_.num_miss_[adr] == 0) {
          // Fading: decay occupied voxels not observed recently
          if (md_.occupancy_buffer_[adr] > mp_.clamp_min_log_) {
            double elapsed = current_time - md_.last_hit_time_[adr];
            if (elapsed > voxel_decay_time_)
              md_.occupancy_buffer_[adr] = std::max(
                md_.occupancy_buffer_[adr] + mp_.prob_miss_log_, mp_.clamp_min_log_);
          }
          continue;
        }
        double log_odds_update = (md_.num_hit_[adr] >= md_.num_miss_[adr] - md_.num_hit_[adr])
                                 ? mp_.prob_hit_log_ : mp_.prob_miss_log_;
        md_.num_hit_[adr] = md_.num_miss_[adr] = 0;
        // FIESTA UpdateOccupancy line 256-259: reset voxels outside previous local range
        bool in_last_range = (x >= md_.last_local_bound_min_(0) && x <= md_.last_local_bound_max_(0) &&
                              y >= md_.last_local_bound_min_(1) && y <= md_.last_local_bound_max_(1) &&
                              z >= md_.last_local_bound_min_(2) && z <= md_.last_local_bound_max_(2));
        if (!in_last_range) {
          md_.occupancy_buffer_[adr] = 0.0;
          md_.esdf_buffer_[adr] = 1e3;
          continue;
        }
        md_.occupancy_buffer_[adr] = std::min(
          std::max(md_.occupancy_buffer_[adr] + log_odds_update, mp_.clamp_min_log_),
          mp_.clamp_max_log_);
      }

  if (md_.local_updated_) clearAndInflateLocalMap();
  occupancy_grid_has_snapshot_ = true;
  occupancy_grid_dirty_ = true;
  publishOccupancyGrid();
  occupancy_grid_dirty_ = false;
  esdf_need_update_.store(true);
  esdf_worker_cv_.notify_one();
  md_.occ_need_update_ = false;
  md_.local_updated_   = false;
}

void GridMap::publishOccupancyGridTimer()
{
  nav_msgs::msg::OccupancyGrid msg;
  const double last_cloud_time = last_cloud_time_.load();
  if (last_cloud_time <= 0.0) return;
  const double cloud_age = node_->now().seconds() - last_cloud_time;
  if (cloud_age > cloud_stale_timeout_sec_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[GridMap] skip occupancy_grid publish: obstacle_cloud stale %.2fs > %.2fs",
      cloud_age, cloud_stale_timeout_sec_);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(occ_grid_msg_mutex_);
    if (!latest_occupancy_grid_valid_) return;
    msg = latest_occupancy_grid_;
  }

  msg.header.stamp = node_->now();
  occ_grid_pub_->publish(msg);
}

void GridMap::updateESDFTimer()
{
  if (!mp_.enable_esdf_) return;

  std::lock_guard<std::mutex> lock(map_mutex_);
  if (!esdf_need_update_.exchange(false)) return;

  updateESDFIncremental();
  if (mp_.enable_negative_esdf_) {
    computeNegativeESDF();
  } else {
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
        for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
          int idx = toAddress(Eigen::Vector3i(x, y, z));
          md_.esdf_buffer_signed_[idx] = md_.esdf_buffer_[idx];
        }
  }
}

void GridMap::esdfWorkerLoop()
{
  while (!esdf_worker_stop_.load()) {
    std::unique_lock<std::mutex> worker_lock(esdf_worker_mutex_);
    esdf_worker_cv_.wait_for(
      worker_lock,
      std::chrono::milliseconds(esdf_update_period_ms_),
      [this]() {return esdf_worker_stop_.load() || esdf_need_update_.load();});
    worker_lock.unlock();

    if (esdf_worker_stop_.load()) break;
    updateESDFTimer();
  }
}

void GridMap::publishOccupancyGrid()
{
  const int nx = mp_.map_voxel_num_(0);
  const int ny = mp_.map_voxel_num_(1);
  const int nz = mp_.map_voxel_num_(2);

  int iz_min = 0;
  int iz_max = 0;
  if (occupancy_grid_project_all_z_) {
    iz_min = static_cast<int>(
      std::floor((occupancy_grid_min_z_ - mp_.map_origin_(2)) * mp_.resolution_inv_));
    iz_max = static_cast<int>(
      std::floor((occupancy_grid_max_z_ - mp_.map_origin_(2)) * mp_.resolution_inv_));
  } else {
    iz_min = static_cast<int>(
      std::floor((pub_z_slice_ - mp_.map_origin_(2)) * mp_.resolution_inv_));
    iz_max = iz_min;
  }

  iz_min = std::max(0, std::min(iz_min, nz - 1));
  iz_max = std::max(0, std::min(iz_max, nz - 1));
  if (iz_min > iz_max) {
    std::swap(iz_min, iz_max);
  }

  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = mp_.frame_id_;
  msg.header.stamp = node_->now();
  msg.info.resolution = static_cast<float>(mp_.resolution_);
  msg.info.width  = static_cast<uint32_t>(nx);
  msg.info.height = static_cast<uint32_t>(ny);
  msg.info.origin.position.x = mp_.map_origin_(0);
  msg.info.origin.position.y = mp_.map_origin_(1);
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data.resize(static_cast<size_t>(nx * ny), 0);
  const double now = node_->now().seconds();
  const size_t grid_size = static_cast<size_t>(nx * ny);
  if (occupancy_grid_last_occupied_time_.size() != grid_size) {
    occupancy_grid_last_occupied_time_.assign(grid_size, -1.0);
  }

  size_t current_occupied_count = 0;
  size_t held_occupied_count = 0;
  for (int iy = 0; iy < ny; ++iy) {
    for (int ix = 0; ix < nx; ++ix) {
      const size_t idx2d = static_cast<size_t>(iy * nx + ix);
      bool occupied = false;
      for (int iz = iz_min; iz <= iz_max; ++iz) {
        if (occupancy_grid_project_all_z_) {
          const double z_center =
            mp_.map_origin_(2) + (static_cast<double>(iz) + 0.5) * mp_.resolution_;
          if (z_center < occupancy_grid_min_z_ || z_center > occupancy_grid_max_z_) {
            continue;
          }
        }
        if (md_.occupancy_buffer_inflate_[toAddress(Eigen::Vector3i(ix, iy, iz))]) {
          occupied = true;
          break;
        }
      }
      if (occupied) {
        occupancy_grid_last_occupied_time_[idx2d] = now;
        msg.data[idx2d] = 100;
        ++current_occupied_count;
      } else if (occupancy_grid_hold_enabled_ &&
        occupancy_grid_last_occupied_time_[idx2d] >= 0.0 &&
        now - occupancy_grid_last_occupied_time_[idx2d] <= occupancy_grid_hold_time_)
      {
        msg.data[idx2d] = 100;
        ++held_occupied_count;
      }
    }
  }

  if (occupancy_grid_debug_) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "[GridMap] occupancy_grid projected current=%zu held=%zu z_range=[%.3f, %.3f] hold=%.2fs",
      current_occupied_count, held_occupied_count,
      occupancy_grid_min_z_, occupancy_grid_max_z_, occupancy_grid_hold_time_);
  }
  {
    std::lock_guard<std::mutex> lock(occ_grid_msg_mutex_);
    latest_occupancy_grid_ = msg;
    latest_occupancy_grid_valid_ = true;
  }
}

// FIESTA-style incremental ESDF: raise/lower queues + closest_obstacle linked-list
// Reference: FIESTA (tommy 2018, HKUST) — UpdateESDF()

void GridMap::insertIntoList(int obs_idx, int vox_idx)
{
  md_.next_[vox_idx] = md_.head_[obs_idx];
  md_.prev_[vox_idx] = -1;
  if (md_.head_[obs_idx] >= 0) md_.prev_[md_.head_[obs_idx]] = vox_idx;
  md_.head_[obs_idx] = vox_idx;
}

void GridMap::deleteFromList(int obs_idx, int vox_idx)
{
  if (obs_idx < 0 || vox_idx < 0) return;
  if (md_.prev_[vox_idx] >= 0) md_.next_[md_.prev_[vox_idx]] = md_.next_[vox_idx];
  else md_.head_[obs_idx] = md_.next_[vox_idx];
  if (md_.next_[vox_idx] >= 0) md_.prev_[md_.next_[vox_idx]] = md_.prev_[vox_idx];
  md_.prev_[vox_idx] = md_.next_[vox_idx] = -1;
}

static const int kDx[6] = {1,-1,0,0,0,0};
static const int kDy[6] = {0,0,1,-1,0,0};
static const int kDz[6] = {0,0,0,0,1,-1};

static const Eigen::Vector3i kUndefined(-1,-1,-1);

void GridMap::updateESDFIncremental()
{
  const int total = int(md_.esdf_buffer_.size());
  if (total == 0) return;

  struct Entry { Eigen::Vector3i pt; double dist; };
  std::queue<Entry> insert_q, delete_q, update_q;

  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        Eigen::Vector3i id(x,y,z);
        int adr = toAddress(id);
        bool occ = md_.occupancy_buffer_inflate_[adr] != 0;
        bool had_closest = md_.closest_obstacle_[adr](0) >= 0;

        if (occ && md_.esdf_buffer_[adr] > 0.0) {
          insert_q.push({id, 0.0});
        } else if (!occ && had_closest) {
          const Eigen::Vector3i & obs = md_.closest_obstacle_[adr];
          if (md_.occupancy_buffer_inflate_[toAddress(obs)] == 0)
            delete_q.push({id, 1e3});
        }
      }

  // --- Insert (lower) ---
  while (!insert_q.empty()) {
    auto [pt, d] = insert_q.front(); insert_q.pop();
    int idx = toAddress(pt);
    const Eigen::Vector3i & old_obs = md_.closest_obstacle_[idx];
    if (old_obs(0) >= 0) deleteFromList(toAddress(old_obs), idx);
    md_.closest_obstacle_[idx] = pt;
    md_.esdf_buffer_[idx] = 0.0;
    insertIntoList(idx, idx);
    update_q.push({pt, 0.0});
  }

  // --- Delete (raise) ---
  while (!delete_q.empty()) {
    auto [pt, d] = delete_q.front(); delete_q.pop();
    int obs_idx = toAddress(pt);
    int v = md_.head_[obs_idx];
    while (v >= 0) {
      int nxt = md_.next_[v];
      md_.closest_obstacle_[v] = kUndefined;
      md_.esdf_buffer_[v] = 1e3;
      Eigen::Vector3i vox(v / (mp_.map_voxel_num_(1)*mp_.map_voxel_num_(2)),
        (v / mp_.map_voxel_num_(2)) % mp_.map_voxel_num_(1),
        v % mp_.map_voxel_num_(2));
      for (int i = 0; i < 6; ++i) {
        Eigen::Vector3i nb(vox(0)+kDx[i], vox(1)+kDy[i], vox(2)+kDz[i]);
        if (!isInMap(nb)) continue;
        int nb_idx = toAddress(nb);
        const Eigen::Vector3i & nb_obs = md_.closest_obstacle_[nb_idx];
        if (nb_obs(0) >= 0 && md_.occupancy_buffer_inflate_[toAddress(nb_obs)]) {
          double dist = (vox.cast<double>() - nb_obs.cast<double>()).norm() * mp_.resolution_;
          if (dist < md_.esdf_buffer_[v]) {
            md_.esdf_buffer_[v] = dist;
            md_.closest_obstacle_[v] = nb_obs;
          }
          break;  // FIESTA: stop after first valid neighbor
        }
      }
      if (md_.closest_obstacle_[v](0) >= 0) {
        insertIntoList(toAddress(md_.closest_obstacle_[v]), v);
        update_q.push({vox, md_.esdf_buffer_[v]});
      }
      v = nxt;
    }
    md_.head_[obs_idx] = -1;
  }

  // --- Propagation BFS (pull all neighbors first, then push — matches FIESTA UpdateESDF) ---
  while (!update_q.empty()) {
    auto [pt, d] = update_q.front(); update_q.pop();
    int idx = toAddress(pt);
    if (std::fabs(d - md_.esdf_buffer_[idx]) > 1e-6) continue;

    // pull: check all neighbors, update self if better
    bool change = false;
    for (int i = 0; i < 6; ++i) {
      Eigen::Vector3i nb(pt(0)+kDx[i], pt(1)+kDy[i], pt(2)+kDz[i]);
      if (!isInMap(nb)) continue;
      int nb_idx = toAddress(nb);
      const Eigen::Vector3i & nb_obs = md_.closest_obstacle_[nb_idx];
      if (nb_obs(0) < 0) continue;
      double tmp = (pt.cast<double>() - nb_obs.cast<double>()).norm() * mp_.resolution_;
      if (tmp < md_.esdf_buffer_[idx]) {
        const Eigen::Vector3i & my_obs = md_.closest_obstacle_[idx];
        if (my_obs(0) >= 0) deleteFromList(toAddress(my_obs), idx);
        md_.esdf_buffer_[idx] = tmp;
        md_.closest_obstacle_[idx] = nb_obs;
        insertIntoList(toAddress(nb_obs), idx);
        change = true;
      }
    }
    if (change) {
      update_q.push({pt, md_.esdf_buffer_[idx]});
      continue;
    }

    // push: propagate to neighbors
    const Eigen::Vector3i & my_obs = md_.closest_obstacle_[idx];
    if (my_obs(0) < 0) continue;
    for (int i = 0; i < 6; ++i) {
      Eigen::Vector3i nb(pt(0)+kDx[i], pt(1)+kDy[i], pt(2)+kDz[i]);
      if (!isInMap(nb)) continue;
      int nb_idx = toAddress(nb);
      double tmp = (nb.cast<double>() - my_obs.cast<double>()).norm() * mp_.resolution_;
      if (tmp < md_.esdf_buffer_[nb_idx]) {
        const Eigen::Vector3i & old_nb_obs = md_.closest_obstacle_[nb_idx];
        if (old_nb_obs(0) >= 0) deleteFromList(toAddress(old_nb_obs), nb_idx);
        md_.esdf_buffer_[nb_idx] = tmp;
        md_.closest_obstacle_[nb_idx] = my_obs;
        insertIntoList(toAddress(my_obs), nb_idx);
        update_q.push({nb, tmp});
      }
    }
  }
}

void GridMap::computeNegativeESDF()
{
  // Reset neg buffer in local region
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        int adr = toAddress(Eigen::Vector3i(x,y,z));
        md_.esdf_buffer_neg_[adr] = 1e3;
        md_.esdf_buffer_signed_[adr] = md_.esdf_buffer_[adr];  // default: positive dist
      }

  struct Entry { Eigen::Vector3i pt; double dist; };
  std::queue<Entry> q;

  // Seed: occupied voxels adjacent to at least one free voxel → dist = resolution
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        Eigen::Vector3i id(x,y,z);
        if (!md_.occupancy_buffer_inflate_[toAddress(id)]) continue;
        for (int i = 0; i < 6; ++i) {
          Eigen::Vector3i nb(x+kDx[i], y+kDy[i], z+kDz[i]);
          if (!isInMap(nb) || md_.occupancy_buffer_inflate_[toAddress(nb)]) continue;
          // id is occupied, nb is free
          int adr = toAddress(id);
          if (md_.esdf_buffer_neg_[adr] > mp_.resolution_) {
            md_.esdf_buffer_neg_[adr] = mp_.resolution_;
            q.push({id, mp_.resolution_});
          }
          break;
        }
      }

  // BFS propagation into occupied region only
  while (!q.empty()) {
    auto [pt, d] = q.front(); q.pop();
    int idx = toAddress(pt);
    if (std::fabs(d - md_.esdf_buffer_neg_[idx]) > 1e-6) continue;
    for (int i = 0; i < 6; ++i) {
      Eigen::Vector3i nb(pt(0)+kDx[i], pt(1)+kDy[i], pt(2)+kDz[i]);
      if (!isInMap(nb) || !md_.occupancy_buffer_inflate_[toAddress(nb)]) continue;
      double nd = d + mp_.resolution_;
      int nb_idx = toAddress(nb);
      if (nd < md_.esdf_buffer_neg_[nb_idx]) {
        md_.esdf_buffer_neg_[nb_idx] = nd;
        q.push({nb, nd});
      }
    }
  }

  // Combine into signed distance (matches sdf_map.cpp:469-472)
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        int idx = toAddress(Eigen::Vector3i(x,y,z));
        if (md_.esdf_buffer_neg_[idx] < 1e2)  // inside obstacle, neg dist valid
          md_.esdf_buffer_signed_[idx] = -(md_.esdf_buffer_neg_[idx] - mp_.resolution_);
      }
}

double GridMap::getSignedDistance2D(double x, double y, double z) const
{
  const Eigen::Vector3d pos(x, y, z);
  if (!isInMap(pos)) return -1e6;  // sentinel: distinct from any valid signed distance
  Eigen::Vector3i id;
  posToIndex(pos, id);
  std::lock_guard<std::mutex> lock(map_mutex_);
  return md_.esdf_buffer_signed_[toAddress(id)];
}

double GridMap::getDistance(const Eigen::Vector3d & pos) const
{
  if (!isInMap(pos)) return -1.0;
  Eigen::Vector3i id;
  posToIndex(pos, id);
  std::lock_guard<std::mutex> lock(map_mutex_);
  return md_.esdf_buffer_[toAddress(id)];
}

double GridMap::getDistance2D(double x, double y, double z) const
{
  const Eigen::Vector3d pos(x, y, z);
  if (!isInMap(pos)) return -1.0;
  Eigen::Vector3i id;
  posToIndex(pos, id);
  std::lock_guard<std::mutex> lock(map_mutex_);
  return md_.esdf_buffer_[toAddress(id)];
}

void GridMap::getGradient2D(double x, double y, double z,
                             double & gx, double & gy) const
{
  double res = mp_.resolution_;
  auto sample_distance = [this](double sx, double sy, double sz) {
    const Eigen::Vector3d pos(sx, sy, sz);
    if (!isInMap(pos)) return -1.0;
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return md_.esdf_buffer_[toAddress(id)];
  };
  std::lock_guard<std::mutex> lock(map_mutex_);
  double d0  = sample_distance(x,      y,      z);
  double dxp = sample_distance(x+res,  y,      z);
  double dxm = sample_distance(x-res,  y,      z);
  double dyp = sample_distance(x,      y+res,  z);
  double dym = sample_distance(x,      y-res,  z);
  // 中心差分为零（窄道对称点）时，取绝对值更大的单侧差分
  double cd_x = (dxp - dxm) * 0.5;
  double cd_y = (dyp - dym) * 0.5;
  gx = (std::abs(cd_x) > 1e-6) ? cd_x / res
     : ((std::abs(dxp - d0) >= std::abs(d0 - dxm)) ? (dxp - d0) / res : (d0 - dxm) / res);
  gy = (std::abs(cd_y) > 1e-6) ? cd_y / res
     : ((std::abs(dyp - d0) >= std::abs(d0 - dym)) ? (dyp - d0) / res : (d0 - dym) / res);
}

void GridMap::publishMapInflate()
{
  if (map_inf_pub_->get_subscription_count() == 0) return;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::lock_guard<std::mutex> lock(map_mutex_);
  Eigen::Vector3i min_cut = md_.local_bound_min_ - Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ + Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut); boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (!md_.occupancy_buffer_inflate_[toAddress(Eigen::Vector3i(x,y,z))]) continue;
        Eigen::Vector3d pos; indexToPos(Eigen::Vector3i(x,y,z), pos);
        if (pos(2) > visualization_truncate_height_) continue;
        cloud.emplace_back(static_cast<float>(pos(0)), static_cast<float>(pos(1)), static_cast<float>(pos(2)));
      }

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = mp_.frame_id_;
  msg.header.stamp = node_->now();
  map_inf_pub_->publish(msg);
}

void GridMap::publishESDF()
{
  if (esdf_pub_->get_subscription_count() == 0) return;

  pcl::PointCloud<pcl::PointXYZI> cloud;
  std::lock_guard<std::mutex> lock(map_mutex_);
  Eigen::Vector3i slice_idx;
  posToIndex(Eigen::Vector3d(0, 0, pub_z_slice_), slice_idx);
  const int iz = std::max(0, std::min(slice_idx(2), mp_.map_voxel_num_(2)-1));

  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {
      double dist = md_.esdf_buffer_[toAddress(Eigen::Vector3i(x, y, iz))];
      if (dist < 0 || dist > 3.0) continue;
      Eigen::Vector3d pos; indexToPos(Eigen::Vector3i(x, y, iz), pos);
      pcl::PointXYZI pt;
      pt.x = static_cast<float>(pos(0));
      pt.y = static_cast<float>(pos(1));
      pt.z = static_cast<float>(pos(2));
      pt.intensity = static_cast<float>(dist);
      cloud.push_back(pt);
    }

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = mp_.frame_id_;
  msg.header.stamp = node_->now();
  esdf_pub_->publish(msg);
}

}  // namespace plan_env
