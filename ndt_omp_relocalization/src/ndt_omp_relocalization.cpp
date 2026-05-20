// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ndt_omp_relocalization/ndt_omp_relocalization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace ndt_omp_relocalization
{

// ---------------------------------------------------------------------------
// Helper: extract yaw from Eigen rotation matrix (avoids tf2 conversion)
// ---------------------------------------------------------------------------
double NdtOmpRelocalizationNode::getYaw(const Eigen::Matrix3d & rot)
{
  return std::atan2(rot(1, 0), rot(0, 0));
}

// ---------------------------------------------------------------------------
// Angle difference normalised to [-π, π]
// ---------------------------------------------------------------------------
static double normalizeAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
NdtOmpRelocalizationNode::NdtOmpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("ndt_omp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity()),
  last_trusted_result_t_(Eigen::Isometry3d::Identity()),
  has_last_trusted_result_(false),
  converged_(false),
  trust_ndt_(false),
  consecutive_converged_count_(0),
  enable_roll_pitch_fix_(true),
  map_ready_(false)
{
  // --- declare parameters ---------------------------------------------------
  // NDT specific
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("ndt_resolution", 1.0);
  this->declare_parameter("ndt_step_size", 0.1);
  this->declare_parameter("ndt_epsilon", 0.01);
  this->declare_parameter("ndt_max_iterations", 30);
  this->declare_parameter("ndt_search_method", 1);  // 0:KDTREE 1:DIRECT7 2:DIRECT1 3:DIRECT26
  this->declare_parameter("fitness_score_threshold", 1.0);
  this->declare_parameter("registration_period_ms", 300);

  // Common
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("min_source_points", 1000);
  this->declare_parameter("min_filtered_points", 120);
  this->declare_parameter("max_scan_age_sec", 0.5);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("enable_roll_pitch_fix", true);
  this->declare_parameter("trust_ndt_threshold", 5);
  this->declare_parameter("jump_threshold_xy", 0.5);
  this->declare_parameter("jump_threshold_yaw", 0.3);
  this->declare_parameter("jump_threshold_rp", 0.1);
  this->declare_parameter("enable_quality_gate", true);
  this->declare_parameter("quality_sample_points", 1500);
  this->declare_parameter("quality_max_corr_dist", 1.0);
  this->declare_parameter("quality_min_valid_correspondences", 120);
  this->declare_parameter("quality_min_overlap_ratio", 0.35);
  this->declare_parameter("quality_max_median_residual", 0.35);
  this->declare_parameter("quality_max_p90_residual", 1.0);

  // Publish policy
  this->declare_parameter("publish_tf_only_when_trusted", false);
  this->declare_parameter("freeze_tf_when_not_trusted", false);

  // --- get parameters -------------------------------------------------------
  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("ndt_resolution", ndt_resolution_);
  this->get_parameter("ndt_step_size", ndt_step_size_);
  this->get_parameter("ndt_epsilon", ndt_epsilon_);
  this->get_parameter("ndt_max_iterations", ndt_max_iterations_);
  this->get_parameter("ndt_search_method", ndt_search_method_);
  this->get_parameter("fitness_score_threshold", fitness_score_threshold_);
  this->get_parameter("registration_period_ms", registration_period_ms_);

  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("min_source_points", min_source_points_);
  this->get_parameter("min_filtered_points", min_filtered_points_);
  this->get_parameter("max_scan_age_sec", max_scan_age_sec_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("enable_roll_pitch_fix", enable_roll_pitch_fix_);
  this->get_parameter("trust_ndt_threshold", trust_ndt_threshold_);
  this->get_parameter("jump_threshold_xy", jump_threshold_xy_);
  this->get_parameter("jump_threshold_yaw", jump_threshold_yaw_);
  this->get_parameter("jump_threshold_rp", jump_threshold_rp_);
  this->get_parameter("enable_quality_gate", enable_quality_gate_);
  this->get_parameter("quality_sample_points", quality_sample_points_);
  this->get_parameter("quality_max_corr_dist", quality_max_corr_dist_);
  this->get_parameter("quality_min_valid_correspondences", quality_min_valid_correspondences_);
  this->get_parameter("quality_min_overlap_ratio", quality_min_overlap_ratio_);
  this->get_parameter("quality_max_median_residual", quality_max_median_residual_);
  this->get_parameter("quality_max_p90_residual", quality_max_p90_residual_);

  this->get_parameter("publish_tf_only_when_trusted", publish_tf_only_when_trusted_);
  this->get_parameter("freeze_tf_when_not_trusted", freeze_tf_when_not_trusted_);

  if (registration_period_ms_ < 50) {
    RCLCPP_WARN(this->get_logger(),
      "registration_period_ms=%d is too small; clamping to 50 ms.", registration_period_ms_);
    registration_period_ms_ = 50;
  }
  if (quality_sample_points_ < 1) {
    RCLCPP_WARN(this->get_logger(),
      "quality_sample_points=%d is invalid; clamping to 1.", quality_sample_points_);
    quality_sample_points_ = 1;
  }
  if (quality_max_corr_dist_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(),
      "quality_max_corr_dist=%.3f is invalid; clamping to 1.0 m.", quality_max_corr_dist_);
    quality_max_corr_dist_ = 1.0;
  }

  // --- frame fallback logic -------------------------------------------------
  if (robot_base_frame_.empty()) {
    robot_base_frame_ = base_frame_;
    RCLCPP_WARN(this->get_logger(),
      "robot_base_frame not set, falling back to base_frame: '%s'", base_frame_.c_str());
  }
  if (base_frame_.empty()) {
    base_frame_ = robot_base_frame_;
    RCLCPP_WARN(this->get_logger(),
      "base_frame not set, falling back to robot_base_frame: '%s'", robot_base_frame_.c_str());
  }
  if (base_frame_.empty()) {
    RCLCPP_ERROR(this->get_logger(),
      "Both 'base_frame' and 'robot_base_frame' are empty! "
      "Please set at least one in your launch/yaml config. Node will not function.");
  }

  // --- init pose ------------------------------------------------------------
  if (init_pose_.size() >= 6) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      (Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX())).toRotationMatrix();
    previous_result_t_ = result_t_;
    last_trusted_result_t_ = result_t_;
    has_last_trusted_result_ = true;
  }

  // --- point clouds ---------------------------------------------------------
  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  // --- NDT OMP setup --------------------------------------------------------
  ndt_ = pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr(
    new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
  ndt_->setResolution(ndt_resolution_);
  ndt_->setStepSize(ndt_step_size_);
  ndt_->setTransformationEpsilon(ndt_epsilon_);
  ndt_->setMaximumIterations(ndt_max_iterations_);
  ndt_->setNumThreads(num_threads_);

  switch (ndt_search_method_) {
    case 0:
      ndt_->setNeighborhoodSearchMethod(pclomp::KDTREE);
      RCLCPP_INFO(this->get_logger(), "NDT search method: KDTREE");
      break;
    case 1:
      ndt_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      RCLCPP_INFO(this->get_logger(), "NDT search method: DIRECT7");
      break;
    case 2:
      ndt_->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      RCLCPP_INFO(this->get_logger(), "NDT search method: DIRECT1");
      break;
    case 3:
      ndt_->setNeighborhoodSearchMethod(pclomp::DIRECT26);
      RCLCPP_INFO(this->get_logger(), "NDT search method: DIRECT26");
      break;
    default:
      ndt_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      RCLCPP_WARN(this->get_logger(), "Unknown search method, defaulting to DIRECT7");
      break;
  }

  // --- TF -------------------------------------------------------------------
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // --- Load global map and set NDT target immediately -----------------------
  loadGlobalMap(prior_pcd_file_);
  setupGlobalMap();

  // --- Subscriptions --------------------------------------------------------
  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "registered_scan", 10,
    std::bind(&NdtOmpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&NdtOmpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  // --- Timers ---------------------------------------------------------------
  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(registration_period_ms_),
    std::bind(&NdtOmpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&NdtOmpRelocalizationNode::publishTransform, this));

  RCLCPP_INFO(this->get_logger(), "NDT OMP Relocalization node initialized.");
  RCLCPP_INFO(this->get_logger(), "  resolution: %.2f, step_size: %.2f, epsilon: %.4f",
    ndt_resolution_, ndt_step_size_, ndt_epsilon_);
  RCLCPP_INFO(this->get_logger(), "  max_iterations: %d, num_threads: %d",
    ndt_max_iterations_, num_threads_);
  RCLCPP_INFO(this->get_logger(),
    "  registration_period_ms: %d, source gates: raw>=%d, filtered>=%d, max_age=%.2fs",
    registration_period_ms_, min_source_points_, min_filtered_points_, max_scan_age_sec_);
  RCLCPP_INFO(this->get_logger(),
    "  quality gate: %s, sample=%d, corr<=%.2fm, min_corr=%d, overlap>=%.2f, "
    "median<=%.2f, p90<=%.2f",
    enable_quality_gate_ ? "enabled" : "disabled", quality_sample_points_,
    quality_max_corr_dist_, quality_min_valid_correspondences_, quality_min_overlap_ratio_,
    quality_max_median_residual_, quality_max_p90_residual_);
  RCLCPP_INFO(this->get_logger(), "  base_frame: '%s', robot_base_frame: '%s'",
    base_frame_.c_str(), robot_base_frame_.c_str());
}

// ---------------------------------------------------------------------------
// Load PCD from disk
// ---------------------------------------------------------------------------
void NdtOmpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points",
    global_map_->points.size());
}

// ---------------------------------------------------------------------------
// Down-sample and set NDT target — no TF needed (PCD is already in map frame)
// ---------------------------------------------------------------------------
bool NdtOmpRelocalizationNode::setupGlobalMap()
{
  if (global_map_->empty()) return false;

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setLeafSize(global_leaf_size_, global_leaf_size_, global_leaf_size_);
  voxel_filter.setInputCloud(global_map_);

  target_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  voxel_filter.filter(*target_cloud_);

  ndt_->setInputTarget(target_cloud_);
  target_kdtree_ = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
  target_kdtree_->setInputCloud(target_cloud_);

  map_ready_ = true;
  RCLCPP_INFO(this->get_logger(),
    "Global map loaded and NDT target set (%zu -> %zu points after downsampling).",
    global_map_->points.size(), target_cloud_->points.size());
  return true;
}

// ---------------------------------------------------------------------------
// Jump detection between two poses
// ---------------------------------------------------------------------------
bool NdtOmpRelocalizationNode::checkJump(
  const Eigen::Isometry3d & new_pose, const Eigen::Isometry3d & prev_pose) const
{
  const double xy_diff = std::hypot(
    new_pose.translation().x() - prev_pose.translation().x(),
    new_pose.translation().y() - prev_pose.translation().y());

  const double new_yaw = getYaw(new_pose.linear());
  const double old_yaw = getYaw(prev_pose.linear());
  const double yaw_diff = std::abs(normalizeAngle(new_yaw - old_yaw));

  // Roll/pitch via Eigen's eulerAngles (ZYX convention → index 1=pitch, 2=roll)
  const Eigen::Vector3d new_rpy = new_pose.linear().eulerAngles(2, 1, 0);  // yaw, pitch, roll
  const Eigen::Vector3d old_rpy = prev_pose.linear().eulerAngles(2, 1, 0);
  const double rp_diff = std::hypot(
    normalizeAngle(new_rpy[2] - old_rpy[2]),   // roll diff
    normalizeAngle(new_rpy[1] - old_rpy[1]));  // pitch diff

  return (xy_diff > jump_threshold_xy_ || yaw_diff > jump_threshold_yaw_ ||
          rp_diff > jump_threshold_rp_);
}

// ---------------------------------------------------------------------------
// Zero roll & pitch, keep yaw only
// ---------------------------------------------------------------------------
Eigen::Isometry3d NdtOmpRelocalizationNode::applyZeroRollPitch(
  const Eigen::Isometry3d & ndt_pose) const
{
  const double yaw = getYaw(ndt_pose.linear());

  Eigen::Isometry3d corrected = Eigen::Isometry3d::Identity();
  corrected.translation() = ndt_pose.translation();
  corrected.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  return corrected;
}

// ---------------------------------------------------------------------------
// Lightweight final quality gate for accepted NDT results
// ---------------------------------------------------------------------------
bool NdtOmpRelocalizationNode::evaluateAlignmentQuality(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
  const Eigen::Isometry3d & map_to_odom,
  QualityMetrics & metrics) const
{
  if (!target_kdtree_ || !target_cloud_ || target_cloud_->empty() || !source || source->empty()) {
    return false;
  }

  const int sample_limit = std::max(1, quality_sample_points_);
  const size_t stride =
    std::max<size_t>(1, source->points.size() / static_cast<size_t>(sample_limit));
  const double max_corr_sq = quality_max_corr_dist_ * quality_max_corr_dist_;

  std::vector<double> residuals;
  residuals.reserve(std::min(source->points.size(), static_cast<size_t>(sample_limit)));

  std::vector<int> indices(1);
  std::vector<float> sq_dists(1);

  for (size_t i = 0; i < source->points.size() && metrics.sampled_points < sample_limit;
    i += stride)
  {
    const auto & point = source->points[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    const Eigen::Vector3d transformed =
      map_to_odom * Eigen::Vector3d(point.x, point.y, point.z);
    pcl::PointXYZ query;
    query.x = static_cast<float>(transformed.x());
    query.y = static_cast<float>(transformed.y());
    query.z = static_cast<float>(transformed.z());

    ++metrics.sampled_points;
    if (target_kdtree_->nearestKSearch(query, 1, indices, sq_dists) > 0 &&
      static_cast<double>(sq_dists[0]) <= max_corr_sq)
    {
      ++metrics.valid_correspondences;
      residuals.push_back(std::sqrt(static_cast<double>(sq_dists[0])));
    }
  }

  if (metrics.sampled_points <= 0) {
    return false;
  }

  metrics.overlap_ratio =
    static_cast<double>(metrics.valid_correspondences) / metrics.sampled_points;

  if (residuals.empty()) {
    metrics.median_residual = std::numeric_limits<double>::infinity();
    metrics.p90_residual = std::numeric_limits<double>::infinity();
  } else {
    std::sort(residuals.begin(), residuals.end());
    metrics.median_residual = residuals[residuals.size() / 2];
    const size_t p90_index =
      std::min(residuals.size() - 1, (residuals.size() * 9) / 10);
    metrics.p90_residual = residuals[p90_index];
  }

  return metrics.valid_correspondences >= quality_min_valid_correspondences_ &&
         metrics.overlap_ratio >= quality_min_overlap_ratio_ &&
         metrics.median_residual <= quality_max_median_residual_ &&
         metrics.p90_residual <= quality_max_p90_residual_;
}

// ---------------------------------------------------------------------------
// Incoming scan accumulation
// ---------------------------------------------------------------------------
void NdtOmpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);

  std::lock_guard<std::mutex> lock(cloud_mutex_);
  latest_cloud_stamp_ = rclcpp::Time(msg->header.stamp);
  has_latest_cloud_stamp_ =
    msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0;

  *accumulated_cloud_ += *scan;

  // Prevent unbounded memory growth — keep only the most recent points
  if (accumulated_cloud_->points.size() > kMaxAccumulatedPoints) {
    const size_t excess = accumulated_cloud_->points.size() - kMaxAccumulatedPoints;
    accumulated_cloud_->points.erase(
      accumulated_cloud_->points.begin(),
      accumulated_cloud_->points.begin() + static_cast<long>(excess));
    accumulated_cloud_->width = static_cast<uint32_t>(accumulated_cloud_->points.size());
    accumulated_cloud_->height = 1;
  }
}

// ---------------------------------------------------------------------------
// NDT registration (called at 2 Hz)
// ---------------------------------------------------------------------------
void NdtOmpRelocalizationNode::performRegistration()
{
  if (!map_ready_) {
    if (!setupGlobalMap()) {
      return;
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_process(new pcl::PointCloud<pcl::PointXYZ>());
  rclcpp::Time cloud_stamp(0, 0, RCL_ROS_TIME);
  bool has_cloud_stamp = false;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (accumulated_cloud_->empty()) {
      return;
    }
    cloud_stamp = latest_cloud_stamp_;
    has_cloud_stamp = has_latest_cloud_stamp_;
    cloud_to_process.swap(accumulated_cloud_);
    accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    latest_cloud_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    has_latest_cloud_stamp_ = false;
  }

  if (cloud_to_process->points.size() < static_cast<size_t>(std::max(0, min_source_points_))) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "Skip NDT: source cloud too small (%zu < %d).",
      cloud_to_process->points.size(), min_source_points_);
    std::lock_guard<std::mutex> lock(pose_mutex_);
    converged_ = false;
    trust_ndt_ = false;
    if (enable_roll_pitch_fix_) {
      consecutive_converged_count_ = 0;
    }
    return;
  }

  if (has_cloud_stamp && max_scan_age_sec_ > 0.0) {
    const double scan_age = (this->now() - cloud_stamp).seconds();
    if (scan_age > max_scan_age_sec_ || scan_age < -0.1) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "Skip NDT: source cloud stamp age %.3fs outside valid range [-0.1, %.3f].",
        scan_age, max_scan_age_sec_);
      std::lock_guard<std::mutex> lock(pose_mutex_);
      converged_ = false;
      trust_ndt_ = false;
      if (enable_roll_pitch_fix_) {
        consecutive_converged_count_ = 0;
      }
      return;
    }
  }

  // Down-sample the source cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setLeafSize(registered_leaf_size_, registered_leaf_size_, registered_leaf_size_);
  voxel_filter.setInputCloud(cloud_to_process);
  voxel_filter.filter(*filtered_cloud);

  if (filtered_cloud->points.size() < static_cast<size_t>(std::max(0, min_filtered_points_))) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "Skip NDT: filtered cloud too small (%zu < %d).",
      filtered_cloud->points.size(), min_filtered_points_);
    std::lock_guard<std::mutex> lock(pose_mutex_);
    converged_ = false;
    trust_ndt_ = false;
    if (enable_roll_pitch_fix_) {
      consecutive_converged_count_ = 0;
    }
    return;
  }

  ndt_->setInputSource(filtered_cloud);

  Eigen::Matrix4f init_guess;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    init_guess = previous_result_t_.matrix().cast<float>();
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
  ndt_->align(*aligned, init_guess);

  const bool ndt_converged = ndt_->hasConverged();
  const double fitness_score = ndt_->getFitnessScore();

  if (ndt_converged && fitness_score < fitness_score_threshold_) {
    Eigen::Isometry3d new_result = Eigen::Isometry3d::Identity();
    new_result.matrix() = ndt_->getFinalTransformation().cast<double>();

    QualityMetrics quality_metrics;
    if (enable_quality_gate_ &&
      !evaluateAlignmentQuality(filtered_cloud, new_result, quality_metrics))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "Reject NDT by quality gate: sampled=%d, valid=%d, overlap=%.3f, "
        "median=%.3f, p90=%.3f.",
        quality_metrics.sampled_points, quality_metrics.valid_correspondences,
        quality_metrics.overlap_ratio, quality_metrics.median_residual,
        quality_metrics.p90_residual);
      std::lock_guard<std::mutex> lock(pose_mutex_);
      converged_ = false;
      trust_ndt_ = false;
      if (enable_roll_pitch_fix_) {
        consecutive_converged_count_ = 0;
      }
      return;
    }

    std::lock_guard<std::mutex> lock(pose_mutex_);

    if (enable_roll_pitch_fix_ && checkJump(new_result, previous_result_t_)) {
      RCLCPP_WARN(this->get_logger(),
        "NDT result jumped too much, keeping previous pose (fitness: %.4f)", fitness_score);
      converged_ = false;
      consecutive_converged_count_ = 0;
      trust_ndt_ = false;
    } else {
      result_t_ = previous_result_t_ = new_result;
      converged_ = true;

      if (enable_roll_pitch_fix_) {
        consecutive_converged_count_++;
        if (consecutive_converged_count_ >= trust_ndt_threshold_) {
          trust_ndt_ = true;
        }
      } else {
        trust_ndt_ = true;
      }

      if (trust_ndt_) {
        last_trusted_result_t_ = result_t_;
        has_last_trusted_result_ = true;
      }

      RCLCPP_DEBUG(this->get_logger(),
        "NDT converged. fitness: %.4f, iterations: %d, trusted: %s, "
        "overlap: %.3f, median: %.3f, p90: %.3f",
        fitness_score, ndt_->getFinalNumIteration(), trust_ndt_ ? "yes" : "no",
        quality_metrics.overlap_ratio, quality_metrics.median_residual,
        quality_metrics.p90_residual);
    }
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "NDT did not converge or fitness too high "
      "(converged: %d, fitness: %.4f, threshold: %.4f).",
      ndt_converged, fitness_score, fitness_score_threshold_);
    std::lock_guard<std::mutex> lock(pose_mutex_);
    converged_ = false;
    trust_ndt_ = false;
    if (enable_roll_pitch_fix_) {
      consecutive_converged_count_ = 0;
    }
  }
}

// ---------------------------------------------------------------------------
// Broadcast map→odom TF
// ---------------------------------------------------------------------------
void NdtOmpRelocalizationNode::broadcastTransform(const Eigen::Isometry3d & pose)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.stamp = this->now();
  ts.header.frame_id = map_frame_;
  ts.child_frame_id = odom_frame_;

  const Eigen::Vector3d t = pose.translation();
  const Eigen::Quaterniond r(pose.rotation());

  ts.transform.translation.x = t.x();
  ts.transform.translation.y = t.y();
  ts.transform.translation.z = t.z();
  ts.transform.rotation.x = r.x();
  ts.transform.rotation.y = r.y();
  ts.transform.rotation.z = r.z();
  ts.transform.rotation.w = r.w();

  tf_broadcaster_->sendTransform(ts);
}

// ---------------------------------------------------------------------------
// Periodic TF publisher (20 Hz)
// ---------------------------------------------------------------------------
void NdtOmpRelocalizationNode::publishTransform()
{
  Eigen::Isometry3d final_pose;
  bool local_trusted = false;

  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    local_trusted = trust_ndt_ && converged_;

    if (local_trusted) {
      final_pose = result_t_;
    } else if (publish_tf_only_when_trusted_) {
      // Only publish when trusted — but we still need the map frame to exist,
      // so fall through to has_last_trusted_result_ check.
      if (has_last_trusted_result_) {
        final_pose = last_trusted_result_t_;
      } else {
        // No trusted result yet — still broadcast init_pose so map frame exists.
        final_pose = result_t_;
      }
    } else if (freeze_tf_when_not_trusted_ && has_last_trusted_result_) {
      final_pose = last_trusted_result_t_;
    } else {
      // Fallback: always publish something so the map frame never disappears.
      final_pose = result_t_;
    }
  }

  if (enable_roll_pitch_fix_ && !local_trusted) {
    final_pose = applyZeroRollPitch(final_pose);
  }

  broadcastTransform(final_pose);
}

// ---------------------------------------------------------------------------
// Initial pose from RViz / external
// ---------------------------------------------------------------------------
void NdtOmpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]",
    msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

  if (robot_base_frame_.empty()) {
    RCLCPP_ERROR(this->get_logger(),
      "Cannot apply initial pose: robot_base_frame (and base_frame) are both empty. "
      "Set at least one in your launch/yaml config.");
    return;
  }

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(
    msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();

  // Try to compute map->odom = map->base * base->odom.
  // If the TF odom->base is not yet available, fall back to treating the given
  // pose directly as map->odom so that initial pose always works.
  Eigen::Isometry3d map_to_odom = map_to_robot_base;  // fallback: assume odom ≈ base

  try {
    auto transform = tf_buffer_->lookupTransform(
      odom_frame_, robot_base_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(0.1));
    Eigen::Isometry3d odom_to_robot_base = tf2::transformToEigen(transform.transform);

    // T_(map <- odom) = T_(map <- base) * T_(base <- odom)
    map_to_odom = map_to_robot_base * odom_to_robot_base.inverse();

    RCLCPP_INFO(this->get_logger(),
      "Initial pose: odom->%s TF found, computing accurate map->odom.",
      robot_base_frame_.c_str());
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(),
      "Could not get odom->%s TF (%s). "
      "Treating the given pose directly as map->odom (fallback).",
      robot_base_frame_.c_str(), ex.what());
  }

  {
    std::lock_guard<std::mutex> lock(pose_mutex_);

    previous_result_t_ = result_t_ = map_to_odom;

    last_trusted_result_t_ = map_to_odom;
    has_last_trusted_result_ = true;

    // Treat a manual pose estimate as a full trusted reset
    converged_ = true;
    trust_ndt_ = true;
    consecutive_converged_count_ = trust_ndt_threshold_;
  }

  // Broadcast immediately
  broadcastTransform(map_to_odom);

  RCLCPP_INFO(this->get_logger(),
    "Initial pose applied: map->odom set to [%.2f, %.2f, %.2f]. TF broadcast immediately.",
    map_to_odom.translation().x(),
    map_to_odom.translation().y(),
    map_to_odom.translation().z());
}

}  // namespace ndt_omp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ndt_omp_relocalization::NdtOmpRelocalizationNode)
