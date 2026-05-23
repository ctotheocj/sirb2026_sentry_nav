#include "sirb_smoother/safe_geometric_smoother.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <mutex>
#include <sstream>

#include "angles/angles.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sirb_smoother
{

namespace
{

class AtomicFlagGuard
{
public:
  explicit AtomicFlagGuard(std::atomic_bool & flag)
  : flag_(flag)
  {}

  ~AtomicFlagGuard()
  {
    flag_.store(false);
  }

  AtomicFlagGuard(const AtomicFlagGuard &) = delete;
  AtomicFlagGuard & operator=(const AtomicFlagGuard &) = delete;

private:
  std::atomic_bool & flag_;
};

uint64_t goalIdFromWaypoints(const std::vector<Eigen::Vector3d> & waypoints)
{
  if (waypoints.empty()) {
    return 0;
  }
  const auto & goal = waypoints.back();
  const auto qx = static_cast<int64_t>(std::llround(goal.x() * 1000.0));
  const auto qy = static_cast<int64_t>(std::llround(goal.y() * 1000.0));
  uint64_t seed = 1469598103934665603ULL;
  auto mix = [&](int64_t v) {
      seed ^= static_cast<uint64_t>(v);
      seed *= 1099511628211ULL;
    };
  mix(qx);
  mix(qy);
  return seed;
}

struct GeometryMetrics
{
  double length{0.0};
  double max_turn{0.0};
  double total_turn{0.0};
};

double poseDistance(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b)
{
  return std::hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y);
}

GeometryMetrics pathGeometryMetrics(const nav_msgs::msg::Path & path)
{
  GeometryMetrics m;
  if (path.poses.size() < 2) {return m;}
  for (size_t i = 1; i < path.poses.size(); ++i) {
    m.length += poseDistance(path.poses[i - 1], path.poses[i]);
    if (i + 1 >= path.poses.size()) {continue;}
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    const auto & c = path.poses[i + 1].pose.position;
    const double dx1 = b.x - a.x;
    const double dy1 = b.y - a.y;
    const double dx2 = c.x - b.x;
    const double dy2 = c.y - b.y;
    if (std::hypot(dx1, dy1) < 1.0e-4 || std::hypot(dx2, dy2) < 1.0e-4) {continue;}
    double turn = std::abs(std::atan2(dy2, dx2) - std::atan2(dy1, dx1));
    if (turn > M_PI) {turn = 2.0 * M_PI - turn;}
    m.max_turn = std::max(m.max_turn, turn);
    m.total_turn += turn;
  }
  return m;
}

double cross2d(double ax, double ay, double bx, double by)
{
  return ax * by - ay * bx;
}

bool segmentsIntersect(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b,
  const geometry_msgs::msg::Point & c,
  const geometry_msgs::msg::Point & d)
{
  const double abx = b.x - a.x;
  const double aby = b.y - a.y;
  const double cdx = d.x - c.x;
  const double cdy = d.y - c.y;
  const double denom = cross2d(abx, aby, cdx, cdy);
  if (std::abs(denom) < 1.0e-9) {return false;}
  const double acx = c.x - a.x;
  const double acy = c.y - a.y;
  const double t = cross2d(acx, acy, cdx, cdy) / denom;
  const double u = cross2d(acx, acy, abx, aby) / denom;
  constexpr double eps = 1.0e-3;
  return t > eps && t < 1.0 - eps && u > eps && u < 1.0 - eps;
}

bool hasSelfIntersection(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 5) {return false;}
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    for (size_t j = i + 2; j < path.poses.size(); ++j) {
      if (i == 1 && j + 1 == path.poses.size()) {continue;}
      const auto & c = path.poses[j - 1].pose.position;
      const auto & d = path.poses[j].pose.position;
      if (segmentsIntersect(a, b, c, d)) {return true;}
    }
  }
  return false;
}

bool hasHairpin(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 3) {return false;}
  for (size_t i = 1; i + 1 < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    const auto & c = path.poses[i + 1].pose.position;
    const double dx1 = b.x - a.x;
    const double dy1 = b.y - a.y;
    const double dx2 = c.x - b.x;
    const double dy2 = c.y - b.y;
    const double l1 = std::hypot(dx1, dy1);
    const double l2 = std::hypot(dx2, dy2);
    if (l1 < 0.05 || l2 < 0.05) {continue;}
    const double dot = (dx1 * dx2 + dy1 * dy2) / (l1 * l2);
    if (dot < -0.65 && std::min(l1, l2) < 0.60) {return true;}
  }
  return false;
}

bool smoothedGeometryReasonable(
  const nav_msgs::msg::Path & input_path,
  const nav_msgs::msg::Path & candidate,
  std::string & reason)
{
  const auto in = pathGeometryMetrics(input_path);
  const auto out = pathGeometryMetrics(candidate);
  if (input_path.poses.size() < 2 || candidate.poses.size() < 2 || in.length < 0.10) {
    reason = "too few poses";
    return false;
  }
  if (out.length > std::max(in.length * 1.45, in.length + 0.80)) {
    reason = "length inflated";
    return false;
  }
  if (in.total_turn < 0.35 && (out.max_turn > 0.60 || out.total_turn > 1.20)) {
    reason = "straight path kinked";
    return false;
  }
  if (hasHairpin(candidate)) {
    reason = "hairpin";
    return false;
  }
  if (hasSelfIntersection(candidate)) {
    reason = "self intersection";
    return false;
  }
  return true;
}

std::string productTypeToString(int value)
{
  switch (value) {
    case 1:
      return "optimized_minco";
    case 2:
      return "cached_minco";
    case 3:
      return "timeout_reference_fallback";
    case 4:
      return "collision_fallback";
    case 5:
      return "geometry_fallback";
    case 6:
      return "path_fallback";
    default:
      return "none";
  }
}

}  // namespace

void SafeGeometricSmoother::setEsdf(plan_env::GridMap::Ptr esdf)
{
  esdf_map_ = esdf;
  minco_optimizer_.setEsdf(esdf);
}

void SafeGeometricSmoother::configure(  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_sub_ = costmap_sub;
  footprint_sub_ = footprint_sub;
  auto node = parent.lock();
  logger_ = node->get_logger();

  auto decl = [&](const std::string & key, auto val) {
      nav2_util::declare_parameter_if_not_declared(node, name_ + "." + key, rclcpp::ParameterValue(val));
    };

  decl("enabled", true);
  decl("fallback_to_input_path", false);
  decl("check_input_path", false);
  decl("shortcut", true);
  decl("resample", true);
  decl("collision_cost_threshold", 253);
  decl("collision_check_step", 0.05);
  decl("resample_resolution", 0.20);
  decl("max_shortcut_dist", 2.0);
  decl("max_shortcut_skip", 30);
  decl("min_clearance_for_removal", 0.25);
  decl("use_minco", false);
  decl("minco_v_ref", 1.5);
  decl("minco_min_segment_time", 0.15);
  decl("minco_max_segment_time", 1.0);
  decl("minco_corner_time_weight", 0.20);
  decl("minco_sample_resolution", 0.10);
  decl("minco_max_pieces", 80);
  decl("minco_use_lbfgs", true);
  decl("minco_guide_fillet_enabled", true);
  decl("minco_guide_fillet_radius", 0.60);
  decl("minco_guide_fillet_min_angle", 0.25);
  decl("minco_obstacle_cost_threshold", 128.0);
  decl("minco_obstacle_finite_diff_step", 0.05);
  decl("minco_obstacle_sample_dt", 0.05);
  decl("minco_w_obstacle_traj", 1.0);
  decl("minco_v_max", 2.0);
  decl("minco_a_max", 3.0);
  decl("minco_w_velocity", 1.0);
  decl("minco_w_acceleration", 0.5);
  decl("minco_dynamics_sample_dt", 0.05);
  decl("minco_dynamic_realloc_max_segment_scale", 2.5);
  decl("minco_pre_max_iterations", 30);
  decl("minco_pre_w_energy", 0.5);
  decl("minco_pre_w_reference", 20.0);
  decl("minco_pre_w_obstacle", 1.0);
  decl("minco_fine_max_iterations", 60);
  decl("minco_fine_w_energy", 2.0);
  decl("minco_fine_w_reference", 5.0);
  decl("minco_fine_w_obstacle", 3.0);
  decl("minco_wei_time", 0.1);
  // ESDF
  decl("minco_use_esdf", false);
  decl("minco_esdf_safe_distance", 0.3);
  decl("minco_esdf_influence_distance", 0.8);
  decl("minco_esdf_gradient_step", 0.05);
  decl("minco_esdf_valley_threshold", 0.5);
  decl("minco_esdf_query_z", 0.3);
  // Phase 0
  decl("minco_phase0_enabled", true);
  decl("minco_phase0_max_iterations", 15);
  decl("minco_phase0_w_energy", 1.0);
  decl("minco_phase0_w_reference", 10.0);
  decl("minco_phase0_w_obstacle", 0.1);
  // Time ratio
  decl("minco_w_time_ratio", 5.0);
  decl("minco_time_ratio_upper", 1.3);
  decl("minco_time_ratio_lower", 0.7);
  // Trapezoidal time
  decl("minco_use_trapezoidal_time", true);
  decl("minco_trapezoidal_k_angle", 0.3);
  // Stitching
  decl("enable_trajectory_stitching", true);
  decl("retain_duration", 0.2);
  decl("stitch_sample_dt", 0.1);
  decl("stitch_max_distance", 0.5);
  decl("stitch_cache_timeout", 4.0);
  decl("reuse_cached_trajectory_on_minco_failure", false);
  decl("allow_reference_fallback_on_bad_geometry", false);
  // Initial state for nonzero-velocity replanning.
  decl("use_odom_initial_state", true);
  decl("odom_topic", std::string("odometry"));
  decl("odom_max_age_sec", 0.25);
  decl("odom_twist_in_child_frame", true);
  decl("initial_velocity_max", 0.0);
  // MPC output
  decl("publish_trajectory_for_mpc", false);
  // Collision
  decl("use_footprint_collision_check", true);
  decl("footprint_collision_cost_threshold", 253);
  decl("allow_unknown", true);
  // Dynamic obstacles
  decl("dynamic_obstacle_topic", std::string("dynamic_obstacles"));
  decl("dynamic_obstacle_enabled", false);
  // Debug
  decl("debug_publish", true);

  auto get = [&](const std::string & key, auto & var) {
      node->get_parameter(name_ + "." + key, var);
    };

  get("enabled", enabled_);
  get("fallback_to_input_path", fallback_to_input_path_);
  get("check_input_path", check_input_path_);
  get("shortcut", do_shortcut_);
  get("resample", do_resample_);
  int cct = static_cast<int>(collision_cost_threshold_);
  get("collision_cost_threshold", cct);
  collision_cost_threshold_ = static_cast<unsigned char>(std::clamp(cct, 0, 255));
  get("collision_check_step", collision_check_step_);
  get("resample_resolution", resample_resolution_);
  get("max_shortcut_dist", max_shortcut_dist_);
  get("max_shortcut_skip", max_shortcut_skip_);
  get("min_clearance_for_removal", min_clearance_for_removal_);
  get("use_minco", use_minco_);
  get("minco_v_ref", minco_options_.v_ref);
  get("minco_min_segment_time", minco_options_.min_segment_time);
  get("minco_max_segment_time", minco_options_.max_segment_time);
  get("minco_corner_time_weight", minco_options_.corner_time_weight);
  get("minco_sample_resolution", minco_options_.sample_resolution);
  get("minco_max_pieces", minco_options_.max_pieces);
  get("minco_use_lbfgs", minco_options_.use_lbfgs);
  get("minco_guide_fillet_enabled", minco_options_.guide_fillet_enabled);
  get("minco_guide_fillet_radius", minco_options_.guide_fillet_radius);
  get("minco_guide_fillet_min_angle", minco_options_.guide_fillet_min_angle);
  get("minco_obstacle_cost_threshold", minco_options_.obstacle_cost_threshold);
  get("minco_obstacle_finite_diff_step", minco_options_.obstacle_finite_diff_step);
  get("minco_obstacle_sample_dt", minco_options_.obstacle_sample_dt);
  get("minco_w_obstacle_traj", minco_options_.w_obstacle_traj);
  get("minco_v_max", minco_options_.v_max);
  get("minco_a_max", minco_options_.a_max);
  get("minco_w_velocity", minco_options_.w_velocity);
  get("minco_w_acceleration", minco_options_.w_acceleration);
  get("minco_dynamics_sample_dt", minco_options_.dynamics_sample_dt);
  get("minco_dynamic_realloc_max_segment_scale",
    minco_options_.dynamic_realloc_max_segment_scale);
  get("minco_pre_max_iterations", minco_options_.pre_max_iterations);
  get("minco_pre_w_energy", minco_options_.pre_w_energy);
  get("minco_pre_w_reference", minco_options_.pre_w_reference);
  get("minco_pre_w_obstacle", minco_options_.pre_w_obstacle);
  get("minco_fine_max_iterations", minco_options_.fine_max_iterations);
  get("minco_fine_w_energy", minco_options_.fine_w_energy);
  get("minco_fine_w_reference", minco_options_.fine_w_reference);
  get("minco_fine_w_obstacle", minco_options_.fine_w_obstacle);
  get("minco_wei_time", minco_options_.wei_time);
  get("minco_use_esdf", minco_options_.use_esdf);
  get("minco_esdf_safe_distance", minco_options_.esdf_safe_distance);
  get("minco_esdf_influence_distance", minco_options_.esdf_influence_distance);
  get("minco_esdf_gradient_step", minco_options_.esdf_gradient_step);
  get("minco_esdf_valley_threshold", minco_options_.esdf_valley_threshold);
  get("minco_esdf_query_z", minco_options_.esdf_query_z);
  get("minco_phase0_enabled", minco_options_.phase0_enabled);
  get("minco_phase0_max_iterations", minco_options_.phase0_max_iterations);
  get("minco_phase0_w_energy", minco_options_.phase0_w_energy);
  get("minco_phase0_w_reference", minco_options_.phase0_w_reference);
  get("minco_phase0_w_obstacle", minco_options_.phase0_w_obstacle);
  get("minco_w_time_ratio", minco_options_.w_time_ratio);
  get("minco_time_ratio_upper", minco_options_.time_ratio_upper);
  get("minco_time_ratio_lower", minco_options_.time_ratio_lower);
  get("minco_use_trapezoidal_time", minco_options_.use_trapezoidal_time);
  get("minco_trapezoidal_k_angle", minco_options_.trapezoidal_k_angle);
  get("enable_trajectory_stitching", enable_stitching_);
  get("retain_duration", retain_duration_);
  get("stitch_sample_dt", stitch_sample_dt_);
  get("stitch_max_distance", stitch_max_distance_);
  get("stitch_cache_timeout", stitch_cache_timeout_);
  get("reuse_cached_trajectory_on_minco_failure", reuse_cached_trajectory_on_minco_failure_);
  get("allow_reference_fallback_on_bad_geometry", allow_reference_fallback_on_bad_geometry_);
  get("use_odom_initial_state", use_odom_initial_state_);
  get("odom_topic", odom_topic_);
  get("odom_max_age_sec", odom_max_age_sec_);
  get("odom_twist_in_child_frame", odom_twist_in_child_frame_);
  get("initial_velocity_max", initial_velocity_max_);
  get("publish_trajectory_for_mpc", publish_trajectory_for_mpc_);
  get("use_footprint_collision_check", use_footprint_collision_check_);
  int fp_ct = static_cast<int>(footprint_collision_cost_threshold_);
  get("footprint_collision_cost_threshold", fp_ct);
  footprint_collision_cost_threshold_ = static_cast<unsigned char>(std::clamp(fp_ct, 0, 255));
  get("allow_unknown", allow_unknown_);
  get("dynamic_obstacle_topic", dynamic_obstacle_topic_);
  get("dynamic_obstacle_enabled", minco_options_.dynamic_obstacle_enabled);
  get("debug_publish", debug_publish_);

  minco_options_.enabled = use_minco_;
  minco_options_.allow_unknown = allow_unknown_;
  min_clearance_for_removal_ = std::max(0.0, min_clearance_for_removal_);
  odom_max_age_sec_ = std::max(0.0, odom_max_age_sec_);
  initial_velocity_max_ = std::max(0.0, initial_velocity_max_);
  minco_optimizer_.setOptions(minco_options_);

  if (minco_options_.v_ref > minco_options_.v_max) {
    RCLCPP_WARN(
      logger_,
      "Motion profile mismatch: minco_v_ref %.2f > minco_v_max %.2f",
      minco_options_.v_ref, minco_options_.v_max);
  }
  if (minco_options_.max_segment_time < minco_options_.min_segment_time) {
    RCLCPP_WARN(
      logger_,
      "Motion profile mismatch: minco_max_segment_time %.2f < minco_min_segment_time %.2f",
      minco_options_.max_segment_time, minco_options_.min_segment_time);
  }

  if (minco_options_.dynamic_obstacle_enabled) {
    dynamic_obs_sub_ =
      node->create_subscription<sentry_nav_interfaces::msg::TrackedObstacleArray>(
        dynamic_obstacle_topic_, rclcpp::SensorDataQoS(),
        [this](const sentry_nav_interfaces::msg::TrackedObstacleArray::SharedPtr msg) {
          std::vector<DynamicObstacle> obs;
          obs.reserve(msg->obstacles.size());
          for (const auto & o : msg->obstacles) {
            DynamicObstacle d;
            d.p0 = Eigen::Vector2d(o.x, o.y);
            d.v = Eigen::Vector2d(o.vx, o.vy);
            d.radius = o.radius;
            d.prediction_dt = static_cast<double>(o.prediction_dt);
            d.predicted_positions.reserve(o.predicted_positions.size());
            for (const auto & pp : o.predicted_positions) {
              d.predicted_positions.emplace_back(pp.x, pp.y);
            }
            obs.push_back(std::move(d));
          }
          std::lock_guard<std::mutex> lk(dynamic_obs_mutex_);
          dynamic_obstacles_ = std::move(obs);
        });
  }

  if (use_odom_initial_state_) {
    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      std::bind(&SafeGeometricSmoother::odomCallback, this, std::placeholders::_1));
  }

  if (debug_publish_) {
    input_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(name_ + "/input_path", 1);
    output_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(name_ + "/output_path", 1);
    minco_waypoints_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      name_ + "/minco_waypoints", 1);
    rejected_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(name_ + "/rejected_path", 1);
    collision_points_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      name_ + "/collision_points", 1);
    metrics_pub_ = node->create_publisher<std_msgs::msg::String>(name_ + "/metrics", 1);
    dynamic_obs_marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      name_ + "/dynamic_obstacles", 1);
  }
  if (publish_trajectory_for_mpc_) {
    mpc_traj_pub_ = node->create_publisher<sentry_nav_interfaces::msg::MincoTrajectory>(
      name_ + "/trajectory_for_mpc", rclcpp::QoS(1));
  }

  // Store node clock before creating the action server; goal callbacks use throttled logs.
  clock_ = node->get_clock();

  generate_action_server_ = rclcpp_action::create_server<GenerateMincoCandidate>(
    node,
    name_ + "/generate_minco_candidate",
    std::bind(
      &SafeGeometricSmoother::handleGenerateGoal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &SafeGeometricSmoother::handleGenerateCancel, this,
      std::placeholders::_1),
    std::bind(
      &SafeGeometricSmoother::handleGenerateAccepted, this,
      std::placeholders::_1));

  acquireEsdfMap("configure", true);

  RCLCPP_INFO(
    logger_,
    "Configured %s (use_minco=%d use_esdf=%d phase0=%d stitching=%d cache_reuse=%d mpc_pub=%d)",
    name_.c_str(), use_minco_, minco_options_.use_esdf,
    minco_options_.phase0_enabled, enable_stitching_,
    reuse_cached_trajectory_on_minco_failure_, publish_trajectory_for_mpc_);
  RCLCPP_INFO(
    logger_,
    "SafeGeometricSmoother fallback policy: reference_on_bad_geometry=%d",
    allow_reference_fallback_on_bad_geometry_);
}

void SafeGeometricSmoother::cleanup()
{
  generate_action_server_.reset();
  odom_sub_.reset();
  costmap_sub_.reset();
  footprint_sub_.reset();
  tf_.reset();
}

void SafeGeometricSmoother::activate()
{
  acquireEsdfMap("activate", true);
}
void SafeGeometricSmoother::deactivate() {}

bool SafeGeometricSmoother::acquireEsdfMap(const char * reason, bool warn_on_failure)
{
  if (!minco_options_.use_esdf) {
    return false;
  }
  if (esdf_map_) {
    return true;
  }

  auto esdf = plan_env::GridMapRegistry::get();
  if (esdf) {
    esdf_map_ = esdf;
    minco_optimizer_.setEsdf(esdf);
    RCLCPP_INFO(logger_, "%s: ESDF acquired from GridMapRegistry (%s)", name_.c_str(), reason);
    return true;
  }

  if (warn_on_failure) {
    RCLCPP_WARN(
      logger_,
    "%s: use_esdf=true but GridMapRegistry is empty during %s. "
      "Ensure plan_env::GridMapComponent is loaded in the same composable container.",
      name_.c_str(), reason);
  }
  return false;
}

void SafeGeometricSmoother::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(odom_mutex_);
  latest_odom_ = *msg;
  has_odom_ = true;
}

SafeGeometricSmoother::InitialState SafeGeometricSmoother::getInitialStateInFrame(
  const std_msgs::msg::Header & target_header) const
{
  InitialState state;
  if (!use_odom_initial_state_) {
    state.reason = "disabled";
    return state;
  }

  nav_msgs::msg::Odometry odom;
  {
    std::lock_guard<std::mutex> lk(odom_mutex_);
    if (!has_odom_) {
      state.reason = "no odom";
      return state;
    }
    odom = latest_odom_;
  }

  const rclcpp::Time now = clock_ ? clock_->now() : rclcpp::Clock().now();
  const rclcpp::Time stamp(odom.header.stamp);
  try {
    state.age = (now - stamp).seconds();
  } catch (const std::runtime_error &) {
    state.reason = "time source mismatch";
    return state;
  }
  if (state.age > odom_max_age_sec_) {
    std::ostringstream out;
    out << "stale odom age=" << state.age;
    state.reason = out.str();
    return state;
  }

  std::string odom_frame = odom.header.frame_id;
  std::string twist_frame = odom_twist_in_child_frame_ && !odom.child_frame_id.empty() ?
    odom.child_frame_id : odom_frame;
  std::string target_frame = target_header.frame_id;
  if (target_frame.empty()) {
    target_frame = odom_frame;
  }
  if (odom_frame.empty() || twist_frame.empty() || target_frame.empty()) {
    state.reason = "missing frame";
    return state;
  }

  tf2::Vector3 v(
    odom.twist.twist.linear.x,
    odom.twist.twist.linear.y,
    odom.twist.twist.linear.z);

  try {
    if (twist_frame != odom_frame) {
      const auto child_to_odom = tf_->lookupTransform(
        odom_frame, twist_frame, tf2::TimePointZero,
        tf2::durationFromSec(0.02));
      const auto & q_msg = child_to_odom.transform.rotation;
      const tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
      v = tf2::Matrix3x3(q) * v;
    }
    if (target_frame != odom_frame) {
      const auto odom_to_target = tf_->lookupTransform(
        target_frame, odom_frame, tf2::TimePointZero,
        tf2::durationFromSec(0.02));
      const auto & q_msg = odom_to_target.transform.rotation;
      const tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
      v = tf2::Matrix3x3(q) * v;
    }
  } catch (const tf2::TransformException & ex) {
    state.reason = ex.what();
    return state;
  }

  state.velocity = Eigen::Vector3d(v.x(), v.y(), v.z());
  state.acceleration = Eigen::Vector3d::Zero();
  state.speed = state.velocity.head<2>().norm();
  if (initial_velocity_max_ > 1.0e-3 && state.speed > initial_velocity_max_) {
    state.velocity.head<2>() *= initial_velocity_max_ / std::max(state.speed, 1.0e-6);
    state.speed = initial_velocity_max_;
  }
  state.valid = std::isfinite(state.velocity.x()) && std::isfinite(state.velocity.y()) &&
    std::isfinite(state.velocity.z());
  state.reason = state.valid ? "ok" : "nonfinite velocity";
  return state;
}

bool SafeGeometricSmoother::buildSafeReferenceFallback(
  const nav_msgs::msg::Path & reference_path,
  const nav_msgs::msg::Path & geometry_baseline,
  const nav2_costmap_2d::Costmap2D & costmap,
  const Footprint & footprint,
  MincoOptimizer::Result & result,
  nav_msgs::msg::Path & candidate,
  std::string & diagnostic,
  const Eigen::Vector3d * initial_velocity,
  const Eigen::Vector3d * initial_acceleration) const
{
  diagnostic.clear();
  candidate = reference_path;
  if (!minco_optimizer_.buildReferenceTrajectory(
      reference_path, result, candidate, &diagnostic,
      initial_velocity, initial_acceleration))
  {
    if (diagnostic.empty()) {
      diagnostic = "reference trajectory build failed";
    }
    return false;
  }

  std::string geometry_reason;
  if (!smoothedGeometryReasonable(geometry_baseline, candidate, geometry_reason)) {
    diagnostic = "reference trajectory bad geometry: " + geometry_reason;
    return false;
  }

  const bool safe_with_footprint = isPathSafe(candidate, costmap, footprint);
  const bool safe_centerline =
    use_footprint_collision_check_ && isPathSafe(candidate, costmap, Footprint{});
  if (!safe_with_footprint && !safe_centerline) {
    diagnostic = "reference trajectory unsafe";
    return false;
  }

  updateOrientations(candidate);
  return true;
}

rclcpp_action::GoalResponse SafeGeometricSmoother::handleGenerateGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const GenerateMincoCandidate::Goal> goal)
{
  if (!enabled_ || goal->input_path.poses.size() < 2) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool expected = false;
  if (!generation_in_flight_.compare_exchange_strong(expected, true)) {
    generation_cancel_requested_.store(true);
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "%s: rejecting GenerateMincoCandidate while previous generation is still running; "
      "cancel requested for active generation",
      name_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  generation_cancel_requested_.store(false);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SafeGeometricSmoother::handleGenerateCancel(
  const std::shared_ptr<GoalHandleGenerateMincoCandidate>)
{
  generation_cancel_requested_.store(true);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SafeGeometricSmoother::handleGenerateAccepted(
  const std::shared_ptr<GoalHandleGenerateMincoCandidate> goal_handle)
{
  std::thread{
    std::bind(&SafeGeometricSmoother::executeGenerateCandidate, this, goal_handle)}.detach();
}

void SafeGeometricSmoother::executeGenerateCandidate(
  const std::shared_ptr<GoalHandleGenerateMincoCandidate> goal_handle)
{
  AtomicFlagGuard in_flight(generation_in_flight_);
  auto result = std::make_shared<GenerateMincoCandidate::Result>();
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<GenerateMincoCandidate::Feedback>();
  feedback->state = "generating";
  goal_handle->publish_feedback(feedback);

  if (goal_handle->is_canceling()) {
    result->success = false;
    result->reason = "candidate generation canceled";
    generation_cancel_requested_.store(false);
    goal_handle->canceled(result);
    return;
  }

  nav_msgs::msg::Path candidate = goal->input_path;
  const rclcpp::Time t0 = clock_ ? clock_->now() : rclcpp::Clock().now();
  rclcpp::Duration max_time(goal->max_smoothing_duration);
  if (max_time.nanoseconds() <= 0) {
    max_time = rclcpp::Duration::from_seconds(0.5);
  }

  bool ok = false;
  sentry_nav_interfaces::msg::MincoTrajectory minco_msg;
  {
    std::lock_guard<std::mutex> lk(generation_mutex_);
    ok = smooth(candidate, max_time);
    if (ok && has_last_candidate_minco_) {
      minco_msg = last_candidate_minco_;
    }
  }

  if (goal_handle->is_canceling() || generation_cancel_requested_.load()) {
    result->success = false;
    result->reason = "candidate generation canceled";
    generation_cancel_requested_.store(false);
    goal_handle->canceled(result);
    return;
  }

  const rclcpp::Time t1 = clock_ ? clock_->now() : rclcpp::Clock().now();
  const int64_t duration_ns = (t1 - t0).nanoseconds();
  result->smoothing_duration.sec = static_cast<int32_t>(duration_ns / 1000000000LL);
  result->smoothing_duration.nanosec =
    static_cast<uint32_t>(duration_ns % 1000000000LL);
  result->smoothed_path = candidate;
  result->candidate_minco = minco_msg;
  result->product_type = last_candidate_product_type_;
  result->prefer_keep_active = last_candidate_prefer_keep_active_;
  result->success =
    ok && minco_msg.waypoints.size() >= 2 && !minco_msg.segment_times.empty();
  if (result->success) {
    result->reason = "candidate generated: " + result->product_type;
    goal_handle->succeed(result);
  } else {
    result->reason = ok ? "smoother produced no MINCO candidate" : "candidate generation failed";
    goal_handle->abort(result);
  }
}

void SafeGeometricSmoother::invalidateCache()
{
  has_cached_traj_ = false;
  cached_waypoints_.clear();
  cached_reuse_waypoints_.clear();
  cached_reuse_times_.resize(0);
}

bool SafeGeometricSmoother::tryStitchPath(nav_msgs::msg::Path & path) const
{
  if (!has_cached_traj_ || path.poses.empty()) {return false;}

  const rclcpp::Time now = clock_->now();
  if ((now - cached_traj_stamp_).seconds() > stitch_cache_timeout_) {
    has_cached_traj_ = false;
    return false;
  }

  // Current position from path start
  const Eigen::Vector2d cur(
    path.poses.front().pose.position.x,
    path.poses.front().pose.position.y);

  // Find projection on cached trajectory (sample at 0.05s)
  const double dur = cached_traj_.getTotalDuration();
  double best_t = 0.0, best_dist = 1e9;
  const int n = std::max(2, static_cast<int>(std::ceil(dur / 0.05)));
  for (int i = 0; i <= n; ++i) {
    const double t = dur * static_cast<double>(i) / static_cast<double>(n);
    const Eigen::Vector3d p = cached_traj_.getPos(t);
    const double d = std::hypot(p.x() - cur.x(), p.y() - cur.y());
    if (d < best_dist) {best_dist = d; best_t = t;}
  }

  if (best_dist > stitch_max_distance_) {
    has_cached_traj_ = false;
    return false;
  }

  auto make_pose = [&](const Eigen::Vector3d & p) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = p.x();
      ps.pose.position.y = p.y();
      ps.pose.position.z = p.z();
      ps.pose.orientation.w = 1.0;
      return ps;
    };

  std::vector<geometry_msgs::msg::PoseStamped> prepend;
  const double t_retain = std::max(0.0, best_t - std::max(0.0, retain_duration_));
  const double sample_dt = std::clamp(stitch_sample_dt_, 0.03, 0.30);
  const Eigen::Vector3d path_front(
    path.poses.front().pose.position.x,
    path.poses.front().pose.position.y,
    path.poses.front().pose.position.z);

  const Eigen::Vector3d p_proj = cached_traj_.getPos(best_t);
  Eigen::Vector2d forward_dir(path_front.x() - p_proj.x(), path_front.y() - p_proj.y());
  if (forward_dir.norm() < 1.0e-3) {
    const double t_forward = std::min(best_t + sample_dt, dur);
    const Eigen::Vector3d p_forward = cached_traj_.getPos(t_forward);
    forward_dir = Eigen::Vector2d(p_forward.x() - p_proj.x(), p_forward.y() - p_proj.y());
  }
  const bool has_forward_dir = forward_dir.norm() > 1.0e-3;
  if (has_forward_dir) {
    forward_dir.normalize();
  } else {
    forward_dir = Eigen::Vector2d::Zero();
  }

  for (double t = t_retain; t < best_t - 1.0e-6; t += sample_dt) {
    const Eigen::Vector3d p = cached_traj_.getPos(t);
    const Eigen::Vector2d to_p(p.x() - path_front.x(), p.y() - path_front.y());
    if (has_forward_dir && to_p.dot(forward_dir) < -0.08) {
      continue;
    }
    if ((p - path_front).head<2>().norm() < 0.03) {
      continue;
    }
    if (!prepend.empty()) {
      const auto & last = prepend.back().pose.position;
      if (std::hypot(p.x() - last.x, p.y() - last.y) < 0.03) {
        continue;
      }
    }
    prepend.push_back(make_pose(p));
  }

  const Eigen::Vector2d to_proj(p_proj.x() - path_front.x(), p_proj.y() - path_front.y());
  const bool proj_is_behind =
    has_forward_dir && to_proj.dot(forward_dir) < -0.08;
  if (!proj_is_behind && (p_proj - path_front).head<2>().norm() >= 0.03) {
    if (prepend.empty()) {
      prepend.push_back(make_pose(p_proj));
    } else {
      const auto & last = prepend.back().pose.position;
      if (std::hypot(p_proj.x() - last.x, p_proj.y() - last.y) >= 0.03) {
        prepend.push_back(make_pose(p_proj));
      }
    }
  }

  if (prepend.empty()) {
    return false;
  }

  path.poses.insert(path.poses.begin(), prepend.begin(), prepend.end());
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 1000,
    "Trajectory stitched: retained=%zu best_dist=%.2f best_t=%.2f retain=%.2f",
    prepend.size(), best_dist, best_t, retain_duration_);
  return true;
}

bool SafeGeometricSmoother::tryReuseCachedTrajectory(
  const nav_msgs::msg::Path & input_path,
  nav_msgs::msg::Path & cached_path) const
{
  if (!has_cached_traj_ || input_path.poses.empty()) {
    return false;
  }

  const rclcpp::Time now = clock_->now();
  if ((now - cached_traj_stamp_).seconds() > stitch_cache_timeout_) {
    has_cached_traj_ = false;
    return false;
  }

  const Eigen::Vector2d cur(
    input_path.poses.front().pose.position.x,
    input_path.poses.front().pose.position.y);
  const double dur = cached_traj_.getTotalDuration();
  if (dur <= 1.0e-3) {
    has_cached_traj_ = false;
    return false;
  }

  double best_t = 0.0;
  double best_dist = std::numeric_limits<double>::max();
  const int n = std::max(2, static_cast<int>(std::ceil(dur / 0.05)));
  for (int i = 0; i <= n; ++i) {
    const double t = dur * static_cast<double>(i) / static_cast<double>(n);
    const Eigen::Vector3d p = cached_traj_.getPos(t);
    const double d = std::hypot(p.x() - cur.x(), p.y() - cur.y());
    if (d < best_dist) {
      best_dist = d;
      best_t = t;
    }
  }

  if (best_dist > stitch_max_distance_) {
    has_cached_traj_ = false;
    return false;
  }

  cached_path.header = input_path.header;
  cached_path.poses.clear();
  cached_reuse_waypoints_.clear();
  cached_reuse_times_.resize(0);

  const double sample_dt = std::clamp(stitch_sample_dt_, 0.03, 0.30);
  std::vector<double> sample_times;
  for (double t = best_t; t < dur + 1.0e-6; t += sample_dt) {
    const double tt = std::min(t, dur);
    const Eigen::Vector3d p = cached_traj_.getPos(tt);
    geometry_msgs::msg::PoseStamped ps;
    ps.header = cached_path.header;
    ps.pose.position.x = p.x();
    ps.pose.position.y = p.y();
    ps.pose.position.z = p.z();
    ps.pose.orientation.w = 1.0;
    if (!cached_path.poses.empty()) {
      const auto & last = cached_path.poses.back().pose.position;
      if (std::hypot(p.x() - last.x, p.y() - last.y) < 0.03) {
        continue;
      }
    }
    cached_path.poses.push_back(ps);
    cached_reuse_waypoints_.push_back(p);
    sample_times.push_back(tt);
  }

  if (!sample_times.empty() && sample_times.back() < dur - 1.0e-6) {
    const Eigen::Vector3d p = cached_traj_.getPos(dur);
    const auto & last = cached_path.poses.back().pose.position;
    if (std::hypot(p.x() - last.x, p.y() - last.y) >= 0.03) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = cached_path.header;
      ps.pose.position.x = p.x();
      ps.pose.position.y = p.y();
      ps.pose.position.z = p.z();
      ps.pose.orientation.w = 1.0;
      cached_path.poses.push_back(ps);
      cached_reuse_waypoints_.push_back(p);
      sample_times.push_back(dur);
    }
  }

  if (cached_path.poses.size() < 2) {
    cached_reuse_waypoints_.clear();
    cached_reuse_times_.resize(0);
    return false;
  }

  cached_reuse_times_.resize(static_cast<Eigen::Index>(sample_times.size() - 1));
  for (size_t i = 1; i < sample_times.size(); ++i) {
    cached_reuse_times_(static_cast<Eigen::Index>(i - 1)) =
      std::max(0.05, sample_times[i] - sample_times[i - 1]);
  }
  cached_reuse_initial_velocity_ = cached_traj_.getVel(best_t);
  cached_reuse_initial_acceleration_ = cached_traj_.getAcc(best_t);

  RCLCPP_WARN_THROTTLE(
    logger_, *clock_, 1000,
    "MINCO fallback: reusing cached trajectory (pts=%zu best_dist=%.2f best_t=%.2f rem=%.2fs)",
    cached_path.poses.size(), best_dist, best_t, dur - best_t);
  return true;
}

bool SafeGeometricSmoother::smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time)
{
  if (!enabled_ || path.poses.size() < 2) {return true;}

  has_last_candidate_minco_ = false;

  enum class LocalTrajectoryProduct
  {
    NONE = 0,
    OPTIMIZED_MINCO = 1,
    CACHED_MINCO = 2,
    TIMEOUT_REFERENCE_FALLBACK = 3,
    COLLISION_FALLBACK = 4,
    GEOMETRY_FALLBACK = 5,
    PATH_FALLBACK = 6,
  };

  const nav_msgs::msg::Path input_path = path;
  const PathMetrics m_in = computePathMetrics(input_path);
  LocalTrajectoryProduct product = LocalTrajectoryProduct::NONE;
  last_candidate_product_type_ = "none";
  last_candidate_prefer_keep_active_ = false;

  if (debug_publish_ && input_path_pub_) {input_path_pub_->publish(input_path);}

  auto costmap = costmap_sub_ ? costmap_sub_->getCostmap() : nullptr;
  if (!costmap) {
    RCLCPP_WARN(logger_, "No costmap, returning input path");
    return fallback_to_input_path_;
  }
  fp_checker_.setCostmap(costmap.get());

  Footprint footprint;
  if (use_footprint_collision_check_ && footprint_sub_) {
    std_msgs::msg::Header h;
    if (!footprint_sub_->getFootprintInRobotFrame(footprint, h)) {footprint.clear();}
  }

  if (check_input_path_ && !isPathSafe(input_path, *costmap, footprint)) {
    RCLCPP_WARN(logger_, "Input path unsafe");
    return false;
  }

  const rclcpp::Time start_time = clock_->now();
  nav_msgs::msg::Path candidate = input_path;
  nav_msgs::msg::Path minco_reference_path = input_path;
  MincoOptimizer::Result minco_result;
  InitialState initial_state;
  Eigen::Vector3d initial_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d initial_acceleration = Eigen::Vector3d::Zero();
  const Eigen::Vector3d * initial_vel = nullptr;
  const Eigen::Vector3d * initial_acc = nullptr;

  if (use_minco_) {
    if (minco_options_.use_esdf && !acquireEsdfMap("smooth", false)) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 10000,
        "%s: use_esdf=true but esdf_map_ is null, falling back to costmap", name_.c_str());
    }
    if (do_shortcut_) {
      minco_reference_path =
        shortcutPath(minco_reference_path, *costmap, footprint, start_time, max_time);
    }

    // 目标变化较大时清空旧轨迹缓存。
    const Eigen::Vector3d new_goal(
      input_path.poses.back().pose.position.x,
      input_path.poses.back().pose.position.y, 0.0);
    if ((new_goal - cached_goal_).norm() > 1.0) {invalidateCache();}
    cached_goal_ = new_goal;

    // 尝试把旧轨迹可执行前段拼接到新候选路径。
    const bool stitched = enable_stitching_ && tryStitchPath(minco_reference_path);
    initial_state = getInitialStateInFrame(minco_reference_path.header);
    if (initial_state.valid) {
      initial_velocity = initial_state.velocity;
      initial_acceleration = initial_state.acceleration;
      initial_vel = &initial_velocity;
      initial_acc = &initial_acceleration;
    }
    if (initial_state.valid) {
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 1000,
        "MINCO initial state from odom: speed=%.2fm/s age=%.3fs frame='%s'",
        initial_state.speed, initial_state.age, minco_reference_path.header.frame_id.c_str());
    } else {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 3000,
        "MINCO initial state unavailable, using zero velocity: %s",
        initial_state.reason.c_str());
    }

    std::vector<DynamicObstacle> local_obs;
    {
      std::lock_guard<std::mutex> lk(dynamic_obs_mutex_);
      local_obs = dynamic_obstacles_;
    }
    const std::vector<DynamicObstacle> * obs_ptr =
      (minco_options_.dynamic_obstacle_enabled && !local_obs.empty()) ? &local_obs : nullptr;

    std::function<bool()> should_cancel = [&]() {
        if (generation_cancel_requested_.load()) {
          return true;
        }
        if (clock_ && (clock_->now() - start_time) > max_time) {
          return true;
        }
        return false;
      };

    nav_msgs::msg::Path minco_candidate = minco_reference_path;
    minco_result = minco_optimizer_.smooth(
      minco_reference_path, minco_candidate, costmap.get(), obs_ptr, &should_cancel,
      initial_vel, initial_acc);
    const double minco_ms = (clock_->now() - start_time).nanoseconds() * 1.0e-6;

    if (should_cancel()) {
      if (generation_cancel_requested_.load()) {
        RCLCPP_WARN(
          logger_, "MINCO canceled after %.1fms: %s",
          minco_ms,
          minco_result.reason.empty() ? "cancel requested" : minco_result.reason.c_str());
        return false;
      }

      MincoOptimizer::Result reference_result;
      nav_msgs::msg::Path reference_candidate;
      std::string fallback_diag;
      if (buildSafeReferenceFallback(
          minco_reference_path, minco_reference_path, *costmap, footprint,
          reference_result, reference_candidate, fallback_diag, initial_vel, initial_acc))
      {
        candidate = reference_candidate;
        minco_result = reference_result;
        product = LocalTrajectoryProduct::TIMEOUT_REFERENCE_FALLBACK;
        RCLCPP_INFO_THROTTLE(
          logger_,
          *clock_,
          2000,
          "MINCO timeout after %.1fms, using reference fallback: dur=%.2fs max_v=%.2f max_a=%.2f",
          minco_ms, minco_result.traj_duration, minco_result.max_velocity,
          minco_result.max_acceleration);
      } else {
        RCLCPP_WARN(
          logger_, "MINCO timeout fallback failed after %.1fms: %s",
          minco_ms, fallback_diag.c_str());
        return false;
      }
    }

    if (minco_result.success && product == LocalTrajectoryProduct::NONE) {
      candidate = minco_candidate;
      product = LocalTrajectoryProduct::OPTIMIZED_MINCO;
    }

    if (product == LocalTrajectoryProduct::OPTIMIZED_MINCO ||
      product == LocalTrajectoryProduct::TIMEOUT_REFERENCE_FALLBACK)
    {
      if (minco_result.success) {
        RCLCPP_INFO_THROTTLE(
          logger_, *clock_, 1000,
          "MINCO product=%s: stitch=%d init_v=%.2f pre=%d fine=%d dur=%.2fs "
          "max_v=%.2f max_a=%.2f time=%.1fms waypoints=%zu",
          productTypeToString(static_cast<int>(product)).c_str(), stitched,
          minco_result.initial_velocity.head<2>().norm(),
          minco_result.pre_ret, minco_result.fine_ret,
          minco_result.traj_duration, minco_result.max_velocity,
          minco_result.max_acceleration, minco_ms,
          minco_result.optimized_waypoints.size());
      }

      if (debug_publish_ && minco_waypoints_pub_) {
        publishWaypointMarkers(
          minco_result.input_waypoints, minco_result.optimized_waypoints, candidate.header);
      }
    } else {
      RCLCPP_WARN(logger_, "MINCO failed: %s (%.1fms)", minco_result.reason.c_str(), minco_ms);
      nav_msgs::msg::Path cached_candidate;
      if (reuse_cached_trajectory_on_minco_failure_ &&
        tryReuseCachedTrajectory(input_path, cached_candidate))
      {
        candidate = cached_candidate;
        product = LocalTrajectoryProduct::CACHED_MINCO;
      } else {
        if (fallback_to_input_path_) {
          if (do_shortcut_) {
            candidate = shortcutPath(input_path, *costmap, footprint, start_time, max_time);
          }
          if (do_resample_) {candidate = resamplePath(candidate);}
          product = LocalTrajectoryProduct::PATH_FALLBACK;
        } else {
          return false;
        }
      }
    }
  } else {
    if (do_shortcut_) {
      candidate = shortcutPath(candidate, *costmap, footprint, start_time, max_time);
    }
    if (do_resample_) {candidate = resamplePath(candidate);}
  }

  updateOrientations(candidate);

  if (!isPathSafe(candidate, *costmap, footprint)) {
    const auto col = collectCollisionPoses(candidate, *costmap, footprint);
    RCLCPP_WARN(logger_, "Smoothed path rejected: %zu collision(s)", col.size());
    if (debug_publish_) {publishRejectedPath(candidate); publishCollisionMarkers(col, candidate.header);}
    if (product == LocalTrajectoryProduct::OPTIMIZED_MINCO ||
      product == LocalTrajectoryProduct::TIMEOUT_REFERENCE_FALLBACK)
    {
      MincoOptimizer::Result reference_result;
      nav_msgs::msg::Path reference_candidate;
      std::string fallback_diag;
      if (buildSafeReferenceFallback(
          minco_reference_path, minco_reference_path, *costmap, footprint,
          reference_result, reference_candidate, fallback_diag, initial_vel, initial_acc))
      {
        candidate = reference_candidate;
        minco_result = reference_result;
        product = LocalTrajectoryProduct::COLLISION_FALLBACK;
        RCLCPP_WARN(
          logger_,
          "MINCO collision fallback accepted: reference trajectory dur=%.2fs "
          "max_v=%.2f max_a=%.2f rejected_collisions=%zu",
          minco_result.traj_duration, minco_result.max_velocity,
          minco_result.max_acceleration, col.size());
      } else {
        RCLCPP_WARN(
          logger_, "MINCO collision fallback failed: %s", fallback_diag.c_str());
        return false;
      }
    } else {
    if (use_minco_ && reuse_cached_trajectory_on_minco_failure_) {
      nav_msgs::msg::Path cached_candidate;
      if (tryReuseCachedTrajectory(input_path, cached_candidate) &&
        isPathSafe(cached_candidate, *costmap, footprint))
      {
        updateOrientations(cached_candidate);
        path = cached_candidate;
        if (debug_publish_ && output_path_pub_) {output_path_pub_->publish(path);}
        last_candidate_minco_ = makeCachedMpcTrajectory(path.header);
        has_last_candidate_minco_ =
          last_candidate_minco_.waypoints.size() >= 2 &&
          !last_candidate_minco_.segment_times.empty();
        last_candidate_product_type_ =
          productTypeToString(static_cast<int>(LocalTrajectoryProduct::CACHED_MINCO));
        last_candidate_prefer_keep_active_ = true;
        if (publish_trajectory_for_mpc_ && mpc_traj_pub_) {
          if (has_last_candidate_minco_) {
            mpc_traj_pub_->publish(last_candidate_minco_);
          }
        }
        return true;
      }
      return false;
    }
    if (fallback_to_input_path_) {
      path = input_path;
      if (debug_publish_ && output_path_pub_) {output_path_pub_->publish(input_path);}
      last_candidate_product_type_ = productTypeToString(static_cast<int>(LocalTrajectoryProduct::PATH_FALLBACK));
      last_candidate_prefer_keep_active_ = true;
      return true;
    }
    return false;
    }
  }

  if (product == LocalTrajectoryProduct::OPTIMIZED_MINCO ||
    product == LocalTrajectoryProduct::TIMEOUT_REFERENCE_FALLBACK ||
    product == LocalTrajectoryProduct::COLLISION_FALLBACK)
  {
    std::string geometry_reason;
    if (!smoothedGeometryReasonable(input_path, candidate, geometry_reason)) {
      const auto m_candidate = pathGeometryMetrics(candidate);
      RCLCPP_WARN(
        logger_,
        "Smoothed path rejected: bad geometry (%s, in_len=%.2f out_len=%.2f "
        "out_max_turn=%.2f out_total_turn=%.2f)",
        geometry_reason.c_str(), m_in.length, m_candidate.length,
        m_candidate.max_turn, m_candidate.total_turn);
      if (debug_publish_) {publishRejectedPath(candidate);}
      if (!allow_reference_fallback_on_bad_geometry_) {
        RCLCPP_WARN(
          logger_,
          "MINCO geometry fallback disabled: keeping active trajectory instead of "
          "publishing reference path");
        return false;
      }
      MincoOptimizer::Result reference_result;
      nav_msgs::msg::Path reference_candidate;
      std::string fallback_diag;
      if (buildSafeReferenceFallback(
          minco_reference_path, minco_reference_path, *costmap, footprint,
          reference_result, reference_candidate, fallback_diag, initial_vel, initial_acc))
      {
        candidate = reference_candidate;
        minco_result = reference_result;
        product = LocalTrajectoryProduct::GEOMETRY_FALLBACK;
        RCLCPP_WARN(
          logger_,
          "MINCO geometry fallback accepted: reference trajectory dur=%.2fs "
          "max_v=%.2f max_a=%.2f",
          minco_result.traj_duration, minco_result.max_velocity,
          minco_result.max_acceleration);
      } else {
        RCLCPP_WARN(
          logger_, "MINCO geometry fallback failed: %s", fallback_diag.c_str());
        return false;
      }
    }
  }

  path = candidate;
  if (debug_publish_ && output_path_pub_) {output_path_pub_->publish(candidate);}

  // Update trajectory cache and publish MPC trajectory only after safety check passes
  if (product == LocalTrajectoryProduct::OPTIMIZED_MINCO ||
    product == LocalTrajectoryProduct::TIMEOUT_REFERENCE_FALLBACK ||
    product == LocalTrajectoryProduct::COLLISION_FALLBACK ||
    product == LocalTrajectoryProduct::GEOMETRY_FALLBACK)
  {
    if (minco_result.optimized_waypoints.size() >= 2 &&
      minco_result.optimized_times.size() > 0)
    {
      Trajectory<5> new_traj;
      const Eigen::Vector3d init_v = minco_result.initial_velocity;
      const Eigen::Vector3d init_a = minco_result.initial_acceleration;
      if (minco_optimizer_.buildTrajectory(
          minco_result.optimized_waypoints, minco_result.optimized_times, new_traj,
          &init_v, &init_a))
      {
        cached_traj_ = new_traj;
        cached_waypoints_ = minco_result.optimized_waypoints;
        cached_times_ = minco_result.optimized_times;
        cached_initial_velocity_ = init_v;
        cached_initial_acceleration_ = init_a;
        cached_reuse_waypoints_.clear();
        cached_reuse_times_.resize(0);
        cached_traj_stamp_ = clock_->now();
        has_cached_traj_ = true;
      }
    }
    last_candidate_minco_ = makeMpcTrajectory(minco_result, path.header);
    has_last_candidate_minco_ =
      last_candidate_minco_.waypoints.size() >= 2 &&
      !last_candidate_minco_.segment_times.empty();
    if (publish_trajectory_for_mpc_ && mpc_traj_pub_ && has_last_candidate_minco_) {
      mpc_traj_pub_->publish(last_candidate_minco_);
    }
    last_candidate_product_type_ = productTypeToString(static_cast<int>(product));
    last_candidate_prefer_keep_active_ =
      product == LocalTrajectoryProduct::TIMEOUT_REFERENCE_FALLBACK ||
      product == LocalTrajectoryProduct::COLLISION_FALLBACK ||
      product == LocalTrajectoryProduct::GEOMETRY_FALLBACK;
  } else if (product == LocalTrajectoryProduct::CACHED_MINCO && has_cached_traj_)
  {
    last_candidate_minco_ = makeCachedMpcTrajectory(path.header);
    has_last_candidate_minco_ =
      last_candidate_minco_.waypoints.size() >= 2 &&
      !last_candidate_minco_.segment_times.empty();
    if (publish_trajectory_for_mpc_ && mpc_traj_pub_ && has_last_candidate_minco_) {
      mpc_traj_pub_->publish(last_candidate_minco_);
    }
    last_candidate_product_type_ = productTypeToString(static_cast<int>(product));
    last_candidate_prefer_keep_active_ = true;
  } else if (product == LocalTrajectoryProduct::PATH_FALLBACK) {
    last_candidate_product_type_ = productTypeToString(static_cast<int>(product));
    last_candidate_prefer_keep_active_ = true;
  }

  const PathMetrics m_out = computePathMetrics(candidate);
  // RCLCPP_INFO(
  //   logger_,
  //   "Smoother: in=%zu(%.2f) minco=%d(pre=%d,fine=%d,cost=%.3f) -> out=%zu(%.2f)",
  //   m_in.n, m_in.length, minco_result.success,
  //   minco_result.pre_ret, minco_result.fine_ret, minco_result.final_cost,
  //   m_out.n, m_out.length);

  if (debug_publish_ && metrics_pub_) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "in: n=%zu len=%.3f\nout: n=%zu len=%.3f", m_in.n, m_in.length, m_out.n, m_out.length);
    std_msgs::msg::String msg; msg.data = buf;
    metrics_pub_->publish(msg);
  }
  return true;
}

bool SafeGeometricSmoother::isPathSafe(
  const nav_msgs::msg::Path & path, const nav2_costmap_2d::Costmap2D & costmap,
  const Footprint & footprint) const
{
  if (path.poses.empty()) {return false;}
  for (const auto & pose : path.poses) {
    if (!isPoseSafe(pose, costmap, footprint)) {return false;}
  }
  for (size_t i = 1; i < path.poses.size(); ++i) {
    if (!isSegmentSafe(path.poses[i-1], path.poses[i], costmap, footprint)) {return false;}
  }
  return true;
}

bool SafeGeometricSmoother::isSegmentSafe(
  const geometry_msgs::msg::PoseStamped & from,
  const geometry_msgs::msg::PoseStamped & to,
  const nav2_costmap_2d::Costmap2D & costmap,
  const Footprint & footprint) const
{
  const double dist = distance(from, to);
  const int steps = std::max(1, static_cast<int>(std::ceil(dist / std::max(collision_check_step_, 1.0e-3))));
  geometry_msgs::msg::Quaternion seg_q = from.pose.orientation;
  if (use_footprint_collision_check_ && !footprint.empty()) {
    tf2::Quaternion q;
    q.setRPY(0, 0, std::atan2(to.pose.position.y - from.pose.position.y,
                               to.pose.position.x - from.pose.position.x));
    seg_q = tf2::toMsg(q);
  }
  for (int i = 0; i <= steps; ++i) {
    auto p = interpolate(from, to, static_cast<double>(i) / static_cast<double>(steps));
    p.pose.orientation = seg_q;
    if (!isPoseSafe(p, costmap, footprint)) {return false;}
  }
  return true;
}

bool SafeGeometricSmoother::isPoseSafe(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav2_costmap_2d::Costmap2D & costmap,
  const Footprint & footprint) const
{
  if (use_footprint_collision_check_ && !footprint.empty()) {
    const double cost = fp_checker_.footprintCostAtPose(
      pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation), footprint);
    if (cost < 0.0) {return false;}
    if (static_cast<unsigned char>(cost) == nav2_costmap_2d::NO_INFORMATION) {return allow_unknown_;}
    return cost < static_cast<double>(footprint_collision_cost_threshold_);
  }
  unsigned int mx = 0, my = 0;
  if (!costmap.worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {return false;}
  const unsigned char c = costmap.getCost(mx, my);
  if (c == nav2_costmap_2d::NO_INFORMATION) {return allow_unknown_;}
  return c < collision_cost_threshold_;
}

double SafeGeometricSmoother::poseClearance(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav2_costmap_2d::Costmap2D & costmap,
  const Footprint & footprint) const
{
  if (!isPoseSafe(pose, costmap, footprint)) {
    return -std::numeric_limits<double>::infinity();
  }

  double footprint_radius = 0.0;
  if (use_footprint_collision_check_ && !footprint.empty()) {
    for (const auto & p : footprint) {
      footprint_radius = std::max(footprint_radius, std::hypot(p.x, p.y));
    }
  }

  if (minco_options_.use_esdf && esdf_map_) {
    const double d = esdf_map_->getDistance2D(
      pose.pose.position.x, pose.pose.position.y, minco_options_.esdf_query_z);
    if (std::isfinite(d) && d > -1.0e5) {
      return d - footprint_radius;
    }
  }

  unsigned int mx = 0, my = 0;
  if (!costmap.worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
    return -std::numeric_limits<double>::infinity();
  }

  const unsigned int sx = costmap.getSizeInCellsX();
  const unsigned int sy = costmap.getSizeInCellsY();
  const double res = costmap.getResolution();
  const int max_cells = std::max(
    1, static_cast<int>(std::ceil(std::max(min_clearance_for_removal_, res) / res)));

  for (int r = 0; r <= max_cells; ++r) {
    bool has_free_unknown = false;
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != r) {continue;}
        const int cx = static_cast<int>(mx) + dx;
        const int cy = static_cast<int>(my) + dy;
        if (cx < 0 || cy < 0 || cx >= static_cast<int>(sx) || cy >= static_cast<int>(sy)) {
          return static_cast<double>(r) * res - footprint_radius;
        }
        const unsigned char c = costmap.getCost(
          static_cast<unsigned int>(cx), static_cast<unsigned int>(cy));
        if (c == nav2_costmap_2d::NO_INFORMATION) {
          if (!allow_unknown_) {
            return static_cast<double>(r) * res - footprint_radius;
          }
          has_free_unknown = true;
          continue;
        }
        if (c != nav2_costmap_2d::FREE_SPACE || c >= collision_cost_threshold_) {
          return static_cast<double>(r) * res - footprint_radius;
        }
      }
    }
    if (r == max_cells && has_free_unknown) {
      return static_cast<double>(max_cells) * res - footprint_radius;
    }
  }
  return static_cast<double>(max_cells) * res - footprint_radius;
}

double SafeGeometricSmoother::segmentClearance(
  const geometry_msgs::msg::PoseStamped & from,
  const geometry_msgs::msg::PoseStamped & to,
  const nav2_costmap_2d::Costmap2D & costmap,
  const Footprint & footprint) const
{
  const double dist = distance(from, to);
  const int steps = std::max(
    1, static_cast<int>(std::ceil(dist / std::max(collision_check_step_, 1.0e-3))));
  double min_clearance = std::numeric_limits<double>::infinity();
  geometry_msgs::msg::Quaternion seg_q = from.pose.orientation;
  if (use_footprint_collision_check_ && !footprint.empty()) {
    tf2::Quaternion q;
    q.setRPY(
      0, 0, std::atan2(
        to.pose.position.y - from.pose.position.y,
        to.pose.position.x - from.pose.position.x));
    seg_q = tf2::toMsg(q);
  }
  for (int i = 0; i <= steps; ++i) {
    auto p = interpolate(from, to, static_cast<double>(i) / static_cast<double>(steps));
    p.pose.orientation = seg_q;
    min_clearance = std::min(min_clearance, poseClearance(p, costmap, footprint));
  }
  return min_clearance;
}

bool SafeGeometricSmoother::isSegmentClear(
  const geometry_msgs::msg::PoseStamped & from,
  const geometry_msgs::msg::PoseStamped & to,
  const nav2_costmap_2d::Costmap2D & costmap,
  const Footprint & footprint) const
{
  return segmentClearance(from, to, costmap, footprint) >= min_clearance_for_removal_;
}

nav_msgs::msg::Path SafeGeometricSmoother::shortcutPath(
  const nav_msgs::msg::Path & path, const nav2_costmap_2d::Costmap2D & costmap,
  const Footprint & footprint,
  const rclcpp::Time & start_time, const rclcpp::Duration & max_time) const
{
  nav_msgs::msg::Path out; out.header = path.header;
  if (path.poses.size() < 3) {return path;}
  size_t i = 0;
  out.poses.push_back(path.poses.front());
  while (i + 1 < path.poses.size()) {
    if ((clock_->now() - start_time) > max_time) {return path;}
    size_t best = i + 1;
    const size_t max_j = std::min(path.poses.size()-1, i + static_cast<size_t>(std::max(1, max_shortcut_skip_)));
    for (size_t j = max_j; j > i + 1; --j) {
      if (distance(path.poses[i], path.poses[j]) > max_shortcut_dist_) {continue;}
      if (isSegmentClear(path.poses[i], path.poses[j], costmap, footprint)) {best = j; break;}
    }
    out.poses.push_back(path.poses[best]);
    i = best;
  }
  return out;
}

nav_msgs::msg::Path SafeGeometricSmoother::resamplePath(const nav_msgs::msg::Path & path) const
{
  nav_msgs::msg::Path out; out.header = path.header;
  if (path.poses.size() < 2 || resample_resolution_ <= 1.0e-3) {return path;}
  out.poses.push_back(path.poses.front());
  double carry = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const double seg = distance(path.poses[i-1], path.poses[i]);
    if (seg <= 1.0e-6) {continue;}
    double target = resample_resolution_ - carry;
    while (target < seg) {
      out.poses.push_back(interpolate(path.poses[i-1], path.poses[i], target / seg));
      target += resample_resolution_;
    }
    carry = seg - (target - resample_resolution_);
    if (carry >= resample_resolution_) {carry = 0.0;}
  }
  if (distance(out.poses.back(), path.poses.back()) > 1.0e-6) {
    out.poses.push_back(path.poses.back());
  }
  return out;
}

void SafeGeometricSmoother::updateOrientations(nav_msgs::msg::Path & path) const
{
  if (path.poses.size() < 2) {return;}
  for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const double dx = path.poses[i+1].pose.position.x - path.poses[i].pose.position.x;
    const double dy = path.poses[i+1].pose.position.y - path.poses[i].pose.position.y;
    if (std::hypot(dx, dy) < 1.0e-6) {continue;}
    tf2::Quaternion q; q.setRPY(0, 0, std::atan2(dy, dx));
    path.poses[i].pose.orientation = tf2::toMsg(q);
  }
  path.poses.back().pose.orientation = path.poses[path.poses.size()-2].pose.orientation;
}

double SafeGeometricSmoother::distance(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b) const
{
  return std::hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y);
}

geometry_msgs::msg::PoseStamped SafeGeometricSmoother::interpolate(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b, double r) const
{
  auto out = a;
  r = std::clamp(r, 0.0, 1.0);
  out.pose.position.x = a.pose.position.x + (b.pose.position.x - a.pose.position.x) * r;
  out.pose.position.y = a.pose.position.y + (b.pose.position.y - a.pose.position.y) * r;
  out.pose.position.z = a.pose.position.z + (b.pose.position.z - a.pose.position.z) * r;
  return out;
}

double SafeGeometricSmoother::computePathLength(const nav_msgs::msg::Path & path) const
{
  double len = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {len += distance(path.poses[i-1], path.poses[i]);}
  return len;
}

SafeGeometricSmoother::PathMetrics SafeGeometricSmoother::computePathMetrics(
  const nav_msgs::msg::Path & path) const
{
  PathMetrics m; m.n = path.poses.size();
  if (m.n < 2) {m.min_step = 0.0; return m;}
  for (size_t i = 1; i < m.n; ++i) {
    const double d = distance(path.poses[i-1], path.poses[i]);
    m.length += d; m.max_step = std::max(m.max_step, d); m.min_step = std::min(m.min_step, d);
    if (i + 1 < m.n) {
      const double dx1 = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
      const double dy1 = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
      const double dx2 = path.poses[i+1].pose.position.x - path.poses[i].pose.position.x;
      const double dy2 = path.poses[i+1].pose.position.y - path.poses[i].pose.position.y;
      if (std::hypot(dx1,dy1) > 1.0e-6 && std::hypot(dx2,dy2) > 1.0e-6) {
        double dyaw = std::abs(std::atan2(dy2,dx2) - std::atan2(dy1,dx1));
        if (dyaw > M_PI) {dyaw = 2*M_PI - dyaw;}
        m.max_dyaw = std::max(m.max_dyaw, dyaw); m.total_dyaw += dyaw;
      }
    }
  }
  m.avg_step = m.length / static_cast<double>(m.n - 1);
  return m;
}

std::vector<geometry_msgs::msg::PoseStamped> SafeGeometricSmoother::collectCollisionPoses(
  const nav_msgs::msg::Path & path, const nav2_costmap_2d::Costmap2D & costmap,
  const Footprint & footprint) const
{
  std::vector<geometry_msgs::msg::PoseStamped> result;
  for (const auto & pose : path.poses) {
    if (!isPoseSafe(pose, costmap, footprint)) {result.push_back(pose);}
  }
  return result;
}

sentry_nav_interfaces::msg::MincoTrajectory SafeGeometricSmoother::makeMpcTrajectory(
  const MincoOptimizer::Result & result, const std_msgs::msg::Header & header) const
{
  sentry_nav_interfaces::msg::MincoTrajectory msg;
  msg.header = header;
  msg.goal_id = goalIdFromWaypoints(result.optimized_waypoints);
  for (const auto & wp : result.optimized_waypoints) {
    geometry_msgs::msg::Point p; p.x = wp.x(); p.y = wp.y(); p.z = wp.z();
    msg.waypoints.push_back(p);
  }
  for (Eigen::Index i = 0; i < result.optimized_times.size(); ++i) {
    msg.segment_times.push_back(result.optimized_times(i));
  }
  msg.initial_velocity.x = result.initial_velocity.x();
  msg.initial_velocity.y = result.initial_velocity.y();
  msg.initial_velocity.z = result.initial_velocity.z();
  msg.initial_acceleration.x = result.initial_acceleration.x();
  msg.initial_acceleration.y = result.initial_acceleration.y();
  msg.initial_acceleration.z = result.initial_acceleration.z();
  return msg;
}

sentry_nav_interfaces::msg::MincoTrajectory SafeGeometricSmoother::makeCachedMpcTrajectory(
  const std_msgs::msg::Header & header) const
{
  sentry_nav_interfaces::msg::MincoTrajectory msg;
  const auto & waypoints =
    cached_reuse_waypoints_.size() >= 2 ? cached_reuse_waypoints_ : cached_waypoints_;
  const Eigen::VectorXd & times =
    cached_reuse_waypoints_.size() >= 2 ? cached_reuse_times_ : cached_times_;

  if (waypoints.size() < 2 ||
    times.size() + 1 != static_cast<Eigen::Index>(waypoints.size()))
  {
    return msg;
  }

  msg.header = header;
  msg.goal_id = goalIdFromWaypoints(waypoints);
  for (const auto & wp : waypoints) {
    geometry_msgs::msg::Point p;
    p.x = wp.x();
    p.y = wp.y();
    p.z = wp.z();
    msg.waypoints.push_back(p);
  }
  for (Eigen::Index i = 0; i < times.size(); ++i) {
    msg.segment_times.push_back(times(i));
  }
  const bool using_reuse = cached_reuse_waypoints_.size() >= 2;
  const Eigen::Vector3d init_v =
    using_reuse ? cached_reuse_initial_velocity_ : cached_initial_velocity_;
  const Eigen::Vector3d init_a =
    using_reuse ? cached_reuse_initial_acceleration_ : cached_initial_acceleration_;
  msg.initial_velocity.x = init_v.x();
  msg.initial_velocity.y = init_v.y();
  msg.initial_velocity.z = init_v.z();
  msg.initial_acceleration.x = init_a.x();
  msg.initial_acceleration.y = init_a.y();
  msg.initial_acceleration.z = init_a.z();
  return msg;
}

void SafeGeometricSmoother::publishWaypointMarkers(
  const std::vector<Eigen::Vector3d> & input_wps,
  const std::vector<Eigen::Vector3d> & opt_wps,
  const std_msgs::msg::Header & header) const
{
  if (!minco_waypoints_pub_) {return;}
  visualization_msgs::msg::MarkerArray ma;
  auto mk = [&](const Eigen::Vector3d & p, int id, float r, float g, float b) {
      visualization_msgs::msg::Marker m;
      m.header = header; m.ns = "minco_waypoints"; m.id = id;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = p.x(); m.pose.position.y = p.y(); m.pose.position.z = p.z();
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = 0.08;
      m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.9f;
      return m;
    };
  int id = 0;
  for (const auto & p : input_wps) {ma.markers.push_back(mk(p, id++, 0.2f, 0.4f, 1.0f));}
  for (const auto & p : opt_wps)   {ma.markers.push_back(mk(p, id++, 0.2f, 1.0f, 0.2f));}
  minco_waypoints_pub_->publish(ma);
}

void SafeGeometricSmoother::publishRejectedPath(const nav_msgs::msg::Path & path) const
{
  if (rejected_path_pub_) {rejected_path_pub_->publish(path);}
}

void SafeGeometricSmoother::publishCollisionMarkers(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses,
  const std_msgs::msg::Header & header) const
{
  if (!collision_points_pub_) {return;}
  visualization_msgs::msg::MarkerArray ma;
  int id = 0;
  for (const auto & pose : poses) {
    visualization_msgs::msg::Marker m;
    m.header = header; m.ns = "collision_points"; m.id = id++;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = pose.pose;
    m.scale.x = m.scale.y = m.scale.z = 0.12;
    m.color.r = 1.0f; m.color.g = 0.1f; m.color.b = 0.1f; m.color.a = 1.0f;
    ma.markers.push_back(m);
  }
  collision_points_pub_->publish(ma);
}

}  // namespace sirb_smoother

PLUGINLIB_EXPORT_CLASS(sirb_smoother::SafeGeometricSmoother, nav2_core::Smoother)
