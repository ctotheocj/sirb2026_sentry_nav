#ifndef PB_NAV2_SMOOTHER__SAFE_GEOMETRIC_SMOOTHER_HPP_
#define PB_NAV2_SMOOTHER__SAFE_GEOMETRIC_SMOOTHER_HPP_

#include <atomic>
#include <limits>
#include <string>
#include <vector>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sirb_smoother/minco_optimizer.hpp"
#include "plan_env/grid_map.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sentry_nav_interfaces/action/generate_minco_candidate.hpp"
#include "sentry_nav_interfaces/msg/tracked_obstacle_array.hpp"
#include "sentry_nav_interfaces/msg/minco_trajectory.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "gcopter/trajectory.hpp"

namespace sirb_smoother
{

class SafeGeometricSmoother : public nav2_core::Smoother
{
public:
  SafeGeometricSmoother() = default;
  ~SafeGeometricSmoother() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setEsdf(plan_env::GridMap::Ptr esdf);  // for unit-test injection only

private:
  using Footprint = std::vector<geometry_msgs::msg::Point>;
  using GenerateMincoCandidate = sentry_nav_interfaces::action::GenerateMincoCandidate;
  using GoalHandleGenerateMincoCandidate =
    rclcpp_action::ServerGoalHandle<GenerateMincoCandidate>;

  bool smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time) override;
  bool acquireEsdfMap(const char * reason, bool warn_on_failure);
  bool buildSafeReferenceFallback(
    const nav_msgs::msg::Path & reference_path,
    const nav_msgs::msg::Path & geometry_baseline,
    const nav2_costmap_2d::Costmap2D & costmap,
    const Footprint & footprint,
    MincoOptimizer::Result & result,
    nav_msgs::msg::Path & candidate,
    std::string & diagnostic,
    const Eigen::Vector3d * initial_velocity = nullptr,
    const Eigen::Vector3d * initial_acceleration = nullptr,
    const Eigen::Vector3d * terminal_velocity = nullptr,
    const Eigen::Vector3d * terminal_acceleration = nullptr) const;

  bool isPathSafe(
    const nav_msgs::msg::Path & path, const nav2_costmap_2d::Costmap2D & costmap,
    const Footprint & footprint) const;
  bool isSegmentSafe(
    const geometry_msgs::msg::PoseStamped & from,
    const geometry_msgs::msg::PoseStamped & to,
    const nav2_costmap_2d::Costmap2D & costmap,
    const Footprint & footprint) const;
  bool isPoseSafe(
    const geometry_msgs::msg::PoseStamped & pose,
    const nav2_costmap_2d::Costmap2D & costmap,
    const Footprint & footprint) const;
  double poseClearance(
    const geometry_msgs::msg::PoseStamped & pose,
    const nav2_costmap_2d::Costmap2D & costmap,
    const Footprint & footprint) const;
  double segmentClearance(
    const geometry_msgs::msg::PoseStamped & from,
    const geometry_msgs::msg::PoseStamped & to,
    const nav2_costmap_2d::Costmap2D & costmap,
    const Footprint & footprint) const;
  bool isSegmentClear(
    const geometry_msgs::msg::PoseStamped & from,
    const geometry_msgs::msg::PoseStamped & to,
    const nav2_costmap_2d::Costmap2D & costmap,
    const Footprint & footprint) const;

  nav_msgs::msg::Path shortcutPath(
    const nav_msgs::msg::Path & path, const nav2_costmap_2d::Costmap2D & costmap,
    const Footprint & footprint,
    const rclcpp::Time & start_time, const rclcpp::Duration & max_time) const;
  nav_msgs::msg::Path resamplePath(const nav_msgs::msg::Path & path) const;
  void updateOrientations(nav_msgs::msg::Path & path) const;

  double distance(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b) const;
  geometry_msgs::msg::PoseStamped interpolate(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b, double ratio) const;
  double computePathLength(const nav_msgs::msg::Path & path) const;
  nav_msgs::msg::Path buildLocalReferenceWindow(
    const nav_msgs::msg::Path & path,
    bool & reaches_goal,
    double & window_length,
    double & remaining_length) const;
  Eigen::Vector3d terminalVelocityForWindow(
    const nav_msgs::msg::Path & path,
    bool reaches_goal) const;

  struct PathMetrics
  {
    size_t n{0};
    double length{0.0};
    double avg_step{0.0};
    double max_step{0.0};
    double min_step{std::numeric_limits<double>::max()};
    double max_dyaw{0.0};
    double total_dyaw{0.0};
  };
  PathMetrics computePathMetrics(const nav_msgs::msg::Path & path) const;

  struct InitialState
  {
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
    bool valid{false};
    double age{0.0};
    double speed{0.0};
    std::string reason;
  };
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  InitialState getInitialStateInFrame(const std_msgs::msg::Header & target_header) const;

  std::vector<geometry_msgs::msg::PoseStamped> collectCollisionPoses(
    const nav_msgs::msg::Path & path, const nav2_costmap_2d::Costmap2D & costmap,
    const Footprint & footprint) const;

  void publishWaypointMarkers(
    const std::vector<Eigen::Vector3d> & input_wps,
    const std::vector<Eigen::Vector3d> & opt_wps,
    const std_msgs::msg::Header & header) const;
  void publishRejectedPath(const nav_msgs::msg::Path & path) const;
  void publishCollisionMarkers(
    const std::vector<geometry_msgs::msg::PoseStamped> & poses,
    const std_msgs::msg::Header & header) const;
  sentry_nav_interfaces::msg::MincoTrajectory makeMpcTrajectory(
    const MincoOptimizer::Result & result,
    const std_msgs::msg::Header & header,
    uint64_t goal_id) const;
  sentry_nav_interfaces::msg::MincoTrajectory makeCachedMpcTrajectory(
    const std_msgs::msg::Header & header) const;
  rclcpp_action::GoalResponse handleGenerateGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GenerateMincoCandidate::Goal> goal);
  rclcpp_action::CancelResponse handleGenerateCancel(
    const std::shared_ptr<GoalHandleGenerateMincoCandidate> goal_handle);
  void handleGenerateAccepted(
    const std::shared_ptr<GoalHandleGenerateMincoCandidate> goal_handle);
  void executeGenerateCandidate(
    const std::shared_ptr<GoalHandleGenerateMincoCandidate> goal_handle);

  // 轨迹拼接只用于生成候选；执行失败保留旧轨迹由 TrajectoryManager 负责。
  void invalidateCache();
  bool tryStitchPath(nav_msgs::msg::Path & path) const;
  bool tryReuseCachedTrajectory(
    const nav_msgs::msg::Path & input_path,
    nav_msgs::msg::Path & cached_path) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  rclcpp::Logger logger_{rclcpp::get_logger("SafeGeometricSmoother")};

  bool enabled_{true};
  bool fallback_to_input_path_{false};
  bool check_input_path_{false};
  bool do_shortcut_{true};
  bool do_resample_{true};
  bool use_minco_{false};
  bool minco_optimize_online_{false};
  unsigned char collision_cost_threshold_{253};
  double collision_check_step_{0.05};
  double resample_resolution_{0.20};
  double max_shortcut_dist_{2.0};
  int max_shortcut_skip_{30};
  double min_clearance_for_removal_{0.25};

  bool use_footprint_collision_check_{true};
  unsigned char footprint_collision_cost_threshold_{253};
  bool allow_unknown_{true};

  MincoOptimizer::Options minco_options_;
  MincoOptimizer minco_optimizer_;

  mutable nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> fp_checker_;

  // ESDF 查询提供连续距离场避障代价。
  plan_env::GridMap::Ptr esdf_map_;

  // 节点时钟跟随 use_sim_time。
  rclcpp::Clock::SharedPtr clock_;

  // 缓存最近一次可执行 MINCO 轨迹。
  mutable bool has_cached_traj_{false};
  mutable rclcpp::Time cached_traj_stamp_{0, 0, RCL_ROS_TIME};
  mutable std::vector<Eigen::Vector3d> cached_waypoints_;
  mutable Eigen::VectorXd cached_times_;
  mutable Trajectory<5> cached_traj_;
  mutable std::vector<Eigen::Vector3d> cached_reuse_waypoints_;
  mutable Eigen::VectorXd cached_reuse_times_;
  mutable Eigen::Vector3d cached_initial_velocity_{Eigen::Vector3d::Zero()};
  mutable Eigen::Vector3d cached_initial_acceleration_{Eigen::Vector3d::Zero()};
  mutable Eigen::Vector3d cached_reuse_initial_velocity_{Eigen::Vector3d::Zero()};
  mutable Eigen::Vector3d cached_reuse_initial_acceleration_{Eigen::Vector3d::Zero()};
  mutable Eigen::Vector3d cached_terminal_velocity_{Eigen::Vector3d::Zero()};
  mutable Eigen::Vector3d cached_terminal_acceleration_{Eigen::Vector3d::Zero()};
  mutable bool cached_terminal_stop_{true};
  mutable uint64_t cached_goal_id_{0};
  mutable Eigen::Vector3d cached_goal_{0, 0, 0};
  bool enable_stitching_{false};
  double retain_duration_{0.3};
  double stitch_sample_dt_{0.1};
  double stitch_max_distance_{0.5};
  double stitch_cache_timeout_{2.0};
  bool reuse_cached_trajectory_on_minco_failure_{false};
  bool allow_reference_fallback_on_bad_geometry_{false};
  double minco_local_horizon_distance_{6.0};
  double minco_goal_stop_distance_{0.8};
  double minco_terminal_pass_speed_{0.0};

  bool debug_publish_{true};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr input_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr output_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr minco_waypoints_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rejected_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr collision_points_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_obs_marker_pub_;
  rclcpp_action::Server<GenerateMincoCandidate>::SharedPtr generate_action_server_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sentry_nav_interfaces::msg::TrackedObstacleArray>::SharedPtr dynamic_obs_sub_;
  mutable std::mutex odom_mutex_;
  nav_msgs::msg::Odometry latest_odom_;
  bool has_odom_{false};
  bool use_odom_initial_state_{true};
  bool odom_twist_in_child_frame_{true};
  std::string odom_topic_{"odometry"};
  double odom_max_age_sec_{0.25};
  double initial_velocity_max_{0.0};
  std::vector<DynamicObstacle> dynamic_obstacles_;
  std::atomic_bool generation_in_flight_{false};
  std::atomic_bool generation_cancel_requested_{false};
  mutable std::mutex dynamic_obs_mutex_;
  mutable std::mutex generation_mutex_;
  mutable sentry_nav_interfaces::msg::MincoTrajectory last_candidate_minco_;
  mutable bool has_last_candidate_minco_{false};
  mutable std::string last_candidate_product_type_{"none"};
  mutable bool last_candidate_prefer_keep_active_{false};
  std::string dynamic_obstacle_topic_{"dynamic_obstacles"};
};

}  // namespace sirb_smoother

#endif  // PB_NAV2_SMOOTHER__SAFE_GEOMETRIC_SMOOTHER_HPP_
