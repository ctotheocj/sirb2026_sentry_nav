#ifndef PB_NAV2_SMOOTHER__MINCO_OPTIMIZER_HPP_
#define PB_NAV2_SMOOTHER__MINCO_OPTIMIZER_HPP_

#include <string>
#include <vector>
#include <functional>

#include "Eigen/Eigen"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "gcopter/lbfgs.hpp"
#include "gcopter/minco.hpp"
#include "plan_env/grid_map.h"

namespace sirb_smoother
{

struct DynamicObstacle
{
  Eigen::Vector2d p0;
  Eigen::Vector2d v;
  double radius{0.3};
  std::vector<Eigen::Vector2d> predicted_positions;
  double prediction_dt{0.1};
};

class MincoOptimizer
{
public:
  struct Options
  {
    bool enabled{false};
    double v_ref{1.5};
    double min_segment_time{0.15};
    double max_segment_time{1.0};
    double corner_time_weight{0.20};
    double sample_resolution{0.10};
    int max_pieces{80};
    bool use_lbfgs{true};
    bool guide_fillet_enabled{true};
    double guide_fillet_radius{0.60};
    double guide_fillet_min_angle{0.25};
    double obstacle_cost_threshold{128.0};
    double obstacle_finite_diff_step{0.05};
    double obstacle_sample_dt{0.05};
    double w_obstacle_traj{1.0};
    // dynamics feasibility
    double v_max{2.0};
    double a_max{3.0};
    double w_velocity{1.0};
    double w_acceleration{0.5};
    double dynamics_sample_dt{0.05};
    double dynamic_realloc_max_segment_scale{2.5};
    bool allow_unknown{true};
    // PRE stage
    int pre_max_iterations{30};
    double pre_w_energy{0.5};
    double pre_w_reference{20.0};
    double pre_w_obstacle{1.0};
    // FINE stage
    int fine_max_iterations{60};
    double fine_w_energy{2.0};
    double fine_w_reference{5.0};
    double fine_w_obstacle{3.0};
    // dynamic obstacles
    bool dynamic_obstacle_enabled{false};
    double dynamic_obstacle_safe_dist{0.5};
    double w_dynamic_obstacle{5.0};
    // time optimization
    double wei_time{50.0};

    // ESDF 参数控制避障距离和梯度查询。
    bool use_esdf{false};
    double esdf_safe_distance{0.3};
    double esdf_influence_distance{0.8};
    double esdf_gradient_step{0.05};
    double esdf_valley_threshold{0.5};
    double esdf_query_z{0.3};  // 二维查询使用的固定高度切片。

    // 粗优化先修正轨迹形状。
    bool phase0_enabled{true};
    int phase0_max_iterations{15};
    double phase0_w_energy{1.0};
    double phase0_w_reference{10.0};
    double phase0_w_obstacle{0.1};

    // 时间比例约束限制相邻段时长突变。
    double w_time_ratio{5.0};
    double time_ratio_upper{1.3};
    double time_ratio_lower{0.7};

    // 梯形时间分配在折角处自然减速。
    bool use_trapezoidal_time{true};
    double trapezoidal_k_angle{0.3};
  };

  struct Result
  {
    bool success{false};
    bool used_lbfgs{false};
    int pre_ret{0};
    int fine_ret{0};
    double pre_cost{0.0};
    double fine_cost{0.0};
    double final_cost{0.0};
    double traj_duration{0.0};
    double max_velocity{0.0};
    double max_acceleration{0.0};
    Eigen::Vector3d initial_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d initial_acceleration{Eigen::Vector3d::Zero()};
    std::vector<Eigen::Vector3d> input_waypoints;
    std::vector<Eigen::Vector3d> optimized_waypoints;
    Eigen::VectorXd optimized_times;
    std::string reason;
  };

  MincoOptimizer();
  explicit MincoOptimizer(const Options & options);

  void setOptions(const Options & options);
  void setEsdf(plan_env::GridMap::Ptr esdf);

  Result smooth(
    const nav_msgs::msg::Path & input, nav_msgs::msg::Path & output,
    const nav2_costmap_2d::Costmap2D * costmap = nullptr,
    const std::vector<DynamicObstacle> * dynamic_obstacles = nullptr,
    const std::function<bool()> * should_cancel = nullptr) const;

  bool buildTrajectory(
    const std::vector<Eigen::Vector3d> & points, const Eigen::VectorXd & times,
    Trajectory<5> & traj) const;
  bool buildReferenceTrajectory(
    const nav_msgs::msg::Path & input, Result & result, nav_msgs::msg::Path & output,
    std::string * diagnostic = nullptr) const;
  bool enforceDynamicFeasibility(
    const std::vector<Eigen::Vector3d> & points, Eigen::VectorXd & times,
    Trajectory<5> & traj, double & max_velocity, double & max_acceleration,
    std::string * diagnostic = nullptr) const;

private:
  struct GuidePath
  {
    std::vector<Eigen::Vector3d> points;
    std::vector<double> arc_lengths;
    double length{0.0};
  };

  struct GuideProjection
  {
    Eigen::Vector3d point{Eigen::Vector3d::Zero()};
    double distance_sq{0.0};
    bool valid{false};
  };

  struct OptimizationData
  {
    const MincoOptimizer * optimizer{nullptr};
    const std::vector<Eigen::Vector3d> * reference_points{nullptr};
    const GuidePath * guide{nullptr};
    const Eigen::VectorXd * times{nullptr};
    const nav2_costmap_2d::Costmap2D * costmap{nullptr};
    double w_energy{1.0};
    double w_reference{10.0};
    double w_obstacle{2.0};
    const std::vector<DynamicObstacle> * dynamic_obstacles{nullptr};
    const std::function<bool()> * should_cancel{nullptr};
    bool is_fine_stage{false};
    int n_pts{0};
  };

  bool buildGuidePath(const nav_msgs::msg::Path & path, GuidePath & guide) const;
  bool buildWaypoints(
    const GuidePath & guide, std::vector<Eigen::Vector3d> & points,
    Eigen::VectorXd & times) const;
  Eigen::Vector3d guidePointAt(const GuidePath & guide, double arc_s) const;
  GuideProjection projectToGuide(
    const GuidePath & guide, const Eigen::Vector3d & point, double hint_s,
    double search_radius) const;
  void assignSegmentTimes(
    const std::vector<Eigen::Vector3d> & points, Eigen::VectorXd & times) const;
  void smoothSegmentTimes(Eigen::VectorXd & times) const;
  int optimizeWaypoints(
    std::vector<Eigen::Vector3d> & points, const GuidePath & guide, Eigen::VectorXd & times,
    const nav2_costmap_2d::Costmap2D * costmap,
    const std::vector<DynamicObstacle> * dynamic_obstacles,
    double w_energy, double w_reference, double w_obstacle,
    int max_iterations, double & final_cost, bool is_fine_stage = false,
    const std::function<bool()> * should_cancel = nullptr) const;
  double evaluateObjective(
    const Eigen::VectorXd & x, Eigen::VectorXd & grad,
    const OptimizationData & data) const;
  static double evaluateCallback(void * instance, const Eigen::VectorXd & x, Eigen::VectorXd & grad);
  static int progressCallback(
    void * instance, const Eigen::VectorXd & x, const Eigen::VectorXd & g,
    const double fx, const double step, const int k, const int ls);
  double obstacleCostAndGradient(
    const Eigen::Vector3d & point, Eigen::Vector2d & grad,
    const nav2_costmap_2d::Costmap2D * costmap) const;
  double esdfCostAndGradient(
    const Eigen::Vector3d & point, Eigen::Vector2d & grad) const;
  double dynamicObstacleCostAndGradient(
    const Eigen::Vector3d & point, double t_global,
    const std::vector<DynamicObstacle> & obstacles,
    Eigen::Vector2d & grad) const;
  double costAt(
    double wx, double wy, const nav2_costmap_2d::Costmap2D * costmap) const;
  double costAtInterpolated(
    double wx, double wy, const nav2_costmap_2d::Costmap2D * costmap) const;
  void setBoundaryStates(
    const std::vector<Eigen::Vector3d> & points,
    const std::vector<Eigen::Vector3d> & ref_points,
    Eigen::Matrix3d & head_state,
    Eigen::Matrix3d & tail_state,
    const Eigen::Vector3d * head_vel_override = nullptr) const;
  double segmentTime(
    const Eigen::Vector3d & prev, const Eigen::Vector3d & current,
    const Eigen::Vector3d & next) const;
  double anglePenalty(
    const Eigen::Vector3d & prev, const Eigen::Vector3d & current,
    const Eigen::Vector3d & next) const;
  nav_msgs::msg::Path sampleTrajectory(
    const nav_msgs::msg::Path & reference, const Trajectory<5> & traj) const;

  Options options_;
  plan_env::GridMap::Ptr esdf_map_;
};

}  // namespace sirb_smoother

#endif  // PB_NAV2_SMOOTHER__MINCO_OPTIMIZER_HPP_
