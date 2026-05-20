#include "f_mpc_controller/mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>

namespace f_mpc_controller
{
std::vector<ObstacleConstraint> MpcController::buildObstacleConstraints(
  const State & current, const std::vector<State> & ref_traj)
{
  // 汇总动态障碍约束，并补齐固定槽位以匹配 MPC 矩阵结构。
  std::vector<ObstacleConstraint> result;
  const int total_slots = max_dynamic_obs_;

  if (ref_traj.empty()) {
    result.resize(total_slots);
    for (auto & o : result) { o.centers.resize(horizon_); o.radii.assign(horizon_, 0.0); }
    return result;
  }

  appendDynamicObstacleConstraints(current, result);

  while ((int)result.size() < total_slots) {
    ObstacleConstraint inactive;
    inactive.centers.resize(horizon_, Eigen::Vector2d::Zero());
    inactive.radii.assign(horizon_, 0.0);
    result.push_back(inactive);
  }
  result.resize(static_cast<size_t>(total_slots));
  return result;
}
void MpcController::appendDynamicObstacleConstraints(
  const State & current, std::vector<ObstacleConstraint> & result)
{
  // 选择距离机器人最近的动态障碍，并按预测时间插值生成 horizon 内约束。
  if (!enable_dynamic_obstacle_avoidance_) {
    return;
  }

  std::vector<sentry_nav_interfaces::msg::TrackedObstacle> local_obs;
  {
    std::lock_guard<std::mutex> lk(obs_mutex_);
    local_obs = dynamic_obstacles_;
  }
  std::sort(local_obs.begin(), local_obs.end(),
    [&](const auto & a, const auto & b) {
      return std::hypot(a.x - current.x, a.y - current.y) <
             std::hypot(b.x - current.x, b.y - current.y);
    });
  if ((int)local_obs.size() > max_dynamic_obs_)
    local_obs.resize(static_cast<size_t>(max_dynamic_obs_));

  for (const auto & obs : local_obs) {
    ObstacleConstraint dyn;
    dyn.centers.resize(horizon_);
    dyn.radii.resize(horizon_, obs.radius + robot_radius_ + dynamic_safety_margin_);
    const double pred_dt = std::max(static_cast<double>(obs.prediction_dt), 1e-3);
    for (int k = 0; k < horizon_; ++k) {
      if (obs.predicted_positions.empty()) {
        dyn.centers[k] = Eigen::Vector2d(obs.x, obs.y);
        continue;
      }
      double t_k = (k + 1) * control_dt_;
      double idx_f = t_k / pred_dt;
      int idx_lo = std::clamp(static_cast<int>(std::floor(idx_f)), 0,
                              static_cast<int>(obs.predicted_positions.size()) - 1);
      int idx_hi = std::clamp(idx_lo + 1, 0,
                              static_cast<int>(obs.predicted_positions.size()) - 1);
      double alpha = idx_f - std::floor(idx_f);
      dyn.centers[k] = Eigen::Vector2d(
        (1.0-alpha)*obs.predicted_positions[idx_lo].x + alpha*obs.predicted_positions[idx_hi].x,
        (1.0-alpha)*obs.predicted_positions[idx_lo].y + alpha*obs.predicted_positions[idx_hi].y);
    }
    result.push_back(std::move(dyn));
  }
}

}  // namespace f_mpc_controller
