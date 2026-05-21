#include "f_mpc_controller/mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace f_mpc_controller
{
double MpcController::computeDistanceToGoal(const geometry_msgs::msg::PoseStamped & current_pose){
  if (global_plan_odom_.poses.empty()) return 0.0;
  const auto & goal = global_plan_odom_.poses.back().pose.position;
  return std::hypot(goal.x - current_pose.pose.position.x,
                    goal.y - current_pose.pose.position.y);
}
double MpcController::computeReferenceTimeScale() const
{
  // 根据横向误差缩放参考时间推进速度，偏离路径越大追踪越保守。
  if (!enable_lateral_error_ref_scaling_ ||
      lateral_error_high_threshold_ <= lateral_error_slow_threshold_)
  {
    return 1.0;
  }

  const double e_lat = computeLateralError(pose_, target_index_);
  const double ratio = std::clamp(
    (e_lat - lateral_error_slow_threshold_) /
    (lateral_error_high_threshold_ - lateral_error_slow_threshold_),
    0.0, 1.0);
  const double scale = 1.0 - ratio * (1.0 - min_lateral_ref_time_scale_);
  return std::clamp(scale, min_lateral_ref_time_scale_, 1.0);
}
bool MpcController::sampleMincoReference(
  double r_x, double r_y, double ref_time_scale)
{
  // 优先从当前有效 MINCO 轨迹采样 MPC 参考点，无法执行时交给路径 fallback。
  MincoReferenceContext ctx;
  if (!loadActiveMincoReference(ctx) || !resolveMincoFrame(ctx)) {
    return false;
  }

  if (auto node = node_.lock()) {
    RCLCPP_INFO_THROTTLE(node->get_logger(), *clock_, 1000,
      "generateRef: using MINCO branch (msg_age=%.2fs, exec=%.2fs, dur=%.2fs)",
      ctx.msg_age, ctx.exec_time, ctx.traj_dur);
  }

  const double t_proj = updateMincoProjection(ctx, r_x, r_y);

  if (t_proj >= ctx.traj_dur - 2.0 * control_dt_) {
    fillTerminalMincoReference(ctx);
    return true;
  }

  sampleMincoHorizon(ctx, t_proj, ref_time_scale);
  return true;
}

bool MpcController::loadActiveMincoReference(MincoReferenceContext & ctx)
{
  // 在轨迹锁内复制当前可执行 MINCO，并按时间窗口过滤过早或过期轨迹。
  auto node = node_.lock();
  {
    std::lock_guard<std::mutex> lk(traj_mutex_);
    if (!has_minco_traj_) {
      return false;
    }

    const rclcpp::Time now = clock_->now();
    try {
      ctx.msg_age = (now - minco_first_receive_stamp_).seconds();
    } catch (const std::runtime_error &) {
      ctx.msg_age = 0.0;
    }
    ctx.exec_time = ctx.msg_age + minco_time_base_offset_sec_;
    const bool too_far_before_start = ctx.exec_time < -minco_timeout_sec_;
    const bool past_finished_trajectory =
      traj_duration_ > 0.0 && ctx.exec_time > traj_duration_ + minco_timeout_sec_;
    if (too_far_before_start || past_finished_trajectory) {
      has_minco_traj_ = false;
      if (node) {
        RCLCPP_INFO_THROTTLE(node->get_logger(), *clock_, 1000,
          "generateRef: MINCO rejected — outside active time window "
          "(exec=%.2fs dur=%.2fs msg_age=%.2fs grace=%.2fs)",
          ctx.exec_time, traj_duration_, ctx.msg_age, minco_timeout_sec_);
      }
      return false;
    }

    ctx.traj = minco_traj_;
    ctx.traj_dur = traj_duration_;
    ctx.start_offset_sec = minco_time_base_offset_sec_;
    ctx.traj_id = minco_trajectory_id_;
    ctx.frame = minco_traj_frame_;
  }
  return true;
}

bool MpcController::resolveMincoFrame(MincoReferenceContext & ctx)
{
  // 将 MINCO 轨迹所在坐标系转换到控制器全局坐标系，保证参考和状态同框架。
  auto node = node_.lock();
  const std::string global_frame = costmap_ros_->getGlobalFrameID();
  try {
    auto ts = tf_->lookupTransform(global_frame, ctx.frame, tf2::TimePointZero);
    tf2::fromMsg(ts.transform, ctx.map_to_odom);
  } catch (tf2::TransformException & ex) {
    if (node) {
      RCLCPP_INFO_THROTTLE(node->get_logger(), *clock_, 1000,
        "generateRef: MINCO rejected — TF failed: %s", ex.what());
    }
    return false;
  }
  return true;
}

double MpcController::updateMincoProjection(
  MincoReferenceContext & ctx,
  double r_x,
  double r_y)
{
  // 在允许的时间窗口内搜索机器人到轨迹的最近投影，并限制投影点单周期跳变。
  auto node = node_.lock();
  double t_proj = 0.0;
  const rclcpp::Time now = clock_->now();
  const double time_seed = std::clamp(ctx.msg_age + ctx.start_offset_sec, 0.0, ctx.traj_dur);
  const bool new_traj = ctx.traj_id != 0 && ctx.traj_id != last_sampled_trajectory_id_;
  if (new_traj) {
    last_sampled_trajectory_id_ = ctx.traj_id;
    last_t_proj_ = 0.0;
    last_projection_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
  double elapsed = control_dt_;
  if (!new_traj && last_projection_update_time_.nanoseconds() != 0) {
    try {
      elapsed = std::clamp((now - last_projection_update_time_).seconds(), 0.0, 0.5);
    } catch (const std::runtime_error &) {
      elapsed = control_dt_;
    }
  }
  const double max_advance = std::max(
    minco_projection_max_advance_sec_, elapsed + 2.0 * control_dt_);
  const double min_time_from_exec =
    std::clamp(time_seed - std::max(0.0, minco_projection_max_lag_sec_), 0.0, ctx.traj_dur);
  const double seed_center = std::clamp(
    std::max(last_t_proj_, min_time_from_exec), 0.0, ctx.traj_dur);
  const double search_lo = std::clamp(
    std::max(
      seed_center - (new_traj ? std::max(cmd_lookahead_sec_, control_dt_) : 0.0),
      min_time_from_exec),
    0.0, ctx.traj_dur);
  const double search_hi = std::min(
    ctx.traj_dur,
    std::max(time_seed + minco_projection_search_ahead_sec_,
      seed_center + std::max(minco_projection_search_ahead_sec_, max_advance)));
  auto dist2_at = [&](double t) {
    Eigen::Vector3d p = ctx.traj.getPos(t);
    tf2::Vector3 po = ctx.map_to_odom * tf2::Vector3(p.x(), p.y(), p.z());
    return (po.x() - r_x) * (po.x() - r_x) + (po.y() - r_y) * (po.y() - r_y);
  };
  const int N_coarse = 10;
  double best_dist2 = std::numeric_limits<double>::max();
  double best_t = search_lo;
  for (int s = 0; s <= N_coarse; ++s) {
    double t_s = search_lo + s * (search_hi - search_lo) / N_coarse;
    double d2 = dist2_at(t_s);
    if (d2 < best_dist2) {
      best_dist2 = d2;
      best_t = t_s;
    }
  }
  double lo = std::max(search_lo, best_t - 0.1);
  double hi = std::min(search_hi, best_t + 0.1);
  for (int iter = 0; iter < 20; ++iter) {
    double m1 = lo + (hi - lo) * 0.382;
    double m2 = lo + (hi - lo) * 0.618;
    if (dist2_at(m1) < dist2_at(m2)) {
      hi = m2;
    } else {
      lo = m1;
    }
  }
  t_proj = std::clamp((lo + hi) * 0.5, search_lo, search_hi);
  const double lower_bound = new_traj ? search_lo : std::max(last_t_proj_, min_time_from_exec);
  const double upper_bound = new_traj ?
    search_hi : std::min(ctx.traj_dur, std::max(time_seed, last_t_proj_ + max_advance));
  last_t_proj_ = std::clamp(t_proj, lower_bound, upper_bound);
  last_projection_update_time_ = now;
  t_proj = last_t_proj_;
  if (node) {
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *clock_, 1000,
      "generateRef: MINCO projection id=%lu t=%.2f seed=%.2f dist=%.2f new=%d "
      "search=[%.2f, %.2f] adv=%.2f",
      static_cast<unsigned long>(ctx.traj_id), t_proj, time_seed, std::sqrt(best_dist2),
      new_traj ? 1 : 0, search_lo, search_hi, max_advance);
  }
  return t_proj;
}

void MpcController::fillTerminalMincoReference(const MincoReferenceContext & ctx)
{
  // 轨迹接近结束时将整个预测窗口固定到终点，并把参考速度置零。
  Eigen::Vector3d goal_pos = ctx.traj.getPos(ctx.traj_dur);
  tf2::Vector3 goal_odom = ctx.map_to_odom *
    tf2::Vector3(goal_pos.x(), goal_pos.y(), goal_pos.z());
  horizon_speed_limits_.assign(horizon_, 0.0);
  for (int i = 0; i < horizon_; ++i) {
    ref.push_back({goal_odom.x(), goal_odom.y()});
    v_ref_.push_back({0.0, 0.0});
  }
}

void MpcController::sampleMincoHorizon(
  const MincoReferenceContext & ctx,
  double t_proj,
  double ref_time_scale)
{
  // 沿 MINCO 轨迹按缩放后的控制周期采样位置和速度，同时叠加刹停与曲率速度限制。
  const double time_scale = std::clamp(ref_time_scale, 0.05, 1.0);
  last_ref_time_scale_ = time_scale;
  last_min_curvature_speed_limit_ = v_ref_max_effective_;
  const double t_cur = std::min(t_proj + cmd_lookahead_sec_, ctx.traj_dur);
  const double a_brake = std::max(0.5, ax_max_ * brake_safety_factor_);
  const double a_lat = std::max(0.1, lateral_accel_limit_);
  horizon_speed_limits_.resize(horizon_);

  std::vector<double> arc_times;
  std::vector<double> arc_remaining;
  {
    const double ds_dt = std::max(control_dt_, 0.05);
    for (double t = t_proj; t < ctx.traj_dur; t += ds_dt) {
      arc_times.push_back(std::clamp(t, 0.0, ctx.traj_dur));
    }
    if (arc_times.empty() || arc_times.back() < ctx.traj_dur) {
      arc_times.push_back(ctx.traj_dur);
    }
    arc_remaining.assign(arc_times.size(), 0.0);
    for (int idx = static_cast<int>(arc_times.size()) - 2; idx >= 0; --idx) {
      const auto p0 = ctx.traj.getPos(arc_times[static_cast<size_t>(idx)]);
      const auto p1 = ctx.traj.getPos(arc_times[static_cast<size_t>(idx + 1)]);
      arc_remaining[static_cast<size_t>(idx)] =
        arc_remaining[static_cast<size_t>(idx + 1)] + (p1 - p0).head<2>().norm();
    }
    current_s_ = 0.0;
    path_total_dist_ = arc_remaining.empty() ? 0.0 : arc_remaining.front();
  }
  auto remainingDistanceAt = [&](double t) {
      if (arc_times.empty()) {
        return 0.0;
      }
      t = std::clamp(t, arc_times.front(), arc_times.back());
      auto it = std::lower_bound(arc_times.begin(), arc_times.end(), t);
      if (it == arc_times.begin()) {
        return arc_remaining.front();
      }
      if (it == arc_times.end()) {
        return 0.0;
      }
      const size_t hi = static_cast<size_t>(std::distance(arc_times.begin(), it));
      const size_t lo = hi - 1;
      const double span = std::max(arc_times[hi] - arc_times[lo], 1.0e-6);
      const double ratio = std::clamp((t - arc_times[lo]) / span, 0.0, 1.0);
      return arc_remaining[lo] +
        (arc_remaining[hi] - arc_remaining[lo]) * ratio;
    };

  for (int i = 0; i < horizon_; ++i) {
    double t_q = std::min(
      t_cur + static_cast<double>(i + 1) * control_dt_ * time_scale,
      ctx.traj_dur);
    Eigen::Vector3d pos = ctx.traj.getPos(t_q);
    tf2::Vector3 pos_odom = ctx.map_to_odom * tf2::Vector3(pos.x(), pos.y(), pos.z());
    ref.push_back({pos_odom.x(), pos_odom.y()});

    if (t_q >= ctx.traj_dur) {
      v_ref_.push_back({0.0, 0.0});
      horizon_speed_limits_[i] = 0.0;
    } else {
      Eigen::Vector3d vel = ctx.traj.getVel(t_q);
      tf2::Vector3 vel_odom = ctx.map_to_odom.getBasis() *
        tf2::Vector3(vel.x(), vel.y(), vel.z());
      Eigen::Vector3d acc = ctx.traj.getAcc(t_q);
      tf2::Vector3 acc_odom = ctx.map_to_odom.getBasis() *
        tf2::Vector3(acc.x(), acc.y(), acc.z());
      double remaining = std::max(0.0, remainingDistanceAt(t_q));
      double v_brake = std::sqrt(2.0 * a_brake * remaining);
      double v_mag = std::hypot(vel_odom.x(), vel_odom.y());
      double v_curv = v_ref_max_effective_;
      if (curvature_speed_limit_enabled_ && v_mag > curvature_speed_limit_min_speed_) {
        const double cross =
          std::abs(vel_odom.x() * acc_odom.y() - vel_odom.y() * acc_odom.x());
        const double kappa = cross / std::max(v_mag * v_mag * v_mag, 1.0e-6);
        if (std::isfinite(kappa) && kappa > 1.0e-6) {
          v_curv = std::sqrt(a_lat / kappa);
        }
      }
      last_min_curvature_speed_limit_ = std::min(last_min_curvature_speed_limit_, v_curv);
      double v_track = std::min(v_mag, v_ref_max_effective_);
      double v_eff = std::min({v_brake, v_track, v_curv}) * time_scale;
      double scale = (v_mag > v_eff && v_mag > 1e-6) ? v_eff / v_mag : 1.0;
      v_ref_.push_back({vel_odom.x() * scale, vel_odom.y() * scale});
      horizon_speed_limits_[i] = std::min(v_ref_max_effective_, v_eff + ax_max_ * control_dt_);
    }
  }
}
void MpcController::samplePathFallback(double r_x, double r_y, double ref_time_scale)
{
  // 在没有可执行 MINCO 时沿全局路径按弧长采样，主要用于预览或允许 fallback 的场景。
  auto node = node_.lock();
  const double time_scale = std::clamp(ref_time_scale, 0.05, 1.0);
  last_ref_time_scale_ = time_scale;

  if (node) RCLCPP_INFO_THROTTLE(node->get_logger(), *clock_, 1000,
    "generateRef: using arc-length fallback (path_pts=%zu)", global_plan_odom_.poses.size());

  double start_dist = path_accumulated_dist_[target_index_];
  if (target_index_ + 1 < global_plan_odom_.poses.size()) {
    double ax = global_plan_odom_.poses[target_index_].pose.position.x;
    double ay = global_plan_odom_.poses[target_index_].pose.position.y;
    double bx = global_plan_odom_.poses[target_index_ + 1].pose.position.x;
    double by = global_plan_odom_.poses[target_index_ + 1].pose.position.y;
    double seg_dx = bx - ax, seg_dy = by - ay;
    double seg_len2 = seg_dx * seg_dx + seg_dy * seg_dy;
    if (seg_len2 > 1e-9) {
      double t = ((r_x - ax) * seg_dx + (r_y - ay) * seg_dy) / seg_len2;
      t = std::clamp(t, 0.0, 1.0);
      start_dist += t * std::sqrt(seg_len2);
    }
  }
  current_s_ = std::clamp(start_dist, 0.0, path_total_dist_);

  size_t plan_idx = target_index_;
  const double goal_x = global_plan_odom_.poses.back().pose.position.x;
  const double goal_y = global_plan_odom_.poses.back().pose.position.y;

  double s_cur = start_dist;
  horizon_speed_limits_.resize(horizon_);

  for (int i = 0; i < horizon_; ++i) {
    size_t spidx_cur = std::min(plan_idx, path_speed_limit_.empty() ?
      size_t(0) : path_speed_limit_.size() - 1);
    double step_v = path_speed_limit_.empty() ? v_ref_max_ : path_speed_limit_[spidx_cur];
    const double step_min_v = constants::kMinStepSize / std::max(control_dt_, 1.0e-3);
    const double fallback_min_v = std::min(fallback_sample_speed_, step_min_v);
    step_v = std::clamp(step_v, fallback_min_v, v_ref_max_);
    const double step_i = std::max(step_v * control_dt_ * time_scale, constants::kMinStepSize);

    double target_dist = s_cur + step_i;
    s_cur = target_dist;

    while (plan_idx < global_plan_odom_.poses.size() - 1 &&
           path_accumulated_dist_[plan_idx + 1] < target_dist) {
      plan_idx++;
    }

    bool past_end = target_dist >= path_total_dist_;
    double gx = goal_x;
    double gy = goal_y;

    if (past_end) {
      horizon_speed_limits_[i] = 0.0;
      ref.push_back({gx, gy});
      v_ref_.push_back({0.0, 0.0});
      continue;
    } else if (plan_idx < global_plan_odom_.poses.size() - 1) {
      double d = path_accumulated_dist_[plan_idx + 1] - path_accumulated_dist_[plan_idx];
      double ratio = (target_dist - path_accumulated_dist_[plan_idx]) / std::max(0.001, d);
      ratio = std::clamp(ratio, 0.0, 1.0);
      gx = global_plan_odom_.poses[plan_idx].pose.position.x +
           ratio * (global_plan_odom_.poses[plan_idx + 1].pose.position.x -
                    global_plan_odom_.poses[plan_idx].pose.position.x);
      gy = global_plan_odom_.poses[plan_idx].pose.position.y +
           ratio * (global_plan_odom_.poses[plan_idx + 1].pose.position.y -
                    global_plan_odom_.poses[plan_idx].pose.position.y);
    }

    ref.push_back({gx, gy});

    size_t spidx = std::min(plan_idx, path_speed_limit_.empty() ?
      size_t(0) : path_speed_limit_.size() - 1);
    double v_ref_val = path_speed_limit_.empty() ? v_ref_max_effective_ : path_speed_limit_[spidx];
    double dist_to_goal = path_total_dist_ - target_dist;
    double a_brake = std::max(0.5, ax_max_ * brake_safety_factor_);
    v_ref_val = std::min(v_ref_val, std::sqrt(2.0 * a_brake * std::max(dist_to_goal, 0.0)));
    v_ref_val = std::min(v_ref_val, v_ref_max_effective_) * time_scale;
    if (enable_goal_slowdown_ && dist_to_goal < goal_slowdown_distance_) {
      v_ref_val *= std::max(0.05, dist_to_goal / goal_slowdown_distance_);
    }

    horizon_speed_limits_[i] = v_ref_val;

    double tx = 0.0, ty = 0.0;
    const auto & poses = global_plan_odom_.poses;
    if (plan_idx + 1 < poses.size()) {
      double tdx = poses[plan_idx+1].pose.position.x - poses[plan_idx].pose.position.x;
      double tdy = poses[plan_idx+1].pose.position.y - poses[plan_idx].pose.position.y;
      double tn = std::hypot(tdx, tdy);
      if (tn > 1e-6) { tx = tdx / tn; ty = tdy / tn; }
    }
    v_ref_.push_back({v_ref_val * tx, v_ref_val * ty});
  }

  if (!ref.empty()) {
    double max_ref_dist = 0.0;
    for (const auto & p : ref)
      max_ref_dist = std::max(max_ref_dist, std::hypot(p.x - r_x, p.y - r_y));
    if (max_ref_dist < 0.1 && computeDistanceToGoal(pose_) > 0.5) {
      if (node) RCLCPP_WARN_THROTTLE(node->get_logger(), *clock_, 1000,
        "All ref points within 0.1m of robot but goal is %.2fm away "
        "(target_idx=%zu, path_size=%zu)",
        computeDistanceToGoal(pose_), target_index_,
        global_plan_odom_.poses.size());
    }
  }

}
void MpcController::enforceNoReverseTrackingCommand(
  geometry_msgs::msg::TwistStamped & cmd,
  double r_x, double r_y, double cp, double sp)
{
  // 移除与参考前进方向相反的速度分量，避免追踪控制主动倒车。
  if (!prevent_tracking_reverse_ || ref.empty()) {
    return;
  }

  double dir_x = 0.0;
  double dir_y = 0.0;
  double ref_speed = 0.0;
  if (!v_ref_.empty()) {
    ref_speed = std::hypot(v_ref_.front().x, v_ref_.front().y);
    if (ref_speed > reverse_guard_min_ref_speed_) {
      dir_x = v_ref_.front().x / ref_speed;
      dir_y = v_ref_.front().y / ref_speed;
    }
  }
  if (std::hypot(dir_x, dir_y) < 1.0e-6) {
    const double dx = ref.front().x - r_x;
    const double dy = ref.front().y - r_y;
    const double d = std::hypot(dx, dy);
    if (d < 1.0e-3) {
      return;
    }
    dir_x = dx / d;
    dir_y = dy / d;
  }

  const double cmd_odom_x = cp * cmd.twist.linear.x - sp * cmd.twist.linear.y;
  const double cmd_odom_y = sp * cmd.twist.linear.x + cp * cmd.twist.linear.y;
  const double reverse_component = cmd_odom_x * dir_x + cmd_odom_y * dir_y;
  const double allowance = ref_speed > reverse_guard_min_ref_speed_ ?
    0.0 : std::max(0.0, reverse_guard_allowance_);
  if (reverse_component >= -allowance) {
    return;
  }

  const double corrected_odom_x = cmd_odom_x - reverse_component * dir_x;
  const double corrected_odom_y = cmd_odom_y - reverse_component * dir_y;
  cmd.twist.linear.x = cp * corrected_odom_x + sp * corrected_odom_y;
  cmd.twist.linear.y = -sp * corrected_odom_x + cp * corrected_odom_y;

  if (auto node = node_.lock()) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *clock_, 500,
      "MPC reverse guard removed %.3fm/s against reference direction", -reverse_component);
  }
}
void MpcController::generateReferenceTrajectory(const tf2::Transform & base_to_odom_tf)
{
  // 每个控制周期先尝试 MINCO 参考，失败后按配置进入等待或路径 fallback。
  ref.clear();
  ref.reserve(horizon_);
  v_ref_.clear();
  v_ref_.reserve(horizon_);
  reference_waiting_for_minco_ = false;
  reference_uses_minco_ = false;

  auto node = node_.lock();
  double r_x = base_to_odom_tf.getOrigin().x();
  double r_y = base_to_odom_tf.getOrigin().y();
  double ref_time_scale = computeReferenceTimeScale();
  last_ref_time_scale_ = std::clamp(ref_time_scale, 0.05, 1.0);
  last_min_curvature_speed_limit_ = v_ref_max_effective_;

  if (sampleMincoReference(r_x, r_y, ref_time_scale)) {
    reference_uses_minco_ = true;
    return;
  }

  if (!allow_path_fallback_without_minco_) {
    reference_waiting_for_minco_ = true;
    samplePathFallback(r_x, r_y, ref_time_scale);
    horizon_speed_limits_.assign(horizon_, 0.0);
    for (auto & v : v_ref_) {
      v.x = 0.0;
      v.y = 0.0;
    }
    if (node) RCLCPP_WARN_THROTTLE(node->get_logger(), *clock_, 1000,
      "generateRef: waiting for MINCO trajectory; publishing fallback preview only");
    return;
  }

  samplePathFallback(r_x, r_y, ref_time_scale);
}

}  // namespace f_mpc_controller
