#include "f_mpc_controller/mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_util/geometry_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace f_mpc_controller
{
void MpcController::setPlan(const nav_msgs::msg::Path & path)
{
  // 接收 Nav2 全局路径并转换到控制坐标系，同时更新弧长、速度剖面和 warm start 策略。
  auto node = node_.lock();
  if (!node || path.poses.empty() || path.header.frame_id.empty()) return;

  const std::string global_frame = costmap_ros_->getGlobalFrameID();
  geometry_msgs::msg::TransformStamped map_to_odom;
  try {
    map_to_odom = tf_->lookupTransform(global_frame, path.header.frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) { return; }

  tf2::Transform tf2_transform;
  tf2::fromMsg(map_to_odom.transform, tf2_transform);

  nav_msgs::msg::Path tmp_plan_odom;
  std::vector<double> tmp_dist;
  tmp_plan_odom.poses.reserve(path.poses.size());
  tmp_dist.reserve(path.poses.size());

  double current_total_dist = 0.0;
  tmp_dist.push_back(0.0);
  for (size_t i = 0; i < path.poses.size(); ++i) {
    tf2::Transform pose_in, pose_out;
    tf2::fromMsg(path.poses[i].pose, pose_in);
    pose_out = tf2_transform * pose_in;
    geometry_msgs::msg::PoseStamped out_pose;
    out_pose.header.frame_id = global_frame;
    tf2::toMsg(pose_out, out_pose.pose);
    tmp_plan_odom.poses.push_back(out_pose);
    if (i > 0) {
      current_total_dist += nav2_util::geometry_utils::euclidean_distance(
        tmp_plan_odom.poses[i - 1], tmp_plan_odom.poses[i]);
      tmp_dist.push_back(current_total_dist);
    }
  }

  double angle_deg = 0.0;
  double new_dx = 0.0, new_dy = 0.0;
  bool found_lookahead = false;
  size_t lookahead_idx = 0;
  for (size_t i = 1; i < tmp_dist.size(); ++i) {
    if (tmp_dist[i] > 0.8) { lookahead_idx = i; found_lookahead = true; break; }
  }
  if (!found_lookahead) lookahead_idx = std::min<size_t>(5, tmp_plan_odom.poses.size() - 1);
  const bool has_lookahead = (lookahead_idx > 0);

  if (has_lookahead) {
    new_dx = tmp_plan_odom.poses[lookahead_idx].pose.position.x -
             tmp_plan_odom.poses[0].pose.position.x;
    new_dy = tmp_plan_odom.poses[lookahead_idx].pose.position.y -
             tmp_plan_odom.poses[0].pose.position.y;
    double norm = std::hypot(new_dx, new_dy);
    if (norm > 1e-3) {
      new_dx /= norm; new_dy /= norm;
    } else {
      lookahead_idx = 0;
    }
  }

  bool has_robot_tf = false;
  tf2::Transform base_to_global_tf;
  double robot_x = 0.0, robot_y = 0.0;
  try {
    auto ts = tf_->lookupTransform(global_frame, costmap_ros_->getBaseFrameID(), tf2::TimePointZero);
    tf2::fromMsg(ts.transform, base_to_global_tf);
    robot_x = base_to_global_tf.getOrigin().x();
    robot_y = base_to_global_tf.getOrigin().y();
    has_robot_tf = true;
  } catch (tf2::TransformException &) {}

  bool reject_plan = false;
  if (has_lookahead && lookahead_idx > 0 && has_robot_tf) {
    double ref_dir_x = 0.0, ref_dir_y = 0.0;
    double cur_v = std::hypot(last_ux, last_uy);
    if (cur_v > constants::kMinVelocityThreshold) {
      // last_ux/last_uy are in odom frame — already in global frame
      ref_dir_x = last_ux / cur_v;
      ref_dir_y = last_uy / cur_v;
    } else {
      double yaw = tf2::getYaw(base_to_global_tf.getRotation());
      ref_dir_x = std::cos(yaw); ref_dir_y = std::sin(yaw);
    }
    double dot = std::clamp(new_dx * ref_dir_x + new_dy * ref_dir_y, -1.0, 1.0);
    angle_deg = std::acos(dot) * 180.0 / M_PI;
    const auto & new_goal = tmp_plan_odom.poses.back().pose.position;
    double dist_to_goal_val = std::hypot(new_goal.x - robot_x, new_goal.y - robot_y);
    if (angle_deg > reject_angle_deg_ && dist_to_goal_val > reject_dist_threshold_) {
      reject_plan = true;
    }
  }

  if (reject_plan) {
    if (++consecutive_reject_count_ <= max_consecutive_rejects_) return;
  }
  consecutive_reject_count_ = 0;

  bool similar_plan = false;
  bool goal_changed = false;
  {
    std::lock_guard<std::mutex> lk(plan_mutex_);
    if (!global_plan_odom_.poses.empty() && !tmp_plan_odom.poses.empty()) {
      const auto & old_goal = global_plan_odom_.poses.back().pose.position;
      const auto & new_goal = tmp_plan_odom.poses.back().pose.position;
      goal_changed = std::hypot(new_goal.x - old_goal.x, new_goal.y - old_goal.y) >
        minco_goal_reset_tolerance_;
    }
    if (preserve_warm_start_on_similar_plan_ && !global_plan_odom_.poses.empty() &&
        !tmp_plan_odom.poses.empty() && has_robot_tf)
    {
      const auto & old_goal = global_plan_odom_.poses.back().pose.position;
      const auto & new_goal = tmp_plan_odom.poses.back().pose.position;
      const double goal_shift = std::hypot(new_goal.x - old_goal.x, new_goal.y - old_goal.y);

      auto nearest_dist = [](const nav_msgs::msg::Path & plan, double x, double y) {
        double best = std::numeric_limits<double>::max();
        for (const auto & ps : plan.poses) {
          const double dx = ps.pose.position.x - x;
          const double dy = ps.pose.position.y - y;
          best = std::min(best, std::hypot(dx, dy));
        }
        return best;
      };
      auto max_forward_deviation = [](const nav_msgs::msg::Path & old_plan,
                                      const nav_msgs::msg::Path & new_plan,
                                      double x, double y, double forward_dist) {
        auto nearest_index = [](const nav_msgs::msg::Path & plan, double px, double py) {
          size_t best_idx = 0;
          double best = std::numeric_limits<double>::max();
          for (size_t i = 0; i < plan.poses.size(); ++i) {
            const double dx = plan.poses[i].pose.position.x - px;
            const double dy = plan.poses[i].pose.position.y - py;
            const double d2 = dx * dx + dy * dy;
            if (d2 < best) {
              best = d2;
              best_idx = i;
            }
          }
          return best_idx;
        };

        if (old_plan.poses.empty() || new_plan.poses.empty()) {
          return std::numeric_limits<double>::infinity();
        }
        size_t idx = nearest_index(old_plan, x, y);
        double travelled = 0.0;
        double max_dev = 0.0;
        while (idx < old_plan.poses.size()) {
          const auto & p = old_plan.poses[idx].pose.position;
          double best = std::numeric_limits<double>::max();
          for (const auto & q_pose : new_plan.poses) {
            const auto & q = q_pose.pose.position;
            best = std::min(best, std::hypot(p.x - q.x, p.y - q.y));
          }
          max_dev = std::max(max_dev, best);
          if (idx + 1 >= old_plan.poses.size()) break;
          const auto & pn = old_plan.poses[idx + 1].pose.position;
          travelled += std::hypot(pn.x - p.x, pn.y - p.y);
          if (travelled >= forward_dist) break;
          ++idx;
        }
        return max_dev;
      };
      const double old_anchor_dist = nearest_dist(global_plan_odom_, robot_x, robot_y);
      const double new_anchor_dist = nearest_dist(tmp_plan_odom, robot_x, robot_y);
      const double forward_deviation = max_forward_deviation(
        global_plan_odom_, tmp_plan_odom, robot_x, robot_y, similar_plan_forward_distance_);
      similar_plan = goal_shift <= similar_plan_goal_tolerance_ &&
        old_anchor_dist <= similar_plan_anchor_tolerance_ &&
        new_anchor_dist <= similar_plan_anchor_tolerance_ &&
        forward_deviation <= similar_plan_max_deviation_;
    }

    global_plan_odom_ = std::move(tmp_plan_odom);
    path_accumulated_dist_ = std::move(tmp_dist);
    path_total_dist_ = path_accumulated_dist_.back();

    // precompute speed profile for arc-length fallback
    {
      const size_t N = global_plan_odom_.poses.size();
      path_speed_limit_.assign(N, v_ref_max_);
      if (N >= 2) {
        path_speed_limit_[N - 1] = 0.0;
        double a_brake = std::max(0.5, ax_max_ * brake_safety_factor_);
        for (int i = static_cast<int>(N) - 2; i >= 0; --i) {
          double ds = path_accumulated_dist_[i+1] - path_accumulated_dist_[i];
          double kappa = 0.0;
          if (i > 0 && i + 1 < static_cast<int>(N)) {
            const auto & poses = global_plan_odom_.poses;
            double dx1 = poses[i].pose.position.x - poses[i-1].pose.position.x;
            double dy1 = poses[i].pose.position.y - poses[i-1].pose.position.y;
            double dx2 = poses[i+1].pose.position.x - poses[i].pose.position.x;
            double dy2 = poses[i+1].pose.position.y - poses[i].pose.position.y;
            double d1 = std::hypot(dx1, dy1), d2 = std::hypot(dx2, dy2);
            if (d1 > 1e-6 && d2 > 1e-6) {
              double cross = std::abs(dx1 * dy2 - dy1 * dx2) / (d1 * d2);
              kappa = std::min(cross / (0.5 * (d1 + d2)), 100.0);
            }
          }
          double v_curv = v_ref_max_ / (1.0 + k_curvature_ * kappa);
          double v_decel = std::sqrt(path_speed_limit_[i+1] * path_speed_limit_[i+1] +
                                     2.0 * a_brake * ds);
          path_speed_limit_[i] = std::min({v_curv, v_decel, v_ref_max_});
        }
      }
    }

    try {
      auto ts = tf_->lookupTransform(global_frame, costmap_ros_->getBaseFrameID(), tf2::TimePointZero);
      double r_x = ts.transform.translation.x;
      double r_y = ts.transform.translation.y;
      double min_dist_sq = std::numeric_limits<double>::max();
      target_index_ = 0;
      for (size_t i = 0; i < global_plan_odom_.poses.size(); ++i) {
        double dx = global_plan_odom_.poses[i].pose.position.x - r_x;
        double dy = global_plan_odom_.poses[i].pose.position.y - r_y;
        double d_sq = dx * dx + dy * dy;
        if (d_sq < min_dist_sq) { min_dist_sq = d_sq; target_index_ = i; }
      }
    } catch (...) { target_index_ = 0; }

    last_target_index_ = target_index_;
    stuck_count_ = 0;
    if (reset_warm_start_on_goal_change_only_) {
      if (goal_changed) {
        mpc_->resetWarmStart();
      }
    } else if (!similar_plan) {
      mpc_->resetWarmStart();
    } else {
      RCLCPP_INFO_THROTTLE(node->get_logger(), *clock_, 1000,
        "setPlan: similar path accepted, preserving MPC warm start");
    }
    mpc_->setLastExecutedU(last_ux, last_uy);
  }  // plan_mutex_ released
}
double MpcController::computeLateralError(
  const geometry_msgs::msg::PoseStamped & pose, size_t idx) const
{
  // 计算机器人相对当前路径切线的横向误差，用于诊断和参考速度缩放。
  if (global_plan_odom_.poses.size() < 2) return 0.0;
  if (idx >= global_plan_odom_.poses.size()) return 0.0;

  size_t a = (idx + 1 < global_plan_odom_.poses.size()) ? idx : (idx > 0 ? idx - 1 : 0);
  size_t b = a + 1;
  if (b >= global_plan_odom_.poses.size()) return 0.0;

  double tx = global_plan_odom_.poses[b].pose.position.x -
              global_plan_odom_.poses[a].pose.position.x;
  double ty = global_plan_odom_.poses[b].pose.position.y -
              global_plan_odom_.poses[a].pose.position.y;
  double t_norm = std::hypot(tx, ty);
  if (t_norm < 1e-6) return 0.0;
  tx /= t_norm; ty /= t_norm;

  double ex = pose.pose.position.x - global_plan_odom_.poses[idx].pose.position.x;
  double ey = pose.pose.position.y - global_plan_odom_.poses[idx].pose.position.y;
  return std::abs(-ex * ty + ey * tx);
}
void MpcController::updateTargetIndex()
{
  // 在当前索引附近搜索最近路径点，限制目标索引异常回退并监测卡滞。
  size_t search_start = (target_index_ > 3) ? target_index_ - 3 : 0;
  size_t search_end = std::min(target_index_ + 80, global_plan_odom_.poses.size());
  double min_d_sq = std::numeric_limits<double>::max();
  size_t best_idx = target_index_;
  for (size_t i = search_start; i < search_end; i++) {
    double dx = pose_.pose.position.x - global_plan_odom_.poses[i].pose.position.x;
    double dy = pose_.pose.position.y - global_plan_odom_.poses[i].pose.position.y;
    double d_sq = dx * dx + dy * dy;
    if (d_sq < min_d_sq) { min_d_sq = d_sq; best_idx = i; }
  }

  if (best_idx + 5 < target_index_) {
    best_idx = target_index_;
  }
  target_index_ = best_idx;

  if (target_index_ == last_target_index_) {
    stuck_count_++;
  } else {
    stuck_count_ = 0;
  }
  last_target_index_ = target_index_;

  if (stuck_count_ > stuck_threshold_frames_) {
    double e_lat = computeLateralError(pose_, target_index_);
    if (e_lat > stuck_lateral_threshold_) {
      auto _node = node_.lock();
      if (_node) {
        RCLCPP_WARN_THROTTLE(_node->get_logger(), *clock_, 2000,
          "Target index stuck for %d frames with lateral error %.2fm",
          stuck_count_, e_lat);
      }
    }
  }
}

}  // namespace f_mpc_controller
