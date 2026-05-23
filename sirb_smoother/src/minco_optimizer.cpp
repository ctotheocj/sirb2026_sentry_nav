#include "sirb_smoother/minco_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"

namespace sirb_smoother
{

MincoOptimizer::MincoOptimizer()
: options_(Options())
{
}

MincoOptimizer::MincoOptimizer(const Options & options)
: options_(options)
{
}

void MincoOptimizer::setOptions(const Options & options)
{
  options_ = options;
}

void MincoOptimizer::setEsdf(plan_env::GridMap::Ptr esdf)
{
  esdf_map_ = esdf;
}

MincoOptimizer::Result MincoOptimizer::smooth(
  const nav_msgs::msg::Path & input, nav_msgs::msg::Path & output,
  const nav2_costmap_2d::Costmap2D * costmap,
  const std::vector<DynamicObstacle> * dynamic_obstacles,
  const std::function<bool()> * should_cancel,
  const Eigen::Vector3d * initial_velocity,
  const Eigen::Vector3d * initial_acceleration,
  const Eigen::Vector3d * terminal_velocity,
  const Eigen::Vector3d * terminal_acceleration) const
{
  Result result;
  output = input;
  if (!options_.enabled || input.poses.size() < 2) {
    result.reason = "disabled or too few poses";
    return result;
  }

  GuidePath guide;
  if (!buildGuidePath(input, guide)) {
    result.reason = "buildGuidePath failed";
    return result;
  }

  std::vector<Eigen::Vector3d> points;
  Eigen::VectorXd times;
  if (!buildWaypoints(guide, points, times, initial_velocity, terminal_velocity) ||
    points.size() < 2)
  {
    result.reason = "buildWaypoints failed";
    return result;
  }

  result.input_waypoints = points;
  if (should_cancel && (*should_cancel)()) {
    result.reason = "MINCO optimization canceled before start";
    return result;
  }

  if (options_.use_lbfgs && points.size() > 2) {
    result.used_lbfgs = true;

    // 粗优化以参考路径为主，先得到稳定形状。
    if (options_.phase0_enabled) {
      double phase0_cost = 0.0;
      optimizeWaypoints(
        points, guide, times, costmap, dynamic_obstacles,
        options_.phase0_w_energy, options_.phase0_w_reference, options_.phase0_w_obstacle,
        options_.phase0_max_iterations, phase0_cost, false, should_cancel,
        initial_velocity, initial_acceleration, terminal_velocity, terminal_acceleration);
      if (should_cancel && (*should_cancel)()) {
        result.reason = "MINCO optimization canceled during PHASE0";
        return result;
      }

      // 粗优化后按新几何形状重新分配时间。
      assignSegmentTimes(points, times, initial_velocity, terminal_velocity);
      smoothSegmentTimes(times);
    }

    // 预优化强跟随参考路径，避免狭窄区域被过早推出。
    result.pre_ret = optimizeWaypoints(
      points, guide, times, costmap, dynamic_obstacles,
      options_.pre_w_energy, options_.pre_w_reference, options_.pre_w_obstacle,
      options_.pre_max_iterations, result.pre_cost, false, should_cancel,
      initial_velocity, initial_acceleration, terminal_velocity, terminal_acceleration);
    if (should_cancel && (*should_cancel)()) {
      result.reason = "MINCO optimization canceled during PRE";
      return result;
    }

    const bool pre_ok = result.pre_ret >= 0 ||
      result.pre_ret == lbfgs::LBFGS_CANCELED ||
      result.pre_ret == lbfgs::LBFGSERR_MAXIMUMITERATION ||
      result.pre_ret == lbfgs::LBFGSERR_MAXIMUMLINESEARCH ||
      result.pre_ret == lbfgs::LBFGSERR_MINIMUMSTEP ||
      result.pre_ret == lbfgs::LBFGSERR_MAXIMUMSTEP ||
      result.pre_ret == lbfgs::LBFGSERR_WIDTHTOOSMALL;
    if (!pre_ok) {
      result.reason = "PRE optimization failed (lbfgs_ret=" +
        std::to_string(result.pre_ret) + ")";
      return result;
    }

    assignSegmentTimes(points, times, initial_velocity, terminal_velocity);
    smoothSegmentTimes(times);

    // 精优化强化动力学平滑并联合调整时间。
    std::vector<Eigen::Vector3d> fine_points = points;
    result.fine_ret = optimizeWaypoints(
      fine_points, guide, times, costmap, dynamic_obstacles,
      options_.fine_w_energy, options_.fine_w_reference, options_.fine_w_obstacle,
      options_.fine_max_iterations, result.fine_cost, true, should_cancel,
      initial_velocity, initial_acceleration, terminal_velocity, terminal_acceleration);
    if (should_cancel && (*should_cancel)()) {
      result.reason = "MINCO optimization canceled during FINE";
      return result;
    }

    const bool fine_ok = result.fine_ret >= 0 ||
      result.fine_ret == lbfgs::LBFGS_CANCELED ||
      result.fine_ret == lbfgs::LBFGSERR_MAXIMUMITERATION ||
      result.fine_ret == lbfgs::LBFGSERR_MAXIMUMLINESEARCH ||
      result.fine_ret == lbfgs::LBFGSERR_MINIMUMSTEP ||
      result.fine_ret == lbfgs::LBFGSERR_MAXIMUMSTEP ||
      result.fine_ret == lbfgs::LBFGSERR_WIDTHTOOSMALL;
    if (fine_ok) {
      points = fine_points;
      result.final_cost = result.fine_cost;
    } else {
      result.final_cost = result.pre_cost;
    }
  }

  result.optimized_waypoints = points;
  result.optimized_times = times;

  Trajectory<5> traj;
  if (!buildTrajectory(
      points, times, traj, initial_velocity, initial_acceleration,
      terminal_velocity, terminal_acceleration))
  {
    result.reason = "buildTrajectory failed";
    return result;
  }
  std::string dyn_diag;
  if (!enforceDynamicFeasibility(
      points, times, traj, result.max_velocity, result.max_acceleration, &dyn_diag,
      initial_velocity, initial_acceleration, terminal_velocity, terminal_acceleration))
  {
    result.reason = "dynamic feasibility reallocation failed: " + dyn_diag;
    return result;
  }
  result.optimized_times = times;
  result.traj_duration = traj.getTotalDuration();
  result.initial_velocity = traj.getVel(0.0);
  result.initial_acceleration = traj.getAcc(0.0);
  const double duration = traj.getTotalDuration();
  result.terminal_velocity = traj.getVel(duration);
  result.terminal_acceleration = traj.getAcc(duration);
  result.terminal_stop =
    result.terminal_velocity.head<2>().norm() < 1.0e-3 &&
    result.terminal_acceleration.head<2>().norm() < 1.0e-3;

  output = sampleTrajectory(input, traj);
  if (output.poses.size() < 2) {
    result.reason = "sampleTrajectory produced too few poses";
    return result;
  }

  result.success = true;
  return result;
}

bool MincoOptimizer::buildGuidePath(const nav_msgs::msg::Path & path, GuidePath & guide) const
{
  guide = GuidePath{};
  if (path.poses.size() < 2) {return false;}

  std::vector<Eigen::Vector3d> raw;
  raw.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    const auto & p = pose.pose.position;
    Eigen::Vector3d point(p.x, p.y, p.z);
    if (raw.empty() || (point - raw.back()).norm() > 1.0e-5) {
      raw.push_back(point);
    }
  }

  if (raw.size() < 2) {return false;}

  auto append_point = [&](const Eigen::Vector3d & point) {
      if (guide.points.empty() || (point - guide.points.back()).norm() > 1.0e-5) {
        guide.points.push_back(point);
      }
    };

  if (!options_.guide_fillet_enabled || raw.size() < 3 ||
    options_.guide_fillet_radius <= 1.0e-3)
  {
    guide.points = raw;
  } else {
    guide.points.reserve(raw.size() * 3);
    append_point(raw.front());
    for (size_t i = 1; i + 1 < raw.size(); ++i) {
      const Eigen::Vector3d prev = raw[i - 1];
      const Eigen::Vector3d cur = raw[i];
      const Eigen::Vector3d next = raw[i + 1];
      const Eigen::Vector3d in_vec = cur - prev;
      const Eigen::Vector3d out_vec = next - cur;
      const double len_in = in_vec.norm();
      const double len_out = out_vec.norm();
      if (len_in < 1.0e-4 || len_out < 1.0e-4) {
        append_point(cur);
        continue;
      }

      const Eigen::Vector3d d_in = in_vec / len_in;
      const Eigen::Vector3d d_out = out_vec / len_out;
      const double turn = std::acos(std::clamp(d_in.dot(d_out), -1.0, 1.0));
      if (turn < options_.guide_fillet_min_angle || std::abs(M_PI - turn) < 1.0e-3) {
        append_point(cur);
        continue;
      }

      const double trim_by_radius = options_.guide_fillet_radius * std::tan(0.5 * turn);
      const double trim = std::min({trim_by_radius, 0.45 * len_in, 0.45 * len_out});
      if (trim < std::max(0.05, 0.5 * options_.sample_resolution)) {
        append_point(cur);
        continue;
      }

      const Eigen::Vector3d entry = cur - d_in * trim;
      const Eigen::Vector3d exit = cur + d_out * trim;
      append_point(entry);

      const double sin_half = std::max(std::sin(0.5 * turn), 1.0e-6);
      const double radius = trim / std::tan(0.5 * turn);
      const Eigen::Vector2d d_in_xy = d_in.head<2>();
      const Eigen::Vector2d d_out_xy = d_out.head<2>();
      const double cross = d_in_xy.x() * d_out_xy.y() - d_in_xy.y() * d_out_xy.x();
      if (std::abs(cross) < 1.0e-6 || sin_half <= 1.0e-6) {
        append_point(exit);
        continue;
      }

      const double side = cross >= 0.0 ? 1.0 : -1.0;
      const Eigen::Vector2d left_normal(-d_in_xy.y(), d_in_xy.x());
      const Eigen::Vector2d center = entry.head<2>() + side * radius * left_normal;

      const double a0 = std::atan2(entry.y() - center.y(), entry.x() - center.x());
      const double a1 = std::atan2(exit.y() - center.y(), exit.x() - center.x());
      double delta = a1 - a0;
      if (cross > 0.0 && delta < 0.0) {
        delta += 2.0 * M_PI;
      } else if (cross < 0.0 && delta > 0.0) {
        delta -= 2.0 * M_PI;
      }

      const double arc_len = std::abs(delta) * radius;
      const int arc_steps = std::clamp(
        static_cast<int>(std::ceil(arc_len / std::max(options_.sample_resolution, 0.05))),
        2, 12);
      for (int s = 1; s < arc_steps; ++s) {
        const double r = static_cast<double>(s) / static_cast<double>(arc_steps);
        const double a = a0 + delta * r;
        Eigen::Vector3d p;
        p.x() = center.x() + radius * std::cos(a);
        p.y() = center.y() + radius * std::sin(a);
        p.z() = cur.z();
        append_point(p);
      }
      append_point(exit);
    }
    append_point(raw.back());
  }

  if (guide.points.size() < 2) {return false;}

  guide.arc_lengths.reserve(guide.points.size());
  guide.arc_lengths.push_back(0.0);
  for (size_t i = 1; i < guide.points.size(); ++i) {
    guide.length += (guide.points[i] - guide.points[i - 1]).norm();
    guide.arc_lengths.push_back(guide.length);
  }
  return guide.length > 1.0e-6;
}

Eigen::Vector3d MincoOptimizer::guidePointAt(const GuidePath & guide, double arc_s) const
{
  if (guide.points.empty()) {return Eigen::Vector3d::Zero();}
  if (guide.points.size() == 1 || guide.length <= 1.0e-9) {return guide.points.front();}
  const double s = std::clamp(arc_s, 0.0, guide.length);
  auto it = std::lower_bound(guide.arc_lengths.begin(), guide.arc_lengths.end(), s);
  if (it == guide.arc_lengths.begin()) {return guide.points.front();}
  if (it == guide.arc_lengths.end()) {return guide.points.back();}
  const size_t hi = static_cast<size_t>(std::distance(guide.arc_lengths.begin(), it));
  const size_t lo = hi - 1;
  const double ds = guide.arc_lengths[hi] - guide.arc_lengths[lo];
  const double r = ds > 1.0e-9 ? (s - guide.arc_lengths[lo]) / ds : 0.0;
  return guide.points[lo] + r * (guide.points[hi] - guide.points[lo]);
}

MincoOptimizer::GuideProjection MincoOptimizer::projectToGuide(
  const GuidePath & guide, const Eigen::Vector3d & point, double hint_s,
  double search_radius) const
{
  GuideProjection best;
  if (guide.points.size() < 2 || guide.arc_lengths.size() != guide.points.size()) {
    return best;
  }

  const double radius = std::max(search_radius, options_.sample_resolution);
  const double begin_s = std::max(0.0, hint_s - radius);
  const double end_s = std::min(guide.length, hint_s + radius);
  auto begin_it = std::upper_bound(guide.arc_lengths.begin(), guide.arc_lengths.end(), begin_s);
  size_t begin_idx = begin_it == guide.arc_lengths.begin() ? 0 :
    static_cast<size_t>(std::distance(guide.arc_lengths.begin(), begin_it) - 1);
  auto end_it = std::lower_bound(guide.arc_lengths.begin(), guide.arc_lengths.end(), end_s);
  size_t end_idx = end_it == guide.arc_lengths.end() ? guide.points.size() - 1 :
    static_cast<size_t>(std::distance(guide.arc_lengths.begin(), end_it));
  begin_idx = std::min(begin_idx, guide.points.size() - 2);
  end_idx = std::min(std::max(end_idx, begin_idx + 1), guide.points.size() - 1);

  best.distance_sq = std::numeric_limits<double>::infinity();
  auto scan_segment = [&](size_t i) {
      const Eigen::Vector3d a = guide.points[i];
      const Eigen::Vector3d b = guide.points[i + 1];
      const Eigen::Vector3d ab = b - a;
      const double len_sq = ab.squaredNorm();
      const double u = len_sq > 1.0e-12 ?
        std::clamp((point - a).dot(ab) / len_sq, 0.0, 1.0) : 0.0;
      const Eigen::Vector3d q = a + u * ab;
      const double d_sq = (point - q).squaredNorm();
      if (d_sq < best.distance_sq) {
        best.point = q;
        best.distance_sq = d_sq;
        best.valid = true;
      }
    };

  for (size_t i = begin_idx; i < end_idx; ++i) {
    scan_segment(i);
  }

  if (!best.valid) {
    for (size_t i = 0; i + 1 < guide.points.size(); ++i) {
      scan_segment(i);
    }
  }
  return best;
}

bool MincoOptimizer::buildWaypoints(
  const GuidePath & guide, std::vector<Eigen::Vector3d> & points,
  Eigen::VectorXd & times,
  const Eigen::Vector3d * initial_velocity,
  const Eigen::Vector3d * terminal_velocity) const
{
  points.clear();
  if (guide.points.size() < 2 || guide.length <= 1.0e-6) {return false;}

  const double spacing_floor = std::max(
    {options_.sample_resolution, options_.min_waypoint_spacing, 0.05});
  const double time_step = std::max(options_.waypoint_time_step, options_.min_segment_time);
  const double nominal_spacing = std::clamp(
    options_.v_ref * time_step,
    spacing_floor, 1.50);
  const int configured_max_waypoints =
    options_.max_waypoints > 0 ? options_.max_waypoints : options_.max_pieces + 1;
  const int max_waypoints = std::max(2, configured_max_waypoints);
  const int target_samples = std::clamp(
    static_cast<int>(std::ceil(guide.length / nominal_spacing)) + 1,
    2, max_waypoints);

  std::vector<double> samples;
  samples.reserve(static_cast<size_t>(target_samples) + guide.points.size());
  for (int i = 0; i < target_samples; ++i) {
    samples.push_back(
      target_samples > 1 ?
      guide.length * static_cast<double>(i) / static_cast<double>(target_samples - 1) : 0.0);
  }
  std::sort(samples.begin(), samples.end());
  samples.erase(
    std::unique(
      samples.begin(), samples.end(),
      [&](double a, double b) {
        return std::abs(a - b) <
          std::max(1.0e-5, 0.25 * std::max(options_.sample_resolution, 1.0e-3));
      }),
    samples.end());

  if (static_cast<int>(samples.size()) > max_waypoints) {
    std::vector<double> protected_samples{0.0, guide.length};
    while (static_cast<int>(samples.size()) > max_waypoints) {
      size_t remove_idx = 0;
      double best_gap = std::numeric_limits<double>::infinity();
      for (size_t i = 1; i + 1 < samples.size(); ++i) {
        const bool protected_point = std::any_of(
          protected_samples.begin(), protected_samples.end(),
          [&](double s) {
            return std::abs(s - samples[i]) <
              std::max(1.0e-5, 0.25 * std::max(options_.sample_resolution, 1.0e-3));
          });
        if (protected_point) {continue;}
        const double gap = samples[i + 1] - samples[i - 1];
        if (gap < best_gap) {
          best_gap = gap;
          remove_idx = i;
        }
      }
      if (remove_idx == 0) {break;}
      samples.erase(samples.begin() + static_cast<std::ptrdiff_t>(remove_idx));
    }
  }

  points.reserve(samples.size());
  for (const double s : samples) {
    const Eigen::Vector3d p = guidePointAt(guide, s);
    if (points.empty() || (p - points.back()).norm() > 1.0e-5) {
      points.push_back(p);
    }
  }
  if ((guide.points.back() - points.back()).norm() > 1.0e-5) {
    points.push_back(guide.points.back());
  }
  if (points.size() < 2) {return false;}

  times.resize(static_cast<int>(points.size()) - 1);
  assignSegmentTimes(points, times, initial_velocity, terminal_velocity);
  return true;
}

void MincoOptimizer::assignSegmentTimes(
  const std::vector<Eigen::Vector3d> & points, Eigen::VectorXd & times,
  const Eigen::Vector3d * initial_velocity,
  const Eigen::Vector3d * terminal_velocity) const
{
  if (points.size() < 2) {
    times.resize(0);
    return;
  }
  times.resize(static_cast<int>(points.size()) - 1);
  if (options_.use_trapezoidal_time) {
    double total_equiv = 0.0;
    double total_dist = 0.0;
    std::vector<double> seg_equiv(points.size() - 1, 0.0);
    for (size_t i = 0; i + 1 < points.size(); ++i) {
      double d = (points[i + 1] - points[i]).norm();
      total_dist += d;
      if (i > 0) {
        d += options_.trapezoidal_k_angle * anglePenalty(points[i - 1], points[i], points[i + 1]);
      }
      seg_equiv[i] = d;
      total_equiv += d;
    }
    if (total_equiv < 1.0e-6) {
      for (size_t i = 0; i + 1 < points.size(); ++i) {
        times(static_cast<int>(i)) = options_.min_segment_time;
      }
    } else {
      const double v = std::max(options_.v_ref, 1.0e-3);
      const double a = std::max(options_.a_max, 1.0e-3);
      const double v0 = boundarySpeedAlongSegment(initial_velocity, points.front(), points[1]);
      const double vf = boundarySpeedAlongSegment(
        terminal_velocity, points[points.size() - 2], points.back());
      const double accel_dist = std::max(0.0, (v * v - v0 * v0) / (2.0 * a));
      const double decel_dist = std::max(0.0, (v * v - vf * vf) / (2.0 * a));
      double T_total = 0.0;
      if (total_dist <= 1.0e-6) {
        T_total = options_.min_segment_time * static_cast<double>(times.size());
      } else if (accel_dist + decel_dist <= total_dist) {
        T_total =
          std::max(0.0, (v - v0) / a) +
          std::max(0.0, (v - vf) / a) +
          (total_dist - accel_dist - decel_dist) / v;
      } else {
        const double v_peak_sq = std::max(0.0, a * total_dist + 0.5 * (v0 * v0 + vf * vf));
        const double v_peak = std::min(v, std::sqrt(v_peak_sq));
        T_total =
          std::max(0.0, (v_peak - v0) / a) +
          std::max(0.0, (v_peak - vf) / a);
      }
      T_total = std::max(T_total, options_.min_segment_time * static_cast<double>(times.size()));
      for (size_t i = 0; i + 1 < points.size(); ++i) {
        times(static_cast<int>(i)) = std::clamp(
          T_total * seg_equiv[i] / total_equiv,
          options_.min_segment_time, options_.max_segment_time);
      }
    }
  } else {
    for (size_t i = 0; i + 1 < points.size(); ++i) {
      const Eigen::Vector3d & prev = i == 0 ? points[i] : points[i - 1];
      times(static_cast<int>(i)) = segmentTime(prev, points[i], points[i + 1]);
    }
  }
}

double MincoOptimizer::boundarySpeedAlongSegment(
  const Eigen::Vector3d * velocity,
  const Eigen::Vector3d & from,
  const Eigen::Vector3d & to) const
{
  if (!velocity) {
    return 0.0;
  }
  const Eigen::Vector2d delta = (to - from).head<2>();
  const double len = delta.norm();
  if (len < 1.0e-6) {
    return 0.0;
  }
  const double along = velocity->head<2>().dot(delta / len);
  return std::clamp(along, 0.0, std::max(options_.v_max, options_.v_ref));
}

void MincoOptimizer::smoothSegmentTimes(Eigen::VectorXd & times) const
{
  if (times.size() <= 2) {return;}
  for (int iter = 0; iter < 3; ++iter) {
    Eigen::VectorXd smoothed = times;
    for (int i = 1; i < times.size() - 1; ++i) {
      smoothed(i) = 0.5 * times(i) + 0.25 * (times(i - 1) + times(i + 1));
    }
    times = smoothed;
  }
}

bool MincoOptimizer::enforceDynamicFeasibility(
  const std::vector<Eigen::Vector3d> & points, Eigen::VectorXd & times,
  Trajectory<5> & traj, double & max_velocity, double & max_acceleration,
  std::string * diagnostic,
  const Eigen::Vector3d * initial_velocity,
  const Eigen::Vector3d * initial_acceleration,
  const Eigen::Vector3d * terminal_velocity,
  const Eigen::Vector3d * terminal_acceleration) const
{
  if (points.size() < 2 || times.size() != static_cast<int>(points.size()) - 1) {
    if (diagnostic) {
      *diagnostic = "invalid points/times size";
    }
    return false;
  }

  auto measure = [&]() {
    max_velocity = 0.0;
    max_acceleration = 0.0;
    for (int k = 0; k < traj.getPieceNum(); ++k) {
      const double T_k = traj[k].getDuration();
      const int n = std::max(
        2, static_cast<int>(std::ceil(T_k / std::max(options_.dynamics_sample_dt, 1.0e-3))));
      for (int s = 0; s <= n; ++s) {
        const double tau = T_k * static_cast<double>(s) / static_cast<double>(n);
        max_velocity = std::max(
          max_velocity, std::hypot(traj[k].getVel(tau).x(), traj[k].getVel(tau).y()));
        max_acceleration = std::max(
          max_acceleration, std::hypot(traj[k].getAcc(tau).x(), traj[k].getAcc(tau).y()));
      }
    }
  };

  auto measure_piece_ratios = [&]() {
    Eigen::VectorXd ratios = Eigen::VectorXd::Ones(times.size());
    max_velocity = 0.0;
    max_acceleration = 0.0;
    for (int k = 0; k < traj.getPieceNum(); ++k) {
      double piece_max_v = 0.0;
      double piece_max_a = 0.0;
      const double T_k = traj[k].getDuration();
      const int n = std::max(
        2, static_cast<int>(std::ceil(T_k / std::max(options_.dynamics_sample_dt, 1.0e-3))));
      for (int s = 0; s <= n; ++s) {
        const double tau = T_k * static_cast<double>(s) / static_cast<double>(n);
        piece_max_v = std::max(
          piece_max_v, std::hypot(traj[k].getVel(tau).x(), traj[k].getVel(tau).y()));
        piece_max_a = std::max(
          piece_max_a, std::hypot(traj[k].getAcc(tau).x(), traj[k].getAcc(tau).y()));
      }
      max_velocity = std::max(max_velocity, piece_max_v);
      max_acceleration = std::max(max_acceleration, piece_max_a);
      const double v_ratio = options_.v_max > 1.0e-3 ? piece_max_v / options_.v_max : 1.0;
      const double a_ratio = options_.a_max > 1.0e-3 ?
        std::sqrt(piece_max_a / options_.a_max) : 1.0;
      ratios(k) = std::max({1.0, v_ratio, a_ratio});
    }
    return ratios;
  };

  const Eigen::VectorXd initial_times = times;
  const double segment_scale_cap = std::max(1.0, options_.dynamic_realloc_max_segment_scale);
  double last_ratio = 1.0;
  double last_v_ratio = 1.0;
  double last_a_ratio = 1.0;
  int applied_iters = 0;
  auto update_ratios = [&]() {
      const double v_ratio = options_.v_max > 1.0e-3 ? max_velocity / options_.v_max : 1.0;
      const double a_ratio = options_.a_max > 1.0e-3 ?
        std::sqrt(max_acceleration / options_.a_max) : 1.0;
      last_ratio = std::max({1.0, v_ratio, a_ratio});
      last_v_ratio = v_ratio;
      last_a_ratio = a_ratio;
    };

  for (int iter = 0; iter < 8; ++iter) {
    const Eigen::VectorXd piece_ratios = measure_piece_ratios();
    update_ratios();
    if (last_ratio <= 1.02) {
      if (diagnostic) {
        *diagnostic = "ok iters=" + std::to_string(applied_iters) +
          " max_v=" + std::to_string(max_velocity) +
          " max_a=" + std::to_string(max_acceleration);
      }
      return true;
    }

    for (Eigen::Index i = 0; i < times.size(); ++i) {
      const double local_scale = std::min(piece_ratios(i) * 1.05, 1.35);
      const double capped_time = initial_times(i) * segment_scale_cap;
      times(i) = std::clamp(
        times(i) * local_scale,
        options_.min_segment_time,
        std::max(options_.min_segment_time, capped_time));
    }
    ++applied_iters;
    if (!buildTrajectory(
        points, times, traj, initial_velocity, initial_acceleration,
        terminal_velocity, terminal_acceleration))
    {
      if (diagnostic) {
        *diagnostic = "buildTrajectory failed after realloc iters=" +
          std::to_string(applied_iters);
      }
      return false;
    }
  }

  measure();
  update_ratios();

  // Local per-piece stretching can stall just above the feasibility threshold when
  // endpoint continuity redistributes acceleration to adjacent pieces. A bounded
  // global stretch preserves the optimized shape while removing the residual
  // dynamic violation.
  int global_stretch_iters = 0;
  for (; global_stretch_iters < 3 && last_ratio > 1.02; ++global_stretch_iters) {
    bool changed = false;
    const double scale = std::clamp(last_ratio * 1.03, 1.0, 1.25);
    for (Eigen::Index i = 0; i < times.size(); ++i) {
      const double old_t = times(i);
      const double max_t = std::max(options_.min_segment_time, options_.max_segment_time);
      times(i) = std::clamp(old_t * scale, options_.min_segment_time, max_t);
      changed = changed || std::abs(times(i) - old_t) > 1.0e-6;
    }
    if (!changed) {
      break;
    }
    if (!buildTrajectory(
        points, times, traj, initial_velocity, initial_acceleration,
        terminal_velocity, terminal_acceleration))
    {
      if (diagnostic) {
        *diagnostic = "buildTrajectory failed after global stretch iters=" +
          std::to_string(global_stretch_iters + 1);
      }
      return false;
    }
    measure();
    update_ratios();
  }

  const bool ok = max_velocity <= options_.v_max * 1.10 &&
    max_acceleration <= options_.a_max * 1.10;
  if (diagnostic) {
    *diagnostic =
      "iters=" + std::to_string(applied_iters) +
      " global=" + std::to_string(global_stretch_iters) +
      " ratio=" + std::to_string(last_ratio) +
      " v_ratio=" + std::to_string(last_v_ratio) +
      " a_ratio=" + std::to_string(last_a_ratio) +
      " max_v=" + std::to_string(max_velocity) +
      " max_a=" + std::to_string(max_acceleration) +
      " dur=" + std::to_string(traj.getTotalDuration()) +
      " time_min=" + std::to_string(times.minCoeff()) +
      " time_max=" + std::to_string(times.maxCoeff());
  }
  return ok;
}

bool MincoOptimizer::buildReferenceTrajectory(
  const nav_msgs::msg::Path & input, Result & result, nav_msgs::msg::Path & output,
  std::string * diagnostic,
  const Eigen::Vector3d * initial_velocity,
  const Eigen::Vector3d * initial_acceleration,
  const Eigen::Vector3d * terminal_velocity,
  const Eigen::Vector3d * terminal_acceleration) const
{
  GuidePath guide;
  if (!buildGuidePath(input, guide)) {
    if (diagnostic) {*diagnostic = "buildGuidePath failed";}
    return false;
  }

  std::vector<Eigen::Vector3d> points;
  Eigen::VectorXd times;
  if (!buildWaypoints(guide, points, times, initial_velocity, terminal_velocity) ||
    points.size() < 2)
  {
    if (diagnostic) {*diagnostic = "buildWaypoints failed";}
    return false;
  }

  Trajectory<5> traj;
  if (!buildTrajectory(
      points, times, traj, initial_velocity, initial_acceleration,
      terminal_velocity, terminal_acceleration))
  {
    if (diagnostic) {*diagnostic = "buildTrajectory failed";}
    return false;
  }

  std::string dyn_diag;
  if (!enforceDynamicFeasibility(
      points, times, traj, result.max_velocity, result.max_acceleration, &dyn_diag,
      initial_velocity, initial_acceleration, terminal_velocity, terminal_acceleration))
  {
    if (diagnostic) {*diagnostic = dyn_diag;}
    return false;
  }

  result.input_waypoints = points;
  result.optimized_waypoints = points;
  result.optimized_times = times;
  result.traj_duration = traj.getTotalDuration();
  result.initial_velocity = traj.getVel(0.0);
  result.initial_acceleration = traj.getAcc(0.0);
  result.terminal_velocity = traj.getVel(result.traj_duration);
  result.terminal_acceleration = traj.getAcc(result.traj_duration);
  result.terminal_stop =
    result.terminal_velocity.head<2>().norm() < 1.0e-3 &&
    result.terminal_acceleration.head<2>().norm() < 1.0e-3;
  output = sampleTrajectory(input, traj);
  result.success = output.poses.size() >= 2;
  if (!result.success && diagnostic) {*diagnostic = "sampleTrajectory produced too few poses";}
  return result.success;
}

int MincoOptimizer::optimizeWaypoints(
  std::vector<Eigen::Vector3d> & points, const GuidePath & guide, Eigen::VectorXd & times,
  const nav2_costmap_2d::Costmap2D * costmap,
  const std::vector<DynamicObstacle> * dynamic_obstacles,
  double w_energy, double w_reference, double w_obstacle,
  int max_iterations, double & final_cost, bool is_fine_stage,
  const std::function<bool()> * should_cancel,
  const Eigen::Vector3d * initial_velocity,
  const Eigen::Vector3d * initial_acceleration,
  const Eigen::Vector3d * terminal_velocity,
  const Eigen::Vector3d * terminal_acceleration) const
{
  if (points.size() < 3) {return -1;}
  if (should_cancel && (*should_cancel)()) {return lbfgs::LBFGS_CANCELED;}

  const int n_pts = static_cast<int>((points.size() - 2) * 2);
  const int n_times = is_fine_stage ? static_cast<int>(points.size()) - 1 : 0;
  Eigen::VectorXd x(n_pts + n_times);
  for (size_t i = 1; i + 1 < points.size(); ++i) {
    const int id = static_cast<int>((i - 1) * 2);
    x(id) = points[i].x();
    x(id + 1) = points[i].y();
  }
  if (is_fine_stage) {
    for (int i = 0; i < n_times; ++i) {
      const double T = times(i);
      x(n_pts + i) = T > 1.0 ? (std::sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - std::sqrt(2.0 / T - 1.0));
    }
  }

  const std::vector<Eigen::Vector3d> reference_points = points;
  OptimizationData data;
  data.optimizer = this;
  data.reference_points = &reference_points;
  data.guide = &guide;
  data.times = &times;
  data.costmap = costmap;
  data.w_energy = w_energy;
  data.w_reference = w_reference;
  data.w_obstacle = w_obstacle;
  data.dynamic_obstacles = dynamic_obstacles;
  data.should_cancel = should_cancel;
  data.initial_velocity = initial_velocity;
  data.initial_acceleration = initial_acceleration;
  data.terminal_velocity = terminal_velocity;
  data.terminal_acceleration = terminal_acceleration;
  data.is_fine_stage = is_fine_stage;
  data.n_pts = n_pts;

  lbfgs::lbfgs_parameter_t params;
  params.mem_size = 8;
  params.g_epsilon = 1.0e-4;
  params.past = 3;
  params.delta = 1.0e-4;
  params.max_iterations = std::max(1, max_iterations);
  params.max_linesearch = 32;

  final_cost = 0.0;
  const int ret = lbfgs::lbfgs_optimize(
    x, final_cost, &MincoOptimizer::evaluateCallback, nullptr,
    &MincoOptimizer::progressCallback, &data, params);

  if (ret >= 0 || ret == lbfgs::LBFGSERR_MAXIMUMITERATION ||
    ret == lbfgs::LBFGS_CANCELED ||
    ret == lbfgs::LBFGSERR_MAXIMUMLINESEARCH ||
    ret == lbfgs::LBFGSERR_MINIMUMSTEP ||
    ret == lbfgs::LBFGSERR_MAXIMUMSTEP ||
    ret == lbfgs::LBFGSERR_WIDTHTOOSMALL)
  {
    for (size_t i = 1; i + 1 < points.size(); ++i) {
      const int id = static_cast<int>((i - 1) * 2);
      points[i].x() = x(id);
      points[i].y() = x(id + 1);
    }
    for (int i = 0; i < n_times; ++i) {
      const double vt = x(n_pts + i);
      times(i) = vt > 0.0 ? ((0.5 * vt + 1.0) * vt + 1.0) : 1.0 / ((0.5 * vt - 1.0) * vt + 1.0);
    }
  }

  return ret;
}

void MincoOptimizer::setBoundaryStates(
  const std::vector<Eigen::Vector3d> & points,
  const std::vector<Eigen::Vector3d> &,
  Eigen::Matrix3d & head_state,
  Eigen::Matrix3d & tail_state,
  const Eigen::Vector3d * head_vel_override,
  const Eigen::Vector3d * head_acc_override,
  const Eigen::Vector3d * tail_vel_override,
  const Eigen::Vector3d * tail_acc_override) const
{
  head_state = Eigen::Matrix3d::Zero();
  tail_state = Eigen::Matrix3d::Zero();
  head_state.col(0) = points.front();
  tail_state.col(0) = points.back();
  if (head_vel_override) {
    head_state.col(1) = *head_vel_override;
  }
  if (head_acc_override) {
    head_state.col(2) = *head_acc_override;
  }
  if (tail_vel_override) {
    tail_state.col(1) = *tail_vel_override;
  }
  if (tail_acc_override) {
    tail_state.col(2) = *tail_acc_override;
  }
}

bool MincoOptimizer::buildTrajectory(
  const std::vector<Eigen::Vector3d> & points, const Eigen::VectorXd & times,
  Trajectory<5> & traj,
  const Eigen::Vector3d * initial_velocity,
  const Eigen::Vector3d * initial_acceleration,
  const Eigen::Vector3d * terminal_velocity,
  const Eigen::Vector3d * terminal_acceleration) const
{
  if (points.size() < 2 || times.size() != static_cast<int>(points.size()) - 1) {
    return false;
  }
  const int piece_num = static_cast<int>(points.size()) - 1;
  Eigen::Matrix3d head_state, tail_state;
  setBoundaryStates(
    points, points, head_state, tail_state,
    initial_velocity, initial_acceleration, terminal_velocity, terminal_acceleration);
  Eigen::Matrix3Xd inner_points(3, std::max(0, piece_num - 1));
  for (int i = 1; i < piece_num; ++i) {
    inner_points.col(i - 1) = points[static_cast<size_t>(i)];
  }
  minco::MINCO_S3NU minco;
  minco.setConditions(head_state, tail_state, piece_num);
  minco.setParameters(inner_points, times);
  minco.getTrajectory(traj);
  return true;
}

double MincoOptimizer::esdfCostAndGradient(
  const Eigen::Vector3d & point, Eigen::Vector2d & grad) const
{
  grad.setZero();
  if (!esdf_map_) {return 0.0;}

  const double res = esdf_map_->getResolution();
  const double res_inv = 1.0 / res;
  const Eigen::Vector3d origin = esdf_map_->getOrigin();
  const double z = options_.esdf_query_z;

  // Continuous grid coordinates (subtract 0.5 so fx/fy are offsets between cell centers)
  const double mx = (point.x() - origin.x()) * res_inv - 0.5;
  const double my = (point.y() - origin.y()) * res_inv - 0.5;
  const int ix = static_cast<int>(std::floor(mx));
  const int iy = static_cast<int>(std::floor(my));
  const double fx = mx - ix;  // ∈ [0,1): fraction between cell centers ix and ix+1
  const double fy = my - iy;

  // Helper: safe ESDF query by grid index — uses signed distance (negative inside obstacles)
  auto esdf = [&](int cx, int cy) -> double {
      const double d = esdf_map_->getSignedDistance2D(
        origin.x() + (cx + 0.5) * res,
        origin.y() + (cy + 0.5) * res,
        z);
      return (d < -1e5) ? -res : d;  // -1e6 sentinel = out of bounds
    };

  // Bilinear interpolation for distance value
  const double d00 = esdf(ix,     iy);
  const double d10 = esdf(ix + 1, iy);
  const double d01 = esdf(ix,     iy + 1);
  const double d11 = esdf(ix + 1, iy + 1);
  const double val = (1 - fx) * (1 - fy) * d00 + fx * (1 - fy) * d10 +
                     (1 - fx) * fy * d01 + fx * fy * d11;

  // Quadratic interpolation for gradient (USTC RoboWalker 5.5.2.2)
  // X gradient: fit parabola through (ix-1,j), (ix,j), (ix+1,j) at each y row
  auto grad_x_at_row = [&](int j) -> double {
      const double dm1 = esdf(ix - 1, j);
      const double d0  = esdf(ix,     j);
      const double dp1 = esdf(ix + 1, j);
      const double a = (dm1 - 2.0 * d0 + dp1) * 0.5;
      const double b = (dp1 - dm1) * 0.5;
      return 2.0 * a * fx + b;
    };
  auto grad_y_at_col = [&](int i) -> double {
      const double dm1 = esdf(i, iy - 1);
      const double d0  = esdf(i, iy);
      const double dp1 = esdf(i, iy + 1);
      const double a = (dm1 - 2.0 * d0 + dp1) * 0.5;
      const double b = (dp1 - dm1) * 0.5;
      return 2.0 * a * fy + b;
    };

  const double gx = ((1.0 - fy) * grad_x_at_row(iy) + fy * grad_x_at_row(iy + 1)) * res_inv;
  const double gy = ((1.0 - fx) * grad_y_at_col(ix) + fx * grad_y_at_col(ix + 1)) * res_inv;

  // Penalty function with smooth transition
  const double d_inf = options_.esdf_influence_distance;
  const double d_safe = options_.esdf_safe_distance;
  if (val >= d_inf) {return 0.0;}

  double cost = 0.0;
  double dCost_dVal = 0.0;
  if (val >= d_safe) {
    const double t = (d_inf - val) / (d_inf - d_safe);
    cost = t * t * t;
    dCost_dVal = -3.0 * t * t / (d_inf - d_safe);
  } else {
    const double excess = d_safe - val;
    cost = excess * excess + 1.0;
    dCost_dVal = -2.0 * excess;
  }

  grad.x() = dCost_dVal * gx;
  grad.y() = dCost_dVal * gy;
  return cost;
}

double MincoOptimizer::evaluateObjective(
  const Eigen::VectorXd & x, Eigen::VectorXd & grad,
  const OptimizationData & data) const
{
  grad.setZero(x.size());
  if (!data.reference_points || !data.times) {return 0.0;}

  std::vector<Eigen::Vector3d> points = *data.reference_points;
  for (size_t i = 1; i + 1 < points.size(); ++i) {
    const int id = static_cast<int>((i - 1) * 2);
    points[i].x() = x(id);
    points[i].y() = x(id + 1);
  }

  const int piece_num = static_cast<int>(points.size()) - 1;
  Eigen::Matrix3d head_state, tail_state;
  setBoundaryStates(
    points, *data.reference_points, head_state, tail_state,
    data.initial_velocity, data.initial_acceleration,
    data.terminal_velocity, data.terminal_acceleration);

  Eigen::Matrix3Xd inner_points(3, std::max(0, piece_num - 1));
  for (int i = 1; i < piece_num; ++i) {
    inner_points.col(i - 1) = points[static_cast<size_t>(i)];
  }

  const int n_times = piece_num;
  Eigen::VectorXd real_times(n_times);
  if (data.n_pts > 0 && x.size() == data.n_pts + n_times) {
    for (int i = 0; i < n_times; ++i) {
      const double vt = x(data.n_pts + i);
      real_times(i) = vt > 0.0 ? ((0.5 * vt + 1.0) * vt + 1.0) : 1.0 / ((0.5 * vt - 1.0) * vt + 1.0);
    }
  } else {
    real_times = *data.times;
  }

  minco::MINCO_S3NU minco;
  minco.setConditions(head_state, tail_state, piece_num);
  minco.setParameters(inner_points, real_times);

  double cost = 0.0;
  double energy = 0.0;
  minco.getEnergy(energy);
  cost += data.w_energy * energy;

  Eigen::MatrixX3d grad_coeffs;
  Eigen::VectorXd grad_times;
  minco.getEnergyPartialGradByCoeffs(grad_coeffs);
  minco.getEnergyPartialGradByTimes(grad_times);
  grad_coeffs *= data.w_energy;
  grad_times *= data.w_energy;

  const bool use_esdf = options_.use_esdf && esdf_map_;
  const bool use_guide_reference = data.guide && data.guide->points.size() >= 2 &&
    data.guide->length > 1.0e-6 && data.w_reference > 0.0;
  const bool need_traj = use_guide_reference ||
    (options_.w_obstacle_traj > 0.0 && (data.costmap || use_esdf)) ||
    options_.w_velocity > 0.0 || options_.w_acceleration > 0.0 ||
    (options_.dynamic_obstacle_enabled && data.dynamic_obstacles &&
    !data.dynamic_obstacles->empty());

  if (need_traj) {
    Trajectory<5> traj;
    minco.getTrajectory(traj);
    double t_offset = 0.0;
    for (int k = 0; k < piece_num; ++k) {
      const double T_k = real_times(k);

      if (use_guide_reference) {
        const int n = std::max(
          1, static_cast<int>(std::ceil(T_k / std::max(options_.obstacle_sample_dt, 1.0e-3))));
        const double duration = std::max(real_times.sum(), 1.0e-6);
        const double search_radius = std::max(
          2.0 * options_.sample_resolution,
          data.guide->length / static_cast<double>(std::max(2, piece_num)));
        for (int s = 0; s <= n; ++s) {
          const double tau = T_k * static_cast<double>(s) / static_cast<double>(n);
          const double global_t = t_offset + tau;
          const double hint_s = data.guide->length * std::clamp(global_t / duration, 0.0, 1.0);
          const Eigen::Vector3d p = traj[k].getPos(tau);
          const GuideProjection proj = projectToGuide(*data.guide, p, hint_s, search_radius);
          if (!proj.valid) {continue;}
          const Eigen::Vector3d diff = p - proj.point;
          const double quadrature = (s == 0 || s == n) ? 0.5 : 1.0;
          const double weight = quadrature * T_k / static_cast<double>(n);
          cost += weight * data.w_reference * diff.squaredNorm();
          const Eigen::Vector2d ref_grad =
            2.0 * weight * data.w_reference * diff.head<2>();
          double tau_pow = 1.0;
          for (int j = 5; j >= 0; --j) {
            grad_coeffs(6 * k + j, 0) += ref_grad.x() * tau_pow;
            grad_coeffs(6 * k + j, 1) += ref_grad.y() * tau_pow;
            tau_pow *= tau;
          }
        }
      }

      if (options_.w_obstacle_traj > 0.0 && (data.costmap || use_esdf)) {
        const int n = std::max(1, static_cast<int>(std::ceil(T_k / options_.obstacle_sample_dt)));
        Eigen::Vector2d prev_grad = Eigen::Vector2d::Zero();
        bool has_prev = false;
        for (int s = 0; s <= n; ++s) {
          const double tau = T_k * static_cast<double>(s) / static_cast<double>(n);
          const Eigen::Vector3d p = traj[k].getPos(tau);
          Eigen::Vector2d obs_grad = Eigen::Vector2d::Zero();
          double obs_cost = 0.0;
          if (use_esdf) {
            obs_cost = esdfCostAndGradient(p, obs_grad);
          } else {
            obs_cost = obstacleCostAndGradient(p, obs_grad, data.costmap);
          }
          if (obs_cost <= 0.0) {has_prev = false; continue;}

          if (data.is_fine_stage) {
            // Velocity-direction projection correction
            const Eigen::Vector3d vel = traj[k].getVel(tau);
            const double vn_sq = vel.x() * vel.x() + vel.y() * vel.y();
            if (vn_sq > 1.0e-8) {
              const double proj = (obs_grad.x() * vel.x() + obs_grad.y() * vel.y()) / vn_sq;
              obs_grad.x() = (obs_grad.x() - proj * vel.x()) + 0.2 * proj * vel.x();
              obs_grad.y() = (obs_grad.y() - proj * vel.y()) + 0.2 * proj * vel.y();
            }

            // Valley detection (USTC 5.5.4.2)
            if (use_esdf && obs_grad.norm() > 1.0e-6) {
              const Eigen::Vector2d step_dir = obs_grad.normalized();
              const Eigen::Vector3d p_step(
                p.x() + options_.esdf_gradient_step * step_dir.x(),
                p.y() + options_.esdf_gradient_step * step_dir.y(),
                p.z());
              Eigen::Vector2d dummy_grad;
              const double d_cur  = esdf_map_->getDistance2D(p.x(), p.y(), options_.esdf_query_z);
              const double d_step = esdf_map_->getDistance2D(p_step.x(), p_step.y(), options_.esdf_query_z);
              const double dir_grad = (d_step - d_cur) / options_.esdf_gradient_step;
              if (dir_grad < options_.esdf_valley_threshold) {
                const double d_safe = options_.esdf_safe_distance;
                const double scale = std::max(0.0, (d_safe - std::max(d_cur, 0.0)) / std::max(d_safe, 1.0e-6));
                obs_grad = step_dir * scale * std::sqrt(obs_grad.norm());
              }
            }

            // EMA gradient smoothing (ESDF mode only)
            if (use_esdf && has_prev && obs_grad.norm() > 1.0e-6 && prev_grad.norm() > 1.0e-6) {
              constexpr double alpha = 0.7;
              obs_grad = alpha * obs_grad + (1.0 - alpha) * prev_grad;
            }
          }
          prev_grad = obs_grad;
          has_prev = true;

          cost += options_.w_obstacle_traj * obs_cost;
          double tau_pow = 1.0;
          for (int j = 5; j >= 0; --j) {
            grad_coeffs(6 * k + j, 0) += options_.w_obstacle_traj * obs_grad.x() * tau_pow;
            grad_coeffs(6 * k + j, 1) += options_.w_obstacle_traj * obs_grad.y() * tau_pow;
            tau_pow *= tau;
          }
        }
      }

      if (options_.w_velocity > 0.0 || options_.w_acceleration > 0.0) {
        const int n = std::max(1, static_cast<int>(std::ceil(T_k / options_.dynamics_sample_dt)));
        for (int s = 0; s <= n; ++s) {
          const double tau = T_k * static_cast<double>(s) / static_cast<double>(n);
          if (options_.w_velocity > 0.0) {
            const Eigen::Vector3d vel = traj[k].getVel(tau);
            const double v_sq = vel.x() * vel.x() + vel.y() * vel.y();
            const double pen = v_sq - options_.v_max * options_.v_max;
            if (pen > 0.0) {
              cost += options_.w_velocity * pen * pen * pen;
              const double dL_dvx = 6.0 * options_.w_velocity * pen * pen * vel.x();
              const double dL_dvy = 6.0 * options_.w_velocity * pen * pen * vel.y();
              double tau_pow = 1.0;
              for (int j = 4; j >= 0; --j) {
                const double c = static_cast<double>(5 - j) * tau_pow;
                grad_coeffs(6 * k + j, 0) += dL_dvx * c;
                grad_coeffs(6 * k + j, 1) += dL_dvy * c;
                tau_pow *= tau;
              }
            }
          }
          if (options_.w_acceleration > 0.0) {
            const Eigen::Vector3d acc = traj[k].getAcc(tau);
            const double a_sq = acc.x() * acc.x() + acc.y() * acc.y();
            const double pen = a_sq - options_.a_max * options_.a_max;
            if (pen > 0.0) {
              cost += options_.w_acceleration * pen * pen * pen;
              const double dL_dax = 6.0 * options_.w_acceleration * pen * pen * acc.x();
              const double dL_day = 6.0 * options_.w_acceleration * pen * pen * acc.y();
              const double tau2 = tau * tau;
              const double da_dc[4] = {20.0 * tau2 * tau, 12.0 * tau2, 6.0 * tau, 2.0};
              for (int j = 0; j <= 3; ++j) {
                grad_coeffs(6 * k + j, 0) += dL_dax * da_dc[j];
                grad_coeffs(6 * k + j, 1) += dL_day * da_dc[j];
              }
            }
          }
        }
      }

      if (options_.dynamic_obstacle_enabled && data.dynamic_obstacles &&
        !data.dynamic_obstacles->empty())
      {
        const int n = std::max(1, static_cast<int>(std::ceil(T_k / options_.dynamics_sample_dt)));
        for (int s = 0; s <= n; ++s) {
          const double tau = T_k * static_cast<double>(s) / static_cast<double>(n);
          const Eigen::Vector3d p = traj[k].getPos(tau);
          Eigen::Vector2d dyn_grad = Eigen::Vector2d::Zero();
          const double dyn_cost = dynamicObstacleCostAndGradient(
            p, t_offset + tau, *data.dynamic_obstacles, dyn_grad);
          if (dyn_cost <= 0.0) {continue;}
          cost += options_.w_dynamic_obstacle * dyn_cost;
          double tau_pow = 1.0;
          for (int j = 5; j >= 0; --j) {
            grad_coeffs(6 * k + j, 0) += options_.w_dynamic_obstacle * dyn_grad.x() * tau_pow;
            grad_coeffs(6 * k + j, 1) += options_.w_dynamic_obstacle * dyn_grad.y() * tau_pow;
            tau_pow *= tau;
          }
        }
      }
      t_offset += T_k;
    }
  }

  Eigen::Matrix3Xd grad_points;
  Eigen::VectorXd grad_T;
  minco.propogateGrad(grad_coeffs, grad_times, grad_points, grad_T);

  cost += options_.wei_time * real_times.sum();
  grad_T.array() += options_.wei_time;

  // Time ratio constraint (FINE stage only)
  if (data.is_fine_stage && data.n_pts > 0 && x.size() == data.n_pts + n_times) {
    const double avg_T = real_times.mean();
    if (avg_T > 1.0e-6) {
      for (int i = 0; i < n_times; ++i) {
        const double ratio = real_times(i) / avg_T;
        if (ratio > options_.time_ratio_upper) {
          const double excess = ratio - options_.time_ratio_upper;
          cost += options_.w_time_ratio * excess * excess;
          grad_T(i) += 2.0 * options_.w_time_ratio * excess / avg_T;
        } else if (ratio < options_.time_ratio_lower) {
          const double deficit = options_.time_ratio_lower - ratio;
          cost += options_.w_time_ratio * deficit * deficit;
          grad_T(i) -= 2.0 * options_.w_time_ratio * deficit / avg_T;
        }
      }
    }
  }

  for (size_t i = 1; i + 1 < points.size(); ++i) {
    const int id = static_cast<int>((i - 1) * 2);
    const int col = static_cast<int>(i - 1);
    grad(id)     += grad_points(0, col);
    grad(id + 1) += grad_points(1, col);

    if (!use_guide_reference) {
      const Eigen::Vector3d diff = points[i] - (*data.reference_points)[i];
      cost += data.w_reference * diff.squaredNorm();
      grad(id)     += 2.0 * data.w_reference * diff.x();
      grad(id + 1) += 2.0 * data.w_reference * diff.y();
    }

    if (options_.w_obstacle_traj <= 0.0) {
      Eigen::Vector2d obs_grad = Eigen::Vector2d::Zero();
      double obs_cost = 0.0;
      if (use_esdf) {
        obs_cost = esdfCostAndGradient(points[i], obs_grad);
      } else {
        obs_cost = obstacleCostAndGradient(points[i], obs_grad, data.costmap);
      }
      cost += data.w_obstacle * obs_cost;
      grad(id)     += data.w_obstacle * obs_grad.x();
      grad(id + 1) += data.w_obstacle * obs_grad.y();
    }
  }

  if (data.n_pts > 0 && x.size() == data.n_pts + n_times) {
    for (int i = 0; i < n_times; ++i) {
      const double vt = x(data.n_pts + i);
      const double dT_dVT = vt > 0.0 ? (vt + 1.0) :
        (1.0 - vt) / (((0.5 * vt - 1.0) * vt + 1.0) * ((0.5 * vt - 1.0) * vt + 1.0));
      grad(data.n_pts + i) += grad_T(i) * dT_dVT;
    }
  }

  if (!std::isfinite(cost) || !grad.allFinite()) {
    grad.setZero();
    cost = 0.0;
    for (size_t i = 1; i + 1 < points.size(); ++i) {
      const int id = static_cast<int>((i - 1) * 2);
      const Eigen::Vector3d diff = points[i] - (*data.reference_points)[i];
      cost += 100.0 * diff.squaredNorm();
      grad(id)     = 200.0 * diff.x();
      grad(id + 1) = 200.0 * diff.y();
    }
    return cost + 1.0e6;
  }
  return cost;
}

double MincoOptimizer::evaluateCallback(
  void * instance, const Eigen::VectorXd & x, Eigen::VectorXd & grad)
{
  const auto * data = static_cast<OptimizationData *>(instance);
  if (data->should_cancel && (*data->should_cancel)()) {
    grad.setZero();
    return 0.0;
  }
  return data->optimizer->evaluateObjective(x, grad, *data);
}

int MincoOptimizer::progressCallback(
  void * instance, const Eigen::VectorXd &, const Eigen::VectorXd &,
  const double, const double, const int, const int)
{
  const auto * data = static_cast<OptimizationData *>(instance);
  return (data->should_cancel && (*data->should_cancel)()) ? 1 : 0;
}

double MincoOptimizer::obstacleCostAndGradient(
  const Eigen::Vector3d & point, Eigen::Vector2d & grad,
  const nav2_costmap_2d::Costmap2D * costmap) const
{
  grad.setZero();
  if (!costmap) {return 0.0;}
  const auto penalty = [this, costmap](double x, double y) {
      const double cost = costAtInterpolated(x, y, costmap);
      const double excess = std::max(0.0, cost - options_.obstacle_cost_threshold);
      const double scale = std::max(1.0, 255.0 - options_.obstacle_cost_threshold);
      const double n = excess / scale;
      return n * n;
    };
  const double base = penalty(point.x(), point.y());
  if (base <= 0.0) {return 0.0;}
  const double h = std::max(options_.obstacle_finite_diff_step, 1.0e-3);
  grad.x() = (penalty(point.x() + h, point.y()) - penalty(point.x() - h, point.y())) / (2.0 * h);
  grad.y() = (penalty(point.x(), point.y() + h) - penalty(point.x(), point.y() - h)) / (2.0 * h);
  return base;
}

double MincoOptimizer::costAt(
  double wx, double wy, const nav2_costmap_2d::Costmap2D * costmap) const
{
  if (!costmap) {return 0.0;}
  unsigned int mx = 0, my = 0;
  if (!costmap->worldToMap(wx, wy, mx, my)) {return 255.0;}
  const unsigned char cost = costmap->getCost(mx, my);
  if (cost == nav2_costmap_2d::NO_INFORMATION) {return options_.allow_unknown ? 0.0 : 255.0;}
  return static_cast<double>(cost);
}

double MincoOptimizer::costAtInterpolated(
  double wx, double wy, const nav2_costmap_2d::Costmap2D * costmap) const
{
  if (!costmap) {return 0.0;}
  const double res = costmap->getResolution();
  const double mx = (wx - costmap->getOriginX()) / res - 0.5;
  const double my = (wy - costmap->getOriginY()) / res - 0.5;
  const int ix = static_cast<int>(std::floor(mx));
  const int iy = static_cast<int>(std::floor(my));
  const double fx = mx - ix, fy = my - iy;
  const int sx = static_cast<int>(costmap->getSizeInCellsX());
  const int sy = static_cast<int>(costmap->getSizeInCellsY());
  auto sc = [&](int cx, int cy) -> double {
      if (cx < 0 || cy < 0 || cx >= sx || cy >= sy) {return 255.0;}
      const unsigned char c = costmap->getCost(cx, cy);
      if (c == nav2_costmap_2d::NO_INFORMATION) {return options_.allow_unknown ? 0.0 : 255.0;}
      return static_cast<double>(c);
    };
  return (1-fx)*(1-fy)*sc(ix,iy) + fx*(1-fy)*sc(ix+1,iy) +
         (1-fx)*fy*sc(ix,iy+1)   + fx*fy*sc(ix+1,iy+1);
}

double MincoOptimizer::dynamicObstacleCostAndGradient(
  const Eigen::Vector3d & point, double t_global,
  const std::vector<DynamicObstacle> & obstacles, Eigen::Vector2d & grad) const
{
  grad.setZero();
  double total = 0.0;
  for (const auto & obs : obstacles) {
    Eigen::Vector2d pos;
    const int idx = static_cast<int>(std::floor(t_global / std::max(obs.prediction_dt, 1.0e-3)));
    if (!obs.predicted_positions.empty() && idx < static_cast<int>(obs.predicted_positions.size())) {
      pos = obs.predicted_positions[static_cast<size_t>(idx)];
    } else {
      pos = obs.p0 + obs.v * t_global;
    }
    const Eigen::Vector2d diff(point.x() - pos.x(), point.y() - pos.y());
    const double d = diff.norm();
    const double excess = options_.dynamic_obstacle_safe_dist + obs.radius - d;
    if (excess <= 0.0) {continue;}
    total += excess * excess;
    if (d > 1.0e-6) {grad -= 2.0 * excess * diff / d;}
  }
  return total;
}

double MincoOptimizer::segmentTime(
  const Eigen::Vector3d & prev, const Eigen::Vector3d & current,
  const Eigen::Vector3d & next) const
{
  const double length = (next - current).norm();
	  const double base_time = length / std::max(options_.v_ref, 1.0e-3);
	  return std::clamp(
	    base_time + options_.corner_time_weight * anglePenalty(prev, current, next),
	    options_.min_segment_time, options_.max_segment_time);
	}

double MincoOptimizer::anglePenalty(
  const Eigen::Vector3d & prev, const Eigen::Vector3d & current,
  const Eigen::Vector3d & next) const
{
  const Eigen::Vector3d a = current - prev, b = next - current;
  if (a.norm() < 1.0e-6 || b.norm() < 1.0e-6) {return 0.0;}
  return std::acos(std::clamp(a.normalized().dot(b.normalized()), -1.0, 1.0));
}

nav_msgs::msg::Path MincoOptimizer::sampleTrajectory(
  const nav_msgs::msg::Path & reference, const Trajectory<5> & traj) const
{
  nav_msgs::msg::Path output;
  output.header = reference.header;
  const double duration = traj.getTotalDuration();
  if (duration <= 1.0e-6) {output = reference; return output;}

  double length = 0.0;
  const int rough = std::max(2, static_cast<int>(std::ceil(duration / 0.05)));
  std::vector<std::pair<double, double>> arc_table;
  arc_table.reserve(static_cast<size_t>(rough) + 1);
  arc_table.push_back({0.0, 0.0});
  Eigen::Vector3d prev_p = traj.getPos(0.0);
  for (int i = 1; i <= rough; ++i) {
    const double t = duration * static_cast<double>(i) / static_cast<double>(rough);
    const Eigen::Vector3d p = traj.getPos(t);
    length += (p - prev_p).norm();
    arc_table.push_back({t, length});
    prev_p = p;
  }

  const int samples = std::max(2, static_cast<int>(std::ceil(
    length / std::max(options_.sample_resolution, 1.0e-3))));
  output.poses.reserve(static_cast<size_t>(samples) + 1);
  for (int i = 0; i <= samples; ++i) {
    const double target_s = length * static_cast<double>(i) / static_cast<double>(samples);
    auto it = std::lower_bound(arc_table.begin(), arc_table.end(), target_s,
      [](const std::pair<double,double> & a, double s){return a.second < s;});
    double t_sample;
    if (it == arc_table.begin()) {t_sample = arc_table.front().first;}
    else if (it == arc_table.end()) {t_sample = arc_table.back().first;}
    else {
      const auto & hi = *it, & lo = *(it - 1);
      const double ds = hi.second - lo.second;
      t_sample = lo.first + (ds > 1.0e-9 ? (target_s - lo.second) / ds : 0.0) * (hi.first - lo.first);
    }
    const Eigen::Vector3d p = traj.getPos(t_sample);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = reference.header;
    pose.pose.position.x = p.x();
    pose.pose.position.y = p.y();
    pose.pose.position.z = p.z();
    output.poses.push_back(pose);
  }
  return output;
}

}  // namespace sirb_smoother
