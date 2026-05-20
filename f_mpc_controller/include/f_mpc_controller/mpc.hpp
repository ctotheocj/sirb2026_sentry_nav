#pragma once
#ifndef F_MPC_CONTROLLER__MPC_HPP_
#define F_MPC_CONTROLLER__MPC_HPP_

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>

struct State { double x, y; };
struct Control { double vx, vy; };
struct SolveResult {
  bool success{false};
  Control control{0.0, 0.0};
  OsqpEigen::Status status{OsqpEigen::Status::Unsolved};
};

struct ObstacleConstraint {
  std::vector<Eigen::Vector2d> centers;  // size N
  std::vector<double> radii;             // size N; <= 0 = inactive
};

class MPC
{
public:
  MPC(double dt, int horizon,
      double vx_min, double vx_max,
      double vy_min, double vy_max,
      double QX, double QY, double R, double S,
      double terminal_weight = 5.0, int terminal_horizon = 2,
      double Qv = 0.5,
      double ax_max = 5.0, double ay_max = 5.0,
      double v_circle_max = 3.0,
      int max_dynamic = 5);

  void setObstacles(const std::vector<ObstacleConstraint> & obstacles,
                    const Eigen::Vector2d & x0,
                    const std::vector<Eigen::Vector2d> & p_prev,
                    const std::vector<State> & ref);

  void setHorizonSpeedLimits(const std::vector<double> & speed_limits) {
    for (int i = 0; i < N_; ++i) {
      double v = (i < static_cast<int>(speed_limits.size())) ? speed_limits[i] : v_circle_max_;
      double fc = std::min(v, v_circle_max_) * std::cos(M_PI / 8.0);
      for (int k = 0; k < 8; ++k) ub_(k * N_ + i) = fc;
    }
  }
  void clearHorizonSpeedLimits() {
    const double fc = v_circle_max_ * std::cos(M_PI / 8.0);
    for (int i = 0; i < N_; ++i) {
      for (int k = 0; k < 8; ++k) ub_(k * N_ + i) = fc;
    }
  }

  void clearObstacles();

  SolveResult solve(const State & current,
                    const std::vector<State> & ref,
                    const std::vector<State> & v_ref);

  void resetWarmStart() {
    has_last_solution_ = false;
    has_last_dual_ = false;
    last_U_.setZero();
  }
  void setLastExecutedU(double vx, double vy) {
    last_executed_u_ = Eigen::Vector2d(vx, vy);
  }
  const Eigen::VectorXd & getLastU() const { return last_U_; }
  int getActiveObstacleCount() const { return active_obs_count_; }

  ~MPC() = default;

private:
  double dt_;
  int N_, max_obs_;
  double QX_, QY_, R_, S_, Qv_;
  double terminal_weight_;
  int terminal_horizon_;
  double ax_max_, ay_max_, v_circle_max_;

  OsqpEigen::Solver solver_;
  Eigen::MatrixXd Bbar_;
  Eigen::MatrixXd Qbar_;
  Eigen::MatrixXd linear_f_mapping_;

  int obs_row_offset_;
  Eigen::MatrixXd A_dense_;
  Eigen::SparseMatrix<double> A_sparse_;

  Eigen::VectorXd last_U_;
  Eigen::Vector2d last_executed_u_ = Eigen::Vector2d::Zero();
  bool has_last_solution_{false};
  Eigen::VectorXd last_dual_;
  bool has_last_dual_{false};
  Eigen::VectorXd lb_, ub_;

  int active_obs_count_{0};
};

#endif  // F_MPC_CONTROLLER__MPC_HPP_
