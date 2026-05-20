#include "f_mpc_controller/mpc.hpp"
#include <cmath>
#include <algorithm>
#include <cstring>

MPC::MPC(double dt, int horizon,
         double /*vx_min*/, double /*vx_max*/,
         double /*vy_min*/, double /*vy_max*/,
         double QX, double QY, double R, double S,
         double terminal_weight, int terminal_horizon, double Qv,
         double ax_max, double ay_max, double v_circle_max,
         int max_dynamic)
: dt_(dt), N_(horizon), max_obs_(max_dynamic),
  QX_(QX), QY_(QY), R_(R), S_(S), Qv_(Qv),
  terminal_weight_(terminal_weight),
  terminal_horizon_(std::clamp(terminal_horizon, 1, horizon)),
  ax_max_(ax_max), ay_max_(ay_max), v_circle_max_(v_circle_max)
{
  // 构建固定尺寸的 OSQP 问题矩阵，后续每周期只更新梯度、边界和障碍线性化行。
  const int nx = 2, nu = 2;
  const int nU = nu * N_;
  obs_row_offset_ = 8 * N_ + nU;
  const int nC = obs_row_offset_ + max_obs_ * N_;

  last_U_ = Eigen::VectorXd::Zero(nU);
  last_dual_ = Eigen::VectorXd::Zero(nC);

  const double fc_bound = v_circle_max_ * std::cos(M_PI / 8.0);
  const double du_max_x = ax_max_ * dt_;
  const double du_max_y = ay_max_ * dt_;

  lb_ = Eigen::VectorXd::Constant(nC, -OsqpEigen::INFTY);
  ub_ = Eigen::VectorXd::Constant(nC,  OsqpEigen::INFTY);
  for (int i = 0; i < 8 * N_; ++i) ub_(i) = fc_bound;
  for (int i = 0; i < N_; ++i) {
    lb_(8*N_ + i*nu)     = -du_max_x;  ub_(8*N_ + i*nu)     = +du_max_x;
    lb_(8*N_ + i*nu + 1) = -du_max_y;  ub_(8*N_ + i*nu + 1) = +du_max_y;
  }

  Bbar_ = Eigen::MatrixXd::Zero(nx * N_, nU);
  for (int i = 0; i < N_; ++i)
    for (int j = 0; j <= i; ++j) {
      Bbar_(i*nx,   j*nu)   = dt_;
      Bbar_(i*nx+1, j*nu+1) = dt_;
    }

  Qbar_ = Eigen::MatrixXd::Zero(nx * N_, nx * N_);
  for (int i = 0; i < N_; ++i) {
    double m = (i >= N_ - terminal_horizon_) ? terminal_weight_ : 1.0;
    Qbar_(i*nx,   i*nx)   = QX_ * m;
    Qbar_(i*nx+1, i*nx+1) = QY_ * m;
  }

  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(nU, nU);
  for (int i = 0; i < N_; ++i)
    for (int d = 0; d < nu; ++d) {
      int idx = i*nu + d;
      D(idx, idx) = 1.0;
      if (i > 0) D(idx, (i-1)*nu + d) = -1.0;
    }

  Eigen::MatrixXd H_dense = 2.0 * (Bbar_.transpose() * Qbar_ * Bbar_ +
                                    (R_ + Qv_) * Eigen::MatrixXd::Identity(nU, nU) +
                                    S_ * D.transpose() * D);
  linear_f_mapping_ = 2.0 * Bbar_.transpose() * Qbar_;

  // 用占位非零保留障碍约束稀疏结构，保证运行时可直接更新矩阵数值。
  A_dense_ = Eigen::MatrixXd::Zero(nC, nU);
  for (int k = 0; k < 8; ++k) {
    double ck = std::cos(2.0*M_PI*k/8.0), sk = std::sin(2.0*M_PI*k/8.0);
    for (int i = 0; i < N_; ++i) {
      A_dense_(k*N_+i, i*nu)   = ck;
      A_dense_(k*N_+i, i*nu+1) = sk;
    }
  }
  A_dense_.block(8*N_, 0, nU, nU) = D;
  for (int obs = 0; obs < max_obs_; ++obs)
    for (int k = 0; k < N_; ++k)
      for (int j = 0; j < nU; ++j)
        A_dense_(obs_row_offset_ + obs*N_ + k, j) = 1.0;

  A_sparse_ = A_dense_.sparseView();

  Eigen::SparseMatrix<double> H = H_dense.sparseView();

  solver_.settings()->setVerbosity(false);
  solver_.settings()->setWarmStart(true);
  solver_.settings()->setAbsoluteTolerance(1e-2);
  solver_.settings()->setMaxIteration(800);
  solver_.data()->setNumberOfVariables(nU);
  solver_.data()->setNumberOfConstraints(nC);
  if (!solver_.data()->setHessianMatrix(H)) return;
  if (!solver_.data()->setLinearConstraintsMatrix(A_sparse_)) return;

  Eigen::VectorXd dummy = Eigen::VectorXd::Zero(nU);
  solver_.data()->setGradient(dummy);
  solver_.data()->setLowerBound(lb_);
  solver_.data()->setUpperBound(ub_);
  if (!solver_.initSolver())
    throw std::runtime_error("Failed to initialize OSQP solver in MPC.");
}

void MPC::setObstacles(const std::vector<ObstacleConstraint> & obstacles,
                       const Eigen::Vector2d & x0,
                       const std::vector<Eigen::Vector2d> & p_prev,
                       const std::vector<State> & ref)
{
  // 将圆形安全距离约束在上一帧预测点处线性化为 OSQP 可处理的半空间约束。
  const int nx = 2, nU = 2 * N_;
  active_obs_count_ = 0;

  for (int obs = 0; obs < max_obs_; ++obs) {
    for (int k = 0; k < N_; ++k) {
      int row = obs_row_offset_ + obs * N_ + k;

      bool inactive = obs >= static_cast<int>(obstacles.size()) ||
                      k   >= static_cast<int>(obstacles[obs].radii.size()) ||
                      obstacles[obs].radii[k] <= 0.0;

      if (inactive) {
        lb_(row) = -OsqpEigen::INFTY;
        ub_(row) =  OsqpEigen::INFTY;
        continue;
      }

      ++active_obs_count_;
      const Eigen::Vector2d & c_k = obstacles[obs].centers[k];
      const double r_safe = obstacles[obs].radii[k];
      const Eigen::Vector2d & p_prev_k = p_prev[std::min(k, (int)p_prev.size()-1)];

      Eigen::Vector2d n_k;
      double dist = (p_prev_k - c_k).norm();
      if (dist < 1e-3) {
        const auto & rk = ref[std::min(k, (int)ref.size()-1)];
        Eigen::Vector2d fallback = Eigen::Vector2d(rk.x, rk.y) - c_k;
        double fd = fallback.norm();
        if (fd < 1e-3) {
          lb_(row) = -OsqpEigen::INFTY;
          ub_(row) =  OsqpEigen::INFTY;
          continue;
        }
        n_k = fallback / fd;
      } else {
        n_k = (p_prev_k - c_k) / dist;
      }

      for (int j = 0; j < nU; ++j)
        A_dense_(row, j) = n_k.x() * Bbar_(k*nx, j) + n_k.y() * Bbar_(k*nx+1, j);

      lb_(row) = r_safe + n_k.dot(c_k) - n_k.dot(x0);
      ub_(row) = OsqpEigen::INFTY;
    }
  }

  for (int j = 0; j < A_sparse_.cols(); ++j)
    for (Eigen::SparseMatrix<double>::InnerIterator it(A_sparse_, j); it; ++it)
      if (it.row() >= obs_row_offset_)
        it.valueRef() = A_dense_(it.row(), it.col());

  solver_.updateLinearConstraintsMatrix(A_sparse_);
}

void MPC::clearObstacles()
{
  // 关闭所有障碍约束边界，保留矩阵结构以便后续周期重新启用。
  active_obs_count_ = 0;
  for (int row = obs_row_offset_; row < lb_.size(); ++row) {
    lb_(row) = -OsqpEigen::INFTY;
    ub_(row) =  OsqpEigen::INFTY;
  }
  solver_.updateBounds(lb_, ub_);
}

SolveResult MPC::solve(const State & current, const std::vector<State> & ref,
                       const std::vector<State> & v_ref)
{
  // 更新当前状态对应的二次规划并求解，返回预测序列第一步作为速度命令。
  const int nx = 2, nu = 2, nU = nu * N_;

  Eigen::VectorXd U_init(nU);
  if (has_last_solution_) {
    // 将上一帧最优控制序列前移一格作为 warm start，提高连续控制求解稳定性。
    for (int i = 0; i < N_-1; ++i)
      U_init.segment<2>(2*i) = last_U_.segment<2>(2*(i+1));
    U_init.segment<2>(2*(N_-1)) = last_U_.segment<2>(2*(N_-1));
    solver_.setPrimalVariable(U_init);
    if (has_last_dual_) solver_.setDualVariable(last_dual_);
  } else {
    U_init.setZero();
    solver_.setPrimalVariable(U_init);
  }

  Eigen::VectorXd error(nx * N_);
  for (int i = 0; i < N_; ++i) {
    const auto & r = ref[std::min(i, (int)ref.size()-1)];
    error(i*nx)   = current.x - r.x;
    error(i*nx+1) = current.y - r.y;
  }

  Eigen::VectorXd f = linear_f_mapping_ * error;
  f(0) -= 2.0 * S_ * last_executed_u_(0);
  f(1) -= 2.0 * S_ * last_executed_u_(1);
  if (Qv_ > 0.0) {
    for (int i = 0; i < N_; ++i) {
      const auto & v = v_ref[std::min(i, (int)v_ref.size()-1)];
      f(2*i)   -= 2.0 * Qv_ * v.x;
      f(2*i+1) -= 2.0 * Qv_ * v.y;
    }
  }

  solver_.updateGradient(f);

  const int rate_offset = 8 * N_;
  const double du_max_x = ax_max_ * dt_, du_max_y = ay_max_ * dt_;
  lb_(rate_offset)   = -du_max_x + last_executed_u_(0);
  ub_(rate_offset)   = +du_max_x + last_executed_u_(0);
  lb_(rate_offset+1) = -du_max_y + last_executed_u_(1);
  ub_(rate_offset+1) = +du_max_y + last_executed_u_(1);
  solver_.updateBounds(lb_, ub_);

  if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
    has_last_solution_ = false; has_last_dual_ = false;
    return {false, {0.0, 0.0}, solver_.getStatus()};
  }

  auto status = solver_.getStatus();
  if (status == OsqpEigen::Status::MaxIterReached) {
    has_last_solution_ = false; has_last_dual_ = false;
    return {false, {0.0, 0.0}, status};
  }

  if (status != OsqpEigen::Status::Solved &&
      status != OsqpEigen::Status::SolvedInaccurate) {
    has_last_solution_ = false; has_last_dual_ = false;
    return {false, {0.0, 0.0}, status};
  }

  Eigen::VectorXd U = solver_.getSolution();

  if (status == OsqpEigen::Status::SolvedInaccurate) {
    // 对带硬障碍或违反速度外包络的不精确解保持保守，避免输出潜在不可行命令。
    if (active_obs_count_ > 0) {
      has_last_solution_ = false; has_last_dual_ = false;
      return {false, {0.0, 0.0}, status};
    }
    const double fc_bound = v_circle_max_ * std::cos(M_PI / 8.0);
    for (int k = 0; k < 8; ++k) {
      double ck = std::cos(2.0*M_PI*k/8.0), sk = std::sin(2.0*M_PI*k/8.0);
      if (ck*U(0) + sk*U(1) > fc_bound + 1e-2) {
        has_last_solution_ = false; has_last_dual_ = false;
        return {false, {0.0, 0.0}, status};
      }
    }
  }

  last_dual_ = solver_.getDualSolution();
  has_last_dual_ = true;
  last_U_ = U;
  has_last_solution_ = true;
  // 保存本次原始最优序列和对偶变量，供下一周期 warm start 与碰撞预测使用。
  return {true, {U(0), U(1)}, status};
}
