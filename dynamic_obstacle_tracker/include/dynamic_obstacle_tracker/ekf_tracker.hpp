// Copyright 2025 Pan
// Licensed under the Apache License, Version 2.0
//
// EKF-SORT 追踪器
// - 状态: [px, py, vx, vy] 恒速模型 (CV)
// - 关联: 最优匈牙利 (Kuhn-Munkres O(N³)) + Mahalanobis 门控距离
// - 生命周期: 新检测 >3 帧确认, 失联时 confidence 衰减, >5 帧删除
//
// Mahalanobis 关联: 阈值为卡方分布 95% 置信 (chi2=5.99, sqrt≈2.45)
//   比欧氏距离阈值更合理: 高速轨迹预测误差大时门控自动放宽
// Confidence Fading: 失联时每帧乘 0.7, 5帧后约 16%, 下游 ESDF 软决策
// 最优匈牙利: O(N³) Kuhn-Munkres算法, 替代O(N!)暴力枚举
// Q_vel=2.0 (大速度噪声) 适应 RM 全向轮急转急停

#pragma once

#include <algorithm>
#include <limits>
#include <numeric>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "dynamic_obstacle_tracker/hungarian.hpp"

namespace dynamic_obstacle_tracker
{

struct TrackedObject
{
  int id;
  Eigen::Vector4d state;    // [px, py, vx, vy]
  Eigen::Matrix4d P;        // 协方差矩阵
  double radius;
  float confidence;         // 0~1
  int age;                  // 存在帧数
  int missed;               // 连续失联帧数
  bool confirmed;           // 是否已确认 (age >= confirm_frames)
};

class EKFTracker
{
public:
  // 参数
  struct Params
  {
    double q_pos{0.01};         // 位置过程噪声
    double q_vel{2.0};          // 速度过程噪声 (RM全向轮急转急停需要大)
    double r_pos{0.05};         // 观测噪声
    double association_threshold{2.45};  // Mahalanobis 距离阈值 (卡方95%: sqrt(5.99)≈2.45)
    int max_missed{5};          // 最大失联帧数
    int confirm_frames{3};      // 确认所需帧数
    double default_radius{0.35};  // 默认包围半径 (m)
  };

  EKFTracker() : params_(Params{}) {}
  explicit EKFTracker(const Params & params) : params_(params) {}

  // 更新: 输入新帧检测结果 (质心列表), 返回当前已确认 tracks
  // dt: 与上帧时间间隔 (s)
  const std::vector<TrackedObject> & update(
    const std::vector<Eigen::Vector2d> & detections,
    const std::vector<double> & det_radii,
    double dt)
  {
    // 1. 预测所有 tracks
    for (auto & t : tracks_) {
      predictEKF(t, dt);
    }

    // 2. 匈牙利匹配
    int M = static_cast<int>(tracks_.size());
    int N = static_cast<int>(detections.size());

    // 代价矩阵: M×N, 使用 Mahalanobis 距离
    // S = H*P*H^T + R (新息协方差), cost = sqrt(diff^T * S^{-1} * diff)
    // 高速轨迹预测不确定性大时门控自动放宽, 避免欧氏距离误拒
    Eigen::Matrix<double, 2, 4> H_assoc = Eigen::Matrix<double, 2, 4>::Zero();
    H_assoc(0, 0) = 1.0;
    H_assoc(1, 1) = 1.0;
    Eigen::Matrix2d R_assoc = Eigen::Matrix2d::Identity() * params_.r_pos;

    std::vector<std::vector<double>> cost(M, std::vector<double>(N, 0.0));
    for (int i = 0; i < M; ++i) {
      Eigen::Vector2d pred_pos = tracks_[i].state.head<2>();
      Eigen::Matrix2d S = H_assoc * tracks_[i].P * H_assoc.transpose() + R_assoc;
      Eigen::Matrix2d S_inv = S.inverse();
      for (int j = 0; j < N; ++j) {
        Eigen::Vector2d diff = detections[j] - pred_pos;
        double maha2 = diff.transpose() * S_inv * diff;
        cost[i][j] = std::sqrt(std::max(0.0, maha2));
      }
    }

    // 简单匈牙利 (Hungarian / Kuhn-Munkres O(N³))
    auto assignment = hungarianAssignment(cost, M, N);

    // 3. 更新匹配的 tracks
    std::vector<bool> det_matched(N, false);
    for (int i = 0; i < M; ++i) {
      int j = assignment[i];
      if (j >= 0 && cost[i][j] <= params_.association_threshold) {
        updateEKF(tracks_[i], detections[j]);
        tracks_[i].radius = det_radii[j];
        tracks_[i].missed = 0;
        tracks_[i].age++;
        if (tracks_[i].age >= params_.confirm_frames) {
          tracks_[i].confirmed = true;
        }
        tracks_[i].confidence = std::min(
          1.0f, static_cast<float>(tracks_[i].age) / (params_.confirm_frames * 2.0f));
        det_matched[j] = true;
      } else {
        tracks_[i].missed++;
        // Confidence Fading: 失联时每帧衰减 30%, 5帧后约 16%
        // 下游 ESDF 可用阈值做软决策, 避免障碍物突然消失引发激进规划
        tracks_[i].confidence *= 0.7f;
      }
    }

    // 4. 未匹配检测 → 创建新 track
    for (int j = 0; j < N; ++j) {
      if (!det_matched[j]) {
        initTrack(detections[j], det_radii[j]);
      }
    }

    // 5. 删除失联超过阈值的 tracks
    tracks_.erase(
      std::remove_if(
        tracks_.begin(), tracks_.end(),
        [this](const TrackedObject & t) { return t.missed > params_.max_missed; }),
      tracks_.end());

    return tracks_;
  }

  const std::vector<TrackedObject> & tracks() const { return tracks_; }

private:
  Params params_;
  std::vector<TrackedObject> tracks_;
  int next_id_{0};

  void predictEKF(TrackedObject & t, double dt)
  {
    // 状态转移: x = F*x
    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;
    t.state = F * t.state;

    // 过程噪声矩阵 Q
    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;
    Q(0, 0) = params_.q_pos * dt4 / 4.0;
    Q(1, 1) = params_.q_pos * dt4 / 4.0;
    Q(2, 2) = params_.q_vel * dt2;
    Q(3, 3) = params_.q_vel * dt2;
    Q(0, 2) = Q(2, 0) = params_.q_pos * dt3 / 2.0;
    Q(1, 3) = Q(3, 1) = params_.q_pos * dt3 / 2.0;

    // P = F*P*F^T + Q
    t.P = F * t.P * F.transpose() + Q;
  }

  void updateEKF(TrackedObject & t, const Eigen::Vector2d & z)
  {
    // 观测矩阵 H = [I | 0]
    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    // 观测噪声 R
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * params_.r_pos;

    // 创新
    Eigen::Vector2d y = z - H * t.state;

    // 卡尔曼增益
    Eigen::Matrix2d S = H * t.P * H.transpose() + R;
    Eigen::Matrix<double, 4, 2> K = t.P * H.transpose() * S.inverse();

    // 状态更新
    t.state += K * y;

    // 协方差更新 (Joseph form, 数值稳定)
    Eigen::Matrix4d I_KH = Eigen::Matrix4d::Identity() - K * H;
    t.P = I_KH * t.P * I_KH.transpose() + K * R * K.transpose();
  }

  void initTrack(const Eigen::Vector2d & pos, double radius)
  {
    TrackedObject t;
    t.id = next_id_++;
    t.state = Eigen::Vector4d(pos.x(), pos.y(), 0.0, 0.0);
    t.P = Eigen::Matrix4d::Identity();
    t.P(2, 2) = 1.0;  // 初始速度不确定
    t.P(3, 3) = 1.0;
    t.radius = radius;
    t.confidence = 0.0f;
    t.age = 1;
    t.missed = 0;
    t.confirmed = false;
    tracks_.push_back(t);
  }

  // 最优匈牙利匹配 (O(N³) Kuhn-Munkres算法)
  // 返回 assignment[i] = j (track i 匹配到 detection j), 或 -1
  // 相比O(N!)暴力枚举: 支持N>10的场景，保证全局最优
  std::vector<int> hungarianAssignment(
    const std::vector<std::vector<double>> & cost,
    int M, int N)
  {
    std::vector<int> assignment;
    HungarianAlgorithm::solve(cost, assignment);

    // 过滤超过阈值的匹配
    for (int i = 0; i < M; ++i) {
      if (assignment[i] >= 0 && assignment[i] < N) {
        if (cost[i][assignment[i]] > params_.association_threshold) {
          assignment[i] = -1;
        }
      }
    }

    return assignment;
  }
};

}  // namespace dynamic_obstacle_tracker
