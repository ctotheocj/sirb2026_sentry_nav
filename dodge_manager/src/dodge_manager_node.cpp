// Licensed under the Apache License, Version 2.0

#include "dodge_manager/dodge_manager_node.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <algorithm>

namespace dodge_manager
{


void Polygon::computeAABB()
{
  if (vertices.empty()) {
    return;
  }
  aabb_min_x = aabb_max_x = vertices[0].x;
  aabb_min_y = aabb_max_y = vertices[0].y;
  for (size_t i = 1; i < vertices.size(); ++i) {
    aabb_min_x = std::min(aabb_min_x, vertices[i].x);
    aabb_max_x = std::max(aabb_max_x, vertices[i].x);
    aabb_min_y = std::min(aabb_min_y, vertices[i].y);
    aabb_max_y = std::max(aabb_max_y, vertices[i].y);
  }
  aabb_valid = true;
}

void Polygon::precompute()
{
  computeAABB();

  int n = static_cast<int>(vertices.size());
  if (n < 3) {
    is_convex = false;
    return;
  }

  // 预计算边向量
  edges.resize(n);
  for (int i = 0; i < n; ++i) {
    int j = (i + 1) % n;
    edges[i].dx = vertices[j].x - vertices[i].x;
    edges[i].dy = vertices[j].y - vertices[i].y;
  }

  // 检测凸性并判断顶点顺序
  bool has_pos = false, has_neg = false;
  double sum_cross = 0.0;
  for (int i = 0; i < n; ++i) {
    int j = (i + 1) % n;
    double cross = edges[i].dx * edges[j].dy - edges[i].dy * edges[j].dx;
    sum_cross += cross;
    if (cross > 0.0) has_pos = true;
    if (cross < 0.0) has_neg = true;
  }
  is_convex = !(has_pos && has_neg);

  // 如果是凸多边形且顶点顺序为顺时针，则翻转为逆时针
  if (is_convex && sum_cross < 0.0) {
    std::reverse(vertices.begin(), vertices.end());
    // 重新计算边向量
    for (int i = 0; i < n; ++i) {
      int j = (i + 1) % n;
      edges[i].dx = vertices[j].x - vertices[i].x;
      edges[i].dy = vertices[j].y - vertices[i].y;
    }
    computeAABB(); // 翻转后重新计算AABB
  }
}

bool Polygon::containsConvex(double x, double y) const
{
  int n = static_cast<int>(vertices.size());
  for (int i = 0; i < n; ++i) {
    double cross = edges[i].dx * (y - vertices[i].y)
                 - edges[i].dy * (x - vertices[i].x);
    if (cross < 0.0) {
      return false;
    }
  }
  return true;
}

bool Polygon::contains(double x, double y) const
{
  int n = static_cast<int>(vertices.size());
  if (n < 3) {
    return false;
  }
  bool inside = false;
  for (int i = 0, j = n - 1; i < n; j = i++) {
    double xi = vertices[i].x, yi = vertices[i].y;
    double xj = vertices[j].x, yj = vertices[j].y;
    double dy_ij = yj - yi;
    if (((yi > y) != (yj > y))) {
      double lhs = (x - xi) * dy_ij;
      double rhs = (xj - xi) * (y - yi);
      if (dy_ij > 0.0 ? (lhs < rhs) : (lhs > rhs)) {
        inside = !inside;
      }
    }
  }
  return inside;
}

bool Polygon::containsFast(double x, double y) const
{
  if (aabb_valid) {
    if (x < aabb_min_x || x > aabb_max_x ||
      y < aabb_min_y || y > aabb_max_y)
    {
      return false;
    }
  }
  if (is_convex) {
    return containsConvex(x, y);
  }
  return contains(x, y);
}

DodgeManagerNode::DodgeManagerNode(const rclcpp::NodeOptions & options)
: Node("dodge_manager", options),
  rng_(std::random_device{}())
{
  this->declare_parameter("enable_dodge", false);
  this->declare_parameter("global_frame", "odom");
  this->declare_parameter("robot_frame", "gimbal_yaw");
  this->declare_parameter("control_frequency", 20.0);

  this->declare_parameter("dodge_zone.x1", 0.0);
  this->declare_parameter("dodge_zone.y1", 0.0);
  this->declare_parameter("dodge_zone.x2", 5.0);
  this->declare_parameter("dodge_zone.y2", 0.0);
  this->declare_parameter("dodge_zone.x3", 5.0);
  this->declare_parameter("dodge_zone.y3", 5.0);
  this->declare_parameter("dodge_zone.x4", 0.0);
  this->declare_parameter("dodge_zone.y4", 5.0);

  this->declare_parameter("robot_status_topic", "/referee/robot_status");
  this->declare_parameter("tracker_target_topic", "/tracker/target");
  this->declare_parameter("cmd_vel_topic", "cmd_vel");
  this->declare_parameter("costmap_topic", "local_costmap/costmap_raw");
  this->declare_parameter("goal_pose_topic", "goal_pose");

  this->declare_parameter("robot_radius", 0.20);
  this->declare_parameter("dodge_distance", 1.0);
  this->declare_parameter("dodge_velocity", 1.5);
  this->declare_parameter("dodge_count", 3);
  this->declare_parameter("safety_margin", 0.15);
  this->declare_parameter("direction_samples", 36);
  this->declare_parameter("costmap_check_resolution", 0.05);
  this->declare_parameter("arrive_threshold", 0.15);

  // --- 获取参数 ---
  this->get_parameter("enable_dodge", enable_dodge_);
  this->get_parameter("global_frame", global_frame_);
  this->get_parameter("robot_frame", robot_frame_);
  this->get_parameter("control_frequency", control_frequency_);

  double x1, y1, x2, y2, x3, y3, x4, y4;
  this->get_parameter("dodge_zone.x1", x1);
  this->get_parameter("dodge_zone.y1", y1);
  this->get_parameter("dodge_zone.x2", x2);
  this->get_parameter("dodge_zone.y2", y2);
  this->get_parameter("dodge_zone.x3", x3);
  this->get_parameter("dodge_zone.y3", y3);
  this->get_parameter("dodge_zone.x4", x4);
  this->get_parameter("dodge_zone.y4", y4);

  dodge_zone_.vertices.resize(4);
  dodge_zone_.vertices[0].x = x1; dodge_zone_.vertices[0].y = y1;
  dodge_zone_.vertices[1].x = x2; dodge_zone_.vertices[1].y = y2;
  dodge_zone_.vertices[2].x = x3; dodge_zone_.vertices[2].y = y3;
  dodge_zone_.vertices[3].x = x4; dodge_zone_.vertices[3].y = y4;
  dodge_zone_.precompute();

  this->get_parameter("robot_status_topic", robot_status_topic_);
  this->get_parameter("tracker_target_topic", tracker_target_topic_);
  this->get_parameter("cmd_vel_topic", cmd_vel_topic_);
  this->get_parameter("costmap_topic", costmap_topic_);

  std::string goal_pose_topic;
  this->get_parameter("goal_pose_topic", goal_pose_topic);

  this->get_parameter("robot_radius", robot_radius_);
  this->get_parameter("dodge_distance", dodge_distance_);
  this->get_parameter("dodge_velocity", dodge_velocity_);
  this->get_parameter("dodge_count", dodge_count_);
  this->get_parameter("safety_margin", safety_margin_);
  this->get_parameter("direction_samples", direction_samples_);
  this->get_parameter("costmap_check_resolution", costmap_check_resolution_);
  this->get_parameter("arrive_threshold", arrive_threshold_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (enable_dodge_) {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    robot_status_sub_ = this->create_subscription<pb_rm_interfaces::msg::RobotStatus>(
      robot_status_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DodgeManagerNode::robotStatusCallback, this, std::placeholders::_1));

    tracker_target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
      tracker_target_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DodgeManagerNode::trackerTargetCallback, this, std::placeholders::_1));

    costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
      costmap_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DodgeManagerNode::costmapCallback, this, std::placeholders::_1));

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_pose_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&DodgeManagerNode::goalPoseCallback, this, std::placeholders::_1));

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_frequency_));
    control_timer_ = this->create_wall_timer(
      period_ns,
      std::bind(&DodgeManagerNode::controlLoop, this));
    RCLCPP_INFO(
      this->get_logger(),
      "DodgeManager 已启用 | freq=%.1fHz | dodge_count=%d | dist=%.2fm | vel=%.2fm/s",
      control_frequency_, dodge_count_, dodge_distance_, dodge_velocity_);
    RCLCPP_INFO(
      this->get_logger(),
      "闪避区域: (%.2f,%.2f) (%.2f,%.2f) (%.2f,%.2f) (%.2f,%.2f)",
      x1, y1, x2, y2, x3, y3, x4, y4);
    RCLCPP_INFO(this->get_logger(), "话题订阅: robot_status=%s tracker=%s costmap=%s goal=%s cmd_vel=%s",
      robot_status_topic_.c_str(), tracker_target_topic_.c_str(),
      costmap_topic_.c_str(), goal_pose_topic.c_str(), cmd_vel_topic_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "DodgeManager 未启用 (enable_dodge=false)");
  }
}


void DodgeManagerNode::robotStatusCallback(
  const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg)
{
  int current_hp = msg->current_hp;

  // 首次接收，只记录 HP
  if (previous_hp_ == -1) {
    previous_hp_ = current_hp;
    RCLCPP_INFO(this->get_logger(), "[RobotStatus] 首次收到血量: %d", current_hp);
    return;
  }

  bool hp_decreased = current_hp < previous_hp_;
  bool is_armor_hit = hp_decreased && (msg->hp_deduction_reason == 0);

  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "[RobotStatus] HP=%d, deduction_reason=%d",
    current_hp, msg->hp_deduction_reason);

  if (hp_decreased && !is_armor_hit) {
    RCLCPP_WARN(this->get_logger(),
      "[RobotStatus] HP 下降 %d→%d, 但 hp_deduction_reason=%d (非装甲板伤害，不触发闪避)",
      previous_hp_, current_hp, msg->hp_deduction_reason);
  }

  previous_hp_ = current_hp;

  if (is_armor_hit) {
    DodgeState cur_state = state_.load();
    const char* state_str = "UNKNOWN";
    switch (cur_state) {
      case DodgeState::IDLE:       state_str = "IDLE";       break;
      case DodgeState::MONITORING: state_str = "MONITORING"; break;
      case DodgeState::DODGING:    state_str = "DODGING";    break;
      case DodgeState::FOUND_STOP: state_str = "FOUND_STOP"; break;
    }
    RCLCPP_WARN(this->get_logger(),
      "[RobotStatus] 装甲板被打! HP: %d→%d | 当前状态: %s",
      previous_hp_ + (previous_hp_ - current_hp), current_hp, state_str);

    if (cur_state != DodgeState::MONITORING) {
      RCLCPP_WARN(this->get_logger(),
        "[RobotStatus] ⚠ 当前不在 MONITORING 状态，is_attacked_ 已置位但闪避不会触发！"
        " (需要先进入闪避区域才会响应)");
    }

    bool expected = false;
    if (is_attacked_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(this->get_logger(), "[RobotStatus] is_attacked_ 置位成功");
    } else {
      RCLCPP_WARN(this->get_logger(), "[RobotStatus] is_attacked_ 已为 true (上一次未处理)");
    }
  }
}

void DodgeManagerNode::trackerTargetCallback(
  const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  is_tracking_enemy_.store(msg->tracking);
}

void DodgeManagerNode::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  latest_costmap_ = msg;
}

void DodgeManagerNode::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pending_goal_mutex_);
  last_nav_goal_ = *msg;
  has_last_nav_goal_ = true;

  if (dodge_in_progress_.load()) {
    has_pending_goal_ = true;
    pending_goal_ = *msg;
    RCLCPP_WARN(
      this->get_logger(), "开闪！！");
    cancelNavigation();
  }
}


bool DodgeManagerNode::getCurrentPose(double & x, double & y, double & yaw)
{
  try {
    auto transform = tf_buffer_->lookupTransform(
      global_frame_, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(0.0));
    x = transform.transform.translation.x;
    y = transform.transform.translation.y;
    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    yaw = tf2::getYaw(q);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "获取位姿失败: %s", ex.what());
    return false;
  }
}

bool DodgeManagerNode::isInsideDodgeZone()
{
  double x, y, yaw;
  if (!getCachedPose(x, y, yaw)) {
    return false;
  }
  return dodge_zone_.containsFast(x, y);
}

bool DodgeManagerNode::getCachedPose(double & x, double & y, double & yaw)
{
  if (pose_cache_valid_) {
    x = cached_x_;
    y = cached_y_;
    yaw = cached_yaw_;
    return true;
  }
  if (!getCurrentPose(x, y, yaw)) {
    return false;
  }
  cached_x_ = x;
  cached_y_ = y;
  cached_yaw_ = yaw;
  pose_cache_valid_ = true;
  return true;
}

void DodgeManagerNode::invalidatePoseCache()
{
  pose_cache_valid_ = false;
}


void DodgeManagerNode::controlLoop()
{
  invalidatePoseCache();

  DodgeState cur = state_.load();

  // 每 5 秒打印一次当前状态，确认控制循环正在运行
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
    "[ControlLoop] 状态: %s | is_attacked=%d | is_tracking=%d | in_zone=%s",
    [cur]() -> const char* {
      switch (cur) {
        case DodgeState::IDLE:       return "IDLE";
        case DodgeState::MONITORING: return "MONITORING";
        case DodgeState::DODGING:    return "DODGING";
        case DodgeState::FOUND_STOP: return "FOUND_STOP";
        default: return "UNKNOWN";
      }
    }(),
    is_attacked_.load() ? 1 : 0,
    is_tracking_enemy_.load() ? 1 : 0,
    (cur == DodgeState::MONITORING || cur == DodgeState::DODGING) ? "YES" : "NO");

  if (cur != DodgeState::FOUND_STOP && is_tracking_enemy_.load()) {
    publishZeroVelocity();
    cancelNavigation();
    state_.store(DodgeState::FOUND_STOP);
    dodge_in_progress_.store(false);
    RCLCPP_WARN(this->get_logger(), "发现目标，停停停!");
    return;
  }

  switch (cur) {
    case DodgeState::IDLE:
      handleIdle();
      break;
    case DodgeState::MONITORING:
      handleMonitoring();
      break;
    case DodgeState::DODGING:
      handleDodging();
      break;
    case DodgeState::FOUND_STOP:
      handleFoundStop();
      break;
  }
}

void DodgeManagerNode::handleIdle()
{
  if (++idle_tick_counter_ < kIdleCheckInterval) {
    return;
  }
  idle_tick_counter_ = 0;

  if (isInsideDodgeZone()) {
    state_.store(DodgeState::MONITORING);
    RCLCPP_INFO(this->get_logger(), "进入闪避区域，开始监听");
  }
}

void DodgeManagerNode::handleMonitoring()
{
  if (!isInsideDodgeZone()) {
    state_.store(DodgeState::IDLE);
    idle_tick_counter_ = 0;
    RCLCPP_INFO(this->get_logger(), "离开闪避区域");
    return;
  }

  if (is_attacked_.load()) {
    is_attacked_.store(false);

    {
      std::lock_guard<std::mutex> lock(pending_goal_mutex_);
      if (has_last_nav_goal_) {
        double gx = last_nav_goal_.pose.position.x;
        double gy = last_nav_goal_.pose.position.y;
        if (!dodge_zone_.containsFast(gx, gy)) {
          RCLCPP_WARN(this->get_logger(), "我先走了哈！！");
          return;
        }
      }
    }

    RCLCPP_WARN(this->get_logger(), "被击打! 我闪！");
    dodge_in_progress_.store(true);
    current_dodge_count_ = 0;
    exec_state_ = DodgeExecState::PLAN;
    cancelNavigation();
    state_.store(DodgeState::DODGING);
  }
}

void DodgeManagerNode::handleDodging()
{
  is_attacked_.store(false);

  switch (exec_state_) {
    case DodgeExecState::PLAN: {
      if (current_dodge_count_ >= dodge_count_) {
        exec_state_ = DodgeExecState::DONE;
        return;
      }

      double dodge_angle;
      if (!findSafeDodgeDirection(dodge_angle)) {
        current_dodge_count_++;
        return;
      }

      double robot_x, robot_y, robot_yaw;
      if (!getCachedPose(robot_x, robot_y, robot_yaw)) {
        return;
      }

      dodge_target_x_ = robot_x + dodge_distance_ * std::cos(dodge_angle);
      dodge_target_y_ = robot_y + dodge_distance_ * std::sin(dodge_angle);
      dodge_move_timeout_ = (dodge_distance_ / dodge_velocity_) * 2.5;
      dodge_move_start_time_ = this->now();

      RCLCPP_INFO(this->get_logger(),
        "我闪 %d/%d: (%.2f,%.2f)->(%.2f,%.2f) 方向%.1f°",
        current_dodge_count_ + 1, dodge_count_,
        robot_x, robot_y, dodge_target_x_, dodge_target_y_,
        dodge_angle * 180.0 / M_PI);

      exec_state_ = DodgeExecState::MOVE;
      break;
    }

    case DodgeExecState::MOVE: {
      double cur_x, cur_y, cur_yaw;
      if (!getCachedPose(cur_x, cur_y, cur_yaw)) {
        publishZeroVelocity();
        return;
      }

      // 已离开闪避区域 → 终止
      if (!dodge_zone_.containsFast(cur_x, cur_y)) {
        publishZeroVelocity();
        RCLCPP_WARN(this->get_logger(), "闪避中离开区域，终止闪避序列");
        exec_state_ = DodgeExecState::DONE;
        return;
      }

      double dx = dodge_target_x_ - cur_x;
      double dy = dodge_target_y_ - cur_y;
      double dist = std::hypot(dx, dy);
      double elapsed = (this->now() - dodge_move_start_time_).seconds();

      // 到达目标 或 超时 → 直接进入下一次 PLAN（无 PAUSE）
      if (dist < arrive_threshold_ || elapsed > dodge_move_timeout_) {
        publishZeroVelocity();
        current_dodge_count_++;
        exec_state_ = DodgeExecState::PLAN;
        return;
      }

      // P-control: 每帧根据 (target - cur) 重新计算全局速度方向
      double angle = std::atan2(dy, dx);
      double global_vx = dodge_velocity_ * std::cos(angle);
      double global_vy = dodge_velocity_ * std::sin(angle);

      // 预测下一步是否出界
      double dt_predict = 2.0 / control_frequency_;
      double predict_x = cur_x + global_vx * dt_predict;
      double predict_y = cur_y + global_vy * dt_predict;
      if (!dodge_zone_.containsFast(predict_x, predict_y)) {
        publishZeroVelocity();
        current_dodge_count_++;
        exec_state_ = DodgeExecState::PLAN;
        return;
      }

      // cur_yaw = gimbal_yaw 在 odom 下的朝向（TF: odom→gimbal_yaw）
      double cos_yaw = std::cos(cur_yaw);
      double sin_yaw = std::sin(cur_yaw);
      double local_vx =  cos_yaw * global_vx + sin_yaw * global_vy;
      double local_vy = -sin_yaw * global_vx + cos_yaw * global_vy;
      publishVelocity(local_vx, local_vy);
      break;
    }

    case DodgeExecState::DONE: {
      RCLCPP_INFO(this->get_logger(), "闪避完成，恢复导航");
      publishZeroVelocity();
      dodge_in_progress_.store(false);
      resumeNavigation();

      if (isInsideDodgeZone()) {
        state_.store(DodgeState::MONITORING);
      } else {
        state_.store(DodgeState::IDLE);
      }
      break;
    }
  }
}

void DodgeManagerNode::handleFoundStop()
{

  if (!is_tracking_enemy_.load()) {
    RCLCPP_INFO(this->get_logger(), "目标丢失，恢复导航");
    resumeNavigation();
    if (isInsideDodgeZone()) {
      state_.store(DodgeState::MONITORING);
    } else {
      state_.store(DodgeState::IDLE);
    }
  }
}

bool DodgeManagerNode::findSafeDodgeDirection(double & out_angle)
{
  nav2_msgs::msg::Costmap::SharedPtr costmap_copy;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap_copy = latest_costmap_;
  }
  if (!costmap_copy) {
    RCLCPP_WARN(this->get_logger(), "无 costmap 数据");
    return false;
  }

  double robot_x, robot_y, robot_yaw;
  if (!getCachedPose(robot_x, robot_y, robot_yaw)) {
    return false;
  }

  const auto & info = costmap_copy->metadata;
  const double resolution = info.resolution;
  if (resolution < 1e-6) {
    RCLCPP_WARN(this->get_logger(), "costmap resolution 异常: %.9f", resolution);
    return false;
  }
  const double inv_resolution = 1.0 / resolution;
  const double origin_x = info.origin.position.x;
  const double origin_y = info.origin.position.y;
  const int width = static_cast<int>(info.size_x);
  const int height = static_cast<int>(info.size_y);
  const auto & data = costmap_copy->data;
  const size_t data_size = data.size();

  const double endpoint_margin = safety_margin_;

  auto costmapBlocked = [&](double px, double py) -> bool {
    const int mx = static_cast<int>(std::floor((px - origin_x) * inv_resolution));
    const int my = static_cast<int>(std::floor((py - origin_y) * inv_resolution));
    if (mx < 0 || mx >= width || my < 0 || my >= height) {
      return true;
    }
    const size_t idx = static_cast<size_t>(my) * static_cast<size_t>(width)
      + static_cast<size_t>(mx);
    return (idx < data_size && data[idx] >= 253);
  };

  std::vector<double> safe_angles;
  safe_angles.reserve(direction_samples_);
  const double angle_step = 2.0 * M_PI / direction_samples_;

  for (int i = 0; i < direction_samples_; ++i) {
    const double angle = i * angle_step;
    const double cos_a = std::cos(angle);
    const double sin_a = std::sin(angle);
    const double end_x = robot_x + dodge_distance_ * cos_a;
    const double end_y = robot_y + dodge_distance_ * sin_a;

    // 合并路径检查和 zone 检查
    bool blocked = false;
    constexpr int zone_check_samples = 5;
    for (int s = 1; s <= zone_check_samples; ++s) {
      double t = static_cast<double>(s) / zone_check_samples;
      double cx = robot_x + dodge_distance_ * t * cos_a;
      double cy = robot_y + dodge_distance_ * t * sin_a;
      if (!dodge_zone_.containsFast(cx, cy) || costmapBlocked(cx, cy)) {
        blocked = true;
        break;
      }
    }
    if (blocked) continue;

    // 终点 margin 检查
    if (costmapBlocked(end_x + endpoint_margin, end_y) ||
        costmapBlocked(end_x - endpoint_margin, end_y) ||
        costmapBlocked(end_x, end_y + endpoint_margin) ||
        costmapBlocked(end_x, end_y - endpoint_margin))
    {
      continue;
    }

    safe_angles.push_back(angle);
  }

  if (safe_angles.empty()) {
    RCLCPP_WARN(this->get_logger(), "无安全方向 (0/%d)", direction_samples_);
    return false;
  }

  std::uniform_int_distribution<size_t> dist(0, safe_angles.size() - 1);
  out_angle = safe_angles[dist(rng_)];

  RCLCPP_INFO(this->get_logger(), "选择方向 %.1f° (%zu/%d 可用)",
    out_angle * 180.0 / M_PI, safe_angles.size(), direction_samples_);
  return true;
}

void DodgeManagerNode::publishZeroVelocity()
{
  geometry_msgs::msg::Twist cmd;
  cmd_vel_pub_->publish(cmd);
}

void DodgeManagerNode::publishVelocity(double vx, double vy)
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = vx;
  cmd.linear.y = vy;
  cmd_vel_pub_->publish(cmd);
}

void DodgeManagerNode::cancelNavigation()
{
  if (!nav_client_ || !nav_client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "navigate_to_pose action server 不可用");
    return;
  }
  nav_client_->async_cancel_all_goals();
  RCLCPP_INFO(this->get_logger(), "已取消导航");
}

void DodgeManagerNode::resumeNavigation()
{
  std::lock_guard<std::mutex> lock(pending_goal_mutex_);

  geometry_msgs::msg::PoseStamped goal_to_send;
  bool should_send = false;

  if (has_pending_goal_) {
    goal_to_send = pending_goal_;
    has_pending_goal_ = false;
    should_send = true;
    RCLCPP_INFO(this->get_logger(), "恢复缓存目标: (%.2f, %.2f)",
      goal_to_send.pose.position.x, goal_to_send.pose.position.y);
  } else if (has_last_nav_goal_) {
    goal_to_send = last_nav_goal_;
    should_send = true;
    RCLCPP_INFO(this->get_logger(), "恢复闪避前目标: (%.2f, %.2f)",
      goal_to_send.pose.position.x, goal_to_send.pose.position.y);
  }

  if (should_send && nav_client_) {
    if (!nav_client_->action_server_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "action server 不可用，无法恢复导航");
      return;
    }
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal_to_send;
    goal_msg.pose.header.stamp.sec = 0;
    goal_msg.pose.header.stamp.nanosec = 0;
    nav_client_->async_send_goal(goal_msg);
  } else if (!should_send) {
    RCLCPP_INFO(this->get_logger(), "无可恢复目标，等待上层发送");
  }
}

}  // namespace dodge_manager


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dodge_manager::DodgeManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
