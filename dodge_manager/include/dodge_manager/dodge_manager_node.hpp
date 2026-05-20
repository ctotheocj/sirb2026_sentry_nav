// Copyright 2025 Pan
// Licensed under the Apache License, Version 2.0

#ifndef DODGE_MANAGER__DODGE_MANAGER_NODE_HPP_
#define DODGE_MANAGER__DODGE_MANAGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <pb_rm_interfaces/msg/robot_status.hpp>
#include <auto_aim_interfaces/msg/target.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <mutex>
#include <atomic>
#include <vector>
#include <cmath>
#include <random>

namespace dodge_manager
{

struct Polygon
{
  std::vector<geometry_msgs::msg::Point> vertices;  // 4 vertices
  bool contains(double x, double y) const;

  // AABB 快速拒绝
  double aabb_min_x{0.0}, aabb_min_y{0.0};
  double aabb_max_x{0.0}, aabb_max_y{0.0};
  bool aabb_valid{false};
  void computeAABB();
  bool containsFast(double x, double y) const;

  // 凸多边形预计算（边向量 + 法线方向），用叉积法代替射线法
  bool is_convex{false};
  struct Edge {
    double dx, dy;  // 边向量 (v[i+1] - v[i])
  };
  std::vector<Edge> edges;
  void precompute();  // 计算 AABB + 凸性 + 边向量
  bool containsConvex(double x, double y) const;  // 叉积法，无除法
};

enum class DodgeState
{
  IDLE,           // 正常导航，不干预
  MONITORING,     // 在区域内，监听话题
  DODGING,        // 正在执行闪避动作
  FOUND_STOP,     // 发现目标，停车
};

// 闪避执行子状态（非阻塞）
enum class DodgeExecState
{
  PLAN,           // 计算闪避方向
  MOVE,           // 移动中（P-control 闭环）
  DONE,           // 本轮闪避序列完成
};

class DodgeManagerNode : public rclcpp::Node
{
public:
  explicit DodgeManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DodgeManagerNode() = default;

private:
  void controlLoop();
  void handleIdle();
  void handleMonitoring();
  void handleDodging();
  void handleFoundStop();

  bool findSafeDodgeDirection(double & out_angle);
  void publishZeroVelocity();
  void publishVelocity(double vx, double vy);

  bool getCurrentPose(double & x, double & y, double & yaw);
  bool isInsideDodgeZone();

  bool getCachedPose(double & x, double & y, double & yaw);
  void invalidatePoseCache();
  bool pose_cache_valid_{false};
  double cached_x_{0.0}, cached_y_{0.0}, cached_yaw_{0.0};

  void robotStatusCallback(const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg);
  void trackerTargetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg);
  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);

  void cancelNavigation();
  void resumeNavigation();

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // ===================== 参数 =====================

  bool enable_dodge_{false};

  Polygon dodge_zone_;

  std::string robot_status_topic_;
  std::string tracker_target_topic_;
  std::string cmd_vel_topic_;
  std::string costmap_topic_;

  double robot_radius_{0.25};
  double dodge_distance_{1.0};
  double dodge_velocity_{1.5};
  int dodge_count_{3};
  double safety_margin_{0.15};
  int direction_samples_{36};
  double costmap_check_resolution_{0.05};

  std::string global_frame_{"odom"};
  std::string robot_frame_{"gimbal_yaw"};

  double arrive_threshold_{0.15};

  // ===================== 主状态 =====================

  std::atomic<DodgeState> state_{DodgeState::IDLE};
  std::atomic<bool> is_attacked_{false};
  std::atomic<bool> is_tracking_enemy_{false};
  std::atomic<bool> dodge_in_progress_{false};
  int previous_hp_{-1};  // 上一帧 HP，-1 表示未初始化
  int current_dodge_count_{0};
  int idle_tick_counter_{0};
  static constexpr int kIdleCheckInterval = 5;

  // ===================== 闪避执行子状态（非阻塞） =====================

  DodgeExecState exec_state_{DodgeExecState::PLAN};
  double dodge_target_x_{0.0};
  double dodge_target_y_{0.0};
  rclcpp::Time dodge_move_start_time_;
  double dodge_move_timeout_{0.0};

  // ===================== 导航目标缓存 =====================

  bool has_pending_goal_{false};
  geometry_msgs::msg::PoseStamped pending_goal_;
  std::mutex pending_goal_mutex_;

  geometry_msgs::msg::PoseStamped last_nav_goal_;
  bool has_last_nav_goal_{false};

  // ===================== ROS 接口 =====================

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr tracker_target_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  std::mutex costmap_mutex_;
  nav2_msgs::msg::Costmap::SharedPtr latest_costmap_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mt19937 rng_;

  double control_frequency_{20.0};
};

}  // namespace dodge_manager

#endif  // DODGE_MANAGER__DODGE_MANAGER_NODE_HPP_
