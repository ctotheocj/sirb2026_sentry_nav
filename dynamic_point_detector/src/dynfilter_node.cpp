// Copyright 2025 Pan — Apache-2.0
// M-detector ROS2 主节点 (移植自 dynfilter_with_odom.cpp)
// 订阅: pointcloud_topic (PointCloud2) + odometry_topic (Odometry)
// Legacy standalone M-detector node. The installed executable is built from
// dynamic_point_detector_node.cpp.
// 发布: dynamic_points  (动态点云)
//       m_detector/std_points (静态点云)

#include <deque>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "m-detector/types.h"
#include "m-detector/DynObjFilter.h"

using namespace std;

class DynFilterNode : public rclcpp::Node
{
public:
  DynFilterNode() : Node("dynamic_point_detector")
  {
    declare_parameter("pointcloud_topic", "/cloud_registered");
    declare_parameter("odometry_topic",   "/Odometry");

    DynObjFilt_ = std::make_shared<DynObjFilter>();
    DynObjFilt_->init(this);

    pub_dyn_  = create_publisher<sensor_msgs::msg::PointCloud2>("dynamic_points", 10);
    pub_frame_= create_publisher<sensor_msgs::msg::PointCloud2>("m_detector/frame_out", 10);
    pub_std_  = create_publisher<sensor_msgs::msg::PointCloud2>("m_detector/std_points", 10);

    pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("pointcloud_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&DynFilterNode::onCloud, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odometry_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&DynFilterNode::onOdom, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&DynFilterNode::onTimer, this));

    RCLCPP_INFO(get_logger(), "dynamic_point_detector (M-detector ROS2) started");
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    Eigen::Quaterniond q(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);
    cur_rot_ = q.matrix();
    cur_pos_ << msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z;
    double t = rclcpp::Time(msg->header.stamp).seconds();
    buf_rots_.push_back(cur_rot_);
    buf_poss_.push_back(cur_pos_);
    buf_times_.push_back(t);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    pcl::fromROSMsg(*msg, *cloud);
    double t = rclcpp::Time(msg->header.stamp).seconds();
    buf_pcs_.push_back({cloud, t});
  }

  void onTimer()
  {
    if (buf_pcs_.empty() || buf_times_.size() < 2) { return; }

    double pc_t = buf_pcs_.front().second;
    double now  = buf_times_.back();
    // M-Detector buffer delay: wait until buffer_delay seconds have elapsed
    if (now - pc_t < DynObjFilt_->buffer_delay) { return; }

    auto [cur_pc, cur_t] = buf_pcs_.front(); buf_pcs_.pop_front();

    // Find closest odometry pose by timestamp
    size_t best = 0;
    double best_dt = std::fabs(buf_times_[0] - cur_t);
    for (size_t i = 1; i < buf_times_.size(); ++i) {
      double dt = std::fabs(buf_times_[i] - cur_t);
      if (dt < best_dt) { best_dt = dt; best = i; }
    }

    DynObjFilt_->filter(cur_pc, buf_rots_[best], buf_poss_[best], cur_t);
    DynObjFilt_->publish_dyn(pub_dyn_, pub_frame_, pub_std_, cur_t);

    // Trim old odometry entries
    while (buf_times_.size() > 200) {
      buf_rots_.pop_front(); buf_poss_.pop_front(); buf_times_.pop_front();
    }
  }

  std::shared_ptr<DynObjFilter> DynObjFilt_;
  Eigen::Matrix3d cur_rot_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d cur_pos_ = Eigen::Vector3d::Zero();

  std::deque<std::pair<PointCloudXYZI::Ptr, double>> buf_pcs_;
  std::deque<Eigen::Matrix3d>      buf_rots_;
  std::deque<Eigen::Vector3d>      buf_poss_;
  std::deque<double>               buf_times_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dyn_, pub_frame_, pub_std_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynFilterNode>());
  rclcpp::shutdown();
  return 0;
}
