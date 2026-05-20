// Copyright 2025 Pan - Apache-2.0
// M-detector ROS 2 node.

#include <cmath>
#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "m-detector/DynObjFilter.h"
#include "m-detector/types.h"

class DynamicPointDetectorNode : public rclcpp::Node
{
public:
  DynamicPointDetectorNode()
  : Node("dynamic_point_detector")
  {
    declare_parameter("pointcloud_topic", "registered_scan");
    declare_parameter("odometry_topic", "odometry");
    declare_parameter("output_topic", "dynamic_points");
    declare_parameter("frame_topic", "m_detector/frame_out");
    declare_parameter("static_points_topic", "m_detector/std_points");

    dyn_obj_filter_ = std::make_shared<DynObjFilter>();
    dyn_obj_filter_->init(this);

    const auto pointcloud_topic = get_parameter("pointcloud_topic").as_string();
    const auto odometry_topic = get_parameter("odometry_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();
    const auto frame_topic = get_parameter("frame_topic").as_string();
    const auto static_points_topic = get_parameter("static_points_topic").as_string();

    dyn_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
    frame_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(frame_topic, 10);
    static_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(static_points_topic, 10);

    pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&DynamicPointDetectorNode::onCloud, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, rclcpp::SensorDataQoS(),
      std::bind(&DynamicPointDetectorNode::onOdom, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&DynamicPointDetectorNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "dynamic_point_detector: %s + %s -> %s, debug=[%s, %s]",
      pointcloud_topic.c_str(), odometry_topic.c_str(), output_topic.c_str(),
      frame_topic.c_str(), static_points_topic.c_str());
  }

private:
  static constexpr std::size_t kMaxCloudQueueSize = 5;

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

    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
    odom_rots_.push_back(cur_rot_);
    odom_positions_.push_back(cur_pos_);
    odom_stamps_.push_back(stamp);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    pcl::fromROSMsg(*msg, *cloud);

    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
    cloud_buffer_.push_back({cloud, stamp});
    while (cloud_buffer_.size() > kMaxCloudQueueSize) {
      cloud_buffer_.pop_front();
    }
  }

  void onTimer()
  {
    if (cloud_buffer_.empty() || odom_stamps_.size() < 2) {
      return;
    }

    const double latest_odom_stamp = odom_stamps_.back();

    std::size_t selected_idx = cloud_buffer_.size();
    for (std::size_t i = cloud_buffer_.size(); i > 0; --i) {
      const std::size_t idx = i - 1;
      const double cloud_stamp = cloud_buffer_[idx].second;
      if (latest_odom_stamp - cloud_stamp >= dyn_obj_filter_->buffer_delay) {
        selected_idx = idx;
        break;
      }
    }

    if (selected_idx == cloud_buffer_.size()) {
      return;
    }

    auto [cloud, stamp] = cloud_buffer_[selected_idx];
    cloud_buffer_.erase(cloud_buffer_.begin(), cloud_buffer_.begin() + selected_idx + 1);

    size_t best_idx = 0;
    double best_dt = std::fabs(odom_stamps_[0] - stamp);
    for (size_t i = 1; i < odom_stamps_.size(); ++i) {
      const double dt = std::fabs(odom_stamps_[i] - stamp);
      if (dt < best_dt) {
        best_dt = dt;
        best_idx = i;
      }
    }

    dyn_obj_filter_->filter(cloud, odom_rots_[best_idx], odom_positions_[best_idx], stamp);
    dyn_obj_filter_->publish_dyn(dyn_pub_, frame_pub_, static_pub_, stamp);

    while (odom_stamps_.size() > 200) {
      odom_rots_.pop_front();
      odom_positions_.pop_front();
      odom_stamps_.pop_front();
    }
  }

  std::shared_ptr<DynObjFilter> dyn_obj_filter_;
  Eigen::Matrix3d cur_rot_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d cur_pos_ = Eigen::Vector3d::Zero();

  std::deque<std::pair<PointCloudXYZI::Ptr, double>> cloud_buffer_;
  std::deque<Eigen::Matrix3d> odom_rots_;
  std::deque<Eigen::Vector3d> odom_positions_;
  std::deque<double> odom_stamps_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dyn_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frame_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicPointDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
