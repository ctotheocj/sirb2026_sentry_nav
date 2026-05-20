// Copyright 2025 Pan — Apache-2.0
//
// dynamic_obstacle_tracker node
// 流程:
//   1. PCL EuclideanClusterExtraction → 聚类
//   2. 最近邻帧间关联
//   3. 卡尔曼滤波 (匀速模型, 状态: x,y,vx,vy)
//   4. 匀速外推 N 步预测
//   5. 发布 TrackedObstacleArray

#include <cmath>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "dynamic_obstacle_tracker/hungarian.hpp"
#include "sentry_nav_interfaces/msg/tracked_obstacle.hpp"
#include "sentry_nav_interfaces/msg/tracked_obstacle_array.hpp"

namespace dynamic_obstacle_tracker
{

struct Track {
  int    id;
  double radius{0.3};
  int    age{0};
  int    missed{0};

  // Kalman filter: state = [x, y, vx, vy]
  Eigen::Vector4d X{0,0,0,0};   // state estimate
  Eigen::Matrix4d P;             // covariance

  double x()  const { return X(0); }
  double y()  const { return X(1); }
  double vx() const { return X(2); }
  double vy() const { return X(3); }

  Track(int id_, double cx, double cy, double r)
  : id(id_), radius(r)
  {
    X << cx, cy, 0, 0;
    P = Eigen::Matrix4d::Identity() * 1.0;
  }

  // Predict step (constant velocity)
  void predict(double dt)
  {
    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0,2) = dt; F(1,3) = dt;
    // process noise
    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    Q(2,2) = Q(3,3) = 0.5;
    X = F * X;
    P = F * P * F.transpose() + Q;
  }

  // Update step (observe position)
  void update(double mx, double my)
  {
    Eigen::Matrix<double,2,4> H = Eigen::Matrix<double,2,4>::Zero();
    H(0,0) = 1; H(1,1) = 1;
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 0.1;
    Eigen::Matrix2d S = H * P * H.transpose() + R;
    Eigen::Matrix<double,4,2> K = P * H.transpose() * S.inverse();
    Eigen::Vector2d z(mx, my);
    X = X + K * (z - H * X);
    P = (Eigen::Matrix4d::Identity() - K * H) * P;
  }
};

class ObstacleTrackerNode : public rclcpp::Node
{
public:
  explicit ObstacleTrackerNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
  : Node("obstacle_tracker", opts)
  {
    declare_parameter("input_topic",   "dynamic_points");
    declare_parameter("output_topic",  "dynamic_obstacles");
    declare_parameter("viz_topic",     "vis/tracked_obstacles");
    declare_parameter("cluster_tolerance",   0.4);   // EuclideanCluster 距离阈值
    declare_parameter("cluster_min_size",    5);
    declare_parameter("cluster_max_size",    500);
    declare_parameter("match_dist_max",      1.5);   // 最近邻匹配门限 (m)
    declare_parameter("vel_alpha",           0.4);   // 低通系数 (0=全平滑,1=无滤波)
    declare_parameter("max_missed_frames",   5);
    declare_parameter("confirm_frames",      3);
    declare_parameter("prediction_steps",    20);    // 预测步数
    declare_parameter("prediction_dt",       0.1);   // 预测步长 (s)
    declare_parameter("max_output_obstacles",7);

    cluster_tol_   = get_parameter("cluster_tolerance").as_double();
    cluster_min_   = get_parameter("cluster_min_size").as_int();
    cluster_max_   = get_parameter("cluster_max_size").as_int();
    match_dist_    = get_parameter("match_dist_max").as_double();
    vel_alpha_     = get_parameter("vel_alpha").as_double();
    max_missed_    = get_parameter("max_missed_frames").as_int();
    confirm_frames_= get_parameter("confirm_frames").as_int();
    pred_steps_    = get_parameter("prediction_steps").as_int();
    pred_dt_       = get_parameter("prediction_dt").as_double();
    max_output_    = get_parameter("max_output_obstacles").as_int();

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("input_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&ObstacleTrackerNode::onCloud, this, std::placeholders::_1));

    obs_pub_ = create_publisher<sentry_nav_interfaces::msg::TrackedObstacleArray>(
      get_parameter("output_topic").as_string(), 10);
    viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      get_parameter("viz_topic").as_string(), 10);

    RCLCPP_INFO(get_logger(), "obstacle_tracker (M-detector): %s → %s",
      get_parameter("input_topic").as_string().c_str(),
      get_parameter("output_topic").as_string().c_str());
  }

private:
  struct Cluster { double x, y, radius; };

  std::vector<Cluster> extractClusters(const sensor_msgs::msg::PointCloud2::SharedPtr & msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    if (!cloud || cloud->empty()) {
      return {};
    }

    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tol_);
    ec.setMinClusterSize(cluster_min_);
    ec.setMaxClusterSize(cluster_max_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(indices);

    std::vector<Cluster> result;
    result.reserve(indices.size());
    for (const auto & idx : indices) {
      double sx = 0, sy = 0;
      for (int i : idx.indices) { sx += cloud->points[i].x; sy += cloud->points[i].y; }
      double cx = sx / idx.indices.size(), cy = sy / idx.indices.size();
      double r = 0;
      for (int i : idx.indices) {
        double d = std::hypot(cloud->points[i].x - cx, cloud->points[i].y - cy);
        if (d > r) { r = d; }
      }
      result.push_back({cx, cy, std::max(r + 0.05, 0.15)});
    }
    return result;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    double dt = 0.1;
    if (last_stamp_.nanoseconds() > 0) {
      dt = (rclcpp::Time(msg->header.stamp) - last_stamp_).seconds();
      if (dt <= 0.0 || dt > 1.0) { dt = 0.1; }
    }
    last_stamp_ = msg->header.stamp;

    auto clusters = extractClusters(msg);

    std::vector<bool> matched_track(tracks_.size(), false);
    std::vector<bool> matched_det(clusters.size(), false);

    for (size_t ti = 0; ti < tracks_.size(); ++ti) {
      tracks_[ti].predict(dt);
    }

    const size_t nT = tracks_.size(), nD = clusters.size();
    if (nT > 0 && nD > 0) {
      std::vector<std::vector<double>> cost(nT, std::vector<double>(nD, 1e9));
      for (size_t ti = 0; ti < nT; ++ti)
        for (size_t dj = 0; dj < nD; ++dj) {
          double d = std::hypot(tracks_[ti].x() - clusters[dj].x,
                                tracks_[ti].y() - clusters[dj].y);
          if (d < match_dist_) cost[ti][dj] = d;
        }
      std::vector<int> assignment;
      HungarianAlgorithm::solve(cost, assignment);
      for (size_t ti = 0; ti < nT; ++ti) {
        int j = (ti < assignment.size()) ? assignment[ti] : -1;
        if (j < 0 || j >= static_cast<int>(nD) || cost[ti][static_cast<size_t>(j)] >= 1e8) continue;
        matched_track[ti] = true;
        matched_det[static_cast<size_t>(j)] = true;
        tracks_[ti].update(clusters[static_cast<size_t>(j)].x, clusters[static_cast<size_t>(j)].y);
        tracks_[ti].radius = clusters[static_cast<size_t>(j)].radius;
        tracks_[ti].missed = 0;
        ++tracks_[ti].age;
      }
    }

    // 未匹配 track: missed++
    for (size_t ti = 0; ti < tracks_.size(); ++ti) {
      if (!matched_track[ti]) { ++tracks_[ti].missed; }
    }

    for (size_t dj = 0; dj < clusters.size(); ++dj) {
      if (!matched_det[dj]) {
        tracks_.emplace_back(next_id_++, clusters[dj].x, clusters[dj].y,
                             clusters[dj].radius);
      }
    }

    // 删除丢失过久的 track
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
      [this](const Track & t){ return t.missed > max_missed_; }), tracks_.end());

    sentry_nav_interfaces::msg::TrackedObstacleArray out;
    out.header = msg->header;
    int count = 0;
    for (const auto & t : tracks_) {
      if (t.age < confirm_frames_ || count >= max_output_) { continue; }
      sentry_nav_interfaces::msg::TrackedObstacle obs;
      obs.id         = t.id;
      obs.x          = t.x();
      obs.y          = t.y();
      obs.vx         = t.vx();
      obs.vy         = t.vy();
      obs.radius     = t.radius;
      obs.confidence = std::min(1.0f, static_cast<float>(t.age) / 10.0f);
      obs.prediction_dt = static_cast<float>(pred_dt_);
      obs.predicted_positions.reserve(pred_steps_);
      for (int k = 1; k <= pred_steps_; ++k) {
        geometry_msgs::msg::Point p;
        p.x = t.x() + t.vx() * k * pred_dt_;
        p.y = t.y() + t.vy() * k * pred_dt_;
        p.z = 0.0;
        obs.predicted_positions.push_back(p);
      }
      out.obstacles.push_back(obs);
      ++count;
    }
    obs_pub_->publish(out);
    if (viz_pub_->get_subscription_count() > 0) { publishViz(out); }
  }

  void publishViz(const sentry_nav_interfaces::msg::TrackedObstacleArray & msg)
  {
    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    del.header = msg.header;
    ma.markers.push_back(del);

    int mid = 0;
    for (const auto & obs : msg.obstacles) {
      visualization_msgs::msg::Marker m;
      m.header = msg.header; m.ns = "obstacles"; m.id = mid++;
      m.type   = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = obs.x; m.pose.position.y = obs.y; m.pose.position.z = 0.5;
      m.pose.orientation.w = 1.0;
      m.scale.x = obs.radius * 2.0; m.scale.y = obs.radius * 2.0; m.scale.z = 1.0;
      m.color.r = 1.0f; m.color.g = 0.4f; m.color.b = 0.0f; m.color.a = 0.7f;
      m.lifetime = rclcpp::Duration(0, 300'000'000);
      ma.markers.push_back(m);

      // 速度箭头
      visualization_msgs::msg::Marker arrow;
      arrow.header = msg.header; arrow.ns = "velocity"; arrow.id = mid++;
      arrow.type   = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      geometry_msgs::msg::Point p0, p1;
      p0.x = obs.x; p0.y = obs.y; p0.z = 0.5;
      p1.x = obs.x + obs.vx; p1.y = obs.y + obs.vy; p1.z = 0.5;
      arrow.points.push_back(p0); arrow.points.push_back(p1);
      arrow.scale.x = 0.05; arrow.scale.y = 0.1; arrow.scale.z = 0.1;
      arrow.color.r = 0.2f; arrow.color.g = 1.0f; arrow.color.b = 0.2f; arrow.color.a = 0.9f;
      arrow.lifetime = rclcpp::Duration(0, 300'000'000);
      ma.markers.push_back(arrow);

      // ID 文字
      visualization_msgs::msg::Marker text;
      text.header = msg.header; text.ns = "id_text"; text.id = mid++;
      text.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = obs.x; text.pose.position.y = obs.y; text.pose.position.z = 1.5;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.3;
      text.color.r = 1.0f; text.color.g = 1.0f; text.color.b = 1.0f; text.color.a = 1.0f;
      text.text = "ID:" + std::to_string(obs.id);
      text.lifetime = rclcpp::Duration(0, 300'000'000);
      ma.markers.push_back(text);
      if (!obs.predicted_positions.empty()) {
        visualization_msgs::msg::Marker line;
        line.header = msg.header; line.ns = "predictions"; line.id = mid++;
        line.type   = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.03;
        line.color.r = 1.0f; line.color.g = 1.0f; line.color.b = 0.0f; line.color.a = 0.5f;
        line.lifetime = rclcpp::Duration(0, 300'000'000);
        geometry_msgs::msg::Point start; start.x = obs.x; start.y = obs.y; start.z = 0.5;
        line.points.push_back(start);
        for (const auto & p : obs.predicted_positions) {
          geometry_msgs::msg::Point pt; pt.x = p.x; pt.y = p.y; pt.z = 0.5;
          line.points.push_back(pt);
        }
        ma.markers.push_back(line);
      }
    }
    viz_pub_->publish(ma);
  }

  double cluster_tol_{0.4};
  int    cluster_min_{5}, cluster_max_{500};
  double match_dist_{1.5};
  double vel_alpha_{0.4};
  int    max_missed_{5}, confirm_frames_{3};
  int    pred_steps_{20};
  double pred_dt_{0.1};
  int    max_output_{7};

  int next_id_{0};
  std::vector<Track> tracks_;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sentry_nav_interfaces::msg::TrackedObstacleArray>::SharedPtr obs_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
};

}  // namespace dynamic_obstacle_tracker

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dynamic_obstacle_tracker::ObstacleTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
