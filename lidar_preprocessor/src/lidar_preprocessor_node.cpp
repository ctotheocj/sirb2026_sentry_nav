#include <algorithm>
#include <string>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "lidar_preprocessor/ground_filter.hpp"

namespace lidar_preprocessor
{

class LidarPreprocessorNode : public rclcpp::Node
{
public:
  explicit LidarPreprocessorNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
  : Node("lidar_preprocessor", opts)
  {
    declare_parameter("pointcloud_topic", "registered_scan");
    declare_parameter("output_topic", "obstacle_cloud");
    declare_parameter("voxel_leaf_size", 0.08);
    declare_parameter("ground_cell_size", 0.4);
    declare_parameter("ground_low_percentile", 0.05);
    declare_parameter("cell_min_points", 5);
    declare_parameter("obstacle_min_height", 0.05);
    declare_parameter("obstacle_max_height", 1.80);
    declare_parameter("visualization_min_height", -0.20);
    declare_parameter("visualization_max_height", 1.80);
    declare_parameter("enable_normal_check", true);
    declare_parameter("normal_min_neighbors", 5);
    declare_parameter("ground_normal_cos_thresh", 0.966);
    declare_parameter("slope_tolerance_height", 0.15);

    voxel_leaf_ = get_parameter("voxel_leaf_size").as_double();
    gf_.cell_size                = get_parameter("ground_cell_size").as_double();
    gf_.low_percentile           = get_parameter("ground_low_percentile").as_double();
    gf_.cell_min_points          = get_parameter("cell_min_points").as_int();
    gf_.obstacle_min_height      = get_parameter("obstacle_min_height").as_double();
    gf_.obstacle_max_height      = get_parameter("obstacle_max_height").as_double();
    visualization_min_height_    = get_parameter("visualization_min_height").as_double();
    visualization_max_height_    = get_parameter("visualization_max_height").as_double();
    gf_.enable_normal_check      = get_parameter("enable_normal_check").as_bool();
    gf_.normal_min_neighbors     = get_parameter("normal_min_neighbors").as_int();
    gf_.ground_normal_cos_thresh = get_parameter("ground_normal_cos_thresh").as_double();
    gf_.slope_tolerance_height   = get_parameter("slope_tolerance_height").as_double();
    if (visualization_min_height_ > visualization_max_height_) {
      std::swap(visualization_min_height_, visualization_max_height_);
    }

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("pointcloud_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&LidarPreprocessorNode::onCloud, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      get_parameter("output_topic").as_string(),
      rclcpp::QoS(5).reliable().durability_volatile());
    vis_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "vis/obstacle_cloud", rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "lidar_preprocessor: %s → %s",
      get_parameter("pointcloud_topic").as_string().c_str(),
      get_parameter("output_topic").as_string().c_str());
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) return;

    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr ds(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::VoxelGrid<pcl::PointXYZI> vg;
      vg.setInputCloud(cloud);
      const float ls = static_cast<float>(voxel_leaf_);
      vg.setLeafSize(ls, ls, ls);
      vg.filter(*ds);
      cloud = ds;
      if (cloud->empty()) return;
    }

    const std::vector<GroundFilterPoint> points = filterGroundDetailed(*cloud, gf_);

    pcl::PointCloud<pcl::PointXYZI> out;
    out.reserve(points.size());
    for (const auto & point : points) out.push_back(cloud->points[point.index]);

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(out, out_msg);
    out_msg.header = msg->header;
    pub_->publish(out_msg);

    if (vis_pub_->get_subscription_count() > 0) {
      const std::vector<GroundFilterPoint> vis_points = collectGroundHeightPoints(
        *cloud, gf_, visualization_min_height_, visualization_max_height_);

      pcl::PointCloud<pcl::PointXYZI> vis;
      vis.reserve(vis_points.size());
      for (const auto & point : vis_points) {
        auto vis_point = cloud->points[point.index];
        vis_point.intensity = static_cast<float>(point.height);
        vis.push_back(vis_point);
      }

      sensor_msgs::msg::PointCloud2 vis_msg;
      pcl::toROSMsg(vis, vis_msg);
      vis_msg.header = msg->header;
      vis_pub_->publish(vis_msg);
    }
  }

  double voxel_leaf_{0.08};
  double visualization_min_height_{-0.20};
  double visualization_max_height_{1.80};
  GroundFilterParams gf_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr vis_pub_;
};

}  // namespace lidar_preprocessor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_preprocessor::LidarPreprocessorNode>());
  rclcpp::shutdown();
  return 0;
}
