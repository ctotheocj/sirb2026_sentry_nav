#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>
#include <optional>
#include <mutex>

class YawFusion : public rclcpp::Node
{
public:
  YawFusion()
  : Node("yaw_fusion"),
    odom_received_(false),
    last_odom_yaw_(0.0),
    current_v_yaw_(0.0),
    fused_yaw_(0.0)
  {
    // 参数声明
    const auto parameter_overrides =
      this->get_node_parameters_interface()->get_parameter_overrides();
    const bool has_yaw_rate_topic = parameter_overrides.count("yaw_rate.topic") > 0;
    const bool has_yaw_rate_sign = parameter_overrides.count("yaw_rate.sign") > 0;
    const bool has_yaw_rate_max_abs = parameter_overrides.count("yaw_rate.max_abs") > 0;
    const bool has_yaw_rate_timeout = parameter_overrides.count("yaw_rate.timeout") > 0;

    std::string odom_topic = this->declare_parameter("odom_topic", "odometry");
    std::string v_yaw_topic = this->declare_parameter("yaw_rate.topic", "/serial/v_yaw");

    // 最大允许预测时间（秒），超过此时间不再预测，防止 odom 长期丢失时的异常
    max_predict_dt_ = this->declare_parameter("max_predict_dt", 0.15);
    yaw_rate_enable_ = this->declare_parameter("yaw_rate.enable", true);
    v_yaw_sign_ = this->declare_parameter("yaw_rate.sign", 1.0);
    v_yaw_max_abs_ = this->declare_parameter("yaw_rate.max_abs", 12.0);
    v_yaw_timeout_ = this->declare_parameter("yaw_rate.timeout", 0.05);

    // Backward compatibility for old yaw_fusion-only parameter names.
    const std::string legacy_v_yaw_topic = this->declare_parameter("v_yaw_topic", v_yaw_topic);
    const double legacy_v_yaw_sign = this->declare_parameter("v_yaw_sign", v_yaw_sign_);
    const double legacy_v_yaw_max_abs =
      this->declare_parameter("v_yaw_max_abs", v_yaw_max_abs_);
    const double legacy_v_yaw_timeout =
      this->declare_parameter("v_yaw_timeout", v_yaw_timeout_);
    if (!has_yaw_rate_topic) {
      v_yaw_topic = legacy_v_yaw_topic;
    }
    if (!has_yaw_rate_sign) {
      v_yaw_sign_ = legacy_v_yaw_sign;
    }
    if (!has_yaw_rate_max_abs) {
      v_yaw_max_abs_ = legacy_v_yaw_max_abs;
    }
    if (!has_yaw_rate_timeout) {
      v_yaw_timeout_ = legacy_v_yaw_timeout;
    }

    v_yaw_sign_ = v_yaw_sign_ >= 0.0 ? 1.0 : -1.0;
    v_yaw_max_abs_ = std::max(0.0, v_yaw_max_abs_);
    v_yaw_timeout_ = std::max(0.001, v_yaw_timeout_);

    // 初始化 TF 缓冲与监听（仅用于可视化箭头）
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // 订阅 odom
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, std::bind(&YawFusion::odomCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribing to odom topic: %s", odom_topic.c_str());

    if (yaw_rate_enable_) {
      // 订阅电控角速度（陀螺仪直读，最准确，无零飘）
      v_yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        v_yaw_topic, 10, std::bind(&YawFusion::vYawCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Subscribing to v_yaw topic: %s", v_yaw_topic.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "yaw_rate.enable=false; /Nav_yaw will mirror odometry yaw.");
    }

    // 发布
    // 发布：/Nav_yaw 绝对路径（电控侧消费，与 /serial/v_yaw 同理不加 ns）
    nav_yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("/Nav_yaw", 10);
    // 内部调试话题：相对路径，随节点 namespace 自动加前缀
    debug_pub_ = this->create_publisher<std_msgs::msg::Float64>("yaw_fusion/debug", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("yaw_fusion/status", 10);
    arrow_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("yaw_fusion/arrow", 10);
    latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("yaw_fusion/odom_latency", 10);

    // 定时器：50Hz (20ms) — 高频输出融合 yaw
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&YawFusion::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
      "YawFusion Node Started (Forward Prediction Mode)\n"
      "  yaw_rate_enable: %s\n"
      "  v_yaw_source: %s (电控陀螺仪直读)\n"
      "  max_predict_dt: %.3f s\n"
      "  v_yaw_sign: %.1f, v_yaw_max_abs: %.3f rad/s, v_yaw_timeout: %.3f s",
      yaw_rate_enable_ ? "true" : "false", v_yaw_topic.c_str(), max_predict_dt_,
      v_yaw_sign_, v_yaw_max_abs_, v_yaw_timeout_);
  }

private:
  void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    double current_odom_yaw = quaternionToYaw(msg->pose.pose.orientation);
    rclcpp::Time current_time = msg->header.stamp;

    last_odom_yaw_ = current_odom_yaw;
    last_odom_stamp_ = current_time;

    if (!odom_received_) {
      odom_received_ = true;
      fused_yaw_ = current_odom_yaw;
      RCLCPP_INFO(this->get_logger(), "First odom received: yaw=%.2f deg",
        current_odom_yaw * 180.0 / M_PI);
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Odom: yaw=%.2f°, stamp_age=%.1f ms",
      current_odom_yaw * 180.0 / M_PI,
      (this->now() - current_time).seconds() * 1000.0);
  }

  void vYawCallback(const std_msgs::msg::Float64::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_v_yaw_ = msg->data * v_yaw_sign_;
    last_v_yaw_stamp_ = this->now();
  }

  void timerCallback()
  {
    if (!odom_received_) return;

    double v_yaw = 0.0;
    double dt = 0.0;
    double base_yaw = 0.0;
    bool v_yaw_valid = false;
    double v_yaw_age = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (last_v_yaw_stamp_.has_value()) {
        v_yaw_age = (this->now() - last_v_yaw_stamp_.value()).seconds();
        const bool fresh = v_yaw_age <= v_yaw_timeout_;
        const bool finite = std::isfinite(current_v_yaw_);
        const bool in_range = v_yaw_max_abs_ <= 0.0 || std::fabs(current_v_yaw_) <= v_yaw_max_abs_;
        v_yaw_valid = fresh && finite && in_range;
        if (v_yaw_valid) {
          v_yaw = current_v_yaw_;
        }
      }
      if (last_odom_stamp_.has_value()) {
        dt = (this->now() - last_odom_stamp_.value()).seconds();
        dt = std::clamp(dt, 0.0, max_predict_dt_);  // 限制最大预测时间
      }
      base_yaw = last_odom_yaw_;
    }

    //    fused_yaw = odom_yaw + v_yaw × dt
    double predicted_yaw = normalizeAngle(base_yaw + v_yaw * dt);
    fused_yaw_ = predicted_yaw;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "fused: %.1f°, odom: %.1f°, v_yaw: %.2f°/s, valid: %s, age: %.1f ms, "
      "dt: %.1f ms, predict_delta: %.2f°",
      fused_yaw_ * 180.0 / M_PI,
      base_yaw * 180.0 / M_PI,
      v_yaw * 180.0 / M_PI,
      v_yaw_valid ? "true" : "false",
      v_yaw_age * 1000.0,
      dt * 1000.0,
      (v_yaw * dt) * 180.0 / M_PI);

    std_msgs::msg::Float64 msg;
    msg.data = fused_yaw_;
    nav_yaw_pub_->publish(msg);
    debug_pub_->publish(msg);

    std_msgs::msg::Float64 latency_msg;
    latency_msg.data = dt * 1000.0;  // ms
    latency_pub_->publish(latency_msg);

    std_msgs::msg::String status_msg;
    status_msg.data = v_yaw_valid ? "v_yaw_valid" : "v_yaw_invalid_or_stale";
    status_pub_->publish(status_msg);

    publishArrow();
  }

  void publishArrow()
  {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = "odom";
    arrow.header.stamp = this->now();
    arrow.ns = "yaw_fusion";
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;

    try {
      auto transform = tf_buffer_->lookupTransform("odom", "base_footprint", rclcpp::Time(0));
      arrow.pose.position.x = transform.transform.translation.x;
      arrow.pose.position.y = transform.transform.translation.y;
      arrow.pose.position.z = transform.transform.translation.z + 0.5;
    } catch (const tf2::TransformException &) {
      arrow.pose.position.x = 0.0;
      arrow.pose.position.y = 0.0;
      arrow.pose.position.z = 0.5;
    }

    tf2::Quaternion q;
    q.setRPY(0, 0, fused_yaw_);
    arrow.pose.orientation = tf2::toMsg(q);

    arrow.scale.x = 0.8;
    arrow.scale.y = 0.1;
    arrow.scale.z = 0.1;
    arrow.color.g = 1.0; arrow.color.a = 1.0;

    arrow_pub_->publish(arrow);
  }

  static double quaternionToYaw(const geometry_msgs::msg::Quaternion & quat)
  {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);
    return y;
  }

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr v_yaw_sub_;  // 电控角速度
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nav_yaw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arrow_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex mutex_;

  // Odom 状态
  bool odom_received_;
  double last_odom_yaw_;
  std::optional<rclcpp::Time> last_odom_stamp_;

  // 电控角速度（陀螺仪直读）
  double current_v_yaw_;
  std::optional<rclcpp::Time> last_v_yaw_stamp_;

  // 输出
  double fused_yaw_;

  // 参数
  double max_predict_dt_;
  bool yaw_rate_enable_;
  double v_yaw_sign_;
  double v_yaw_max_abs_;
  double v_yaw_timeout_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<YawFusion>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
