#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class OdomEarthPoseLoggerNode : public rclcpp::Node
{
public:
  explicit OdomEarthPoseLoggerNode(const rclcpp::NodeOptions & options)
  : Node("odom_earth_pose_logger", options)
  {
    input_odometry_topic_ = this->declare_parameter<std::string>("input_odometry_topic", "");
    map_save_interval_sec_ = this->declare_parameter<double>("map_save_interval_sec", 5.0);
    map_save_ply_path_ = this->declare_parameter<std::string>("map_save_ply_path", "");
    map_save_append_start_timestamp_ = this->declare_parameter<bool>(
      "map_save_append_start_timestamp", true);
    earth_frame_id_ = this->declare_parameter<std::string>("earth_frame_id", "earth");
    tf_lookup_timeout_sec_ = this->declare_parameter<double>("tf_lookup_timeout_sec", 0.1);
    run_id_ = this->declare_parameter<std::string>("run_id", "");

    validate_configuration();

    experiment_start_timestamp_ = run_id_.empty() ? make_start_timestamp() : run_id_;
    resolved_pose_log_path_ = resolve_pose_log_path(
      map_save_ply_path_,
      map_save_append_start_timestamp_,
      experiment_start_timestamp_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_odometry_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&OdomEarthPoseLoggerNode::odometry_callback, this, std::placeholders::_1));

    if (map_save_interval_sec_ > 0.0) {
      const auto save_period = std::chrono::duration<double>(std::max(0.2, map_save_interval_sec_));
      save_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(save_period),
        std::bind(&OdomEarthPoseLoggerNode::save_timer_callback, this));
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Odom-earth pose logger: odometry=%s earth_frame=%s map_save_interval_sec=%.2f output=%s",
      input_odometry_topic_.c_str(),
      earth_frame_id_.c_str(),
      map_save_interval_sec_,
      resolved_pose_log_path_.c_str());

    if (map_save_interval_sec_ <= 0.0) {
      RCLCPP_INFO(
        this->get_logger(),
        "Periodic pose logging disabled (map_save_interval_sec=%.3f).",
        map_save_interval_sec_);
    }
  }

private:
  static double stamp_to_seconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) +
      static_cast<double>(stamp.nanosec) * 1e-9;
  }

  static std::string make_start_timestamp()
  {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now{};
#ifdef _WIN32
    localtime_s(&tm_now, &now_time);
#else
    localtime_r(&now_time, &tm_now);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
    return oss.str();
  }

  static std::string resolve_pose_log_path(
    const std::string & map_save_ply_path,
    bool append_start_timestamp,
    const std::string & start_timestamp)
  {
    const std::filesystem::path configured_path(map_save_ply_path);
    std::filesystem::path output_dir;
    if (configured_path.has_extension()) {
      output_dir = configured_path.parent_path();
    } else {
      output_dir = configured_path;
    }

    if (output_dir.empty()) {
      output_dir = ".";
    }

    std::string filename = "odom_earth_pose_log.csv";
    if (append_start_timestamp) {
      filename = "odom_earth_pose_log_" + start_timestamp + ".csv";
    }

    return (output_dir / filename).string();
  }

  void validate_configuration() const
  {
    if (input_odometry_topic_.empty()) {
      throw std::runtime_error("input_odometry_topic must not be empty");
    }

    if (map_save_interval_sec_ < 0.0) {
      throw std::runtime_error("map_save_interval_sec must be >= 0");
    }

    if (map_save_ply_path_.empty()) {
      throw std::runtime_error("map_save_ply_path must not be empty");
    }

    if (earth_frame_id_.empty()) {
      throw std::runtime_error("earth_frame_id must not be empty");
    }

    if (tf_lookup_timeout_sec_ < 0.0) {
      throw std::runtime_error("tf_lookup_timeout_sec must be >= 0");
    }
  }

  void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
  {
    std::lock_guard<std::mutex> lock(latest_odom_mutex_);
    latest_odom_msg_ = odom_msg;
  }

  void save_timer_callback()
  {
    save_latest_pose();
  }

  bool save_latest_pose()
  {
    nav_msgs::msg::Odometry::ConstSharedPtr odom_msg;
    {
      std::lock_guard<std::mutex> lock(latest_odom_mutex_);
      odom_msg = latest_odom_msg_;
    }

    if (!odom_msg) {
      return false;
    }

    if (odom_msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Skipping pose logging because odometry header frame_id is empty.");
      return false;
    }

    geometry_msgs::msg::TransformStamped earth_t_odom;
    try {
      earth_t_odom = tf_buffer_->lookupTransform(
        earth_frame_id_,
        odom_msg->header.frame_id,
        odom_msg->header.stamp,
        tf2::durationFromSec(tf_lookup_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Failed to lookup transform %s<- %s: %s",
        earth_frame_id_.c_str(),
        odom_msg->header.frame_id.c_str(),
        ex.what());
      return false;
    }

    const std::filesystem::path output_path(resolved_pose_log_path_);
    const std::filesystem::path output_dir = output_path.parent_path();
    if (!output_dir.empty()) {
      std::error_code ec;
      std::filesystem::create_directories(output_dir, ec);
      if (ec) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          3000,
          "Failed to create output directory '%s': %s",
          output_dir.string().c_str(),
          ec.message().c_str());
        return false;
      }
    }

    const bool write_header =
      !std::filesystem::exists(output_path) ||
      std::filesystem::is_empty(output_path);

    std::ofstream out(resolved_pose_log_path_, std::ios::app);
    if (!out.is_open()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Failed to open pose log file '%s' for append.",
        resolved_pose_log_path_.c_str());
      return false;
    }

    if (write_header) {
      out << "odometry_stamp_sec,node_time_sec,target_frame,source_frame,tx,ty,tz,qx,qy,qz,qw\n";
    }

    const double odom_stamp_sec = stamp_to_seconds(odom_msg->header.stamp);
    const double node_time_sec = this->get_clock()->now().seconds();
    const auto & tr = earth_t_odom.transform.translation;
    const auto & q = earth_t_odom.transform.rotation;

    out << std::fixed << std::setprecision(9)
        << odom_stamp_sec << ","
        << node_time_sec << ","
        << earth_t_odom.header.frame_id << ","
        << earth_t_odom.child_frame_id << ","
        << tr.x << ","
        << tr.y << ","
        << tr.z << ","
        << q.x << ","
        << q.y << ","
        << q.z << ","
        << q.w << "\n";

    return true;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr save_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string input_odometry_topic_;
  double map_save_interval_sec_ = 5.0;
  std::string map_save_ply_path_;
  bool map_save_append_start_timestamp_ = true;
  std::string earth_frame_id_ = "earth";
  double tf_lookup_timeout_sec_ = 0.1;
  std::string run_id_;
  std::string experiment_start_timestamp_;
  std::string resolved_pose_log_path_;

  std::mutex latest_odom_mutex_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_msg_;
};

#ifndef POINTCLOUD_COLORIZER_BUILD_STANDALONE
RCLCPP_COMPONENTS_REGISTER_NODE(OdomEarthPoseLoggerNode)
#else
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomEarthPoseLoggerNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif