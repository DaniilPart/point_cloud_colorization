#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <mutex>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "pointcloud_colorizer/colored_cloud_map_aggregator_core.hpp"
#include "pointcloud_colorizer/csv_keyframe_pose_provider.hpp"
#include "pointcloud_colorizer/transform_utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rmw/qos_profiles.h>
#include <tf2_ros/transform_broadcaster.h>

using std::placeholders::_1;
using std::placeholders::_2;

class ColoredCloudMapAggregatorNode : public rclcpp::Node
{
  enum class PoseSource
  {
    Odometry,
    CsvKeyframes
  };

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry,
    sensor_msgs::msg::PointCloud2>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

public:
  explicit ColoredCloudMapAggregatorNode(const rclcpp::NodeOptions & options)
  : Node("colored_cloud_map_aggregator", options)
  {
    input_colored_cloud_topic_ = this->declare_parameter<std::string>("input_colored_cloud_topic", "");
    input_odometry_topic_ = this->declare_parameter<std::string>("input_odometry_topic", "");
    output_map_topic_ = this->declare_parameter<std::string>("output_map_topic", "");
    map_frame_id_ = this->declare_parameter<std::string>("map_frame_id", "");
    map_voxel_size_ = static_cast<float>(this->declare_parameter<double>("map_voxel_size", 0.3));
    map_publish_interval_sec_ = this->declare_parameter<double>("map_publish_interval_sec", 1.0);
    map_save_interval_sec_ = this->declare_parameter<double>("map_save_interval_sec", 5.0);
    map_save_ply_path_ = this->declare_parameter<std::string>("map_save_ply_path", "");
    map_save_append_start_timestamp_ = this->declare_parameter<bool>(
      "map_save_append_start_timestamp", true);
    map_save_service_name_ = this->declare_parameter<std::string>("map_save_service_name", "~/save_map");
    publish_colorized_to_odom_colorized_tf_ = this->declare_parameter<bool>(
      "publish_colorized_to_odom_colorized_tf", true);
    odom_colorized_frame_id_ = this->declare_parameter<std::string>(
      "odom_colorized_frame_id", "odom_colorized");
    sync_queue_size_ = std::max<int>(1, this->declare_parameter<int>("sync_queue_size", 10));
    pose_source_name_ = this->declare_parameter<std::string>("pose_source", "odometry");
    keyframes_csv_path_ = this->declare_parameter<std::string>("keyframes_csv_path", "");
    csv_pose_match_tolerance_sec_ = this->declare_parameter<double>("csv_pose_match_tolerance_sec", 0.05);
    csv_pose_frame_id_ = this->declare_parameter<std::string>("csv_pose_frame_id", "odom");
    run_id_ = this->declare_parameter<std::string>("run_id", "");

    color_burnin_samples_ = this->declare_parameter<int>("color_burnin_samples", 5);
    color_step_max_ = this->declare_parameter<int>("color_step_max", 16);
    color_ignore_placeholder_gray_ = this->declare_parameter<bool>("color_ignore_placeholder_gray", true);
    placeholder_gray_value_ = this->declare_parameter<int>("placeholder_gray_value", 128);
    color_hash_initial_capacity_ = static_cast<std::size_t>(
      std::max<int>(0, this->declare_parameter<int>("color_hash_initial_capacity", 262144)));
    color_hash_max_load_factor_ = static_cast<float>(
      this->declare_parameter<double>("color_hash_max_load_factor", 0.7));

    pose_source_ = parse_pose_source(pose_source_name_);
    validate_configuration();

    if (pose_source_ == PoseSource::CsvKeyframes) {
      csv_pose_provider_ = std::make_unique<pointcloud_colorizer::CsvKeyframePoseProvider>(
        keyframes_csv_path_, csv_pose_match_tolerance_sec_);
    }

    experiment_start_timestamp_ = run_id_.empty() ? make_start_timestamp() : run_id_;
    resolved_map_save_ply_path_ = resolve_map_save_path(
      map_save_ply_path_,
      map_save_append_start_timestamp_,
      experiment_start_timestamp_);

    pointcloud_colorizer::ColoredCloudMapAggregatorCoreConfig core_config;
    core_config.map_builder.estimator.voxel_size = map_voxel_size_;
    core_config.map_builder.estimator.burn_in_samples = color_burnin_samples_;
    core_config.map_builder.estimator.step_max = color_step_max_;
    core_config.map_builder.estimator.ignore_placeholder_gray = color_ignore_placeholder_gray_;
    core_config.map_builder.estimator.placeholder_gray_value = placeholder_gray_value_;
    core_config.map_builder.estimator.hash_initial_capacity = color_hash_initial_capacity_;
    core_config.map_builder.estimator.hash_max_load_factor = color_hash_max_load_factor_;
    aggregator_core_ = std::make_unique<pointcloud_colorizer::ColoredCloudMapAggregatorCore>(core_config);

    if (publish_colorized_to_odom_colorized_tf_) {
      colorized_to_odom_colorized_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    if (pose_source_ == PoseSource::Odometry) {
      const auto sensor_qos = rmw_qos_profile_sensor_data;
      odometry_.subscribe(this, input_odometry_topic_, sensor_qos);
      colored_cloud_.subscribe(this, input_colored_cloud_topic_, sensor_qos);
      sync_ = std::make_shared<Sync>(SyncPolicy(sync_queue_size_), odometry_, colored_cloud_);
      sync_->registerCallback(
        std::bind(&ColoredCloudMapAggregatorNode::sync_callback, this, _1, _2));
    } else {
      colored_cloud_only_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_colored_cloud_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&ColoredCloudMapAggregatorNode::colored_cloud_only_callback, this, _1));
    }

    map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_map_topic_, 10);

    const auto map_publish_period = std::chrono::duration<double>(std::max(0.2, map_publish_interval_sec_));
    map_publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(map_publish_period),
      std::bind(&ColoredCloudMapAggregatorNode::map_publish_timer_callback, this));

    const auto map_save_period = std::chrono::duration<double>(std::max(0.2, map_save_interval_sec_));
    if (map_save_interval_sec_ > 0.0) {
      map_save_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(map_save_period),
        std::bind(&ColoredCloudMapAggregatorNode::map_save_timer_callback, this));
    }

    map_save_service_ = this->create_service<std_srvs::srv::Trigger>(
      map_save_service_name_,
      std::bind(
        &ColoredCloudMapAggregatorNode::map_save_service_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      this->get_logger(),
      "Colored cloud map aggregator: pose_source=%s colored_cloud=%s odometry=%s output_map=%s map_frame=%s map_voxel_size=%.3f map_publish_interval_sec=%.2f map_save_interval_sec=%.2f map_save_ply_path=%s sync_queue_size=%d",
      pose_source_name_.c_str(),
      input_colored_cloud_topic_.c_str(),
      input_odometry_topic_.c_str(),
      output_map_topic_.c_str(),
      map_frame_id_.empty() ? "<odom_header>" : map_frame_id_.c_str(),
      map_voxel_size_,
      map_publish_interval_sec_,
      map_save_interval_sec_,
      resolved_map_save_ply_path_.c_str(),
      sync_queue_size_);

    if (pose_source_ == PoseSource::CsvKeyframes) {
      RCLCPP_INFO(
        this->get_logger(),
        "Loaded %zu keyframe poses from CSV '%s' with match tolerance %.6f sec.",
        csv_pose_provider_->size(),
        keyframes_csv_path_.c_str(),
        csv_pose_match_tolerance_sec_);
    }

    if (map_save_interval_sec_ <= 0.0) {
      RCLCPP_INFO(
        this->get_logger(),
        "Periodic map saving disabled (map_save_interval_sec=%.3f). Use service '%s' to save on demand.",
        map_save_interval_sec_,
        map_save_service_name_.c_str());
    }

    if (publish_colorized_to_odom_colorized_tf_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Publishing TF <cloud_header.frame_id>->%s from %s messages.",
        odom_colorized_frame_id_.c_str(),
        pose_source_name_.c_str());
    }
  }

private:
  static PoseSource parse_pose_source(const std::string & value)
  {
    if (value == "odometry") {
      return PoseSource::Odometry;
    }

    if (value == "csv_keyframes") {
      return PoseSource::CsvKeyframes;
    }

    throw std::runtime_error("pose_source must be 'odometry' or 'csv_keyframes'");
  }

  static double stamp_to_seconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) +
      static_cast<double>(stamp.nanosec) * 1e-9;
  }

  void validate_configuration() const
  {
    pointcloud_colorizer::require_non_empty(input_colored_cloud_topic_, "input_colored_cloud_topic");
    pointcloud_colorizer::require_non_empty(output_map_topic_, "output_map_topic");

    if (pose_source_ == PoseSource::Odometry) {
      pointcloud_colorizer::require_non_empty(input_odometry_topic_, "input_odometry_topic");
    } else {
      pointcloud_colorizer::require_non_empty(keyframes_csv_path_, "keyframes_csv_path");
      if (csv_pose_match_tolerance_sec_ < 0.0) {
        throw std::runtime_error("csv_pose_match_tolerance_sec must be >= 0");
      }
      pointcloud_colorizer::require_non_empty(csv_pose_frame_id_, "csv_pose_frame_id");
    }

    if (map_voxel_size_ <= 0.0f) {
      throw std::runtime_error("map_voxel_size must be > 0");
    }

    if (map_save_interval_sec_ < 0.0) {
      throw std::runtime_error("map_save_interval_sec must be >= 0");
    }

    if (publish_colorized_to_odom_colorized_tf_) {
      pointcloud_colorizer::require_non_empty(odom_colorized_frame_id_, "odom_colorized_frame_id");
    }

    pointcloud_colorizer::require_non_empty(map_save_ply_path_, "map_save_ply_path");
  }

  void sync_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr colored_cloud_msg)
  {
    publish_colorized_to_odom_colorized_tf(colored_cloud_msg->header, *odom_msg);

    auto cloud_in = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromROSMsg(*colored_cloud_msg, *cloud_in);

    std::string error_message;
    if (!aggregator_core_->update_from_synced_messages(*odom_msg, *cloud_in, map_frame_id_, &error_message)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Skipping synchronized callback: %s",
        error_message.c_str());
    }
  }

  void colored_cloud_only_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr colored_cloud_msg)
  {
    if (!csv_pose_provider_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Skipping cloud callback in CSV mode: CSV pose provider is not initialized.");
      return;
    }

    const double cloud_stamp_sec = stamp_to_seconds(colored_cloud_msg->header.stamp);
    Eigen::Matrix4f pose_transform = Eigen::Matrix4f::Identity();
    if (!csv_pose_provider_->find_nearest_pose(cloud_stamp_sec, pose_transform)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Skipping colored cloud at %.9f sec: no CSV pose within tolerance %.6f sec.",
        cloud_stamp_sec,
        csv_pose_match_tolerance_sec_);
      return;
    }

    publish_colorized_to_odom_colorized_tf(colored_cloud_msg->header, pose_transform);

    auto cloud_in = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromROSMsg(*colored_cloud_msg, *cloud_in);

    std_msgs::msg::Header source_header = colored_cloud_msg->header;
    if (source_header.frame_id.empty()) {
      source_header.frame_id = csv_pose_frame_id_;
    }

    std::string error_message;
    if (!aggregator_core_->update_from_pose_and_cloud(
        pose_transform,
        source_header,
        *cloud_in,
        map_frame_id_,
        &error_message))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Skipping cloud callback in CSV mode: %s",
        error_message.c_str());
    }
  }

  void publish_colorized_to_odom_colorized_tf(
    const std_msgs::msg::Header & source_header,
    const nav_msgs::msg::Odometry & odom_msg)
  {
    if (!publish_colorized_to_odom_colorized_tf_ || !colorized_to_odom_colorized_tf_broadcaster_) {
      return;
    }

    const auto & q_msg = odom_msg.pose.pose.orientation;
    Eigen::Quaternionf q(
      static_cast<float>(q_msg.w),
      static_cast<float>(q_msg.x),
      static_cast<float>(q_msg.y),
      static_cast<float>(q_msg.z));
    if (q.norm() == 0.0f) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Skipping TF publish <cloud_header.frame_id>->%s: invalid odometry pose rotation.",
        odom_colorized_frame_id_.c_str());
      return;
    }
    q.normalize();

    Eigen::Matrix4f pose_transform = Eigen::Matrix4f::Identity();
    pose_transform.block<3, 3>(0, 0) = q.toRotationMatrix();
    pose_transform(0, 3) = static_cast<float>(odom_msg.pose.pose.position.x);
    pose_transform(1, 3) = static_cast<float>(odom_msg.pose.pose.position.y);
    pose_transform(2, 3) = static_cast<float>(odom_msg.pose.pose.position.z);

    publish_colorized_to_odom_colorized_tf(source_header, pose_transform);
  }

  void publish_colorized_to_odom_colorized_tf(
    const std_msgs::msg::Header & source_header,
    const Eigen::Matrix4f & source_pose_transform)
  {
    if (!publish_colorized_to_odom_colorized_tf_ || !colorized_to_odom_colorized_tf_broadcaster_) {
      return;
    }

    if (source_header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Skipping TF publish <cloud_header.frame_id>->%s: source header frame_id is empty.",
        odom_colorized_frame_id_.c_str());
      return;
    }

    if (!has_latched_tf_parent_frame_id_) {
      latched_tf_parent_frame_id_ = source_header.frame_id;
      has_latched_tf_parent_frame_id_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Latched TF parent frame_id='%s' for %s.",
        latched_tf_parent_frame_id_.c_str(),
        odom_colorized_frame_id_.c_str());
    } else if (source_header.frame_id != latched_tf_parent_frame_id_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Skipping TF publish because source frame_id changed from '%s' to '%s'.",
        latched_tf_parent_frame_id_.c_str(),
        source_header.frame_id.c_str());
      return;
    }

    const Eigen::Matrix4f tf_matrix = source_pose_transform.inverse();

    const Eigen::Matrix3f rotation = tf_matrix.block<3, 3>(0, 0);
    Eigen::Quaternionf q(rotation);
    if (q.norm() == 0.0f) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Skipping TF publish %s->%s: invalid pose rotation.",
        latched_tf_parent_frame_id_.c_str(),
        odom_colorized_frame_id_.c_str());
      return;
    }
    q.normalize();

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = source_header;
    tf_msg.header.frame_id = latched_tf_parent_frame_id_;
    tf_msg.child_frame_id = odom_colorized_frame_id_;
    tf_msg.transform.translation.x = static_cast<double>(tf_matrix(0, 3));
    tf_msg.transform.translation.y = static_cast<double>(tf_matrix(1, 3));
    tf_msg.transform.translation.z = static_cast<double>(tf_matrix(2, 3));
    tf_msg.transform.rotation.w = static_cast<double>(q.w());
    tf_msg.transform.rotation.x = static_cast<double>(q.x());
    tf_msg.transform.rotation.y = static_cast<double>(q.y());
    tf_msg.transform.rotation.z = static_cast<double>(q.z());
    colorized_to_odom_colorized_tf_broadcaster_->sendTransform(tf_msg);
  }

  void map_publish_timer_callback()
  {
    if (!aggregator_core_->has_pending_publish()) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_cloud;
    std_msgs::msg::Header map_header;
    if (!aggregator_core_->take_map_cloud(map_cloud, map_header)) {
      return;
    }

    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*map_cloud, map_msg);
    map_msg.header = map_header;
    if (!map_frame_id_.empty()) {
      map_msg.header.frame_id = map_frame_id_;
    }
    map_publisher_->publish(map_msg);

    {
      std::lock_guard<std::mutex> lock(latest_map_mutex_);
      latest_map_cloud_ = map_cloud;
      latest_map_header_ = map_header;
    }
  }

  void map_save_timer_callback()
  {
    save_latest_map_snapshot(true);
  }

  void map_save_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::string status_message;
    response->success = save_latest_map_snapshot(false, &status_message);
    response->message = status_message;
  }

  bool save_latest_map_snapshot(bool throttle_logs, std::string * status_message = nullptr)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_cloud;
    {
      std::lock_guard<std::mutex> lock(latest_map_mutex_);
      map_cloud = latest_map_cloud_;
    }

    if (map_cloud == nullptr || map_cloud->empty()) {
      if (status_message != nullptr) {
        *status_message = "No map snapshot available yet (map has not been published).";
      }
      return false;
    }

    const std::filesystem::path ply_path(resolved_map_save_ply_path_);
    const std::filesystem::path parent = ply_path.parent_path();
    if (!parent.empty()) {
      std::error_code ec;
      std::filesystem::create_directories(parent, ec);
      if (ec) {
        const std::string message =
          "Failed to create directory for map_save_ply_path '" + parent.string() + "': " + ec.message();
        if (throttle_logs) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            3000,
            "%s",
            message.c_str());
        } else {
          RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
        }
        if (status_message != nullptr) {
          *status_message = message;
        }
        return false;
      }
    }

    const int rc = pcl::io::savePLYFileBinary(resolved_map_save_ply_path_, *map_cloud);
    if (rc != 0) {
      const std::string message =
        "Failed to save PLY map to '" + resolved_map_save_ply_path_ + "' (rc=" + std::to_string(rc) + ")";
      if (throttle_logs) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          3000,
          "%s",
          message.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
      }
      if (status_message != nullptr) {
        *status_message = message;
      }
      return false;
    }

    const std::string success_message =
      "Saved map snapshot to '" + resolved_map_save_ply_path_ + "' with " +
      std::to_string(map_cloud->size()) + " points.";
    if (status_message != nullptr) {
      *status_message = success_message;
    }

    RCLCPP_INFO(this->get_logger(), "%s", success_message.c_str());
    return true;
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

  static std::string resolve_map_save_path(
    const std::string & configured_path,
    bool append_start_timestamp,
    const std::string & start_timestamp)
  {
    if (!append_start_timestamp) {
      return configured_path;
    }

    const std::filesystem::path path(configured_path);
    const std::string stem = path.stem().string();
    const std::string ext = path.extension().string();
    const std::string effective_ext = ext.empty() ? ".ply" : ext;
    const std::filesystem::path parent = path.parent_path();

    const std::filesystem::path filename = stem.empty() ?
      std::filesystem::path(start_timestamp + effective_ext) :
      std::filesystem::path(stem + "_" + start_timestamp + effective_ext);

    return (parent / filename).string();
  }

  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> colored_cloud_;
  std::shared_ptr<Sync> sync_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr colored_cloud_only_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::TimerBase::SharedPtr map_publish_timer_;
  rclcpp::TimerBase::SharedPtr map_save_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> colorized_to_odom_colorized_tf_broadcaster_;

  std::unique_ptr<pointcloud_colorizer::ColoredCloudMapAggregatorCore> aggregator_core_;

  std::string input_colored_cloud_topic_;
  std::string input_odometry_topic_;
  std::string output_map_topic_;
  std::string map_frame_id_;
  PoseSource pose_source_ = PoseSource::Odometry;
  std::string pose_source_name_ = "odometry";
  std::string keyframes_csv_path_;
  double csv_pose_match_tolerance_sec_ = 0.05;
  std::string csv_pose_frame_id_ = "odom";
  std::string run_id_;
  std::unique_ptr<pointcloud_colorizer::CsvKeyframePoseProvider> csv_pose_provider_;

  float map_voxel_size_ = 0.3f;
  double map_publish_interval_sec_ = 1.0;
  double map_save_interval_sec_ = 5.0;
  std::string map_save_ply_path_;
  bool map_save_append_start_timestamp_ = true;
  std::string map_save_service_name_;
  bool publish_colorized_to_odom_colorized_tf_ = true;
  std::string odom_colorized_frame_id_ = "odom_colorized";
  std::string latched_tf_parent_frame_id_;
  bool has_latched_tf_parent_frame_id_ = false;
  std::string experiment_start_timestamp_;
  std::string resolved_map_save_ply_path_;
  int sync_queue_size_ = 10;

  std::mutex latest_map_mutex_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr latest_map_cloud_;
  std_msgs::msg::Header latest_map_header_;

  int color_burnin_samples_ = 5;
  int color_step_max_ = 16;
  bool color_ignore_placeholder_gray_ = true;
  int placeholder_gray_value_ = 128;
  std::size_t color_hash_initial_capacity_ = 262144;
  float color_hash_max_load_factor_ = 0.7f;
};

#ifndef POINTCLOUD_COLORIZER_BUILD_STANDALONE
RCLCPP_COMPONENTS_REGISTER_NODE(ColoredCloudMapAggregatorNode)
#else
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ColoredCloudMapAggregatorNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
