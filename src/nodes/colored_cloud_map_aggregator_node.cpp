#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "pointcloud_colorizer/colored_cloud_map_aggregator_core.hpp"
#include "pointcloud_colorizer/transform_utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rmw/qos_profiles.h>

using std::placeholders::_1;
using std::placeholders::_2;

class ColoredCloudMapAggregatorNode : public rclcpp::Node
{
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
    sync_queue_size_ = std::max<int>(1, this->declare_parameter<int>("sync_queue_size", 10));

    color_burnin_samples_ = this->declare_parameter<int>("color_burnin_samples", 5);
    color_step_max_ = this->declare_parameter<int>("color_step_max", 16);
    color_ignore_placeholder_gray_ = this->declare_parameter<bool>("color_ignore_placeholder_gray", true);
    placeholder_gray_value_ = this->declare_parameter<int>("placeholder_gray_value", 128);
    color_hash_initial_capacity_ = static_cast<std::size_t>(
      std::max<int>(0, this->declare_parameter<int>("color_hash_initial_capacity", 262144)));
    color_hash_max_load_factor_ = static_cast<float>(
      this->declare_parameter<double>("color_hash_max_load_factor", 0.7));

    validate_configuration();

    pointcloud_colorizer::ColoredCloudMapAggregatorCoreConfig core_config;
    core_config.map_builder.estimator.voxel_size = map_voxel_size_;
    core_config.map_builder.estimator.burn_in_samples = color_burnin_samples_;
    core_config.map_builder.estimator.step_max = color_step_max_;
    core_config.map_builder.estimator.ignore_placeholder_gray = color_ignore_placeholder_gray_;
    core_config.map_builder.estimator.placeholder_gray_value = placeholder_gray_value_;
    core_config.map_builder.estimator.hash_initial_capacity = color_hash_initial_capacity_;
    core_config.map_builder.estimator.hash_max_load_factor = color_hash_max_load_factor_;
    aggregator_core_ = std::make_unique<pointcloud_colorizer::ColoredCloudMapAggregatorCore>(core_config);

    const auto sensor_qos = rmw_qos_profile_sensor_data;
    odometry_.subscribe(this, input_odometry_topic_, sensor_qos);
    colored_cloud_.subscribe(this, input_colored_cloud_topic_, sensor_qos);
    sync_ = std::make_shared<Sync>(SyncPolicy(sync_queue_size_), odometry_, colored_cloud_);
    sync_->registerCallback(
      std::bind(&ColoredCloudMapAggregatorNode::sync_callback, this, _1, _2));

    map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_map_topic_, 10);

    const auto map_publish_period = std::chrono::duration<double>(std::max(0.2, map_publish_interval_sec_));
    map_publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(map_publish_period),
      std::bind(&ColoredCloudMapAggregatorNode::map_publish_timer_callback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Colored cloud map aggregator: colored_cloud=%s odometry=%s output_map=%s map_frame=%s map_voxel_size=%.3f map_publish_interval_sec=%.2f sync_queue_size=%d",
      input_colored_cloud_topic_.c_str(),
      input_odometry_topic_.c_str(),
      output_map_topic_.c_str(),
      map_frame_id_.empty() ? "<odom_header>" : map_frame_id_.c_str(),
      map_voxel_size_,
      map_publish_interval_sec_,
      sync_queue_size_);
  }

private:
  void validate_configuration() const
  {
    pointcloud_colorizer::require_non_empty(input_colored_cloud_topic_, "input_colored_cloud_topic");
    pointcloud_colorizer::require_non_empty(input_odometry_topic_, "input_odometry_topic");
    pointcloud_colorizer::require_non_empty(output_map_topic_, "output_map_topic");

    if (map_voxel_size_ <= 0.0f) {
      throw std::runtime_error("map_voxel_size must be > 0");
    }
  }

  void sync_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr colored_cloud_msg)
  {
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
  }

  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> colored_cloud_;
  std::shared_ptr<Sync> sync_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::TimerBase::SharedPtr map_publish_timer_;

  std::unique_ptr<pointcloud_colorizer::ColoredCloudMapAggregatorCore> aggregator_core_;

  std::string input_colored_cloud_topic_;
  std::string input_odometry_topic_;
  std::string output_map_topic_;
  std::string map_frame_id_;

  float map_voxel_size_ = 0.3f;
  double map_publish_interval_sec_ = 1.0;
  int sync_queue_size_ = 10;

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
