#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "pointcloud_colorizer/transform_utils.hpp"
#include "pointcloud_colorizer/voxel_color_estimator.hpp"

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rmw/qos_profiles.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using pointcloud_colorizer::TransformSource;

class RegisteredCloudColorizerNode : public rclcpp::Node
{
  using RegisteredSyncPolicy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry,
    sensor_msgs::msg::CompressedImage,
    sensor_msgs::msg::PointCloud2>;
  using RegisteredSync = message_filters::Synchronizer<RegisteredSyncPolicy>;

public:
  RegisteredCloudColorizerNode()
  : Node("registered_cloud_colorizer")
  {
    input_registered_cloud_topic_ = this->declare_parameter<std::string>(
        "input_registered_cloud_topic", "");
    input_odometry_topic_ = this->declare_parameter<std::string>(
        "input_odometry_topic", "");
    input_image_topic_ = this->declare_parameter<std::string>(
        "input_image_topic", "");
    camera_info_topic_ = this->declare_parameter<std::string>(
        "camera_info_topic", "");
    output_cloud_topic_ = this->declare_parameter<std::string>(
        "output_cloud_topic", "");
    output_map_topic_ = this->declare_parameter<std::string>(
        "output_map_topic", "");
    output_frame_id_ = this->declare_parameter<std::string>(
        "output_frame_id", "");
    map_frame_id_ = this->declare_parameter<std::string>(
        "map_frame_id", "");
    map_voxel_size_ = static_cast<float>(this->declare_parameter<double>(
        "map_voxel_size", 0.3));
    publish_only_colored_points_ = this->declare_parameter<bool>(
        "publish_only_colored_points", true);
    transform_source_string_ = this->declare_parameter<std::string>(
        "transform_source", "config");
    camera_frame_id_ = this->declare_parameter<std::string>(
        "camera_frame_id", "");
    lidar_frame_id_ = this->declare_parameter<std::string>(
        "lidar_frame_id", "");
    camera_to_lidar_matrix_values_ = this->declare_parameter<std::vector<double>>(
        "camera_to_lidar_matrix", std::vector<double>{});
    transform_lookup_timeout_ = rclcpp::Duration::from_seconds(
        this->declare_parameter<double>("transform_lookup_timeout_sec", 0.1));
    sync_queue_size_ = std::max<int>(1, this->declare_parameter<int>("sync_queue_size", 10));
    publish_diagnostics_ = this->declare_parameter<bool>("publish_diagnostics", true);
    diagnostics_topic_ = this->declare_parameter<std::string>(
      "diagnostics_topic", "/diagnostics");
    health_report_interval_sec_ = this->declare_parameter<double>(
      "health_report_interval_sec", 5.0);
    input_stale_timeout_sec_ = this->declare_parameter<double>(
      "input_stale_timeout_sec", 2.0);
    warning_throttle_sec_ = this->declare_parameter<double>(
      "warning_throttle_sec", 5.0);
    startup_grace_period_sec_ = this->declare_parameter<double>(
      "startup_grace_period_sec", 8.0);
    color_burnin_samples_ = this->declare_parameter<int>("color_burnin_samples", 5);
    color_step_max_ = this->declare_parameter<int>("color_step_max", 16);
    color_ignore_placeholder_gray_ = this->declare_parameter<bool>(
      "color_ignore_placeholder_gray", true);
    placeholder_gray_value_ = this->declare_parameter<int>("placeholder_gray_value", 128);

    startup_time_ = this->now();

    transform_source_ = pointcloud_colorizer::parse_transform_source(transform_source_string_);
    validate_configuration();
    initialize_transform();

    pointcloud_colorizer::VoxelColorEstimatorConfig estimator_config;
    estimator_config.voxel_size = map_voxel_size_;
    estimator_config.burn_in_samples = color_burnin_samples_;
    estimator_config.step_max = color_step_max_;
    estimator_config.ignore_placeholder_gray = color_ignore_placeholder_gray_;
    estimator_config.placeholder_gray_value = placeholder_gray_value_;
    voxel_color_estimator_ = std::make_unique<pointcloud_colorizer::VoxelColorEstimator>(
      estimator_config);

    const auto sensor_qos = rmw_qos_profile_sensor_data;

    odometry_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_odometry_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&RegisteredCloudColorizerNode::odometry_input_callback, this, std::placeholders::_1));
    image_state_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      input_image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&RegisteredCloudColorizerNode::image_input_callback, this, std::placeholders::_1));
    cloud_state_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_registered_cloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&RegisteredCloudColorizerNode::cloud_input_callback, this, std::placeholders::_1));

    odometry_.subscribe(this, input_odometry_topic_, sensor_qos);
    image_.subscribe(this, input_image_topic_, sensor_qos);
    registered_cloud_.subscribe(this, input_registered_cloud_topic_, sensor_qos);

    sync_ = std::make_shared<RegisteredSync>(
      RegisteredSyncPolicy(sync_queue_size_),
        odometry_,
        image_,
        registered_cloud_);
    sync_->registerCallback(
        std::bind(&RegisteredCloudColorizerNode::topic_callback, this, _1, _2, _3));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_cloud_topic_, 10);
    map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_map_topic_, 10);

    if (publish_diagnostics_) {
      diagnostics_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        diagnostics_topic_,
        10);
    }

    const auto health_period = std::chrono::duration<double>(
      std::max(0.2, health_report_interval_sec_));
    health_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(health_period),
      std::bind(&RegisteredCloudColorizerNode::health_timer_callback, this));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(
          &RegisteredCloudColorizerNode::camera_info_callback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
        this->get_logger(),
        "Registered colorizer topics: odom=%s image=%s cloud=%s camera_info=%s output_cloud=%s output_map=%s output_frame=%s map_frame=%s map_voxel_size=%.3f publish_only_colored_points=%s transform_source=%s",
        input_odometry_topic_.c_str(),
        input_image_topic_.c_str(),
        input_registered_cloud_topic_.c_str(),
        camera_info_topic_.c_str(),
        output_cloud_topic_.c_str(),
        output_map_topic_.c_str(),
        output_frame_id_.empty() ? "<cloud>" : output_frame_id_.c_str(),
        map_frame_id_.empty() ? "<cloud>" : map_frame_id_.c_str(),
        map_voxel_size_,
        publish_only_colored_points_ ? "true" : "false",
        transform_source_string_.c_str());
      RCLCPP_INFO(
        this->get_logger(),
        "Registered colorizer health: sync_queue_size=%d health_report_interval_sec=%.2f input_stale_timeout_sec=%.2f startup_grace_period_sec=%.2f warning_throttle_sec=%.2f diagnostics=%s diagnostics_topic=%s",
        sync_queue_size_,
        health_report_interval_sec_,
        input_stale_timeout_sec_,
        startup_grace_period_sec_,
        warning_throttle_sec_,
        publish_diagnostics_ ? "enabled" : "disabled",
        diagnostics_topic_.c_str());
      RCLCPP_INFO(
        this->get_logger(),
        "Registered color filter: burnin_samples=%d step_max=%d ignore_placeholder_gray=%s placeholder_gray_value=%d",
        color_burnin_samples_,
        color_step_max_,
        color_ignore_placeholder_gray_ ? "true" : "false",
        placeholder_gray_value_);
  }

private:
  void validate_configuration() const
  {
    pointcloud_colorizer::require_non_empty(
      input_registered_cloud_topic_, "input_registered_cloud_topic");
    pointcloud_colorizer::require_non_empty(input_odometry_topic_, "input_odometry_topic");
    pointcloud_colorizer::require_non_empty(input_image_topic_, "input_image_topic");
    pointcloud_colorizer::require_non_empty(camera_info_topic_, "camera_info_topic");
    pointcloud_colorizer::require_non_empty(output_cloud_topic_, "output_cloud_topic");
    pointcloud_colorizer::require_non_empty(output_map_topic_, "output_map_topic");

    if (map_voxel_size_ <= 0.0f) {
      throw std::runtime_error("map_voxel_size must be > 0");
    }

    if (transform_source_ == TransformSource::TfTree) {
      pointcloud_colorizer::require_non_empty(camera_frame_id_, "camera_frame_id");
      pointcloud_colorizer::require_non_empty(lidar_frame_id_, "lidar_frame_id");
    }
  }

  void initialize_transform()
  {
    if (transform_source_ == TransformSource::Config) {
      camera_to_lidar_transform_ = pointcloud_colorizer::matrix_from_row_major_values(
        camera_to_lidar_matrix_values_, "camera_to_lidar_matrix");

      RCLCPP_INFO(this->get_logger(), "Using lidar/camera transform from ROS parameters.");
      return;
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(
        this->get_logger(),
        "Using lidar/camera transform from TF tree: target lidar_frame_id=%s source camera_frame_id=%s",
        lidar_frame_id_.c_str(),
        camera_frame_id_.c_str());
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    ++camera_info_received_count_;
    camera_info_seen_ = true;
    last_camera_info_receive_time_ = this->now();

    if (camera_info_received_) {
      return;
    }

    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
      msg->k[0], msg->k[1], msg->k[2],
      msg->k[3], msg->k[4], msg->k[5],
      msg->k[6], msg->k[7], msg->k[8]);
    dist_coeffs_ = cv::Mat(msg->d).clone();

    camera_info_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Camera calibration parameters successfully received.");
  }

  void odometry_input_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odometry_seen_ = true;
    ++odometry_received_count_;
    last_odometry_receive_time_ = this->now();
    last_odometry_stamp_ = rclcpp::Time(msg->header.stamp);
  }

  void image_input_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
  {
    image_seen_ = true;
    ++image_received_count_;
    last_image_receive_time_ = this->now();
    last_image_stamp_ = rclcpp::Time(msg->header.stamp);
  }

  void cloud_input_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    cloud_seen_ = true;
    ++cloud_received_count_;
    last_cloud_receive_time_ = this->now();
    last_cloud_stamp_ = rclcpp::Time(msg->header.stamp);
  }

  bool get_camera_to_lidar_transform(
    const rclcpp::Time & stamp,
    Eigen::Matrix4f & transform) const
  {
    if (transform_source_ == TransformSource::Config) {
      transform = camera_to_lidar_transform_;
      return true;
    }

    try {
      const auto transform_msg = tf_buffer_->lookupTransform(
        lidar_frame_id_,
        camera_frame_id_,
        stamp,
        transform_lookup_timeout_);
      transform = pointcloud_colorizer::matrix_from_transform_msg(transform_msg.transform);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not lookup camera->lidar transform: %s", ex.what());
      return false;
    } catch (const std::runtime_error & ex) {
      RCLCPP_WARN(this->get_logger(), "Invalid camera->lidar transform: %s", ex.what());
      return false;
    }
  }

  void topic_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr img_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
  {
    ++sync_callback_count_;
    last_sync_callback_time_ = this->now();

    if (!camera_info_received_) {
      ++skipped_missing_camera_info_count_;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        static_cast<int64_t>(warning_throttle_sec_ * 1000.0),
        "Skipping synchronized callback: camera_info has not been received yet.");
      return;
    }

    const rclcpp::Time odom_time(odom_msg->header.stamp);
    const rclcpp::Time cloud_time(cloud_msg->header.stamp);
    const auto dt_ns = (odom_time - cloud_time).nanoseconds();
    RCLCPP_INFO(this->get_logger(), "dt = %ld ns", dt_ns);

    auto cloud_in = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*cloud_msg, *cloud_in);

    cv::Mat cv_image;
    try {
      cv_image = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
    } catch (cv_bridge::Exception & e) {
      ++image_decode_failure_count_;
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    Eigen::Quaternionf q_odom_lidar(
      odom_msg->pose.pose.orientation.w,
      odom_msg->pose.pose.orientation.x,
      odom_msg->pose.pose.orientation.y,
      odom_msg->pose.pose.orientation.z);

    Eigen::Matrix4f t_odom_lidar = Eigen::Matrix4f::Identity();
    t_odom_lidar.block<3, 3>(0, 0) = q_odom_lidar.toRotationMatrix();
    t_odom_lidar(0, 3) = odom_msg->pose.pose.position.x;
    t_odom_lidar(1, 3) = odom_msg->pose.pose.position.y;
    t_odom_lidar(2, 3) = odom_msg->pose.pose.position.z;

    Eigen::Matrix4f t_camera_to_lidar = Eigen::Matrix4f::Identity();
    if (!get_camera_to_lidar_transform(rclcpp::Time(cloud_msg->header.stamp), t_camera_to_lidar)) {
      ++transform_lookup_failure_count_;
      return;
    }

    const Eigen::Matrix4f t_odom_cam = t_odom_lidar * t_camera_to_lidar;
    const Eigen::Matrix4f t_cam_odom = t_odom_cam.inverse();

    const double fx = camera_matrix_.at<double>(0, 0);
    const double fy = camera_matrix_.at<double>(1, 1);
    const double cx = camera_matrix_.at<double>(0, 2);
    const double cy = camera_matrix_.at<double>(1, 2);

    auto cloud_out = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    cloud_out->points.reserve(cloud_in->points.size());

    for (const auto & point : cloud_in->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      std::size_t output_index = 0;
      if (!publish_only_colored_points_) {
        pcl::PointXYZRGB color_point;
        color_point.x = point.x;
        color_point.y = point.y;
        color_point.z = point.z;
        color_point.r = 128;
        color_point.g = 128;
        color_point.b = 128;
        cloud_out->points.push_back(color_point);
        output_index = cloud_out->points.size() - 1;
      }

      const Eigen::Vector4f pt_odom(point.x, point.y, point.z, 1.0f);
      const Eigen::Vector4f pt_camera = t_cam_odom * pt_odom;

      if (pt_camera.z() <= 0.0f) {
        continue;
      }

      const double u = fx * (pt_camera.x() / pt_camera.z()) + cx;
      const double v = fy * (pt_camera.y() / pt_camera.z()) + cy;
      const int pixel_u = static_cast<int>(std::round(u));
      const int pixel_v = static_cast<int>(std::round(v));

      if (pixel_u < 0 || pixel_u >= cv_image.cols ||
        pixel_v < 0 || pixel_v >= cv_image.rows)
      {
        continue;
      }

      const cv::Vec3b & color = cv_image.at<cv::Vec3b>(pixel_v, pixel_u);
      if (publish_only_colored_points_) {
        pcl::PointXYZRGB color_point;
        color_point.x = point.x;
        color_point.y = point.y;
        color_point.z = point.z;
        color_point.b = color[0];
        color_point.g = color[1];
        color_point.r = color[2];
        cloud_out->points.push_back(color_point);
      } else {
        auto & color_point = cloud_out->points[output_index];
        color_point.b = color[0];
        color_point.g = color[1];
        color_point.r = color[2];
      }
    }

    cloud_out->width = cloud_out->points.size();
    cloud_out->height = 1;
    cloud_out->is_dense = true;

    if (cloud_out->points.empty()) {
      ++zero_output_cloud_count_;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        static_cast<int64_t>(warning_throttle_sec_ * 1000.0),
        "Synchronized callback produced an empty colored cloud.");
    }

    sensor_msgs::msg::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud_out, msg_out);
    msg_out.header = cloud_msg->header;
    if (!output_frame_id_.empty()) {
      msg_out.header.frame_id = output_frame_id_;
    }
    publisher_->publish(msg_out);
    ++output_publish_count_;
    last_output_publish_time_ = this->now();

    for (const auto & point : cloud_out->points) {
      voxel_color_estimator_->update(point);
    }

    const auto map_cloud = voxel_color_estimator_->build_cloud();

    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*map_cloud, map_msg);
    map_msg.header = cloud_msg->header;
    if (!map_frame_id_.empty()) {
      map_msg.header.frame_id = map_frame_id_;
    }
    map_publisher_->publish(map_msg);
    ++map_publish_count_;
  }

  bool is_stale(bool seen, const rclcpp::Time & last_time, const rclcpp::Time & now) const
  {
    if (!seen) {
      return true;
    }
    return (now - last_time).seconds() > input_stale_timeout_sec_;
  }

  double age_seconds(bool seen, const rclcpp::Time & last_time, const rclcpp::Time & now) const
  {
    if (!seen) {
      return -1.0;
    }
    return (now - last_time).seconds();
  }

  void publish_diagnostics(
    const rclcpp::Time & now,
    std::uint8_t level,
    const std::string & summary,
    const std::string & detail)
  {
    if (!publish_diagnostics_ || !diagnostics_publisher_) {
      return;
    }

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = now;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "pointcloud_colorizer/registered_cloud_colorizer";
    status.hardware_id = "pointcloud_colorizer";
    status.level = level;
    status.message = summary;

    const auto add_value = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = key;
      pair.value = value;
      status.values.push_back(pair);
    };

    add_value("detail", detail);
    add_value("odometry_seen", odometry_seen_ ? "true" : "false");
    add_value("image_seen", image_seen_ ? "true" : "false");
    add_value("cloud_seen", cloud_seen_ ? "true" : "false");
    add_value("camera_info_received", camera_info_received_ ? "true" : "false");
    add_value("odometry_age_sec", std::to_string(age_seconds(odometry_seen_, last_odometry_receive_time_, now)));
    add_value("image_age_sec", std::to_string(age_seconds(image_seen_, last_image_receive_time_, now)));
    add_value("cloud_age_sec", std::to_string(age_seconds(cloud_seen_, last_cloud_receive_time_, now)));
    add_value("camera_info_age_sec", std::to_string(age_seconds(camera_info_seen_, last_camera_info_receive_time_, now)));
    add_value("sync_age_sec", std::to_string(age_seconds(sync_callback_count_ > 0, last_sync_callback_time_, now)));
    add_value("output_age_sec", std::to_string(age_seconds(output_publish_count_ > 0, last_output_publish_time_, now)));
    add_value("odometry_received_count", std::to_string(odometry_received_count_));
    add_value("image_received_count", std::to_string(image_received_count_));
    add_value("cloud_received_count", std::to_string(cloud_received_count_));
    add_value("camera_info_received_count", std::to_string(camera_info_received_count_));
    add_value("sync_callback_count", std::to_string(sync_callback_count_));
    add_value("output_publish_count", std::to_string(output_publish_count_));
    add_value("map_publish_count", std::to_string(map_publish_count_));
    add_value("image_decode_failure_count", std::to_string(image_decode_failure_count_));
    add_value("transform_lookup_failure_count", std::to_string(transform_lookup_failure_count_));
    add_value("skipped_missing_camera_info_count", std::to_string(skipped_missing_camera_info_count_));
    add_value("zero_output_cloud_count", std::to_string(zero_output_cloud_count_));
    add_value("sync_queue_size", std::to_string(sync_queue_size_));
    add_value("input_stale_timeout_sec", std::to_string(input_stale_timeout_sec_));
    add_value("voxel_count", std::to_string(voxel_color_estimator_->voxel_count()));
    add_value("voxel_updates_total", std::to_string(voxel_color_estimator_->stats().voxel_updates_total));
    add_value("voxel_new_cells", std::to_string(voxel_color_estimator_->stats().voxel_new_cells));
    add_value("voxel_burnin_updates", std::to_string(voxel_color_estimator_->stats().voxel_burnin_updates));
    add_value("voxel_filtered_updates", std::to_string(voxel_color_estimator_->stats().voxel_filtered_updates));
    add_value("voxel_skipped_placeholder", std::to_string(voxel_color_estimator_->stats().voxel_skipped_placeholder));

    diagnostics.status.push_back(status);
    diagnostics_publisher_->publish(diagnostics);
  }

  void health_timer_callback()
  {
    const auto now = this->now();
    const bool in_startup_grace = (now - startup_time_).seconds() < startup_grace_period_sec_;

    std::uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string summary = "registered_cloud_colorizer healthy";
    std::ostringstream detail;

    std::vector<std::string> missing_inputs;
    if (!odometry_seen_) {
      missing_inputs.push_back("odometry");
    }
    if (!image_seen_) {
      missing_inputs.push_back("image");
    }
    if (!cloud_seen_) {
      missing_inputs.push_back("registered_cloud");
    }

    if (!missing_inputs.empty()) {
      level = in_startup_grace ?
        diagnostic_msgs::msg::DiagnosticStatus::WARN :
        diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      summary = "Waiting for input stream(s): ";
      for (std::size_t i = 0; i < missing_inputs.size(); ++i) {
        summary += missing_inputs[i];
        if (i + 1 < missing_inputs.size()) {
          summary += ", ";
        }
      }
    } else if (!camera_info_received_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      summary = "Inputs received but camera_info has not been received";
    } else {
      const bool odom_stale = is_stale(odometry_seen_, last_odometry_receive_time_, now);
      const bool image_stale = is_stale(image_seen_, last_image_receive_time_, now);
      const bool cloud_stale = is_stale(cloud_seen_, last_cloud_receive_time_, now);

      if (odom_stale || image_stale || cloud_stale) {
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        summary = "Input stream stale: ";
        if (odom_stale) {
          summary += "odometry ";
        }
        if (image_stale) {
          summary += "image ";
        }
        if (cloud_stale) {
          summary += "registered_cloud ";
        }
      } else {
        const double sync_age = age_seconds(sync_callback_count_ > 0, last_sync_callback_time_, now);
        const double output_age = age_seconds(output_publish_count_ > 0, last_output_publish_time_, now);

        if (sync_callback_count_ == 0 || sync_age > input_stale_timeout_sec_) {
          level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
          summary = "Inputs are active but synchronization is not producing callbacks";
        } else if (output_publish_count_ == 0 || output_age > input_stale_timeout_sec_) {
          level = (image_decode_failure_count_ > 0 || transform_lookup_failure_count_ > 0) ?
            diagnostic_msgs::msg::DiagnosticStatus::ERROR :
            diagnostic_msgs::msg::DiagnosticStatus::WARN;
          summary = "Synchronized callbacks exist but colored cloud output is missing";
        }
      }
    }

    detail
      << "odom_age_sec=" << age_seconds(odometry_seen_, last_odometry_receive_time_, now)
      << " image_age_sec=" << age_seconds(image_seen_, last_image_receive_time_, now)
      << " cloud_age_sec=" << age_seconds(cloud_seen_, last_cloud_receive_time_, now)
      << " camera_info_age_sec=" << age_seconds(camera_info_seen_, last_camera_info_receive_time_, now)
      << " sync_age_sec=" << age_seconds(sync_callback_count_ > 0, last_sync_callback_time_, now)
      << " output_age_sec=" << age_seconds(output_publish_count_ > 0, last_output_publish_time_, now)
      << " sync_count=" << sync_callback_count_
      << " output_count=" << output_publish_count_
      << " decode_failures=" << image_decode_failure_count_
      << " transform_failures=" << transform_lookup_failure_count_
      << " skipped_missing_camera_info=" << skipped_missing_camera_info_count_;

    const bool reason_changed = summary != last_health_summary_ || level != last_health_level_;
    const bool throttle_elapsed =
      (now - last_health_log_time_).seconds() >= warning_throttle_sec_;

    if (level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
      if (last_health_level_ != diagnostic_msgs::msg::DiagnosticStatus::OK) {
        RCLCPP_INFO(this->get_logger(), "Health recovered: %s | %s", summary.c_str(), detail.str().c_str());
      }
    } else if (reason_changed || throttle_elapsed) {
      if (level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
        RCLCPP_ERROR(this->get_logger(), "Health state: %s | %s", summary.c_str(), detail.str().c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "Health state: %s | %s", summary.c_str(), detail.str().c_str());
      }
      last_health_log_time_ = now;
    }

    last_health_summary_ = summary;
    last_health_level_ = level;

    publish_diagnostics(now, level, summary, detail.str());
  }

  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_;
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> image_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> registered_cloud_;
  std::shared_ptr<RegisteredSync> sync_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  std::unique_ptr<pointcloud_colorizer::VoxelColorEstimator> voxel_color_estimator_;
  std::string input_registered_cloud_topic_;
  std::string input_odometry_topic_;
  std::string input_image_topic_;
  std::string camera_info_topic_;
  std::string output_cloud_topic_;
  std::string output_map_topic_;
  std::string output_frame_id_;
  std::string map_frame_id_;
  std::string transform_source_string_;
  std::string camera_frame_id_;
  std::string lidar_frame_id_;
  std::vector<double> camera_to_lidar_matrix_values_;
  TransformSource transform_source_ = TransformSource::Config;
  rclcpp::Duration transform_lookup_timeout_ = rclcpp::Duration::from_seconds(0.1);
  float map_voxel_size_ = 0.3f;
  int sync_queue_size_ = 10;
  bool publish_only_colored_points_ = true;
  bool publish_diagnostics_ = true;
  bool camera_info_received_ = false;
  bool camera_info_seen_ = false;
  bool odometry_seen_ = false;
  bool image_seen_ = false;
  bool cloud_seen_ = false;
  double health_report_interval_sec_ = 5.0;
  double input_stale_timeout_sec_ = 2.0;
  double warning_throttle_sec_ = 5.0;
  double startup_grace_period_sec_ = 8.0;
  int color_burnin_samples_ = 5;
  int color_step_max_ = 16;
  bool color_ignore_placeholder_gray_ = true;
  int placeholder_gray_value_ = 128;
  std::string diagnostics_topic_ = "/diagnostics";
  std::uint8_t last_health_level_ = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string last_health_summary_;
  rclcpp::Time startup_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_health_log_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odometry_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_image_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cloud_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_camera_info_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_sync_callback_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_output_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odometry_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_image_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cloud_stamp_{0, 0, RCL_ROS_TIME};
  std::uint64_t odometry_received_count_ = 0;
  std::uint64_t image_received_count_ = 0;
  std::uint64_t cloud_received_count_ = 0;
  std::uint64_t camera_info_received_count_ = 0;
  std::uint64_t sync_callback_count_ = 0;
  std::uint64_t output_publish_count_ = 0;
  std::uint64_t map_publish_count_ = 0;
  std::uint64_t image_decode_failure_count_ = 0;
  std::uint64_t transform_lookup_failure_count_ = 0;
  std::uint64_t skipped_missing_camera_info_count_ = 0;
  std::uint64_t zero_output_cloud_count_ = 0;
  Eigen::Matrix4f camera_to_lidar_transform_ = Eigen::Matrix4f::Identity();
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RegisteredCloudColorizerNode>());
  rclcpp::shutdown();
  return 0;
}
