#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "pointcloud_colorizer/raw_cloud_colorizer_core.hpp"
#include "pointcloud_colorizer/transform_utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rmw/qos_profiles.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using std::placeholders::_1;
using std::placeholders::_2;
using pointcloud_colorizer::TransformSource;

class RawCloudColorizerColorNode : public rclcpp::Node
{
  using CompressedSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::CompressedImage>;
  using CompressedSync = message_filters::Synchronizer<CompressedSyncPolicy>;
  using UncompressedSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::Image>;
  using UncompressedSync = message_filters::Synchronizer<UncompressedSyncPolicy>;

  enum class ImageSource
  {
    Unknown,
    Uncompressed,
    Compressed,
  };

public:
  explicit RawCloudColorizerColorNode(const rclcpp::NodeOptions & options)
  : Node("raw_cloud_colorizer", options)
  {
    input_cloud_topic_ = this->declare_parameter<std::string>("input_cloud_topic", "");
    fixed_sync_image_stream_ = this->declare_parameter<std::string>(
      "fixed_sync_image_stream", "compressed");
    input_compressed_image_topic_ = this->declare_parameter<std::string>("input_image_topic", "");
    input_uncompressed_image_topic_ = this->declare_parameter<std::string>("input_image_topic_raw", "");
    // Backward-compatible aliases.
    const auto legacy_input_compressed_image_topic = this->declare_parameter<std::string>(
      "input_compressed_image_topic", "");
    const auto legacy_input_uncompressed_image_topic = this->declare_parameter<std::string>(
      "input_uncompressed_image_topic", "");
    if (input_compressed_image_topic_.empty()) {
      input_compressed_image_topic_ = legacy_input_compressed_image_topic;
    }
    if (input_uncompressed_image_topic_.empty()) {
      input_uncompressed_image_topic_ = legacy_input_uncompressed_image_topic;
    }
    camera_info_topic_ = this->declare_parameter<std::string>("camera_info_topic", "");
    output_cloud_topic_ = this->declare_parameter<std::string>("output_cloud_topic", "");
    output_frame_id_ = this->declare_parameter<std::string>("output_frame_id", "");
    publish_colorized_identity_tf_ = this->declare_parameter<bool>("publish_colorized_identity_tf", true);
    colorized_frame_id_ = this->declare_parameter<std::string>("colorized_frame_id", "colorized_lidar_link");
    publish_only_colored_points_ = this->declare_parameter<bool>("publish_only_colored_points", true);
    debug_image_topic_ = this->declare_parameter<std::string>(
      "debug_image_topic", "/colorizer/raw/debug_overlay");
    debug_overlay_point_radius_ = std::max<int>(1, this->declare_parameter<int>("debug_overlay_point_radius", 2));
    use_fixed_sync_ = this->declare_parameter<bool>("use_fixed_sync", true);
    // Backward-compatible alias.
    use_fixed_sync_ = this->declare_parameter<bool>("use_fixed_compressed_sync", use_fixed_sync_);
    // Backward-compatible alias.
    fixed_sync_image_stream_ = this->declare_parameter<std::string>(
      "image_stream", fixed_sync_image_stream_);
    source_selector_max_compressed_frames_ = std::max<int>(
      1,
      this->declare_parameter<int>("source_selector_max_compressed_frames", 10));
    source_selector_max_wait_ = rclcpp::Duration::from_seconds(
      this->declare_parameter<double>("source_selector_max_wait_sec", 0.5));
    transform_source_string_ = this->declare_parameter<std::string>("transform_source", "config");
    camera_frame_id_ = this->declare_parameter<std::string>("camera_frame_id", "");
    lidar_frame_id_ = this->declare_parameter<std::string>("lidar_frame_id", "");
    camera_to_lidar_matrix_values_ = this->declare_parameter<std::vector<double>>(
      "camera_to_lidar_matrix", std::vector<double>{});
    transform_lookup_timeout_ = rclcpp::Duration::from_seconds(
      this->declare_parameter<double>("transform_lookup_timeout_sec", 0.1));
    sync_queue_size_ = std::max<int>(1, this->declare_parameter<int>("sync_queue_size", 10));

    sky_filter_enabled_ = this->declare_parameter<bool>("sky_filter_enabled", true);
    sky_region_max_y_fraction_ = this->declare_parameter<double>("sky_region_max_y_fraction", 0.5);
    sky_region_invert_y_ = this->declare_parameter<bool>("sky_region_invert_y", false);
    sky_blue_h_min_ = this->declare_parameter<int>("sky_blue_h_min", 90);
    sky_blue_h_max_ = this->declare_parameter<int>("sky_blue_h_max", 130);
    sky_blue_s_min_ = this->declare_parameter<int>("sky_blue_s_min", 25);
    sky_blue_v_min_ = this->declare_parameter<int>("sky_blue_v_min", 135);
    sky_cloud_s_max_ = this->declare_parameter<int>("sky_cloud_s_max", 40);
    sky_cloud_v_min_ = this->declare_parameter<int>("sky_cloud_v_min", 155);

    if (!use_fixed_sync_ && input_uncompressed_image_topic_.empty() &&
      ends_with(input_compressed_image_topic_, "/compressed"))
    {
      input_uncompressed_image_topic_ =
        input_compressed_image_topic_.substr(
        0,
        input_compressed_image_topic_.size() - std::string("/compressed").size());
    }

    if (fixed_sync_image_stream_ != "compressed" && fixed_sync_image_stream_ != "raw") {
      throw std::runtime_error("fixed_sync_image_stream must be one of: compressed, raw");
    }

    transform_source_ = pointcloud_colorizer::parse_transform_source(transform_source_string_);
    validate_configuration();
    initialize_core();
    initialize_transform();
    if (publish_colorized_identity_tf_) {
      colorized_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    output_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);
    
    debug_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      debug_image_topic_,
      rclcpp::SensorDataQoS());

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&RawCloudColorizerColorNode::camera_info_callback, this, std::placeholders::_1));

    if (use_fixed_sync_) {
      if (fixed_sync_image_stream_ == "raw") {
        initialize_uncompressed_synchronizer();
        selected_source_ = ImageSource::Uncompressed;
      } else {
        initialize_compressed_synchronizer();
        selected_source_ = ImageSource::Compressed;
      }
    } else {
      initialize_source_selector();
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Raw colorizer(color-only): cloud=%s image_compressed=%s image_raw=%s selector=%s camera_info=%s output=%s output_frame=%s publish_only_colored_points=%s",
      input_cloud_topic_.c_str(),
      input_compressed_image_topic_.c_str(),
      input_uncompressed_image_topic_.c_str(),
      use_fixed_sync_ ? "disabled" : "enabled",
      camera_info_topic_.c_str(),
      output_cloud_topic_.c_str(),
      output_frame_id_.empty() ? "<input>" : output_frame_id_.c_str(),
      publish_only_colored_points_ ? "true" : "false");

    if (debug_image_topic_.empty()) {
      RCLCPP_INFO(this->get_logger(), "Debug image publisher disabled (debug_image_topic is empty)");
    } else {
      RCLCPP_INFO(this->get_logger(), "Debug image topic: %s", debug_image_topic_.c_str());
    }

    if (publish_colorized_identity_tf_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Publishing identity TF link from input cloud frame to %s",
        colorized_frame_id_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Identity TF publishing disabled");
    }
  }

private:
  void validate_configuration() const
  {
    pointcloud_colorizer::require_non_empty(input_cloud_topic_, "input_cloud_topic");
    pointcloud_colorizer::require_non_empty(input_compressed_image_topic_, "input_image_topic");
    pointcloud_colorizer::require_non_empty(camera_info_topic_, "camera_info_topic");
    pointcloud_colorizer::require_non_empty(output_cloud_topic_, "output_cloud_topic");

    if (!use_fixed_sync_) {
      pointcloud_colorizer::require_non_empty(input_uncompressed_image_topic_, "input_image_topic_raw");
    }

    if (use_fixed_sync_ && fixed_sync_image_stream_ == "raw") {
      pointcloud_colorizer::require_non_empty(input_uncompressed_image_topic_, "input_image_topic_raw");
    }

    if (transform_source_ == TransformSource::TfTree) {
      pointcloud_colorizer::require_non_empty(camera_frame_id_, "camera_frame_id");
      pointcloud_colorizer::require_non_empty(lidar_frame_id_, "lidar_frame_id");
    }

    if (publish_colorized_identity_tf_) {
      pointcloud_colorizer::require_non_empty(colorized_frame_id_, "colorized_frame_id");
    }

    if (sky_region_max_y_fraction_ <= 0.0 || sky_region_max_y_fraction_ > 1.0) {
      throw std::runtime_error("sky_region_max_y_fraction must be in (0, 1]");
    }

    if (sky_blue_h_min_ < 0 || sky_blue_h_max_ > 179 || sky_blue_h_min_ > sky_blue_h_max_) {
      throw std::runtime_error("sky blue hue range must satisfy 0 <= sky_blue_h_min <= sky_blue_h_max <= 179");
    }

    if (sky_blue_s_min_ < 0 || sky_blue_s_min_ > 255 || sky_blue_v_min_ < 0 || sky_blue_v_min_ > 255 ||
      sky_cloud_s_max_ < 0 || sky_cloud_s_max_ > 255 || sky_cloud_v_min_ < 0 || sky_cloud_v_min_ > 255)
    {
      throw std::runtime_error("sky HSV thresholds must be in [0, 255]");
    }
  }

  void publish_identity_colorized_tf(const std_msgs::msg::Header & source_header)
  {
    if (!publish_colorized_identity_tf_ || !colorized_tf_broadcaster_) {
      return;
    }

    if (source_header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Skipping identity TF publish because source cloud frame is empty.");
      return;
    }

    if (source_header.frame_id == colorized_frame_id_) {
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = source_header;
    tf_msg.child_frame_id = colorized_frame_id_;
    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;
    colorized_tf_broadcaster_->sendTransform(tf_msg);
  }

  static bool ends_with(const std::string & value, const std::string & suffix)
  {
    if (value.size() < suffix.size()) {
      return false;
    }
    return value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
  }

  void initialize_source_selector()
  {
    pointcloud_colorizer::require_non_empty(input_uncompressed_image_topic_, "input_image_topic_raw");

    compressed_detection_count_ = 0;
    selected_source_ = ImageSource::Unknown;
    source_selector_start_time_ = this->now();

    uncompressed_detection_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      input_uncompressed_image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&RawCloudColorizerColorNode::uncompressed_detection_callback, this, std::placeholders::_1));

    compressed_detection_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      input_compressed_image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&RawCloudColorizerColorNode::compressed_detection_callback, this, std::placeholders::_1));

    selector_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&RawCloudColorizerColorNode::source_selector_timer_callback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Image source selector started: waiting up to %d compressed frames or %.3fs for raw image topic=%s",
      source_selector_max_compressed_frames_,
      source_selector_max_wait_.seconds(),
      input_uncompressed_image_topic_.c_str());
  }

  void initialize_compressed_synchronizer()
  {
    const auto sensor_qos = rmw_qos_profile_sensor_data;
    cloud_sub_.unsubscribe();
    compressed_image_sub_.unsubscribe();
    uncompressed_image_sub_.unsubscribe();

    cloud_sub_.subscribe(this, input_cloud_topic_, sensor_qos);
    compressed_image_sub_.subscribe(this, input_compressed_image_topic_, sensor_qos);
    compressed_sync_ = std::make_shared<CompressedSync>(
      CompressedSyncPolicy(sync_queue_size_),
      cloud_sub_,
      compressed_image_sub_);
    compressed_sync_->registerCallback(
      std::bind(&RawCloudColorizerColorNode::compressed_topic_callback, this, _1, _2));
    uncompressed_sync_.reset();
  }

  void initialize_uncompressed_synchronizer()
  {
    const auto sensor_qos = rmw_qos_profile_sensor_data;
    cloud_sub_.unsubscribe();
    compressed_image_sub_.unsubscribe();
    uncompressed_image_sub_.unsubscribe();

    cloud_sub_.subscribe(this, input_cloud_topic_, sensor_qos);
    uncompressed_image_sub_.subscribe(this, input_uncompressed_image_topic_, sensor_qos);
    uncompressed_sync_ = std::make_shared<UncompressedSync>(
      UncompressedSyncPolicy(sync_queue_size_),
      cloud_sub_,
      uncompressed_image_sub_);
    uncompressed_sync_->registerCallback(
      std::bind(&RawCloudColorizerColorNode::uncompressed_topic_callback, this, _1, _2));
    compressed_sync_.reset();
  }

  void stop_source_selector_subscriptions()
  {
    if (selector_timer_) {
      selector_timer_->cancel();
      selector_timer_.reset();
    }
    compressed_detection_sub_.reset();
    uncompressed_detection_sub_.reset();
  }

  void select_source_and_initialize(const ImageSource source, const char * reason)
  {
    {
      std::lock_guard<std::mutex> lock(source_selector_mutex_);
      if (selected_source_ != ImageSource::Unknown) {
        return;
      }
      selected_source_ = source;
    }

    stop_source_selector_subscriptions();

    if (source == ImageSource::Uncompressed) {
      initialize_uncompressed_synchronizer();
      RCLCPP_INFO(this->get_logger(), "Selected uncompressed image source: %s", reason);
      return;
    }

    initialize_compressed_synchronizer();
    RCLCPP_INFO(this->get_logger(), "Selected compressed image source: %s", reason);
  }

  void uncompressed_detection_callback(const sensor_msgs::msg::Image::ConstSharedPtr)
  {
    select_source_and_initialize(ImageSource::Uncompressed, "raw image detected during selector phase");
  }

  void compressed_detection_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr)
  {
    {
      std::lock_guard<std::mutex> lock(source_selector_mutex_);
      if (selected_source_ != ImageSource::Unknown) {
        return;
      }
      ++compressed_detection_count_;
      if (compressed_detection_count_ < source_selector_max_compressed_frames_) {
        return;
      }
    }

    select_source_and_initialize(
      ImageSource::Compressed,
      "raw image not detected within compressed frame threshold");
  }

  void source_selector_timer_callback()
  {
    {
      std::lock_guard<std::mutex> lock(source_selector_mutex_);
      if (selected_source_ != ImageSource::Unknown || compressed_detection_count_ == 0) {
        return;
      }

      if ((this->now() - source_selector_start_time_) < source_selector_max_wait_) {
        return;
      }
    }

    select_source_and_initialize(
      ImageSource::Compressed,
      "raw image not detected before selector timeout");
  }

  void initialize_core()
  {
    pointcloud_colorizer::RawCloudColorizerCoreConfig config;
    config.publish_only_colored_points = publish_only_colored_points_;
    config.sky_filter.enabled = sky_filter_enabled_;
    config.sky_filter.region_max_y_fraction = sky_region_max_y_fraction_;
    config.sky_filter.region_invert_y = sky_region_invert_y_;
    config.sky_filter.blue_h_min = sky_blue_h_min_;
    config.sky_filter.blue_h_max = sky_blue_h_max_;
    config.sky_filter.blue_s_min = sky_blue_s_min_;
    config.sky_filter.blue_v_min = sky_blue_v_min_;
    config.sky_filter.cloud_s_max = sky_cloud_s_max_;
    config.sky_filter.cloud_v_min = sky_cloud_v_min_;
    config.debug_point_radius = debug_overlay_point_radius_;
    colorizer_core_ = std::make_unique<pointcloud_colorizer::RawCloudColorizerCore>(config);
  }

  void initialize_transform()
  {
    if (transform_source_ == TransformSource::Config) {
      camera_to_lidar_transform_ = pointcloud_colorizer::matrix_from_row_major_values(
        camera_to_lidar_matrix_values_, "camera_to_lidar_matrix");
      lidar_to_camera_transform_ = camera_to_lidar_transform_.inverse();
      return;
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (camera_info_received_) {
      return;
    }

    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
      msg->k[0], msg->k[1], msg->k[2],
      msg->k[3], msg->k[4], msg->k[5],
      msg->k[6], msg->k[7], msg->k[8]);
    dist_coeffs_ = cv::Mat(msg->d).clone();

    camera_info_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Camera calibration parameters received.");
  }

  bool get_lidar_to_camera_transform(const rclcpp::Time & stamp, Eigen::Matrix4f & transform)
  {
    if (transform_source_ == TransformSource::Config) {
      transform = lidar_to_camera_transform_;
      return true;
    }

    try {
      const auto transform_msg = tf_buffer_->lookupTransform(
        camera_frame_id_,
        lidar_frame_id_,
        stamp,
        transform_lookup_timeout_);
      transform = pointcloud_colorizer::matrix_from_transform_msg(transform_msg.transform);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not lookup lidar->camera transform: %s", ex.what());
      return false;
    } catch (const std::runtime_error & ex) {
      RCLCPP_WARN(this->get_logger(), "Invalid lidar->camera transform: %s", ex.what());
      return false;
    }
  }

  void compressed_topic_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr img_msg)
  {
    cv::Mat cv_image;
    try {
      cv_image = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    process_coloring(cloud_msg, img_msg->header, cv_image);
  }

  void uncompressed_topic_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
  {
    cv::Mat cv_image;
    try {
      cv_image = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    process_coloring(cloud_msg, img_msg->header, cv_image);
  }

  void process_coloring(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
    const std_msgs::msg::Header & image_header,
    const cv::Mat & cv_image)
  {
    if (!camera_info_received_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Skipping callback: camera_info has not been received yet.");
      return;
    }

    auto cloud_in = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*cloud_msg, *cloud_in);

    Eigen::Matrix4f t_lidar_camera = Eigen::Matrix4f::Identity();
    if (!get_lidar_to_camera_transform(rclcpp::Time(cloud_msg->header.stamp), t_lidar_camera)) {
      return;
    }

    const bool publish_debug_overlay = debug_image_publisher_ &&
      (debug_image_publisher_->get_subscription_count() > 0 ||
      debug_image_publisher_->get_intra_process_subscription_count() > 0);

    pointcloud_colorizer::RawCloudColorizerProcessInput process_input;
    process_input.cloud = cloud_in;
    process_input.bgr_image = cv_image;
    process_input.lidar_to_camera_transform = t_lidar_camera;
    process_input.camera_matrix = camera_matrix_;
    process_input.dist_coeffs = dist_coeffs_;

    pointcloud_colorizer::RawCloudColorizerProcessOutput process_output;
    try {
      process_output = colorizer_core_->process(process_input, publish_debug_overlay);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "Colorizer core processing failed: %s", ex.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*process_output.colored_cloud, output_msg);
    output_msg.header = cloud_msg->header;
    if (!output_frame_id_.empty()) {
      output_msg.header.frame_id = output_frame_id_;
    }

    publish_identity_colorized_tf(cloud_msg->header);

    output_publisher_->publish(output_msg);

    if (publish_debug_overlay && process_output.has_debug_overlay) {
      sensor_msgs::msg::Image::SharedPtr debug_msg =
        cv_bridge::CvImage(image_header, "bgr8", process_output.debug_overlay).toImageMsg();
      debug_image_publisher_->publish(*debug_msg);
    }
  }

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> compressed_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> uncompressed_image_sub_;
  std::shared_ptr<CompressedSync> compressed_sync_;
  std::shared_ptr<UncompressedSync> uncompressed_sync_;

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr uncompressed_detection_sub_;
  rclcpp::TimerBase::SharedPtr selector_timer_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_publisher_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> colorized_tf_broadcaster_;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  std::mutex source_selector_mutex_;
  std::unique_ptr<pointcloud_colorizer::RawCloudColorizerCore> colorizer_core_;

  std::string input_cloud_topic_;
  std::string input_compressed_image_topic_;
  std::string input_uncompressed_image_topic_;
  std::string camera_info_topic_;
  std::string output_cloud_topic_;
  std::string debug_image_topic_;
  std::string output_frame_id_;
  std::string colorized_frame_id_;
  std::string transform_source_string_;
  std::string camera_frame_id_;
  std::string lidar_frame_id_;

  std::vector<double> camera_to_lidar_matrix_values_;
  TransformSource transform_source_ = TransformSource::Config;
  rclcpp::Duration transform_lookup_timeout_ = rclcpp::Duration::from_seconds(0.1);
  rclcpp::Duration source_selector_max_wait_ = rclcpp::Duration::from_seconds(0.5);
  rclcpp::Time source_selector_start_time_;

  int sync_queue_size_ = 10;
  int source_selector_max_compressed_frames_ = 10;
  int debug_overlay_point_radius_ = 2;
  int compressed_detection_count_ = 0;
  bool publish_colorized_identity_tf_ = true;
  bool publish_only_colored_points_ = true;
  bool use_fixed_sync_ = true;
  bool camera_info_received_ = false;
  ImageSource selected_source_ = ImageSource::Unknown;
  std::string fixed_sync_image_stream_ = "compressed";

  bool sky_filter_enabled_ = true;
  double sky_region_max_y_fraction_ = 0.5;
  bool sky_region_invert_y_ = false;
  int sky_blue_h_min_ = 90;
  int sky_blue_h_max_ = 130;
  int sky_blue_s_min_ = 25;
  int sky_blue_v_min_ = 135;
  int sky_cloud_s_max_ = 40;
  int sky_cloud_v_min_ = 155;

  Eigen::Matrix4f camera_to_lidar_transform_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f lidar_to_camera_transform_ = Eigen::Matrix4f::Identity();
};

#ifndef POINTCLOUD_COLORIZER_BUILD_STANDALONE
RCLCPP_COMPONENTS_REGISTER_NODE(RawCloudColorizerColorNode)
#else
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RawCloudColorizerColorNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
