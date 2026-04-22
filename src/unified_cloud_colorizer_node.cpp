#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <rmw/qos_profiles.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class UnifiedCloudColorizerNode : public rclcpp::Node
{
  using RawSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::CompressedImage>;
  using RawSync = message_filters::Synchronizer<RawSyncPolicy>;

  using RegisteredSyncPolicy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry,
    sensor_msgs::msg::CompressedImage,
    sensor_msgs::msg::PointCloud2>;
  using RegisteredSync = message_filters::Synchronizer<RegisteredSyncPolicy>;

  enum class Mode
  {
    Raw,
    Registered
  };

public:
  UnifiedCloudColorizerNode()
  : Node("unified_cloud_colorizer")
  {
    mode_string_ = this->declare_parameter<std::string>("mode", "raw");
    mode_ = parse_mode(mode_string_);

    input_cloud_topic_ = this->declare_parameter<std::string>(
      "input_cloud_topic", "/os1/points");
    input_registered_cloud_topic_ = this->declare_parameter<std::string>(
      "input_registered_cloud_topic", "/liorf/mapping/cloud_registered");
    input_odometry_topic_ = this->declare_parameter<std::string>(
      "input_odometry_topic", "/liorf/mapping/odometry");
    input_image_topic_ = this->declare_parameter<std::string>(
      "input_image_topic", "/camera_front/image_raw/compressed");
    camera_info_topic_ = this->declare_parameter<std::string>(
      "camera_info_topic", "/camera_front/camera_info");
    output_cloud_topic_ = this->declare_parameter<std::string>(
      "output_cloud_topic",
      mode_ == Mode::Raw ?
      "/colorizer/raw/colored_cloud" :
      "/colorizer/registered/colored_cloud");
    output_map_topic_ = this->declare_parameter<std::string>(
      "output_map_topic", "/colorizer/registered/naive_map");
    publish_only_colored_points_ = this->declare_parameter<bool>(
      "publish_only_colored_points", true);
    map_voxel_size_ = static_cast<float>(this->declare_parameter<double>(
      "map_voxel_size", 0.3));

    validate_configuration();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&UnifiedCloudColorizerNode::camera_info_callback, this, std::placeholders::_1));

    initialize_mode();
    log_configuration();
  }

private:
  static Eigen::Matrix4f make_transform_matrix(const geometry_msgs::msg::Transform & transform_msg)
  {
    Eigen::Quaternionf quaternion(
      transform_msg.rotation.w,
      transform_msg.rotation.x,
      transform_msg.rotation.y,
      transform_msg.rotation.z);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    transform(0, 3) = transform_msg.translation.x;
    transform(1, 3) = transform_msg.translation.y;
    transform(2, 3) = transform_msg.translation.z;
    return transform;
  }

  static Eigen::Matrix4f make_pose_matrix(const geometry_msgs::msg::Pose & pose_msg)
  {
    Eigen::Quaternionf quaternion(
      pose_msg.orientation.w,
      pose_msg.orientation.x,
      pose_msg.orientation.y,
      pose_msg.orientation.z);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    transform(0, 3) = pose_msg.position.x;
    transform(1, 3) = pose_msg.position.y;
    transform(2, 3) = pose_msg.position.z;
    return transform;
  }

  Mode parse_mode(const std::string & mode) const
  {
    if (mode == "raw") {
      return Mode::Raw;
    }

    if (mode == "registered") {
      return Mode::Registered;
    }

    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid mode '%s'. Expected 'raw' or 'registered'.",
      mode.c_str());
    throw std::runtime_error("Invalid unified_cloud_colorizer mode");
  }

  void validate_configuration() const
  {
    if (camera_info_topic_.empty() || input_image_topic_.empty() || output_cloud_topic_.empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "camera_info_topic, input_image_topic, and output_cloud_topic must be non-empty.");
      throw std::runtime_error("Unified colorizer received empty mandatory topic parameter");
    }

    if (mode_ == Mode::Raw) {
      if (input_cloud_topic_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "input_cloud_topic must be set in raw mode.");
        throw std::runtime_error("Missing input_cloud_topic in raw mode");
      }
      return;
    }

    if (input_registered_cloud_topic_.empty() || input_odometry_topic_.empty() ||
      output_map_topic_.empty())
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "input_registered_cloud_topic, input_odometry_topic, and output_map_topic must be set in registered mode.");
      throw std::runtime_error("Missing registered-mode topic configuration");
    }

    if (map_voxel_size_ <= 0.0f) {
      RCLCPP_ERROR(this->get_logger(), "map_voxel_size must be > 0, got %.3f", map_voxel_size_);
      throw std::runtime_error("Invalid map_voxel_size");
    }
  }

  void initialize_mode()
  {
    const auto sensor_qos = rmw_qos_profile_sensor_data;

    if (mode_ == Mode::Raw) {
      raw_lidar_.subscribe(this, input_cloud_topic_, sensor_qos);
      raw_image_.subscribe(this, input_image_topic_, sensor_qos);
      raw_sync_ = std::make_shared<RawSync>(RawSyncPolicy(10), raw_lidar_, raw_image_);
      raw_sync_->registerCallback(
        std::bind(&UnifiedCloudColorizerNode::raw_callback, this, _1, _2));

      output_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_cloud_topic_, 10);
      return;
    }

    registered_odometry_.subscribe(this, input_odometry_topic_, sensor_qos);
    registered_image_.subscribe(this, input_image_topic_, sensor_qos);
    registered_cloud_.subscribe(this, input_registered_cloud_topic_, sensor_qos);
    registered_sync_ = std::make_shared<RegisteredSync>(
      RegisteredSyncPolicy(10),
      registered_odometry_,
      registered_image_,
      registered_cloud_);
    registered_sync_->registerCallback(
      std::bind(&UnifiedCloudColorizerNode::registered_callback, this, _1, _2, _3));

    output_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_cloud_topic_, 10);
    map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_map_topic_, 10);
  }

  void log_configuration() const
  {
    if (mode_ == Mode::Raw) {
      RCLCPP_INFO(
        this->get_logger(),
        "Unified colorizer running in raw mode: cloud=%s image=%s camera_info=%s output=%s publish_only_colored_points=%s",
        input_cloud_topic_.c_str(),
        input_image_topic_.c_str(),
        camera_info_topic_.c_str(),
        output_cloud_topic_.c_str(),
        publish_only_colored_points_ ? "true" : "false");
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Unified colorizer running in registered mode: odom=%s cloud=%s image=%s camera_info=%s output=%s map=%s map_voxel_size=%.3f publish_only_colored_points=%s",
      input_odometry_topic_.c_str(),
      input_registered_cloud_topic_.c_str(),
      input_image_topic_.c_str(),
      camera_info_topic_.c_str(),
      output_cloud_topic_.c_str(),
      output_map_topic_.c_str(),
      map_voxel_size_,
      publish_only_colored_points_ ? "true" : "false");
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
    RCLCPP_INFO(this->get_logger(), "Camera calibration parameters successfully received.");
  }

  cv::Mat decode_image(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_msg) const
  {
    return cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  }

  void raw_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr img_msg)
  {
    if (!camera_info_received_) {
      return;
    }

    auto cloud_in = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*cloud_msg, *cloud_in);

    cv::Mat cv_image;
    try {
      cv_image = decode_image(img_msg);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform("pylon_camera", "os1/os_lidar", cloud_msg->header.stamp);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not transform os1/os_lidar to pylon_camera: %s",
        ex.what());
      return;
    }

    const Eigen::Matrix4f transform = make_transform_matrix(tf_msg.transform);

    auto cloud_out = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    cloud_out->points.reserve(cloud_in->points.size());

    std::vector<cv::Point3f> camera_points;
    std::vector<pcl::PointXYZ> candidate_points;
    std::vector<std::size_t> projected_indices;
    camera_points.reserve(cloud_in->points.size());
    candidate_points.reserve(cloud_in->points.size());
    projected_indices.reserve(cloud_in->points.size());

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

      const Eigen::Vector4f pt_lidar(point.x, point.y, point.z, 1.0f);
      const Eigen::Vector4f pt_camera = transform * pt_lidar;

      if (pt_camera.z() <= 0.0f) {
        continue;
      }

      camera_points.emplace_back(pt_camera.x(), pt_camera.y(), pt_camera.z());
      if (publish_only_colored_points_) {
        candidate_points.push_back(point);
      } else {
        projected_indices.push_back(output_index);
      }
    }

    if (!camera_points.empty()) {
      std::vector<cv::Point2f> image_points;
      cv::projectPoints(
        camera_points,
        zero_rotation_,
        zero_translation_,
        camera_matrix_,
        dist_coeffs_,
        image_points);

      for (std::size_t i = 0; i < image_points.size(); ++i) {
        const int u = cvRound(image_points[i].x);
        const int v = cvRound(image_points[i].y);

        if (u < 0 || u >= cv_image.cols || v < 0 || v >= cv_image.rows) {
          continue;
        }

        const cv::Vec3b & color = cv_image.at<cv::Vec3b>(v, u);
        if (publish_only_colored_points_) {
          pcl::PointXYZRGB color_point;
          color_point.x = candidate_points[i].x;
          color_point.y = candidate_points[i].y;
          color_point.z = candidate_points[i].z;
          color_point.b = color[0];
          color_point.g = color[1];
          color_point.r = color[2];
          cloud_out->points.push_back(color_point);
        } else {
          auto & color_point = cloud_out->points[projected_indices[i]];
          color_point.b = color[0];
          color_point.g = color[1];
          color_point.r = color[2];
        }
      }
    }

    publish_cloud(cloud_out, cloud_msg->header);
  }

  void registered_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr img_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
  {
    if (!camera_info_received_) {
      return;
    }

    const rclcpp::Time odom_time(odom_msg->header.stamp);
    const rclcpp::Time cloud_time(cloud_msg->header.stamp);
    RCLCPP_INFO(
      this->get_logger(),
      "Registered mode dt = %ld ns",
      (odom_time - cloud_time).nanoseconds());

    auto cloud_in = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*cloud_msg, *cloud_in);

    cv::Mat cv_image;
    try {
      cv_image = decode_image(img_msg);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat undistorted_image;
    cv::undistort(cv_image, undistorted_image, camera_matrix_, dist_coeffs_);

    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform(
        "os1/os_lidar",
        "pylon_camera",
        cloud_msg->header.stamp);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not transform pylon_camera to os1/os_lidar: %s",
        ex.what());
      return;
    }

    const Eigen::Matrix4f t_odom_lidar = make_pose_matrix(odom_msg->pose.pose);
    const Eigen::Matrix4f t_lidar_cam = make_transform_matrix(tf_msg.transform);
    const Eigen::Matrix4f t_odom_cam = t_odom_lidar * t_lidar_cam;
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

      if (pixel_u < 0 || pixel_u >= undistorted_image.cols ||
        pixel_v < 0 || pixel_v >= undistorted_image.rows)
      {
        continue;
      }

      const cv::Vec3b & color = undistorted_image.at<cv::Vec3b>(pixel_v, pixel_u);
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

    publish_cloud(cloud_out, cloud_msg->header);

    *accumulated_map_ += *cloud_out;

    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(accumulated_map_);
    voxel_filter.setLeafSize(map_voxel_size_, map_voxel_size_, map_voxel_size_);

    auto downsampled_map = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    voxel_filter.filter(*downsampled_map);
    downsampled_map->width = downsampled_map->points.size();
    downsampled_map->height = 1;
    downsampled_map->is_dense = true;
    accumulated_map_ = downsampled_map;

    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*accumulated_map_, map_msg);
    map_msg.header = cloud_msg->header;
    map_publisher_->publish(map_msg);
  }

  void publish_cloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out,
    const std_msgs::msg::Header & header) const
  {
    cloud_out->width = cloud_out->points.size();
    cloud_out->height = 1;
    cloud_out->is_dense = true;

    sensor_msgs::msg::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud_out, msg_out);
    msg_out.header = header;
    output_publisher_->publish(msg_out);
  }

  Mode mode_;
  std::string mode_string_;

  std::string input_cloud_topic_;
  std::string input_registered_cloud_topic_;
  std::string input_odometry_topic_;
  std::string input_image_topic_;
  std::string camera_info_topic_;
  std::string output_cloud_topic_;
  std::string output_map_topic_;
  bool publish_only_colored_points_ = true;
  float map_voxel_size_ = 0.3f;

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> raw_lidar_;
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> raw_image_;
  std::shared_ptr<RawSync> raw_sync_;

  message_filters::Subscriber<nav_msgs::msg::Odometry> registered_odometry_;
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> registered_image_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> registered_cloud_;
  std::shared_ptr<RegisteredSync> registered_sync_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  const cv::Vec3d zero_rotation_{0.0, 0.0, 0.0};
  const cv::Vec3d zero_translation_{0.0, 0.0, 0.0};
  bool camera_info_received_ = false;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_map_ =
    std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UnifiedCloudColorizerNode>());
  rclcpp::shutdown();
  return 0;
}
