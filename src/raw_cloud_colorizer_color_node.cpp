#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "pointcloud_colorizer/transform_utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

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
#include <tf2_ros/transform_listener.h>

using std::placeholders::_1;
using std::placeholders::_2;
using pointcloud_colorizer::TransformSource;

class RawCloudColorizerColorNode : public rclcpp::Node
{
  using RawSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::CompressedImage>;
  using RawSync = message_filters::Synchronizer<RawSyncPolicy>;

public:
  explicit RawCloudColorizerColorNode(const rclcpp::NodeOptions & options)
  : Node("raw_cloud_colorizer", options)
  {
    input_cloud_topic_ = this->declare_parameter<std::string>("input_cloud_topic", "");
    input_image_topic_ = this->declare_parameter<std::string>("input_image_topic", "");
    camera_info_topic_ = this->declare_parameter<std::string>("camera_info_topic", "");
    output_cloud_topic_ = this->declare_parameter<std::string>("output_cloud_topic", "");
    output_frame_id_ = this->declare_parameter<std::string>("output_frame_id", "");
    publish_only_colored_points_ = this->declare_parameter<bool>("publish_only_colored_points", true);
    transform_source_string_ = this->declare_parameter<std::string>("transform_source", "config");
    camera_frame_id_ = this->declare_parameter<std::string>("camera_frame_id", "");
    lidar_frame_id_ = this->declare_parameter<std::string>("lidar_frame_id", "");
    camera_to_lidar_matrix_values_ = this->declare_parameter<std::vector<double>>(
      "camera_to_lidar_matrix", std::vector<double>{});
    transform_lookup_timeout_ = rclcpp::Duration::from_seconds(
      this->declare_parameter<double>("transform_lookup_timeout_sec", 0.1));
    sync_queue_size_ = std::max<int>(1, this->declare_parameter<int>("sync_queue_size", 10));

    transform_source_ = pointcloud_colorizer::parse_transform_source(transform_source_string_);
    validate_configuration();
    initialize_transform();

    const auto sensor_qos = rmw_qos_profile_sensor_data;
    cloud_sub_.subscribe(this, input_cloud_topic_, sensor_qos);
    image_sub_.subscribe(this, input_image_topic_, sensor_qos);

    sync_ = std::make_shared<RawSync>(RawSyncPolicy(sync_queue_size_), cloud_sub_, image_sub_);
    sync_->registerCallback(std::bind(&RawCloudColorizerColorNode::topic_callback, this, _1, _2));

    output_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&RawCloudColorizerColorNode::camera_info_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Raw colorizer(color-only): cloud=%s image=%s camera_info=%s output=%s output_frame=%s publish_only_colored_points=%s",
      input_cloud_topic_.c_str(),
      input_image_topic_.c_str(),
      camera_info_topic_.c_str(),
      output_cloud_topic_.c_str(),
      output_frame_id_.empty() ? "<input>" : output_frame_id_.c_str(),
      publish_only_colored_points_ ? "true" : "false");
  }

private:
  void validate_configuration() const
  {
    pointcloud_colorizer::require_non_empty(input_cloud_topic_, "input_cloud_topic");
    pointcloud_colorizer::require_non_empty(input_image_topic_, "input_image_topic");
    pointcloud_colorizer::require_non_empty(camera_info_topic_, "camera_info_topic");
    pointcloud_colorizer::require_non_empty(output_cloud_topic_, "output_cloud_topic");

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

  void topic_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr img_msg)
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

    cv::Mat cv_image;
    try {
      cv_image = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    Eigen::Matrix4f t_lidar_camera = Eigen::Matrix4f::Identity();
    if (!get_lidar_to_camera_transform(rclcpp::Time(cloud_msg->header.stamp), t_lidar_camera)) {
      return;
    }

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
      const Eigen::Vector4f pt_camera = t_lidar_camera * pt_lidar;
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

    cloud_out->width = cloud_out->points.size();
    cloud_out->height = 1;
    cloud_out->is_dense = true;

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_out, output_msg);
    output_msg.header = cloud_msg->header;
    if (!output_frame_id_.empty()) {
      output_msg.header.frame_id = output_frame_id_;
    }

    output_publisher_->publish(output_msg);
  }

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> image_sub_;
  std::shared_ptr<RawSync> sync_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_publisher_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  const cv::Vec3d zero_rotation_{0.0, 0.0, 0.0};
  const cv::Vec3d zero_translation_{0.0, 0.0, 0.0};

  std::string input_cloud_topic_;
  std::string input_image_topic_;
  std::string camera_info_topic_;
  std::string output_cloud_topic_;
  std::string output_frame_id_;
  std::string transform_source_string_;
  std::string camera_frame_id_;
  std::string lidar_frame_id_;

  std::vector<double> camera_to_lidar_matrix_values_;
  TransformSource transform_source_ = TransformSource::Config;
  rclcpp::Duration transform_lookup_timeout_ = rclcpp::Duration::from_seconds(0.1);

  int sync_queue_size_ = 10;
  bool publish_only_colored_points_ = true;
  bool camera_info_received_ = false;

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
