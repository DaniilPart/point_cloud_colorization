#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include <pcl_conversions/pcl_conversions.h>
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

class RawCloudColorizerNode : public rclcpp::Node
{
  using MySyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::CompressedImage>;
  using Sync = message_filters::Synchronizer<MySyncPolicy>;

public:
  RawCloudColorizerNode()
  : Node("raw_cloud_colorizer")
  {
    const auto sensor_qos = rmw_qos_profile_sensor_data;

    lidar_.subscribe(this, "/os1/points", sensor_qos);
    camera_.subscribe(this, "/camera_front/image_raw/compressed", sensor_qos);
    sync_ = std::make_shared<Sync>(MySyncPolicy(10), lidar_, camera_);
    sync_->registerCallback(std::bind(&RawCloudColorizerNode::topic_callback, this, _1, _2));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/colorizer/raw/colored_cloud", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_front/camera_info",
      rclcpp::SensorDataQoS(),
      std::bind(&RawCloudColorizerNode::camera_info_callback, this, std::placeholders::_1));
  }

private:
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
    RCLCPP_INFO(
      this->get_logger(),
      "Camera calibration parameters successfully received and stored.");
  }

  void topic_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr img_msg) const
  {
    if (!camera_info_received_) {
      return;
    }

    auto cloud_in = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*msg, *cloud_in);

    cv::Mat cv_image;
    try {
      cv_image = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform("pylon_camera", "os1/os_lidar", msg->header.stamp);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not transform os1/os_lidar to pylon_camera: %s",
        ex.what());
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Translation vector: [%.3f, %.3f, %.3f]",
      t.transform.translation.x,
      t.transform.translation.y,
      t.transform.translation.z);

    Eigen::Quaternionf q(
      t.transform.rotation.w,
      t.transform.rotation.x,
      t.transform.rotation.y,
      t.transform.rotation.z);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = q.toRotationMatrix();
    transform(0, 3) = t.transform.translation.x;
    transform(1, 3) = t.transform.translation.y;
    transform(2, 3) = t.transform.translation.z;

    auto cloud_out = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    cloud_out->points.reserve(cloud_in->points.size());

    std::vector<cv::Point3f> camera_points;
    std::vector<pcl::PointXYZ> candidate_points;
    camera_points.reserve(cloud_in->points.size());
    candidate_points.reserve(cloud_in->points.size());

    for (const auto & point : cloud_in->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      Eigen::Vector4f pt_lidar(point.x, point.y, point.z, 1.0f);
      Eigen::Vector4f pt_camera = transform * pt_lidar;

      if (pt_camera.z() <= 0) {
        continue;
      }

      camera_points.emplace_back(pt_camera.x(), pt_camera.y(), pt_camera.z());
      candidate_points.push_back(point);
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

        if (u >= 0 && u < cv_image.cols && v >= 0 && v < cv_image.rows) {
          const cv::Vec3b & color = cv_image.at<cv::Vec3b>(v, u);
          pcl::PointXYZRGB color_point;
          color_point.x = candidate_points[i].x;
          color_point.y = candidate_points[i].y;
          color_point.z = candidate_points[i].z;
          color_point.b = color[0];
          color_point.g = color[1];
          color_point.r = color[2];
          cloud_out->points.push_back(color_point);
        }
      }
    }

    cloud_out->width = cloud_out->points.size();
    cloud_out->height = 1;
    cloud_out->is_dense = true;

    sensor_msgs::msg::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud_out, msg_out);
    msg_out.header = msg->header;
    publisher_->publish(msg_out);
  }

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_;
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> camera_;
  std::shared_ptr<Sync> sync_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  const cv::Vec3d zero_rotation_{0.0, 0.0, 0.0};
  const cv::Vec3d zero_translation_{0.0, 0.0, 0.0};
  bool camera_info_received_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RawCloudColorizerNode>());
  rclcpp::shutdown();
  return 0;
}
