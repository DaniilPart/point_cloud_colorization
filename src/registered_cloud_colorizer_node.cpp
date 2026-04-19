#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
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
    const auto sensor_qos = rmw_qos_profile_sensor_data;

    odometry_.subscribe(this, "/liorf/mapping/odometry", sensor_qos);
    image_.subscribe(this, "/camera_front/image_raw/compressed", sensor_qos);
    registered_cloud_.subscribe(this, "/liorf/mapping/cloud_registered", sensor_qos);

    sync_ = std::make_shared<RegisteredSync>(
      RegisteredSyncPolicy(10),
      odometry_,
      image_,
      registered_cloud_);
    sync_->registerCallback(
      std::bind(&RegisteredCloudColorizerNode::topic_callback, this, _1, _2, _3));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/colorizer/registered/colored_cloud", 10);
    map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/colorizer/registered/naive_map", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_front/camera_info",
      rclcpp::SensorDataQoS(),
      std::bind(&RegisteredCloudColorizerNode::camera_info_callback, this, std::placeholders::_1));
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
      "Registered colorizer received camera calibration parameters.");
  }

  void topic_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr img_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
  {
    if (!camera_info_received_) {
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
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat undistorted_image;
    cv::undistort(cv_image, undistorted_image, camera_matrix_, dist_coeffs_);

    geometry_msgs::msg::TransformStamped t_lidar_cam_msg;
    try {
      t_lidar_cam_msg = tf_buffer_->lookupTransform(
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

    Eigen::Quaternionf q_lidar_cam(
      t_lidar_cam_msg.transform.rotation.w,
      t_lidar_cam_msg.transform.rotation.x,
      t_lidar_cam_msg.transform.rotation.y,
      t_lidar_cam_msg.transform.rotation.z);

    Eigen::Matrix4f t_lidar_cam = Eigen::Matrix4f::Identity();
    t_lidar_cam.block<3, 3>(0, 0) = q_lidar_cam.toRotationMatrix();
    t_lidar_cam(0, 3) = t_lidar_cam_msg.transform.translation.x;
    t_lidar_cam(1, 3) = t_lidar_cam_msg.transform.translation.y;
    t_lidar_cam(2, 3) = t_lidar_cam_msg.transform.translation.z;

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
      pcl::PointXYZRGB color_point;
      color_point.x = point.x;
      color_point.y = point.y;
      color_point.z = point.z;
      color_point.b = color[0];
      color_point.g = color[1];
      color_point.r = color[2];
      cloud_out->points.push_back(color_point);
    }

    cloud_out->width = cloud_out->points.size();
    cloud_out->height = 1;
    cloud_out->is_dense = true;

    sensor_msgs::msg::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud_out, msg_out);
    msg_out.header = cloud_msg->header;
    publisher_->publish(msg_out);

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

  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_;
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> image_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> registered_cloud_;
  std::shared_ptr<RegisteredSync> sync_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_map_ =
    std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  const float map_voxel_size_ = 0.1f;
  bool camera_info_received_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RegisteredCloudColorizerNode>());
  rclcpp::shutdown();
  return 0;
}
