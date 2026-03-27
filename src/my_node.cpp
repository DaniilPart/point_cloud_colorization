#include <memory>
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


using std::placeholders::_1;
using std::placeholders::_2;


class MinimalSubscriber : public rclcpp::Node
{
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2,
      sensor_msgs::msg::CompressedImage
    > MySyncPolicy;


    typedef message_filters::Synchronizer<MySyncPolicy> Sync;


  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      lidar.subscribe(this, "/os1/points" );
      camera.subscribe(this, "/camera_front/image_raw/compressed" );
      sync_.reset(new Sync(MySyncPolicy(10), lidar, camera));
      sync_->registerCallback(std::bind(&MinimalSubscriber::topic_callback, this, _1, _2));


      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/colorizer/debug/colored_cloud", 10);


      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


      camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
          "/camera_front/camera_info", 10,
          std::bind(&MinimalSubscriber::camera_info_callback, this, std::placeholders::_1));
    }
  private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (camera_info_received_) {
            return;
        }


        K_ << msg->k[0], msg->k[1], msg->k[2],
              msg->k[3], msg->k[4], msg->k[5],
              msg->k[6], msg->k[7], msg->k[8];


        D_ = msg->d;


        camera_info_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera calibration parameters successfully received and stored.");
    }


    void topic_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, const sensor_msgs::msg::CompressedImage::ConstSharedPtr img_msg) const
    {
      if (!camera_info_received_) {
          return;
      }


      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*msg, *cloud_in);


      cv::Mat cv_image;
      try {
          cv_image = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
      } catch (cv_bridge::Exception& e) {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
          return;
      }


      cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
          K_(0,0), K_(0,1), K_(0,2),
          K_(1,0), K_(1,1), K_(1,2),
          K_(2,0), K_(2,1), K_(2,2));


      cv::Mat dist_coeffs(D_);


      cv::Mat undistorted_image;
      cv::undistort(cv_image, undistorted_image, camera_matrix, dist_coeffs);


      geometry_msgs::msg::TransformStamped t;
      try {
          t = tf_buffer_->lookupTransform(
              "pylon_camera",
              "os1/os_lidar",
              msg->header.stamp
          );
      } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN(this->get_logger(), "Could not transform os1/os_lidar to pylon_camera: %s", ex.what());
          return;
      }


      RCLCPP_INFO(this->get_logger(), "Translation vector: [%.3f, %.3f, %.3f]",
                  t.transform.translation.x,
                  t.transform.translation.y,
                  t.transform.translation.z);


      Eigen::Quaternionf q(
          t.transform.rotation.w,
          t.transform.rotation.x,
          t.transform.rotation.y,
          t.transform.rotation.z
      );
      
      Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
      transform.block<3, 3>(0, 0) = q.toRotationMatrix();
      transform(0, 3) = t.transform.translation.x;
      transform(1, 3) = t.transform.translation.y;
      transform(2, 3) = t.transform.translation.z;


      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
      cloud_out->points.reserve(cloud_in->points.size());


      for (const auto& point : cloud_in->points) {
          pcl::PointXYZRGB color_point;
          color_point.x = point.x;
          color_point.y = point.y;
          color_point.z = point.z;
          color_point.r = 128;
          color_point.g = 128;
          color_point.b = 128;


          Eigen::Vector4f pt_lidar(point.x, point.y, point.z, 1.0f);
          Eigen::Vector4f pt_camera = transform * pt_lidar;


          if (pt_camera.z() <= 0) {
              cloud_out->points.push_back(color_point);
              continue;
          }


          float x_norm = pt_camera.x() / pt_camera.z();
          float y_norm = pt_camera.y() / pt_camera.z();
          Eigen::Vector3f pt_norm(x_norm, y_norm, 1.0f);


          Eigen::Vector3f pixel_coords = K_ * pt_norm;
          int u = std::round(pixel_coords.x());
          int v = std::round(pixel_coords.y());


          if (u >= 0 && u < undistorted_image.cols && v >= 0 && v < undistorted_image.rows) {
              cv::Vec3b color = undistorted_image.at<cv::Vec3b>(v, u);
              color_point.b = color[0];
              color_point.g = color[1];
              color_point.r = color[2];
          }


          cloud_out->points.push_back(color_point);
      }


      sensor_msgs::msg::PointCloud2 msg_out;
      pcl::toROSMsg(*cloud_out, msg_out);
      msg_out.header = msg->header;
      publisher_->publish(msg_out);
    }


    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar;
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> camera;
    std::shared_ptr<Sync> sync_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;


    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    Eigen::Matrix3f K_;
    std::vector<double> D_;
    bool camera_info_received_ = false;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}