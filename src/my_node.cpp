#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/os1/points", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

      subscription1_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/camera_front/image_raw/compressed", 10, std::bind(&MinimalSubscriber::image_callback, this, _1));

      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/colorizer/debug/purple_cloud", 10);
    }
    
  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*msg, *cloud_in);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);

      for (size_t i = 0; i < cloud_in->points.size(); ++i) {
          pcl::PointXYZRGB color_point;
          color_point.x = cloud_in->points[i].x;
          color_point.y = cloud_in->points[i].y;
          color_point.z = cloud_in->points[i].z;
          
          color_point.r = 128;
          color_point.g = 0;
          color_point.b = 128;

          cloud_out->points.push_back(color_point);
      }

      sensor_msgs::msg::PointCloud2 msg_out;
      pcl::toROSMsg(*cloud_out, msg_out);
      msg_out.header = msg->header;

      publisher_->publish(msg_out);
    }

    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) const
    {
      cv::Mat cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
      int w = cv_image.cols;
      int h = cv_image.rows; 
      int size = h * w;
      RCLCPP_INFO(this->get_logger(), "Image received with shape: %d x %d", w, h);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription1_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
