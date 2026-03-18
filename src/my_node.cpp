#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

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

    }
  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, const sensor_msgs::msg::CompressedImage::ConstSharedPtr img_msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Synchronized pair received at time %d.%d and %d.%d", 
                  msg->header.stamp.sec, msg->header.stamp.nanosec, img_msg->header.stamp.sec, img_msg->header.stamp.nanosec);
    }

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar;
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> camera;
    std::shared_ptr<Sync> sync_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
