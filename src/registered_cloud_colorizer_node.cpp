#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
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

    RCLCPP_INFO(
      this->get_logger(),
      "Registered colorizer is waiting for synchronized odometry, image, and registered cloud.");
  }

private:
  void topic_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr img_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg) const
  {
    (void)img_msg;

    const rclcpp::Time odom_time(odom_msg->header.stamp);
    const rclcpp::Time cloud_time(cloud_msg->header.stamp);
    //const double dt = (odom_time - cloud_time).seconds();
    const auto dt_ns = (odom_time - cloud_time).nanoseconds();
    RCLCPP_INFO(this->get_logger(), "dt = %ld ns", dt_ns);

  }

  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_;
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> image_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> registered_cloud_;
  std::shared_ptr<RegisteredSync> sync_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RegisteredCloudColorizerNode>());
  rclcpp::shutdown();
  return 0;
}
