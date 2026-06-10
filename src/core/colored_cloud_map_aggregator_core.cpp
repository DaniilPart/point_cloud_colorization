#include "pointcloud_colorizer/colored_cloud_map_aggregator_core.hpp"

namespace pointcloud_colorizer
{

ColoredCloudMapAggregatorCore::ColoredCloudMapAggregatorCore(
  const ColoredCloudMapAggregatorCoreConfig & config)
: voxel_map_builder_(config.map_builder)
{
}

bool ColoredCloudMapAggregatorCore::build_odom_transform(
  const nav_msgs::msg::Odometry & odometry,
  Eigen::Matrix4f & odom_to_lidar,
  std::string * error_message)
{
  Eigen::Quaternionf q_odom_lidar(
    static_cast<float>(odometry.pose.pose.orientation.w),
    static_cast<float>(odometry.pose.pose.orientation.x),
    static_cast<float>(odometry.pose.pose.orientation.y),
    static_cast<float>(odometry.pose.pose.orientation.z));

  if (q_odom_lidar.norm() == 0.0f) {
    if (error_message != nullptr) {
      *error_message = "invalid odometry quaternion";
    }
    return false;
  }
  q_odom_lidar.normalize();

  odom_to_lidar = Eigen::Matrix4f::Identity();
  odom_to_lidar.block<3, 3>(0, 0) = q_odom_lidar.toRotationMatrix();
  odom_to_lidar(0, 3) = static_cast<float>(odometry.pose.pose.position.x);
  odom_to_lidar(1, 3) = static_cast<float>(odometry.pose.pose.position.y);
  odom_to_lidar(2, 3) = static_cast<float>(odometry.pose.pose.position.z);
  return true;
}

void ColoredCloudMapAggregatorCore::transform_colored_cloud_in_place(
  pcl::PointCloud<pcl::PointXYZRGB> & cloud,
  const Eigen::Matrix4f & transform)
{
  for (auto & point : cloud.points) {
    const Eigen::Vector4f pt(point.x, point.y, point.z, 1.0f);
    const Eigen::Vector4f transformed = transform * pt;
    point.x = transformed.x();
    point.y = transformed.y();
    point.z = transformed.z();
  }
}

bool ColoredCloudMapAggregatorCore::update_from_synced_messages(
  const nav_msgs::msg::Odometry & odometry,
  const pcl::PointCloud<pcl::PointXYZRGB> & colored_cloud,
  const std::string & map_frame_id,
  std::string * error_message)
{
  Eigen::Matrix4f t_odom_lidar = Eigen::Matrix4f::Identity();
  if (!build_odom_transform(odometry, t_odom_lidar, error_message)) {
    return false;
  }

  pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud = colored_cloud;
  transform_colored_cloud_in_place(transformed_cloud, t_odom_lidar);

  std_msgs::msg::Header map_header = odometry.header;
  if (!map_frame_id.empty()) {
    map_header.frame_id = map_frame_id;
  }

  voxel_map_builder_.update_from_colored_cloud(transformed_cloud, map_header);
  return true;
}

bool ColoredCloudMapAggregatorCore::has_pending_publish() const
{
  return voxel_map_builder_.has_pending_publish();
}

bool ColoredCloudMapAggregatorCore::take_map_cloud(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  std_msgs::msg::Header & header)
{
  return voxel_map_builder_.take_map_cloud(cloud, header);
}

}  // namespace pointcloud_colorizer
