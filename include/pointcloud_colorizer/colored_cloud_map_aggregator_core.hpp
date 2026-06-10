#pragma once

#include <memory>
#include <string>

#include "pointcloud_colorizer/voxel_map_builder.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pointcloud_colorizer
{

struct ColoredCloudMapAggregatorCoreConfig
{
  VoxelMapBuilderConfig map_builder;
};

class ColoredCloudMapAggregatorCore
{
public:
  explicit ColoredCloudMapAggregatorCore(const ColoredCloudMapAggregatorCoreConfig & config);

  bool update_from_synced_messages(
    const nav_msgs::msg::Odometry & odometry,
    const pcl::PointCloud<pcl::PointXYZRGB> & colored_cloud,
    const std::string & map_frame_id,
    std::string * error_message = nullptr);

  bool has_pending_publish() const;

  bool take_map_cloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
    std_msgs::msg::Header & header);

private:
  static bool build_odom_transform(
    const nav_msgs::msg::Odometry & odometry,
    Eigen::Matrix4f & odom_to_lidar,
    std::string * error_message);

  static void transform_colored_cloud_in_place(
    pcl::PointCloud<pcl::PointXYZRGB> & cloud,
    const Eigen::Matrix4f & transform);

  VoxelMapBuilder voxel_map_builder_;
};

}  // namespace pointcloud_colorizer
