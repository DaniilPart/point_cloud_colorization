#pragma once

#include <memory>
#include <string>

#include "pointcloud_colorizer/voxel_color_estimator.hpp"

#include "std_msgs/msg/header.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pointcloud_colorizer
{

struct VoxelMapBuilderConfig
{
  VoxelColorEstimatorConfig estimator;
};

class VoxelMapBuilder
{
public:
  explicit VoxelMapBuilder(const VoxelMapBuilderConfig & config);

  void update_from_colored_cloud(
    const pcl::PointCloud<pcl::PointXYZRGB> & cloud,
    const std_msgs::msg::Header & source_header);

  bool has_pending_publish() const;

  bool take_map_cloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
    std_msgs::msg::Header & header);

  const VoxelColorEstimatorStats & estimator_stats() const;
  std::size_t voxel_count() const;
  std::size_t bucket_count() const;
  float load_factor() const;
  float max_load_factor() const;

private:
  VoxelColorEstimator estimator_;
  std_msgs::msg::Header last_source_header_;
  bool has_header_ = false;
  bool pending_publish_ = false;
};

}  // namespace pointcloud_colorizer
