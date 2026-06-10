#include "pointcloud_colorizer/voxel_map_builder.hpp"

namespace pointcloud_colorizer
{

VoxelMapBuilder::VoxelMapBuilder(const VoxelMapBuilderConfig & config)
: estimator_(config.estimator)
{
}

void VoxelMapBuilder::update_from_colored_cloud(
  const pcl::PointCloud<pcl::PointXYZRGB> & cloud,
  const std_msgs::msg::Header & source_header)
{
  for (const auto & point : cloud.points) {
    estimator_.update(point);
  }

  last_source_header_ = source_header;
  has_header_ = true;
  pending_publish_ = true;
}

bool VoxelMapBuilder::has_pending_publish() const
{
  return pending_publish_ && has_header_;
}

bool VoxelMapBuilder::take_map_cloud(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  std_msgs::msg::Header & header)
{
  if (!has_pending_publish()) {
    return false;
  }

  cloud = estimator_.build_cloud();
  header = last_source_header_;
  pending_publish_ = false;
  return true;
}

const VoxelColorEstimatorStats & VoxelMapBuilder::estimator_stats() const
{
  return estimator_.stats();
}

std::size_t VoxelMapBuilder::voxel_count() const
{
  return estimator_.voxel_count();
}

std::size_t VoxelMapBuilder::bucket_count() const
{
  return estimator_.bucket_count();
}

float VoxelMapBuilder::load_factor() const
{
  return estimator_.load_factor();
}

float VoxelMapBuilder::max_load_factor() const
{
  return estimator_.max_load_factor();
}

}  // namespace pointcloud_colorizer
