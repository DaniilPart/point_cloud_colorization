#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pointcloud_colorizer
{

struct VoxelColorEstimatorConfig
{
  float voxel_size = 0.3f;
  int burn_in_samples = 5;
  int step_max = 16;
  bool ignore_placeholder_gray = true;
  int placeholder_gray_value = 128;
  std::size_t hash_initial_capacity = 262144;
  float hash_max_load_factor = 0.7f;
};

struct VoxelColorEstimatorStats
{
  std::uint64_t voxel_updates_total = 0;
  std::uint64_t voxel_new_cells = 0;
  std::uint64_t voxel_burnin_updates = 0;
  std::uint64_t voxel_filtered_updates = 0;
  std::uint64_t voxel_skipped_placeholder = 0;
};

class VoxelColorEstimator
{
public:
  explicit VoxelColorEstimator(const VoxelColorEstimatorConfig & config);

  void reset();
  void update(const pcl::PointXYZRGB & point);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr build_cloud() const;

  const VoxelColorEstimatorStats & stats() const;
  std::size_t voxel_count() const;
  std::size_t bucket_count() const;
  float load_factor() const;
  float max_load_factor() const;

private:
  struct VoxelKey
  {
    int x;
    int y;
    int z;

    bool operator==(const VoxelKey & other) const;
  };

  struct VoxelKeyHash
  {
    std::size_t operator()(const VoxelKey & key) const;
  };

  struct VoxelState
  {
    std::uint64_t sample_count = 0;

    float mean_x = 0.0f;
    float mean_y = 0.0f;
    float mean_z = 0.0f;

    double mean_r = 0.0;
    double mean_g = 0.0;
    double mean_b = 0.0;

    std::array<int, 3> estimate{{0, 0, 0}};
    std::array<int, 3> step{{1, 1, 1}};
    std::array<int, 3> direction{{0, 0, 0}};
  };

  VoxelKey make_key(float x, float y, float z) const;
  bool is_placeholder_gray(const pcl::PointXYZRGB & point) const;
  void update_channel(int observation, int channel, VoxelState & state);

  VoxelColorEstimatorConfig config_;
  VoxelColorEstimatorStats stats_;
  std::unordered_map<VoxelKey, VoxelState, VoxelKeyHash> voxels_;
};

}  // namespace pointcloud_colorizer
