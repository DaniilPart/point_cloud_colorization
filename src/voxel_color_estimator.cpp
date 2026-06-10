#include "pointcloud_colorizer/voxel_color_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace pointcloud_colorizer
{

namespace
{

int clamp_color(int value)
{
  return std::clamp(value, 0, 255);
}

std::uint64_t splitmix64(std::uint64_t x)
{
  x += 0x9e3779b97f4a7c15ULL;
  x = (x ^ (x >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  x = (x ^ (x >> 27U)) * 0x94d049bb133111ebULL;
  return x ^ (x >> 31U);
}

}  // namespace

VoxelColorEstimator::VoxelColorEstimator(const VoxelColorEstimatorConfig & config)
: config_(config)
{
  if (config_.voxel_size <= 0.0f) {
    config_.voxel_size = 0.3f;
  }
  config_.burn_in_samples = std::max(1, config_.burn_in_samples);
  config_.step_max = std::max(1, config_.step_max);
  config_.placeholder_gray_value = clamp_color(config_.placeholder_gray_value);
  config_.hash_max_load_factor = std::clamp(config_.hash_max_load_factor, 0.5f, 0.95f);

  voxels_.max_load_factor(config_.hash_max_load_factor);
  if (config_.hash_initial_capacity > 0) {
    voxels_.reserve(config_.hash_initial_capacity);
  }
}

void VoxelColorEstimator::reset()
{
  voxels_.clear();
  stats_ = VoxelColorEstimatorStats{};
}

bool VoxelColorEstimator::VoxelKey::operator==(const VoxelKey & other) const
{
  return x == other.x && y == other.y && z == other.z;
}

std::size_t VoxelColorEstimator::VoxelKeyHash::operator()(const VoxelKey & key) const
{
  const std::uint64_t x = splitmix64(static_cast<std::uint32_t>(key.x));
  const std::uint64_t y = splitmix64(static_cast<std::uint32_t>(key.y));
  const std::uint64_t z = splitmix64(static_cast<std::uint32_t>(key.z));
  return static_cast<std::size_t>(x ^ (y << 1U) ^ (z << 2U));
}

VoxelColorEstimator::VoxelKey VoxelColorEstimator::make_key(float x, float y, float z) const
{
  const float inv = 1.0f / config_.voxel_size;
  return VoxelKey{
    static_cast<int>(std::floor(x * inv)),
    static_cast<int>(std::floor(y * inv)),
    static_cast<int>(std::floor(z * inv))
  };
}

bool VoxelColorEstimator::is_placeholder_gray(const pcl::PointXYZRGB & point) const
{
  if (!config_.ignore_placeholder_gray) {
    return false;
  }

  const int gray = config_.placeholder_gray_value;
  return point.r == gray && point.g == gray && point.b == gray;
}

void VoxelColorEstimator::update_channel(int observation, int channel, VoxelState & state)
{
  int & estimate = state.estimate[channel];
  int & step = state.step[channel];
  int & direction = state.direction[channel];

  if (observation > estimate) {
    if (direction > 0) {
      step = std::min(step + 1, config_.step_max);
    } else {
      step = 1;
    }
    direction = 1;
    estimate = clamp_color(estimate + step);
    return;
  }

  if (observation < estimate) {
    if (direction < 0) {
      step = std::min(step + 1, config_.step_max);
    } else {
      step = 1;
    }
    direction = -1;
    estimate = clamp_color(estimate - step);
    return;
  }

  direction = 0;
  if (step > 1) {
    --step;
  }
}

void VoxelColorEstimator::update(const pcl::PointXYZRGB & point)
{
  if (is_placeholder_gray(point)) {
    ++stats_.voxel_skipped_placeholder;
    return;
  }

  ++stats_.voxel_updates_total;

  const VoxelKey key = make_key(point.x, point.y, point.z);
  auto [it, inserted] = voxels_.try_emplace(key, VoxelState{});
  VoxelState & state = it->second;

  if (inserted) {
    ++stats_.voxel_new_cells;
  }

  state.sample_count += 1;

  const double n = static_cast<double>(state.sample_count);
  state.mean_x += static_cast<float>((point.x - state.mean_x) / n);
  state.mean_y += static_cast<float>((point.y - state.mean_y) / n);
  state.mean_z += static_cast<float>((point.z - state.mean_z) / n);

  if (state.sample_count <= static_cast<std::uint64_t>(config_.burn_in_samples)) {
    state.mean_r += (static_cast<double>(point.r) - state.mean_r) / n;
    state.mean_g += (static_cast<double>(point.g) - state.mean_g) / n;
    state.mean_b += (static_cast<double>(point.b) - state.mean_b) / n;

    state.estimate[0] = clamp_color(static_cast<int>(std::lround(state.mean_r)));
    state.estimate[1] = clamp_color(static_cast<int>(std::lround(state.mean_g)));
    state.estimate[2] = clamp_color(static_cast<int>(std::lround(state.mean_b)));
    state.step = {1, 1, 1};
    state.direction = {0, 0, 0};
    ++stats_.voxel_burnin_updates;
    return;
  }

  update_channel(static_cast<int>(point.r), 0, state);
  update_channel(static_cast<int>(point.g), 1, state);
  update_channel(static_cast<int>(point.b), 2, state);
  ++stats_.voxel_filtered_updates;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr VoxelColorEstimator::build_cloud() const
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  cloud->points.reserve(voxels_.size());

  for (const auto & entry : voxels_) {
    const VoxelState & state = entry.second;
    pcl::PointXYZRGB point;
    point.x = state.mean_x;
    point.y = state.mean_y;
    point.z = state.mean_z;
    point.r = static_cast<std::uint8_t>(state.estimate[0]);
    point.g = static_cast<std::uint8_t>(state.estimate[1]);
    point.b = static_cast<std::uint8_t>(state.estimate[2]);
    cloud->points.push_back(point);
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

const VoxelColorEstimatorStats & VoxelColorEstimator::stats() const
{
  return stats_;
}

std::size_t VoxelColorEstimator::voxel_count() const
{
  return voxels_.size();
}

std::size_t VoxelColorEstimator::bucket_count() const
{
  return voxels_.bucket_count();
}

float VoxelColorEstimator::load_factor() const
{
  return voxels_.load_factor();
}

float VoxelColorEstimator::max_load_factor() const
{
  return voxels_.max_load_factor();
}

}  // namespace pointcloud_colorizer
