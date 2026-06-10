#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pointcloud_colorizer
{

struct SkyFilterConfig
{
  bool enabled = true;
  double region_max_y_fraction = 0.5;
  bool region_invert_y = false;
  int blue_h_min = 90;
  int blue_h_max = 130;
  int blue_s_min = 25;
  int blue_v_min = 135;
  int cloud_s_max = 40;
  int cloud_v_min = 155;
};

struct RawCloudColorizerCoreConfig
{
  bool publish_only_colored_points = true;
  bool pre_cleaning_filter_enabled = true;
  SkyFilterConfig sky_filter;

  int debug_point_radius = 2;
  int debug_outline_thickness = 1;
  cv::Scalar debug_kept_point_color_bgr{0.0, 255.0, 0.0};
  cv::Scalar debug_skipped_point_color_bgr{0.0, 0.0, 255.0};
  cv::Scalar debug_outline_color_bgr{0.0, 0.0, 0.0};
};

struct RawCloudColorizerProcessInput
{
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
  cv::Mat bgr_image;
  Eigen::Matrix4f lidar_to_camera_transform = Eigen::Matrix4f::Identity();
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
};

struct RawCloudColorizerProcessOutput
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
  cv::Mat debug_overlay;
  bool has_debug_overlay = false;
};

class RawCloudColorizerCore
{
public:
  explicit RawCloudColorizerCore(const RawCloudColorizerCoreConfig & config);

  RawCloudColorizerProcessOutput process(
    const RawCloudColorizerProcessInput & input,
    bool generate_debug_overlay) const;

private:
  bool is_sky_pixel(const cv::Vec3b & hsv_pixel, int v, int image_rows) const;

  RawCloudColorizerCoreConfig config_;
  const cv::Vec3d zero_rotation_{0.0, 0.0, 0.0};
  const cv::Vec3d zero_translation_{0.0, 0.0, 0.0};
};

}  // namespace pointcloud_colorizer
