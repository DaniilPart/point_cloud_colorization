#include "pointcloud_colorizer/raw_cloud_colorizer_core.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace pointcloud_colorizer
{

RawCloudColorizerCore::RawCloudColorizerCore(const RawCloudColorizerCoreConfig & config)
: config_(config)
{
}

bool RawCloudColorizerCore::is_sky_pixel(const cv::Vec3b & hsv_pixel, const int v, const int image_rows) const
{
  if (!config_.sky_filter.enabled) {
    return false;
  }

  const int upper_region_limit =
    static_cast<int>(std::ceil(config_.sky_filter.region_max_y_fraction * static_cast<double>(image_rows)));
  const bool in_sky_region = config_.sky_filter.region_invert_y
    ? (v >= (image_rows - upper_region_limit))
    : (v < upper_region_limit);
  if (!in_sky_region) {
    return false;
  }

  const int h = static_cast<int>(hsv_pixel[0]);
  const int s = static_cast<int>(hsv_pixel[1]);
  const int val = static_cast<int>(hsv_pixel[2]);

  const bool blue_sky =
    h >= config_.sky_filter.blue_h_min && h <= config_.sky_filter.blue_h_max &&
    s >= config_.sky_filter.blue_s_min && val >= config_.sky_filter.blue_v_min;
  const bool bright_cloud = s <= config_.sky_filter.cloud_s_max && val >= config_.sky_filter.cloud_v_min;
  return blue_sky || bright_cloud;
}

RawCloudColorizerProcessOutput RawCloudColorizerCore::process(
  const RawCloudColorizerProcessInput & input,
  const bool generate_debug_overlay) const
{
  if (!input.cloud) {
    throw std::invalid_argument("RawCloudColorizerCore::process: input.cloud must not be null");
  }
  if (input.bgr_image.empty()) {
    throw std::invalid_argument("RawCloudColorizerCore::process: input.bgr_image must not be empty");
  }
  if (input.camera_matrix.empty()) {
    throw std::invalid_argument("RawCloudColorizerCore::process: input.camera_matrix must not be empty");
  }

  auto cloud_out = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  cloud_out->points.reserve(input.cloud->points.size());

  cv::Mat hsv_image;
  if (config_.sky_filter.enabled) {
    cv::cvtColor(input.bgr_image, hsv_image, cv::COLOR_BGR2HSV);
  }

  cv::Mat debug_overlay;
  if (generate_debug_overlay) {
    debug_overlay = input.bgr_image.clone();
  }

  std::vector<cv::Point3f> camera_points;
  std::vector<pcl::PointXYZ> candidate_points;
  std::vector<std::size_t> projected_indices;
  std::vector<bool> remove_mask;
  camera_points.reserve(input.cloud->points.size());
  candidate_points.reserve(input.cloud->points.size());
  projected_indices.reserve(input.cloud->points.size());
  if (!config_.publish_only_colored_points) {
    remove_mask.resize(input.cloud->points.size(), false);
  }

  for (const auto & point : input.cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    std::size_t output_index = 0;
    if (!config_.publish_only_colored_points) {
      pcl::PointXYZRGB color_point;
      color_point.x = point.x;
      color_point.y = point.y;
      color_point.z = point.z;
      color_point.r = 128;
      color_point.g = 128;
      color_point.b = 128;
      cloud_out->points.push_back(color_point);
      output_index = cloud_out->points.size() - 1;
    }

    const Eigen::Vector4f pt_lidar(point.x, point.y, point.z, 1.0F);
    const Eigen::Vector4f pt_camera = input.lidar_to_camera_transform * pt_lidar;
    if (pt_camera.z() <= 0.0F) {
      continue;
    }

    camera_points.emplace_back(pt_camera.x(), pt_camera.y(), pt_camera.z());
    if (config_.publish_only_colored_points) {
      candidate_points.push_back(point);
    } else {
      projected_indices.push_back(output_index);
    }
  }

  if (!camera_points.empty()) {
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(
      camera_points,
      zero_rotation_,
      zero_translation_,
      input.camera_matrix,
      input.dist_coeffs,
      image_points);

    for (std::size_t i = 0; i < image_points.size(); ++i) {
      const int u = cvRound(image_points[i].x);
      const int v = cvRound(image_points[i].y);

      if (u < 0 || u >= input.bgr_image.cols || v < 0 || v >= input.bgr_image.rows) {
        continue;
      }

      const cv::Point pixel(u, v);
      const cv::Vec3b & color = input.bgr_image.at<cv::Vec3b>(v, u);
      const bool is_sky = config_.sky_filter.enabled &&
        is_sky_pixel(hsv_image.at<cv::Vec3b>(v, u), v, input.bgr_image.rows);

      if (is_sky) {
        if (!config_.publish_only_colored_points) {
          remove_mask[projected_indices[i]] = true;
        }

        if (generate_debug_overlay) {
          cv::circle(
            debug_overlay,
            pixel,
            config_.debug_point_radius + 1,
            config_.debug_outline_color_bgr,
            config_.debug_outline_thickness,
            cv::LINE_AA);
          cv::circle(
            debug_overlay,
            pixel,
            config_.debug_point_radius,
            config_.debug_skipped_point_color_bgr,
            cv::FILLED,
            cv::LINE_AA);
        }
        continue;
      }

      if (config_.publish_only_colored_points) {
        pcl::PointXYZRGB color_point;
        color_point.x = candidate_points[i].x;
        color_point.y = candidate_points[i].y;
        color_point.z = candidate_points[i].z;
        color_point.b = color[0];
        color_point.g = color[1];
        color_point.r = color[2];
        cloud_out->points.push_back(color_point);
      } else {
        auto & color_point = cloud_out->points[projected_indices[i]];
        color_point.b = color[0];
        color_point.g = color[1];
        color_point.r = color[2];
      }

      if (generate_debug_overlay) {
        cv::circle(
          debug_overlay,
          pixel,
          config_.debug_point_radius + 1,
          config_.debug_outline_color_bgr,
          config_.debug_outline_thickness,
          cv::LINE_AA);
        cv::circle(
          debug_overlay,
          pixel,
          config_.debug_point_radius,
          config_.debug_kept_point_color_bgr,
          cv::FILLED,
          cv::LINE_AA);
      }
    }
  }

  if (!config_.publish_only_colored_points && !remove_mask.empty()) {
    auto compacted_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    compacted_cloud->points.reserve(cloud_out->points.size());
    for (std::size_t i = 0; i < cloud_out->points.size(); ++i) {
      if (!remove_mask[i]) {
        compacted_cloud->points.push_back(cloud_out->points[i]);
      }
    }
    cloud_out = compacted_cloud;
  }

  cloud_out->width = cloud_out->points.size();
  cloud_out->height = 1;
  cloud_out->is_dense = true;

  RawCloudColorizerProcessOutput output;
  output.colored_cloud = cloud_out;
  output.has_debug_overlay = generate_debug_overlay;
  if (generate_debug_overlay) {
    output.debug_overlay = std::move(debug_overlay);
  }

  return output;
}

}  // namespace pointcloud_colorizer
