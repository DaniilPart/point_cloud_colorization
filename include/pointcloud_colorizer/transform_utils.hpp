#pragma once

#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include <Eigen/Dense>
#include <geometry_msgs/msg/transform.hpp>

namespace pointcloud_colorizer
{

enum class TransformSource
{
  Config,
  TfTree
};

inline TransformSource parse_transform_source(const std::string & value)
{
  if (value == "config") {
    return TransformSource::Config;
  }

  if (value == "tf_tree") {
    return TransformSource::TfTree;
  }

  throw std::runtime_error("transform_source must be 'config' or 'tf_tree'");
}

inline void require_non_empty(const std::string & value, const std::string & parameter_name)
{
  if (value.empty()) {
    throw std::runtime_error(parameter_name + " must be set");
  }
}

inline Eigen::Matrix4f matrix_from_row_major_values(
  const std::vector<double> & values,
  const std::string & parameter_name)
{
  if (values.size() != 16) {
    throw std::runtime_error(parameter_name + " must contain exactly 16 row-major values");
  }

  Eigen::Matrix4f transform;
  for (std::size_t row = 0; row < 4; ++row) {
    for (std::size_t col = 0; col < 4; ++col) {
      transform(row, col) = static_cast<float>(values[row * 4 + col]);
    }
  }

  return transform;
}

inline Eigen::Matrix4f matrix_from_transform_msg(const geometry_msgs::msg::Transform & msg)
{
  Eigen::Quaternionf quaternion(
    static_cast<float>(msg.rotation.w),
    static_cast<float>(msg.rotation.x),
    static_cast<float>(msg.rotation.y),
    static_cast<float>(msg.rotation.z));

  if (quaternion.norm() == 0.0f) {
    throw std::runtime_error("Received TF transform with zero-length quaternion");
  }

  quaternion.normalize();

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
  transform(0, 3) = static_cast<float>(msg.translation.x);
  transform(1, 3) = static_cast<float>(msg.translation.y);
  transform(2, 3) = static_cast<float>(msg.translation.z);
  return transform;
}

}  // namespace pointcloud_colorizer
