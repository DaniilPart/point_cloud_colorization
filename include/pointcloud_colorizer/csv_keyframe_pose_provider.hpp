#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace pointcloud_colorizer
{

class CsvKeyframePoseProvider
{
public:
  CsvKeyframePoseProvider(const std::string & csv_path, double match_tolerance_sec);

  bool find_nearest_pose(
    double timestamp_sec,
    Eigen::Matrix4f & transform,
    double * abs_dt_sec = nullptr) const;

  std::size_t size() const;
  double match_tolerance_sec() const;

private:
  struct Entry
  {
    double timestamp_sec = 0.0;
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  };

  static std::vector<std::string> split_csv_line(const std::string & line);
  static Eigen::Matrix4f pose_from_rpy_xyz(
    double x,
    double y,
    double z,
    double roll,
    double pitch,
    double yaw);

  void load_from_csv(const std::string & csv_path);

  std::vector<Entry> entries_;
  double match_tolerance_sec_ = 0.05;
};

}  // namespace pointcloud_colorizer
