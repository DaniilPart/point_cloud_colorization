#include "pointcloud_colorizer/csv_keyframe_pose_provider.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <Eigen/Geometry>

namespace pointcloud_colorizer
{

CsvKeyframePoseProvider::CsvKeyframePoseProvider(
  const std::string & csv_path,
  double match_tolerance_sec)
: match_tolerance_sec_(match_tolerance_sec)
{
  if (match_tolerance_sec_ < 0.0) {
    throw std::runtime_error("csv_pose_match_tolerance_sec must be >= 0");
  }

  load_from_csv(csv_path);
}

std::vector<std::string> CsvKeyframePoseProvider::split_csv_line(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

Eigen::Matrix4f CsvKeyframePoseProvider::pose_from_rpy_xyz(
  double x,
  double y,
  double z,
  double roll,
  double pitch,
  double yaw)
{
  const Eigen::AngleAxisf yaw_aa(static_cast<float>(yaw), Eigen::Vector3f::UnitZ());
  const Eigen::AngleAxisf pitch_aa(static_cast<float>(pitch), Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf roll_aa(static_cast<float>(roll), Eigen::Vector3f::UnitX());

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = (yaw_aa * pitch_aa * roll_aa).toRotationMatrix();
  transform(0, 3) = static_cast<float>(x);
  transform(1, 3) = static_cast<float>(y);
  transform(2, 3) = static_cast<float>(z);
  return transform;
}

void CsvKeyframePoseProvider::load_from_csv(const std::string & csv_path)
{
  std::ifstream input(csv_path);
  if (!input.is_open()) {
    throw std::runtime_error("Failed to open keyframes_csv_path: '" + csv_path + "'");
  }

  std::string header_line;
  if (!std::getline(input, header_line)) {
    throw std::runtime_error("keyframes_csv_path is empty: '" + csv_path + "'");
  }

  const auto header = split_csv_line(header_line);
  auto find_column_index = [&header](const std::string & column_name) -> std::size_t {
      const auto it = std::find(header.begin(), header.end(), column_name);
      if (it == header.end()) {
        throw std::runtime_error("Missing CSV column: '" + column_name + "'");
      }
      return static_cast<std::size_t>(std::distance(header.begin(), it));
    };

  const std::size_t timestamp_idx = find_column_index("timestamp_sec");
  const std::size_t x_idx = find_column_index("x_local");
  const std::size_t y_idx = find_column_index("y_local");
  const std::size_t z_idx = find_column_index("z_local");
  const std::size_t roll_idx = find_column_index("roll");
  const std::size_t pitch_idx = find_column_index("pitch");
  const std::size_t yaw_idx = find_column_index("yaw");
  const std::size_t required_max_idx =
    std::max({timestamp_idx, x_idx, y_idx, z_idx, roll_idx, pitch_idx, yaw_idx});

  entries_.clear();

  std::string line;
  std::size_t line_number = 1;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty()) {
      continue;
    }

    const auto fields = split_csv_line(line);
    if (fields.size() <= required_max_idx) {
      throw std::runtime_error(
              "Malformed CSV line " + std::to_string(line_number) +
              " in '" + csv_path + "'");
    }

    Entry entry;
    try {
      entry.timestamp_sec = std::stod(fields[timestamp_idx]);
      const double x = std::stod(fields[x_idx]);
      const double y = std::stod(fields[y_idx]);
      const double z = std::stod(fields[z_idx]);
      const double roll = std::stod(fields[roll_idx]);
      const double pitch = std::stod(fields[pitch_idx]);
      const double yaw = std::stod(fields[yaw_idx]);
      entry.transform = pose_from_rpy_xyz(x, y, z, roll, pitch, yaw);
    } catch (const std::exception & e) {
      throw std::runtime_error(
              "Failed to parse CSV line " + std::to_string(line_number) +
              " in '" + csv_path + "': " + e.what());
    }

    entries_.push_back(entry);
  }

  if (entries_.empty()) {
    throw std::runtime_error("No keyframe poses loaded from: '" + csv_path + "'");
  }

  std::sort(
    entries_.begin(),
    entries_.end(),
    [](const Entry & lhs, const Entry & rhs) {
      return lhs.timestamp_sec < rhs.timestamp_sec;
    });
}

bool CsvKeyframePoseProvider::find_nearest_pose(
  double timestamp_sec,
  Eigen::Matrix4f & transform,
  double * abs_dt_sec) const
{
  if (entries_.empty()) {
    return false;
  }

  auto cmp = [](const Entry & entry, double ts) {
      return entry.timestamp_sec < ts;
    };
  const auto lower = std::lower_bound(entries_.begin(), entries_.end(), timestamp_sec, cmp);

  const Entry * best = nullptr;
  double best_abs_dt = std::numeric_limits<double>::infinity();

  if (lower != entries_.end()) {
    best = &(*lower);
    best_abs_dt = std::abs(lower->timestamp_sec - timestamp_sec);
  }

  if (lower != entries_.begin()) {
    const auto prev = std::prev(lower);
    const double prev_abs_dt = std::abs(prev->timestamp_sec - timestamp_sec);
    if (best == nullptr || prev_abs_dt < best_abs_dt) {
      best = &(*prev);
      best_abs_dt = prev_abs_dt;
    }
  }

  if (best == nullptr || best_abs_dt > match_tolerance_sec_) {
    return false;
  }

  transform = best->transform;
  if (abs_dt_sec != nullptr) {
    *abs_dt_sec = best_abs_dt;
  }
  return true;
}

std::size_t CsvKeyframePoseProvider::size() const
{
  return entries_.size();
}

double CsvKeyframePoseProvider::match_tolerance_sec() const
{
  return match_tolerance_sec_;
}

}  // namespace pointcloud_colorizer
