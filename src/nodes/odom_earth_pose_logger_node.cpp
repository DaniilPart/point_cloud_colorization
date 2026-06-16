#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class OdomEarthPoseLoggerNode : public rclcpp::Node
{
public:
  explicit OdomEarthPoseLoggerNode(const rclcpp::NodeOptions & options)
  : Node("odom_earth_pose_logger", options)
  {
    input_odometry_topic_ = this->declare_parameter<std::string>("input_odometry_topic", "");
    map_save_interval_sec_ = this->declare_parameter<double>("map_save_interval_sec", 5.0);
    output_dir_ = this->declare_parameter<std::string>("output_dir", "");
    earth_frame_id_ = this->declare_parameter<std::string>("earth_frame_id", "earth");
    tf_lookup_timeout_sec_ = this->declare_parameter<double>("tf_lookup_timeout_sec", 0.1);
    run_id_ = this->declare_parameter<std::string>("run_id", "");

    validate_configuration();

    experiment_start_timestamp_ = run_id_.empty() ? make_start_timestamp() : run_id_;
    output_directory_ = resolve_run_output_directory(output_dir_, experiment_start_timestamp_);

    std::error_code dir_ec;
    std::filesystem::create_directories(output_directory_, dir_ec);
    if (dir_ec) {
      throw std::runtime_error(
              "Failed to create output directory '" + output_directory_.string() + "': " +
              dir_ec.message());
    }

    legacy_pose_log_path_ = resolve_legacy_pose_log_path(output_directory_);
    odometry_csv_path_ = (output_directory_ / "odometry.csv").string();
    ecef_csv_path_ = (output_directory_ / "ecef.csv").string();
    utm_csv_path_ = (output_directory_ / "utm.csv").string();
    latest_transform_yaml_path_ = (output_directory_ / "earth_to_odom_transform.yaml").string();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_odometry_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&OdomEarthPoseLoggerNode::odometry_callback, this, std::placeholders::_1));

    if (map_save_interval_sec_ > 0.0) {
      const auto save_period = std::chrono::duration<double>(std::max(0.2, map_save_interval_sec_));
      save_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(save_period),
        std::bind(&OdomEarthPoseLoggerNode::save_timer_callback, this));
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Odom-earth pose logger: odometry=%s earth_frame=%s map_save_interval_sec=%.2f run_id=%s output_dir=%s",
      input_odometry_topic_.c_str(),
      earth_frame_id_.c_str(),
      map_save_interval_sec_,
      experiment_start_timestamp_.c_str(),
      output_directory_.string().c_str());

    if (map_save_interval_sec_ <= 0.0) {
      RCLCPP_INFO(
        this->get_logger(),
        "Periodic pose logging disabled (map_save_interval_sec=%.3f).",
        map_save_interval_sec_);
    }
  }

private:
  static double stamp_to_seconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) +
      static_cast<double>(stamp.nanosec) * 1e-9;
  }

  static std::string make_start_timestamp()
  {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now{};
#ifdef _WIN32
    localtime_s(&tm_now, &now_time);
#else
    localtime_r(&now_time, &tm_now);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
    return oss.str();
  }

  static std::string expand_user_path(const std::string & path)
  {
    if (path.empty() || path[0] != '~') {
      return path;
    }

    if (path.size() > 1 && path[1] != '/') {
      return path;
    }

    const char * home = std::getenv("HOME");
    if (home == nullptr || home[0] == '\0') {
      return path;
    }

    return std::string(home) + path.substr(1);
  }

  static std::filesystem::path resolve_run_output_directory(
    const std::string & output_dir,
    const std::string & run_id)
  {
    const std::filesystem::path root(expand_user_path(output_dir));
    return root / run_id;
  }

  static std::string resolve_legacy_pose_log_path(const std::filesystem::path & output_dir)
  {
    return (output_dir / "odom_earth_pose_log.csv").string();
  }

  static char latitude_to_utm_band(double latitude_deg)
  {
    if (latitude_deg < -80.0 || latitude_deg > 84.0) {
      return 'Z';
    }

    constexpr const char * bands = "CDEFGHJKLMNPQRSTUVWX";
    int index = static_cast<int>(std::floor((latitude_deg + 80.0) / 8.0));
    if (latitude_deg >= 72.0) {
      index = 19;
    }

    index = std::max(0, std::min(index, 19));
    return bands[index];
  }

  static tf2::Quaternion normalized_quaternion(
    double x,
    double y,
    double z,
    double w,
    const tf2::Quaternion & fallback = tf2::Quaternion(0.0, 0.0, 0.0, 1.0))
  {
    tf2::Quaternion q(x, y, z, w);
    if (q.length2() < 1e-12) {
      return fallback;
    }

    q.normalize();
    return q;
  }

  void validate_configuration() const
  {
    if (input_odometry_topic_.empty()) {
      throw std::runtime_error("input_odometry_topic must not be empty");
    }

    if (map_save_interval_sec_ < 0.0) {
      throw std::runtime_error("map_save_interval_sec must be >= 0");
    }

    if (output_dir_.empty()) {
      throw std::runtime_error("output_dir must not be empty");
    }

    if (earth_frame_id_.empty()) {
      throw std::runtime_error("earth_frame_id must not be empty");
    }

    if (tf_lookup_timeout_sec_ < 0.0) {
      throw std::runtime_error("tf_lookup_timeout_sec must be >= 0");
    }
  }

  void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
  {
    std::lock_guard<std::mutex> lock(latest_odom_mutex_);
    latest_odom_msg_ = odom_msg;
  }

  void save_timer_callback()
  {
    save_latest_pose();
  }

  bool ensure_output_directory_exists() const
  {
    std::error_code ec;
    std::filesystem::create_directories(output_directory_, ec);
    if (ec) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Failed to create output directory '%s': %s",
        output_directory_.string().c_str(),
        ec.message().c_str());
      return false;
    }

    return true;
  }

  bool write_legacy_pose_csv(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
    const geometry_msgs::msg::TransformStamped & earth_t_odom)
  {
    const std::filesystem::path output_path(legacy_pose_log_path_);
    const bool write_header =
      !std::filesystem::exists(output_path) ||
      std::filesystem::is_empty(output_path);

    std::ofstream out(legacy_pose_log_path_, std::ios::app);
    if (!out.is_open()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Failed to open legacy pose log file '%s' for append.",
        legacy_pose_log_path_.c_str());
      return false;
    }

    if (write_header) {
      out << "odometry_stamp_sec,node_time_sec,target_frame,source_frame,tx,ty,tz,qx,qy,qz,qw\n";
    }

    const double odom_stamp_sec = stamp_to_seconds(odom_msg->header.stamp);
    const double node_time_sec = this->get_clock()->now().seconds();
    const auto & tr = earth_t_odom.transform.translation;
    const auto & q = earth_t_odom.transform.rotation;

    out << std::fixed << std::setprecision(9)
        << odom_stamp_sec << ","
        << node_time_sec << ","
        << earth_t_odom.header.frame_id << ","
        << earth_t_odom.child_frame_id << ","
        << tr.x << ","
        << tr.y << ","
        << tr.z << ","
        << q.x << ","
        << q.y << ","
        << q.z << ","
        << q.w << "\n";

    return true;
  }

  bool write_latest_transform_yaml(
    const geometry_msgs::msg::TransformStamped & earth_t_odom,
    double timestamp_sec)
  {
    std::ofstream out(latest_transform_yaml_path_, std::ios::trunc);
    if (!out.is_open()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Failed to open latest transform YAML file '%s' for write.",
        latest_transform_yaml_path_.c_str());
      return false;
    }

    const auto & t = earth_t_odom.transform.translation;
    const auto & q = earth_t_odom.transform.rotation;

    out << std::fixed << std::setprecision(9);
    out << "timestamp: " << timestamp_sec << "\n";
    out << "parent_frame: " << earth_t_odom.header.frame_id << "\n";
    out << "child_frame: " << earth_t_odom.child_frame_id << "\n";
    out << "translation:\n";
    out << "  x: " << t.x << "\n";
    out << "  y: " << t.y << "\n";
    out << "  z: " << t.z << "\n";
    out << "rotation:\n";
    out << "  qx: " << q.x << "\n";
    out << "  qy: " << q.y << "\n";
    out << "  qz: " << q.z << "\n";
    out << "  qw: " << q.w << "\n";

    return true;
  }

  bool append_odometry_csv(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
    double timestamp_sec)
  {
    const std::filesystem::path output_path(odometry_csv_path_);
    const bool write_header =
      !std::filesystem::exists(output_path) ||
      std::filesystem::is_empty(output_path);

    std::ofstream out(odometry_csv_path_, std::ios::app);
    if (!out.is_open()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Failed to open odometry CSV '%s' for append.",
        odometry_csv_path_.c_str());
      return false;
    }

    if (write_header) {
      out << "timestamp,frame,x,y,z,qx,qy,qz,qw\n";
    }

    const auto & p = odom_msg->pose.pose.position;
    const auto & q = odom_msg->pose.pose.orientation;
    out << std::fixed << std::setprecision(9)
        << timestamp_sec << ","
        << odom_msg->header.frame_id << ","
        << p.x << ","
        << p.y << ","
        << p.z << ","
        << q.x << ","
        << q.y << ","
        << q.z << ","
        << q.w << "\n";

    return true;
  }

  bool append_ecef_csv(
    double timestamp_sec,
    const tf2::Vector3 & ecef_position,
    const tf2::Quaternion & ecef_orientation)
  {
    const std::filesystem::path output_path(ecef_csv_path_);
    const bool write_header =
      !std::filesystem::exists(output_path) ||
      std::filesystem::is_empty(output_path);

    std::ofstream out(ecef_csv_path_, std::ios::app);
    if (!out.is_open()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Failed to open ECEF CSV '%s' for append.",
        ecef_csv_path_.c_str());
      return false;
    }

    if (write_header) {
      out << "timestamp,frame,x,y,z,qx,qy,qz,qw\n";
    }

    out << std::fixed << std::setprecision(9)
        << timestamp_sec << ","
        << earth_frame_id_ << ","
        << ecef_position.x() << ","
        << ecef_position.y() << ","
        << ecef_position.z() << ","
        << ecef_orientation.x() << ","
        << ecef_orientation.y() << ","
        << ecef_orientation.z() << ","
        << ecef_orientation.w() << "\n";

    return true;
  }

  bool append_utm_csv(
    double timestamp_sec,
    int zone,
    char band,
    bool north_hemisphere,
    double easting,
    double northing,
    double altitude)
  {
    const std::filesystem::path output_path(utm_csv_path_);
    const bool write_header =
      !std::filesystem::exists(output_path) ||
      std::filesystem::is_empty(output_path);

    std::ofstream out(utm_csv_path_, std::ios::app);
    if (!out.is_open()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Failed to open UTM CSV '%s' for append.",
        utm_csv_path_.c_str());
      return false;
    }

    if (write_header) {
      out << "timestamp,zone,band,hemisphere,easting,northing,altitude\n";
    }

    out << std::fixed << std::setprecision(9)
        << timestamp_sec << ","
        << zone << ","
        << band << ","
        << (north_hemisphere ? "N" : "S") << ","
        << easting << ","
        << northing << ","
        << altitude << "\n";

    return true;
  }

  bool compute_utm_from_ecef(
    const tf2::Vector3 & ecef_position,
    int & zone,
    char & band,
    bool & north_hemisphere,
    double & easting,
    double & northing,
    double & altitude)
  {
    try {
      double latitude_deg = 0.0;
      double longitude_deg = 0.0;
      GeographicLib::Geocentric::WGS84().Reverse(
        ecef_position.x(),
        ecef_position.y(),
        ecef_position.z(),
        latitude_deg,
        longitude_deg,
        altitude);

      GeographicLib::UTMUPS::Forward(
        latitude_deg,
        longitude_deg,
        zone,
        north_hemisphere,
        easting,
        northing);

      band = latitude_to_utm_band(latitude_deg);
      return true;
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Failed to convert ECEF to UTM: %s",
        ex.what());
      return false;
    }
  }

  bool save_latest_pose()
  {
    nav_msgs::msg::Odometry::ConstSharedPtr odom_msg;
    {
      std::lock_guard<std::mutex> lock(latest_odom_mutex_);
      odom_msg = latest_odom_msg_;
    }

    if (!odom_msg) {
      return false;
    }

    if (odom_msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Skipping pose logging because odometry header frame_id is empty.");
      return false;
    }

    geometry_msgs::msg::TransformStamped earth_t_odom;
    try {
      earth_t_odom = tf_buffer_->lookupTransform(
        earth_frame_id_,
        odom_msg->header.frame_id,
        odom_msg->header.stamp,
        tf2::durationFromSec(tf_lookup_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Failed to lookup transform %s<- %s: %s",
        earth_frame_id_.c_str(),
        odom_msg->header.frame_id.c_str(),
        ex.what());
      return false;
    }

    if (!ensure_output_directory_exists()) {
      return false;
    }

    const double timestamp_sec = stamp_to_seconds(odom_msg->header.stamp);

    const auto & tf_t = earth_t_odom.transform.translation;
    const auto & tf_q = earth_t_odom.transform.rotation;
    const auto & odom_p = odom_msg->pose.pose.position;
    const auto & odom_q = odom_msg->pose.pose.orientation;

    const tf2::Vector3 t_earth_odom(tf_t.x, tf_t.y, tf_t.z);
    const tf2::Quaternion q_earth_odom = normalized_quaternion(tf_q.x, tf_q.y, tf_q.z, tf_q.w);
    const tf2::Vector3 p_odom(odom_p.x, odom_p.y, odom_p.z);
    const tf2::Quaternion q_odom = normalized_quaternion(odom_q.x, odom_q.y, odom_q.z, odom_q.w);

    const tf2::Vector3 p_ecef = tf2::quatRotate(q_earth_odom, p_odom) + t_earth_odom;
    tf2::Quaternion q_ecef = q_earth_odom * q_odom;
    q_ecef.normalize();

    int utm_zone = 0;
    char utm_band = 'Z';
    bool utm_north = true;
    double utm_easting = 0.0;
    double utm_northing = 0.0;
    double utm_altitude = 0.0;
    if (!compute_utm_from_ecef(
        p_ecef,
        utm_zone,
        utm_band,
        utm_north,
        utm_easting,
        utm_northing,
        utm_altitude))
    {
      return false;
    }

    const bool ok_legacy = write_legacy_pose_csv(odom_msg, earth_t_odom);
    const bool ok_yaml = write_latest_transform_yaml(earth_t_odom, timestamp_sec);
    const bool ok_odom = append_odometry_csv(odom_msg, timestamp_sec);
    const bool ok_ecef = append_ecef_csv(timestamp_sec, p_ecef, q_ecef);
    const bool ok_utm = append_utm_csv(
      timestamp_sec,
      utm_zone,
      utm_band,
      utm_north,
      utm_easting,
      utm_northing,
      utm_altitude);

    return ok_legacy && ok_yaml && ok_odom && ok_ecef && ok_utm;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr save_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string input_odometry_topic_;
  double map_save_interval_sec_ = 5.0;
  std::string output_dir_;
  std::string earth_frame_id_ = "earth";
  double tf_lookup_timeout_sec_ = 0.1;
  std::string run_id_;
  std::string experiment_start_timestamp_;

  std::filesystem::path output_directory_;
  std::string legacy_pose_log_path_;
  std::string odometry_csv_path_;
  std::string ecef_csv_path_;
  std::string utm_csv_path_;
  std::string latest_transform_yaml_path_;

  std::mutex latest_odom_mutex_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_msg_;
};

#ifndef POINTCLOUD_COLORIZER_BUILD_STANDALONE
RCLCPP_COMPONENTS_REGISTER_NODE(OdomEarthPoseLoggerNode)
#else
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomEarthPoseLoggerNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
