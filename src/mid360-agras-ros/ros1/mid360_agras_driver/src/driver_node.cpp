#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <mid360_agras_driver/CustomMsg.h>

#include "mid360_agras/pack.hpp"

#include <cmath>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

namespace {

int readInt(const ros::NodeHandle& nh, const char* name, int fallback) {
  int value = fallback;
  if (nh.getParam(name, value)) {
    return value;
  }
  double number = 0.0;
  if (nh.getParam(name, number)) {
    return static_cast<int>(std::lround(number));
  }
  return fallback;
}

double readDouble(const ros::NodeHandle& nh, const char* name, double fallback) {
  double value = fallback;
  nh.getParam(name, value);
  return value;
}

bool readBool(const ros::NodeHandle& nh, const char* name, bool fallback) {
  bool value = fallback;
  nh.getParam(name, value);
  return value;
}

std::string readString(const ros::NodeHandle& nh, const char* name, const std::string& fallback) {
  std::string value = fallback;
  nh.getParam(name, value);
  return value;
}

uint16_t readPort(const ros::NodeHandle& nh, const char* name, int fallback) {
  const int value = readInt(nh, name, fallback);
  if (value <= 0 || value > 65535) {
    return static_cast<uint16_t>(fallback);
  }
  return static_cast<uint16_t>(value);
}

void fillPointCloud(sensor_msgs::PointCloud2& cloud, const std::vector<mid360_agras::LivoxPointXyzrtlt>& points,
                    uint64_t stamp_ns, const std::string& frame_id) {
  cloud.header.frame_id = frame_id;
  cloud.header.stamp.fromNSec(stamp_ns);
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(points.size());
  cloud.fields.resize(7);
  const auto field = [&](size_t index, const char* name, uint32_t offset, uint8_t datatype) {
    cloud.fields[index].name = name;
    cloud.fields[index].offset = offset;
    cloud.fields[index].datatype = datatype;
    cloud.fields[index].count = 1;
  };
  field(0, "x", 0, sensor_msgs::PointField::FLOAT32);
  field(1, "y", 4, sensor_msgs::PointField::FLOAT32);
  field(2, "z", 8, sensor_msgs::PointField::FLOAT32);
  field(3, "intensity", 12, sensor_msgs::PointField::FLOAT32);
  field(4, "tag", 16, sensor_msgs::PointField::UINT8);
  field(5, "line", 17, sensor_msgs::PointField::UINT8);
  field(6, "timestamp", 18, sensor_msgs::PointField::FLOAT64);
  cloud.is_bigendian = false;
  cloud.point_step = sizeof(mid360_agras::LivoxPointXyzrtlt);
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.is_dense = true;
  cloud.data.resize(points.size() * sizeof(mid360_agras::LivoxPointXyzrtlt));
  if (!points.empty()) {
    std::memcpy(cloud.data.data(), points.data(), cloud.data.size());
  }
}

class Driver {
 public:
  Driver() : private_nh_("~") {
    mid360_agras::Options options;
    options.lidar_ip = readString(private_nh_, "lidar_ip", options.lidar_ip);
    options.host_ip = readString(private_nh_, "host_ip", options.host_ip);
    options.cmd_port = readPort(private_nh_, "cmd_port", options.cmd_port);
    options.point_port = readPort(private_nh_, "point_port", options.point_port);
    options.imu_port = readPort(private_nh_, "imu_port", options.imu_port);
    options.discover_port = readPort(private_nh_, "discover_port", options.discover_port);
    options.fov_start_deg = readInt(private_nh_, "fov_start", options.fov_start_deg);
    options.fov_stop_deg = readInt(private_nh_, "fov_stop", options.fov_stop_deg);
    options.auto_sampling = readBool(private_nh_, "auto_sampling", options.auto_sampling);
    options.configure_dest = readBool(private_nh_, "configure_dest", options.configure_dest);
    options.publish_fill = readBool(private_nh_, "publish_fill", options.publish_fill);
    options.standby_on_exit = readBool(private_nh_, "standby_on_exit", options.standby_on_exit);
    options.max_range_m = readDouble(private_nh_, "max_range", 80.0);
    options.line_num = readInt(private_nh_, "line_num", options.line_num);
    options.discover_sec = readDouble(private_nh_, "discover_sec", options.discover_sec);

    xfer_format_ = readInt(private_nh_, "xfer_format", 0);
    if (xfer_format_ != 0 && xfer_format_ != 1) {
      ROS_WARN("xfer_format 只支持 0（PointCloud2）和 1（CustomMsg），已改用 0");
      xfer_format_ = 0;
    }
    frame_id_ = readString(private_nh_, "frame_id", "livox_frame");
    const std::string lidar_topic = readString(private_nh_, "lidar_topic", "livox/lidar");
    const std::string imu_topic = readString(private_nh_, "imu_topic", "livox/imu");
    double publish_freq = readDouble(private_nh_, "publish_freq", 10.0);
    if (publish_freq < 0.5) {
      publish_freq = 0.5;
    } else if (publish_freq > 100.0) {
      publish_freq = 100.0;
    }

    if (xfer_format_ == 1) {
      custom_pub_ = nh_.advertise<mid360_agras_driver::CustomMsg>(lidar_topic, 16);
    } else {
      cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(lidar_topic, 16);
    }
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic, 400);

    std::string error;
    if (!session_.start(options, error)) {
      ROS_ERROR_STREAM(error);
      return;
    }
    ROS_INFO_STREAM(session_.summary());
    ok_ = true;
    cloud_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_freq), &Driver::publishCloud, this);
    imu_timer_ = nh_.createTimer(ros::Duration(0.005), &Driver::publishImu, this);
    stat_timer_ = nh_.createTimer(ros::Duration(5.0), &Driver::publishStats, this);
  }

  bool ok() const { return ok_; }

 private:
  void publishCloud(const ros::TimerEvent&) {
    std::vector<mid360_agras::CloudPoint> points;
    session_.takePoints(points);
    if (points.empty()) {
      return;
    }
    if (xfer_format_ == 1) {
      std::vector<mid360_agras::CustomPointRaw> raw;
      const uint64_t base = mid360_agras::packCustomPoints(points, raw);
      mid360_agras_driver::CustomMsg msg;
      msg.header.frame_id = frame_id_;
      msg.header.stamp.fromNSec(base);
      msg.timebase = base;
      msg.point_num = static_cast<uint32_t>(raw.size());
      msg.lidar_id = 0;
      msg.points.resize(raw.size());
      for (size_t i = 0; i < raw.size(); ++i) {
        msg.points[i].offset_time = raw[i].offset_time;
        msg.points[i].x = raw[i].x;
        msg.points[i].y = raw[i].y;
        msg.points[i].z = raw[i].z;
        msg.points[i].reflectivity = raw[i].reflectivity;
        msg.points[i].tag = raw[i].tag;
        msg.points[i].line = raw[i].line;
      }
      custom_pub_.publish(msg);
      return;
    }
    std::vector<mid360_agras::LivoxPointXyzrtlt> packed;
    const uint64_t base = mid360_agras::packLivoxPoints(points, packed);
    sensor_msgs::PointCloud2 cloud;
    fillPointCloud(cloud, packed, base, frame_id_);
    cloud_pub_.publish(cloud);
  }

  void publishImu(const ros::TimerEvent&) {
    std::vector<mid360_agras::ImuSample> samples;
    session_.takeImu(samples);
    for (const mid360_agras::ImuSample& sample : samples) {
      sensor_msgs::Imu msg;
      msg.header.frame_id = frame_id_;
      msg.header.stamp.fromNSec(sample.time_ns);
      msg.orientation_covariance[0] = -1.0;
      msg.angular_velocity.x = sample.gyro_x;
      msg.angular_velocity.y = sample.gyro_y;
      msg.angular_velocity.z = sample.gyro_z;
      msg.linear_acceleration.x = sample.acc_x;
      msg.linear_acceleration.y = sample.acc_y;
      msg.linear_acceleration.z = sample.acc_z;
      imu_pub_.publish(msg);
    }
  }

  void publishStats(const ros::TimerEvent&) {
    const mid360_agras::Stats now = session_.stats();
    const auto rate = [&](uint64_t current, uint64_t previous) {
      return static_cast<double>(current - previous) / 5.0;
    };
    std::ostringstream text;
    text << "点云 " << rate(now.point_packets, last_stats_.point_packets) << " 包/s，发布 "
         << rate(now.points_kept, last_stats_.points_kept) << " 点/s，填充丢弃 "
         << rate(now.points_fill, last_stats_.points_fill) << "，视场外 "
         << rate(now.points_fov, last_stats_.points_fov) << "，丢包 "
         << (now.lost_point_packets - last_stats_.lost_point_packets) << "，IMU "
         << rate(now.imu_packets, last_stats_.imu_packets) << " 包/s";
    ROS_INFO_STREAM(text.str());
    if (now.point_packets == 0) {
      ROS_WARN_STREAM("还没有点云。确认雷达 IP、本机网段、防火墙放行 UDP 点云/IMU 端口，并且没有别的程序占用这些端口。");
    }
    last_stats_ = now;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher cloud_pub_;
  ros::Publisher custom_pub_;
  ros::Publisher imu_pub_;
  ros::Timer cloud_timer_;
  ros::Timer imu_timer_;
  ros::Timer stat_timer_;
  mid360_agras::LidarSession session_;
  mid360_agras::Stats last_stats_;
  std::string frame_id_;
  int xfer_format_ = 0;
  bool ok_ = false;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "mid360_agras_driver");
  Driver driver;
  if (!driver.ok()) {
    return 1;
  }
  ros::spin();
  return 0;
}
