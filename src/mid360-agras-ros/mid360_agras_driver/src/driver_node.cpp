#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <mid360_agras_driver/msg/custom_msg.hpp>

#include "mid360_agras/pack.hpp"

#include <cmath>
#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

class Driver : public rclcpp::Node {
 public:
  Driver() : rclcpp::Node("mid360_agras_driver") {
    declare_parameter<std::string>("lidar_ip", "192.168.1.10");
    declare_parameter<std::string>("host_ip", "192.168.1.20");
    declare_parameter<int>("cmd_port", 60000);
    declare_parameter<int>("point_port", 60001);
    declare_parameter<int>("imu_port", 60003);
    declare_parameter<int>("discover_port", 50000);
    declare_parameter<int>("fov_start", 0);
    declare_parameter<int>("fov_stop", 360);
    declare_parameter<int>("xfer_format", 0);
    declare_parameter<double>("publish_freq", 20.0);
    declare_parameter<std::string>("frame_id", "livox_frame");
    declare_parameter<std::string>("lidar_topic", "livox/lidar");
    declare_parameter<std::string>("pointcloud_topic", "");
    declare_parameter<std::string>("imu_topic", "livox/imu");
    declare_parameter<bool>("auto_sampling", true);
    declare_parameter<bool>("configure_dest", false);
    declare_parameter<bool>("publish_fill", false);
    declare_parameter<bool>("standby_on_exit", true);
    declare_parameter<double>("max_range", 80.0);
    declare_parameter<int>("line_num", mid360_agras::kLineNumberMid360);
    declare_parameter<double>("discover_sec", 1.0);

    mid360_agras::Options options;
    options.lidar_ip = get_parameter("lidar_ip").as_string();
    options.host_ip = get_parameter("host_ip").as_string();
    options.cmd_port = readPort("cmd_port", 60000);
    options.point_port = readPort("point_port", 60001);
    options.imu_port = readPort("imu_port", 60003);
    options.discover_port = readPort("discover_port", 50000);
    options.fov_start_deg = static_cast<int>(get_parameter("fov_start").as_int());
    options.fov_stop_deg = static_cast<int>(get_parameter("fov_stop").as_int());
    options.auto_sampling = get_parameter("auto_sampling").as_bool();
    options.configure_dest = get_parameter("configure_dest").as_bool();
    options.publish_fill = get_parameter("publish_fill").as_bool();
    options.standby_on_exit = get_parameter("standby_on_exit").as_bool();
    options.max_range_m = get_parameter("max_range").as_double();
    options.line_num = static_cast<int>(get_parameter("line_num").as_int());
    options.discover_sec = get_parameter("discover_sec").as_double();

    int xfer_format = static_cast<int>(get_parameter("xfer_format").as_int());
    if (xfer_format != 0 && xfer_format != 1 && xfer_format != 4) {
      RCLCPP_WARN(get_logger(), "xfer_format 只支持 0（PointCloud2）、1（CustomMsg）和 4（双格式），已改用 0");
      xfer_format = 0;
    }
    frame_id_ = get_parameter("frame_id").as_string();
    double publish_freq = get_parameter("publish_freq").as_double();
    if (publish_freq < 0.5) {
      publish_freq = 0.5;
    } else if (publish_freq > 100.0) {
      publish_freq = 100.0;
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(16));
    const auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(400));
    const auto lidar_topic = get_parameter("lidar_topic").as_string();
    if (xfer_format == 1 || xfer_format == 4) {
      custom_pub_ = create_publisher<mid360_agras_driver::msg::CustomMsg>(lidar_topic, qos);
    }
    if (xfer_format == 0 || xfer_format == 4) {
      auto pointcloud_topic = get_parameter("pointcloud_topic").as_string();
      if (pointcloud_topic.empty()) {
        pointcloud_topic = xfer_format == 4 ? lidar_topic + "/pointcloud" : lidar_topic;
      }
      if (custom_pub_ && pointcloud_topic == lidar_topic) {
        RCLCPP_ERROR(get_logger(), "双格式模式需要不同的 lidar_topic 和 pointcloud_topic");
        return;
      }
      cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, qos);
    }
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(get_parameter("imu_topic").as_string(), imu_qos);

    std::string error;
    if (!session_.start(options, error)) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "%s", session_.summary().c_str());
    ok_ = true;
    cloud_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_freq),
                                     std::bind(&Driver::publishCloud, this));
    imu_timer_ = create_wall_timer(std::chrono::milliseconds(5), std::bind(&Driver::publishImu, this));
    stat_timer_ = create_wall_timer(std::chrono::seconds(5), std::bind(&Driver::publishStats, this));
  }

  bool ok() const { return ok_; }

 private:
  uint16_t readPort(const char* name, int fallback) {
    const int value = static_cast<int>(get_parameter(name).as_int());
    if (value <= 0 || value > 65535) {
      return static_cast<uint16_t>(fallback);
    }
    return static_cast<uint16_t>(value);
  }

  void publishCloud() {
    std::vector<mid360_agras::CloudPoint> points;
    session_.takePoints(points);
    if (points.empty()) {
      return;
    }
    if (custom_pub_) {
      publishCustomCloud(points);
    }
    if (cloud_pub_) {
      publishPointCloud2(points);
    }
  }

  void publishCustomCloud(const std::vector<mid360_agras::CloudPoint>& points) {
    std::vector<mid360_agras::CustomPointRaw> raw;
    const uint64_t base = mid360_agras::packCustomPoints(points, raw);
    mid360_agras_driver::msg::CustomMsg msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(base));
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
    custom_pub_->publish(msg);
  }

  void publishPointCloud2(const std::vector<mid360_agras::CloudPoint>& points) {
    std::vector<mid360_agras::LivoxPointXyzrtlt> packed;
    const uint64_t base = mid360_agras::packLivoxPoints(points, packed);
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = frame_id_;
    cloud.header.stamp = rclcpp::Time(static_cast<int64_t>(base));
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(packed.size());
    cloud.fields.resize(7);
    const auto field = [&](size_t index, const char* name, uint32_t offset, uint8_t datatype) {
      cloud.fields[index].name = name;
      cloud.fields[index].offset = offset;
      cloud.fields[index].datatype = datatype;
      cloud.fields[index].count = 1;
    };
    field(0, "x", 0, sensor_msgs::msg::PointField::FLOAT32);
    field(1, "y", 4, sensor_msgs::msg::PointField::FLOAT32);
    field(2, "z", 8, sensor_msgs::msg::PointField::FLOAT32);
    field(3, "intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
    field(4, "tag", 16, sensor_msgs::msg::PointField::UINT8);
    field(5, "line", 17, sensor_msgs::msg::PointField::UINT8);
    field(6, "timestamp", 18, sensor_msgs::msg::PointField::FLOAT64);
    cloud.is_bigendian = false;
    cloud.point_step = sizeof(mid360_agras::LivoxPointXyzrtlt);
    cloud.row_step = cloud.width * cloud.point_step;
    cloud.is_dense = true;
    cloud.data.resize(packed.size() * sizeof(mid360_agras::LivoxPointXyzrtlt));
    if (!packed.empty()) {
      std::memcpy(cloud.data.data(), packed.data(), cloud.data.size());
    }
    cloud_pub_->publish(cloud);
  }

  void publishImu() {
    std::vector<mid360_agras::ImuSample> samples;
    session_.takeImu(samples);
    for (const mid360_agras::ImuSample& sample : samples) {
      sensor_msgs::msg::Imu msg;
      msg.header.frame_id = frame_id_;
      msg.header.stamp = rclcpp::Time(static_cast<int64_t>(sample.time_ns));
      msg.orientation_covariance[0] = -1.0;
      msg.angular_velocity.x = sample.gyro_x;
      msg.angular_velocity.y = sample.gyro_y;
      msg.angular_velocity.z = sample.gyro_z;
      msg.linear_acceleration.x = sample.acc_x;
      msg.linear_acceleration.y = sample.acc_y;
      msg.linear_acceleration.z = sample.acc_z;
      imu_pub_->publish(msg);
    }
  }

  void publishStats() {
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
    RCLCPP_INFO(get_logger(), "%s", text.str().c_str());
    if (now.point_packets == 0) {
      RCLCPP_WARN(get_logger(), "还没有点云。确认雷达 IP、本机网段、防火墙放行 UDP 点云/IMU 端口，并且没有别的程序占用这些端口。");
    }
    last_stats_ = now;
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<mid360_agras_driver::msg::CustomMsg>::SharedPtr custom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr cloud_timer_;
  rclcpp::TimerBase::SharedPtr imu_timer_;
  rclcpp::TimerBase::SharedPtr stat_timer_;
  mid360_agras::LidarSession session_;
  mid360_agras::Stats last_stats_;
  std::string frame_id_;
  bool ok_ = false;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Driver>();
  if (!node->ok()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
