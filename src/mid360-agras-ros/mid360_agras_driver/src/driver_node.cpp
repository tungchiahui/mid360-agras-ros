#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <mid360_agras_driver/msg/custom_msg.hpp>

#include "mid360_agras/pack.hpp"

#include <algorithm>
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
    rcl_interfaces::msg::ParameterDescriptor startup;
    startup.read_only = true;
    const bool compatible = declare_parameter<bool>("livox_compatibility", false, startup);
    startup.read_only = compatible;
    declare_parameter<std::string>("lidar_ip", "192.168.1.10");
    declare_parameter<std::string>("host_ip", "192.168.1.20");
    declare_parameter<int>("cmd_port", 60000);
    declare_parameter<int>("point_port", 60001);
    declare_parameter<int>("imu_port", 60003);
    declare_parameter<int>("discover_port", 50000);
    declare_parameter<int>("fov_start", 0);
    declare_parameter<int>("fov_stop", 360);
    const auto requested_format = declare_parameter<int64_t>("xfer_format", 0, startup);
    const auto requested_freq = declare_parameter<double>("publish_freq", 20.0, startup);
    frame_id_ = declare_parameter<std::string>("frame_id", "livox_frame", startup);
    const auto lidar_topic = declare_parameter<std::string>("lidar_topic", "livox/lidar", startup);
    auto pointcloud_topic = declare_parameter<std::string>("pointcloud_topic", "", startup);
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "livox/imu", startup);
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

    if (!std::isfinite(requested_freq)) {
      RCLCPP_ERROR(get_logger(), "publish_freq 必须是有限数值");
      return;
    }
    const double publish_freq = std::clamp(requested_freq, 0.5, 100.0);
    size_t cloud_depth = 16;
    size_t imu_depth = 400;
    imu_frame_id_ = frame_id_;
    if (compatible) {
      const auto multi_topic = declare_parameter<int64_t>("multi_topic", 0, startup);
      const auto data_src = declare_parameter<int64_t>("data_src", 0, startup);
      const auto output_type = declare_parameter<int64_t>("output_data_type", 0, startup);
      const auto timestamp_source = declare_parameter<std::string>("timestamp_source", "host", startup);
      imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "livox_frame", startup);
      if ((requested_format != 0 && requested_format != 1) || multi_topic < 0 || multi_topic > 1 ||
          data_src != 0 || output_type != 0 || options.line_num != mid360_agras::kLineNumberMid360 ||
          (timestamp_source != "host" && timestamp_source != "device")) {
        RCLCPP_ERROR(get_logger(), "Livox 兼容模式需要 xfer_format=0/1、multi_topic=0/1、"
                     "data_src=0、output_data_type=0、line_num=4、timestamp_source=host/device");
        return;
      }
      options.timestamp_source = timestamp_source == "host" ?
          mid360_agras::TimestampSource::Host : mid360_agras::TimestampSource::Device;
      options.cloud_period_ns = static_cast<uint64_t>(1e9 / publish_freq);
      cloud_depth = imu_depth = multi_topic == 0 ? 256 : 64;
      uint8_t lidar_ip[4] = {};
      if (mid360_agras::parseIpv4(options.lidar_ip, lidar_ip)) {
        // The official SDK uses sin_addr.s_addr as the handle on Humble (little-endian).
        lidar_id_ = lidar_ip[0];
      }
      lidar_ip_ = options.lidar_ip;
      timestamp_source_ = timestamp_source;
      diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
      RCLCPP_INFO(get_logger(), "Livox 兼容模式: 时间源 %s，点云 %.3f Hz，QoS 深度 %zu",
                  timestamp_source.c_str(), publish_freq, cloud_depth);
    }

    auto xfer_format = requested_format;
    if (xfer_format != 0 && xfer_format != 1 && xfer_format != 4) {
      RCLCPP_WARN(get_logger(), "xfer_format 只支持 0（PointCloud2）、1（CustomMsg）和 4（双格式），已改用 0");
      xfer_format = 0;
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(cloud_depth));
    const auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(imu_depth));
    if (xfer_format == 1 || xfer_format == 4) {
      custom_pub_ = create_publisher<mid360_agras_driver::msg::CustomMsg>(lidar_topic, qos);
    }
    if (xfer_format == 0 || xfer_format == 4) {
      if (pointcloud_topic.empty()) {
        pointcloud_topic = xfer_format == 4 ? lidar_topic + "/pointcloud" : lidar_topic;
      }
      if (custom_pub_ && pointcloud_topic == lidar_topic) {
        RCLCPP_ERROR(get_logger(), "双格式模式需要不同的 lidar_topic 和 pointcloud_topic");
        return;
      }
      cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, qos);
    }
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic, imu_qos);

    std::string error;
    if (!session_.start(options, error)) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "%s", session_.summary().c_str());
    ok_ = true;
    if (compatible) {
      // UDP arrivals have already produced complete frames. This timer only transfers them to ROS.
      cloud_timer_ = create_wall_timer(std::chrono::milliseconds(2), std::bind(&Driver::publishFrames, this));
      stat_timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&Driver::publishDiagnostics, this));
    } else {
      cloud_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_freq),
                                       std::bind(&Driver::publishCloud, this));
      stat_timer_ = create_wall_timer(std::chrono::seconds(5), std::bind(&Driver::publishStats, this));
    }
    imu_timer_ = create_wall_timer(std::chrono::milliseconds(5), std::bind(&Driver::publishImu, this));
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
    publishPoints(points);
  }

  void publishFrames() {
    std::vector<std::vector<mid360_agras::CloudPoint>> frames;
    session_.takeFrames(frames);
    for (const auto& frame : frames) {
      publishPoints(frame);
    }
  }

  void publishPoints(const std::vector<mid360_agras::CloudPoint>& points) {
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
    msg.lidar_id = lidar_id_;
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
      msg.header.frame_id = imu_frame_id_;
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

  void publishDiagnostics() {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const auto stats = session_.stats();
    const auto quality = session_.compatibilityStats();
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = diagnostic_clock_.now();
    const auto status = [&](const std::string& name) -> DiagnosticStatus& {
      auto& item = message.status.emplace_back();
      item.name = std::string(get_fully_qualified_name()) + ": " + name;
      item.hardware_id = lidar_ip_;
      item.level = DiagnosticStatus::OK;
      item.message = "数据流正常";
      return item;
    };
    const auto add = [](DiagnosticStatus& item, const std::string& key, const auto& value) {
      auto& entry = item.values.emplace_back();
      entry.key = key;
      std::ostringstream text;
      text << value;
      entry.value = text.str();
    };
    const auto add_clock = [&](DiagnosticStatus& item, const std::string& stream, const auto& clock) {
      add(item, stream + ".time_type", clock.time_type);
      add(item, stream + ".raw_timestamp_ns", clock.raw_timestamp_ns);
      add(item, stream + ".invalid_or_unsynchronized", clock.invalid);
      add(item, stream + ".out_of_order", clock.out_of_order);
      add(item, stream + ".jumps", clock.jumps);
    };

    auto& clock = status("timestamps");
    add(clock, "timestamp_source", timestamp_source_);
    add(clock, "dropped_timestamp_packets", quality.dropped_timestamp_packets);
    add_clock(clock, "pointcloud", quality.point_clock);
    add_clock(clock, "imu", quality.imu_clock);
    if (!quality.clock_fault.empty()) {
      clock.level = DiagnosticStatus::ERROR;
      clock.message = quality.clock_fault + "，兼容输出已暂停，恢复时钟后重启节点";
      RCLCPP_ERROR_THROTTLE(get_logger(), steady_clock_, 5000, "%s", clock.message.c_str());
    } else if (!quality.clock_ready) {
      clock.level = DiagnosticStatus::WARN;
      clock.message = "等待点云和 IMU 设备时间校验通过";
    } else if (timestamp_source_ == "host") {
      clock.message = "点云和 IMU 使用主机收包时间，设备时间仅供诊断";
      if (quality.point_clock.invalid || quality.imu_clock.invalid ||
          quality.point_clock.jumps || quality.imu_clock.jumps ||
          quality.point_clock.out_of_order || quality.imu_clock.out_of_order) {
        clock.level = DiagnosticStatus::WARN;
      }
    } else if (quality.dropped_timestamp_packets != last_quality_.dropped_timestamp_packets) {
      clock.level = DiagnosticStatus::WARN;
      clock.message = "已拒收时间不合格或乱序的数据包";
    }

    auto& cloud = status("pointcloud");
    add(cloud, "packets", stats.point_packets);
    add(cloud, "estimated_lost_packets", stats.lost_point_packets);
    add(cloud, "filtered_fill", stats.points_fill);
    add(cloud, "filtered_fov", stats.points_fov);
    add(cloud, "filtered_range", stats.points_range);
    add(cloud, "dropped_late_points", quality.dropped_late_points);
    add(cloud, "dropped_cloud_points", quality.dropped_cloud_points);
    if (stats.point_packets == last_stats_.point_packets) {
      cloud.level = DiagnosticStatus::WARN;
      cloud.message = "最近一秒未收到点云包";
    } else if (quality.dropped_late_points != last_quality_.dropped_late_points ||
               quality.dropped_cloud_points != last_quality_.dropped_cloud_points) {
      cloud.level = DiagnosticStatus::WARN;
      cloud.message = "点云时间重叠或队列丢弃";
    }

    auto& imu = status("imu");
    add(imu, "packets", stats.imu_packets);
    add(imu, "estimated_lost_packets", stats.lost_imu_packets);
    add(imu, "dropped_imu_samples", quality.dropped_imu_samples);
    add(imu, "invalid_imu_samples", quality.invalid_imu_samples);
    add(imu, "consecutive_identical_samples", quality.repeated_imu_samples);
    add(imu, "peak_abs_gyro_raw", quality.peak_gyro);
    add(imu, "peak_abs_acceleration_raw", quality.peak_acceleration);
    if (quality.invalid_imu_samples != last_quality_.invalid_imu_samples) {
      imu.level = DiagnosticStatus::ERROR;
      imu.message = "已拒发包含 NaN 或 Inf 的 IMU 样本";
    } else if (stats.imu_packets == last_stats_.imu_packets) {
      imu.level = DiagnosticStatus::WARN;
      imu.message = "最近一秒未收到 IMU 包";
    } else if (quality.dropped_imu_samples != last_quality_.dropped_imu_samples) {
      imu.level = DiagnosticStatus::WARN;
      imu.message = "IMU 队列丢弃样本";
    } else if (quality.repeated_imu_samples >= 32) {
      imu.level = DiagnosticStatus::WARN;
      imu.message = "IMU 六轴数据连续重复";
    }
    if (imu.level != DiagnosticStatus::OK) {
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000, "%s", imu.message.c_str());
    }
    diagnostics_pub_->publish(message);
    last_stats_ = stats;
    last_quality_ = quality;
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
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr cloud_timer_;
  rclcpp::TimerBase::SharedPtr imu_timer_;
  rclcpp::TimerBase::SharedPtr stat_timer_;
  mid360_agras::LidarSession session_;
  mid360_agras::Stats last_stats_;
  mid360_agras::CompatibilityStats last_quality_;
  rclcpp::Clock diagnostic_clock_{RCL_SYSTEM_TIME};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  std::string frame_id_;
  std::string imu_frame_id_;
  std::string lidar_ip_;
  std::string timestamp_source_;
  uint8_t lidar_id_ = 0;
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
