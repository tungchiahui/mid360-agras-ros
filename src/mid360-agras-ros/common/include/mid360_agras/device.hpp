#pragma once

#include "mid360_agras/protocol.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace mid360_agras {

struct CloudPoint {
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;
  uint8_t reflectivity = 0;
  uint8_t tag = 0;
  uint8_t line = 0;
  uint64_t time_ns = 0;
};

struct ImuSample {
  uint64_t time_ns = 0;
  float gyro_x = 0.f;
  float gyro_y = 0.f;
  float gyro_z = 0.f;
  float acc_x = 0.f;
  float acc_y = 0.f;
  float acc_z = 0.f;
};

enum class TimestampSource { Native, Host, Device };

inline constexpr uint64_t kDefaultCloudPeriodNs = 50000000;

struct Options {
  std::string lidar_ip = "192.168.1.10";
  std::string host_ip = "192.168.1.20";
  uint16_t cmd_port = 60000;
  uint16_t point_port = 60001;
  uint16_t imu_port = 60003;
  uint16_t discover_port = 50000;
  int fov_start_deg = 0;
  int fov_stop_deg = 360;
  bool auto_sampling = true;
  bool configure_dest = false;
  bool publish_fill = false;
  bool standby_on_exit = true;
  double max_range_m = 80.0;
  int line_num = kLineNumberMid360;
  double discover_sec = 1.0;
  TimestampSource timestamp_source = TimestampSource::Native;
  uint64_t cloud_period_ns = kDefaultCloudPeriodNs;
};

struct Stats {
  uint64_t point_packets = 0;
  uint64_t imu_packets = 0;
  uint64_t points_kept = 0;
  uint64_t points_fill = 0;
  uint64_t points_fov = 0;
  uint64_t points_range = 0;
  uint64_t points_queue_drop = 0;
  uint64_t lost_point_packets = 0;
  uint64_t lost_imu_packets = 0;
};

struct ClockStats {
  int time_type = -1;
  uint64_t raw_timestamp_ns = 0;
  uint64_t invalid = 0;
  uint64_t out_of_order = 0;
  uint64_t jumps = 0;
};

struct CompatibilityStats {
  ClockStats point_clock;
  ClockStats imu_clock;
  bool clock_ready = false;
  std::string clock_fault;
  uint64_t dropped_timestamp_packets = 0;
  uint64_t dropped_late_points = 0;
  uint64_t dropped_cloud_points = 0;
  uint64_t dropped_imu_samples = 0;
  uint64_t invalid_imu_samples = 0;
  uint64_t repeated_imu_samples = 0;
  double peak_gyro = 0.0;
  double peak_acceleration = 0.0;
};

class LidarSession {
 public:
  LidarSession();
  ~LidarSession();
  LidarSession(const LidarSession&) = delete;
  LidarSession& operator=(const LidarSession&) = delete;

  bool start(const Options& options, std::string& error);
  void stop();
  void takePoints(std::vector<CloudPoint>& points);
  void takeFrames(std::vector<std::vector<CloudPoint>>& frames);
  void takeImu(std::vector<ImuSample>& samples);
  Stats stats() const;
  CompatibilityStats compatibilityStats() const;
  std::string summary() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mid360_agras
