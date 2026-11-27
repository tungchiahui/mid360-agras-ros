#pragma once

#include "mid360_agras/device.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iterator>
#include <limits>

namespace mid360_agras {

// The receiver owns this state. LidarSession serializes access with its queue mutex.
class CompatibilityStream {
 public:
  using Clock = std::chrono::steady_clock;
  static constexpr size_t kImuCapacity = 4000;

  CompatibilityStream() = default;
  CompatibilityStream(TimestampSource source, uint64_t period)
      : source_(source), period_ns_(period) {
    stats.clock_ready = source == TimestampSource::Host;
  }

  uint64_t packetTime(const DataPacket& packet, uint64_t host_ns, Clock::time_point received,
                      uint64_t span_ns) {
    auto& track = packet.has_imu ? imu_clock_ : point_clock_;
    auto& clock_stats = packet.has_imu ? stats.imu_clock : stats.point_clock;
    const auto result = observeDeviceClock(packet, host_ns, received, span_ns, track, clock_stats);
    if (!stats.clock_fault.empty()) {
      return dropPacket();
    }
    if (source_ == TimestampSource::Device) {
      return selectDeviceTime(packet, result);
    }

    if (host_ns == 0 || host_ns > kMaxRosTime - span_ns ||
        (track.last_host != 0 && std::abs(elapsed(host_ns, track.last_host) -
                                        durationNs(received - track.last_host_receive)) > kJumpNs)) {
      fault("主机时钟跳变或超出 ROS 时间范围");
      return dropPacket();
    }
    if (host_ns < track.last_host) {
      return dropPacket();
    }
    track.last_host = host_ns;
    track.last_host_receive = received;
    return host_ns;
  }

  bool acceptImu(const ImuSample& sample) {
    const std::array<float, 6> values = {sample.gyro_x, sample.gyro_y, sample.gyro_z,
                                         sample.acc_x, sample.acc_y, sample.acc_z};
    if (!std::all_of(values.begin(), values.end(), [](float value) { return std::isfinite(value); })) {
      ++stats.invalid_imu_samples;
      stats.repeated_imu_samples = 0;
      return false;
    }
    stats.repeated_imu_samples = values == last_imu_ ? stats.repeated_imu_samples + 1 : 1;
    last_imu_ = values;
    for (size_t i = 0; i < 3; ++i) {
      stats.peak_gyro = std::max(stats.peak_gyro, std::abs(static_cast<double>(values[i])));
      stats.peak_acceleration = std::max(stats.peak_acceleration, std::abs(static_cast<double>(values[3 + i])));
    }
    return true;
  }

  void addPoints(const std::vector<CloudPoint>& points, uint64_t packet_end, Clock::time_point received) {
    if (last_cloud_receive_ != Clock::time_point{} &&
        received - last_cloud_receive_ > std::chrono::nanoseconds(2 * period_ns_)) {
      finishFrame();
      host_deadline_ = {};
      device_deadline_ = 0;
    }
    last_cloud_receive_ = received;
    if (!pending_.empty() && !points.empty() &&
        points.back().time_ns > pending_.front().time_ns + kMaxOffset) {
      finishFrame();
    }
    if (pending_.size() + points.size() > kCloudCapacity) {
      finishFrame();
    }
    pending_.insert(pending_.end(), points.begin(), points.end());

    if (source_ == TimestampSource::Device) {
      // Match the driver's millisecond boundaries. Roll the deadline forward
      // even if no packet lands in the exact boundary millisecond.
      const uint64_t device_period = period_ns_ - period_ns_ % kMillisecond;
      if (device_deadline_ == 0 && !pending_.empty()) {
        const uint64_t minimum_end = pending_.front().time_ns + period_ns_ - kMillisecond;
        device_deadline_ = ((minimum_end + device_period - 1) / device_period) * device_period;
      }
      if (device_deadline_ != 0 && packet_end >= device_deadline_) {
        finishFrame();
        device_deadline_ = packet_end - packet_end % device_period + device_period;
      }
    } else if (host_deadline_ == Clock::time_point{}) {
      host_deadline_ = received + std::chrono::nanoseconds(period_ns_);
    } else if (received >= host_deadline_) {
      finishFrame();
      const auto period = std::chrono::nanoseconds(period_ns_);
      host_deadline_ = received - host_deadline_ >= period ? received + period : host_deadline_ + period;
    }
  }

  void takeFrames(std::vector<std::vector<CloudPoint>>& frames) {
    frames.clear();
    frames.swap(frames_);
    queued_points_ = 0;
  }

  CompatibilityStats stats;

 private:
  static constexpr uint64_t kSecond = 1000000000;
  static constexpr uint64_t kMillisecond = 1000000;
  static constexpr uint64_t kMaxRosTime =
      (static_cast<uint64_t>(std::numeric_limits<int32_t>::max()) + 1) * kSecond - 1;
  static constexpr uint64_t kMaxOffset = std::numeric_limits<uint32_t>::max();
  static constexpr int64_t kJumpNs = kSecond;
  static constexpr int64_t kReorderNs = 100 * kMillisecond;
  static constexpr int64_t kEpochToleranceNs = 24 * 60 * 60 * static_cast<int64_t>(kSecond);
  static constexpr size_t kCloudCapacity = 1000000;
  static constexpr size_t kFrameCapacity = 32;

  enum class ClockResult { Valid, Invalid, OutOfOrder, Jump };

  struct ClockTrack {
    int type = -1;
    uint64_t device = 0;
    Clock::time_point received;
    unsigned valid_packets = 0;
    uint64_t last_host = 0;
    Clock::time_point last_host_receive;
  };

  static int64_t durationNs(Clock::duration duration) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
  }

  static int64_t elapsed(uint64_t newer, uint64_t older) {
    return static_cast<int64_t>(newer) - static_cast<int64_t>(older);
  }

  static ClockResult observeDeviceClock(const DataPacket& packet, uint64_t host_ns,
                                        Clock::time_point received, uint64_t span_ns,
                                        ClockTrack& track, ClockStats& stats) {
    stats.time_type = packet.time_type;
    stats.raw_timestamp_ns = packet.timestamp;
    if ((packet.time_type != 1 && packet.time_type != 2) ||
        packet.timestamp == 0 || packet.timestamp > kMaxRosTime - span_ns ||
        std::abs(elapsed(packet.timestamp, host_ns)) > kEpochToleranceNs) {
      ++stats.invalid;
      track.valid_packets = 0;
      return ClockResult::Invalid;
    }
    if (track.valid_packets > 0) {
      const auto step = elapsed(packet.timestamp, track.device);
      if (packet.time_type != track.type || step < -kReorderNs ||
          std::abs(step - durationNs(received - track.received)) > kJumpNs) {
        ++stats.jumps;
        track.valid_packets = 0;
        return ClockResult::Jump;
      }
      if (step <= 0) {
        ++stats.out_of_order;
        return ClockResult::OutOfOrder;
      }
    }
    track.type = packet.time_type;
    track.device = packet.timestamp;
    track.received = received;
    track.valid_packets = std::min(2u, track.valid_packets + 1);
    return ClockResult::Valid;
  }

  uint64_t selectDeviceTime(const DataPacket& packet, ClockResult result) {
    if (result == ClockResult::Invalid || result == ClockResult::Jump) {
      if (stats.clock_ready) {
        fault(packet.has_imu ? "IMU 设备时间失效或跳变" : "点云设备时间失效或跳变");
      }
      return dropPacket();
    }
    if (result == ClockResult::OutOfOrder || point_clock_.valid_packets < 2 || imu_clock_.valid_packets < 2) {
      return dropPacket();
    }

    const auto skew = elapsed(point_clock_.device, imu_clock_.device) -
                      durationNs(point_clock_.received - imu_clock_.received);
    if (point_clock_.type != imu_clock_.type || std::abs(skew) > kJumpNs) {
      if (stats.clock_ready) {
        fault("点云和 IMU 的设备时钟类型或时间偏差不一致");
      }
      return dropPacket();
    }
    stats.clock_ready = true;
    return packet.timestamp;
  }

  void fault(const char* reason) {
    stats.clock_fault = reason;
    stats.clock_ready = false;
    stats.dropped_cloud_points += pending_.size() + queued_points_;
    pending_.clear();
    frames_.clear();
    queued_points_ = 0;
  }

  uint64_t dropPacket() {
    ++stats.dropped_timestamp_packets;
    return 0;
  }

  void finishFrame() {
    if (pending_.empty()) {
      return;
    }
    const auto earlier = [](const CloudPoint& a, const CloudPoint& b) { return a.time_ns < b.time_ns; };
    if (!std::is_sorted(pending_.begin(), pending_.end(), earlier)) {
      std::stable_sort(pending_.begin(), pending_.end(), earlier);
    }
    const auto first = std::lower_bound(pending_.begin(), pending_.end(), last_point_ns_,
        [](const CloudPoint& point, uint64_t stamp) { return point.time_ns < stamp; });
    stats.dropped_late_points += std::distance(pending_.begin(), first);
    pending_.erase(pending_.begin(), first);
    if (!pending_.empty()) {
      last_point_ns_ = pending_.back().time_ns;
      if (frames_.size() == kFrameCapacity || queued_points_ + pending_.size() > kCloudCapacity) {
        stats.dropped_cloud_points += pending_.size();
        pending_.clear();
        return;
      }
      queued_points_ += pending_.size();
      frames_.emplace_back(std::move(pending_));
      pending_.clear();
    }
  }

  TimestampSource source_ = TimestampSource::Native;
  uint64_t period_ns_ = kDefaultCloudPeriodNs;
  ClockTrack point_clock_;
  ClockTrack imu_clock_;
  std::array<float, 6> last_imu_{};
  Clock::time_point host_deadline_;
  Clock::time_point last_cloud_receive_;
  uint64_t device_deadline_ = 0;
  uint64_t last_point_ns_ = 0;
  size_t queued_points_ = 0;
  std::vector<CloudPoint> pending_;
  std::vector<std::vector<CloudPoint>> frames_;
};

}  // namespace mid360_agras
