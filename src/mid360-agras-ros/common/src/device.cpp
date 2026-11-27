#include "mid360_agras/device.hpp"
#include "mid360_agras/compatibility_stream.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <set>
#include <sstream>
#include <thread>
#include <utility>

namespace mid360_agras {
namespace {

using Clock = std::chrono::steady_clock;

void logLine(const std::string& text) {
  std::fprintf(stderr, "[mid360_agras] %s\n", text.c_str());
  std::fflush(stderr);
}

class Socket {
 public:
  Socket() = default;
  ~Socket() { reset(); }
  Socket(const Socket&) = delete;
  Socket& operator=(const Socket&) = delete;
  Socket(Socket&& other) noexcept : fd_(other.fd_) { other.fd_ = -1; }
  Socket& operator=(Socket&& other) noexcept {
    if (this != &other) {
      reset();
      fd_ = other.fd_;
      other.fd_ = -1;
    }
    return *this;
  }

  void reset(int fd = -1) {
    if (fd_ >= 0) {
      ::close(fd_);
    }
    fd_ = fd;
  }
  int get() const { return fd_; }
  bool valid() const { return fd_ >= 0; }

 private:
  int fd_ = -1;
};

Socket bindUdp(const std::string& ip, uint16_t port, std::string& error) {
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    error = "创建 UDP socket 失败";
    return {};
  }
  const int reuse = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  const int buffer = 8 * 1024 * 1024;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &buffer, sizeof(buffer));

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  if (ip.empty() || ip == "0.0.0.0") {
    address.sin_addr.s_addr = htonl(INADDR_ANY);
  } else if (::inet_pton(AF_INET, ip.c_str(), &address.sin_addr) != 1) {
    ::close(fd);
    error = "本机 IP 无效: " + ip;
    return {};
  }
  if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
    error = "绑定 " + (ip.empty() ? std::string("0.0.0.0") : ip) + ":" + std::to_string(port) +
            " 失败: " + std::strerror(errno);
    ::close(fd);
    return {};
  }
  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags >= 0) {
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  }
  Socket socket;
  socket.reset(fd);
  return socket;
}

bool ipIsLocal(const std::string& ip) {
  ifaddrs* interfaces = nullptr;
  if (::getifaddrs(&interfaces) != 0) {
    return false;
  }
  bool found = false;
  for (ifaddrs* item = interfaces; item != nullptr; item = item->ifa_next) {
    if (item->ifa_addr == nullptr || item->ifa_addr->sa_family != AF_INET) {
      continue;
    }
    char text[INET_ADDRSTRLEN] = {};
    const auto* address = reinterpret_cast<sockaddr_in*>(item->ifa_addr);
    if (::inet_ntop(AF_INET, &address->sin_addr, text, sizeof(text)) != nullptr && ip == text) {
      found = true;
      break;
    }
  }
  ::freeifaddrs(interfaces);
  return found;
}

bool looksLikeWireOrder(const uint8_t raw[4]) {
  if (raw[0] == 192 && raw[1] == 168) {
    return true;
  }
  if (raw[0] == 10) {
    return true;
  }
  return raw[0] == 172 && raw[1] >= 16 && raw[1] <= 31;
}

bool looksLikeLittleEndian(const uint8_t raw[4]) {
  if (raw[3] == 192 && raw[2] == 168) {
    return true;
  }
  if (raw[3] == 10) {
    return true;
  }
  return raw[3] == 172 && raw[2] >= 16 && raw[2] <= 31;
}

std::string asciiPrefix(const std::vector<uint8_t>& data, size_t size) {
  size_t count = std::min(size, data.size());
  size_t end = 0;
  while (end < count && data[end] >= 32 && data[end] < 127) {
    ++end;
  }
  return std::string(reinterpret_cast<const char*>(data.data()), end);
}

uint64_t unixNowNs() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::system_clock::now().time_since_epoch())
                                   .count());
}

uint64_t packetTimeNs(const DataPacket& packet) {
  if ((packet.time_type == 1 || packet.time_type == 2) && packet.timestamp > 1000000000000000000ULL &&
      packet.timestamp < 4000000000000000000ULL) {
    return packet.timestamp;
  }
  return unixNowNs();
}

}  // namespace

struct LidarSession::Impl {
  Options options;
  FovPlan fov;
  std::string summary = "未连接";
  std::string discovered_sn;
  in_addr_t lidar_addr = 0;
  std::atomic<bool> running{false};
  std::atomic<bool> sampling_started{false};
  std::thread thread;
  Socket point_fd;
  Socket imu_fd;
  Socket cmd_fd;
  uint16_t seq = 1;

  std::mutex mutex;
  std::deque<CloudPoint> points;
  std::deque<ImuSample> imu;
  CompatibilityStream compatibility;
  static constexpr size_t kPointCap = 250000;

  std::atomic<uint64_t> point_packets{0};
  std::atomic<uint64_t> imu_packets{0};
  std::atomic<uint64_t> points_kept{0};
  std::atomic<uint64_t> points_fill{0};
  std::atomic<uint64_t> points_fov{0};
  std::atomic<uint64_t> points_range{0};
  std::atomic<uint64_t> points_queue_drop{0};
  std::atomic<uint64_t> lost_point_packets{0};
  std::atomic<uint64_t> lost_imu_packets{0};
  uint16_t last_point_udp = 0;
  uint16_t last_imu_udp = 0;
  bool have_point_udp = false;
  bool have_imu_udp = false;
  bool logged_point = false;
  bool logged_imu = false;

  void resetCounters() {
    point_packets = 0;
    imu_packets = 0;
    points_kept = 0;
    points_fill = 0;
    points_fov = 0;
    points_range = 0;
    points_queue_drop = 0;
    lost_point_packets = 0;
    lost_imu_packets = 0;
    have_point_udp = false;
    have_imu_udp = false;
    logged_point = false;
    logged_imu = false;
    std::lock_guard<std::mutex> lock(mutex);
    points.clear();
    imu.clear();
  }

  bool request(uint8_t id, const uint8_t* data, size_t size, std::vector<uint8_t>& reply, int timeout_ms,
               int attempts) {
    if (!cmd_fd.valid()) {
      return false;
    }
    for (int attempt = 0; attempt < attempts; ++attempt) {
      const uint16_t seq_now = seq++;
      const std::vector<uint8_t> frame = encodeFrame(id, data, size, seq_now);
      if (frame.empty()) {
        return false;
      }
      sockaddr_in dest{};
      dest.sin_family = AF_INET;
      dest.sin_port = htons(options.cmd_port);
      dest.sin_addr.s_addr = lidar_addr;
      const ssize_t sent = ::sendto(cmd_fd.get(), frame.data(), frame.size(), 0,
                                    reinterpret_cast<sockaddr*>(&dest), sizeof(dest));
      if (sent != static_cast<ssize_t>(frame.size())) {
        continue;
      }
      const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
      while (Clock::now() < deadline) {
        pollfd pfd{};
        pfd.fd = cmd_fd.get();
        pfd.events = POLLIN;
        const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
        const int waited = ::poll(&pfd, 1, static_cast<int>(left < 0 ? 0 : left));
        if (waited < 0) {
          if (errno == EINTR) {
            continue;
          }
          break;
        }
        if (waited == 0) {
          break;
        }
        uint8_t buffer[4096];
        sockaddr_in source{};
        socklen_t source_size = sizeof(source);
        const ssize_t got = ::recvfrom(cmd_fd.get(), buffer, sizeof(buffer), 0,
                                       reinterpret_cast<sockaddr*>(&source), &source_size);
        if (got < 0) {
          if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
            continue;
          }
          break;
        }
        if (source.sin_addr.s_addr != lidar_addr) {
          continue;
        }
        for (const Frame& item : decodeFrames(buffer, static_cast<size_t>(got))) {
          if (item.ack && item.receiver == kHostAddress && item.sender == kDeviceAddress && item.seq == seq_now &&
              item.id == id && item.set == kCmdSet) {
            reply = item.data;
            return true;
          }
        }
      }
    }
    return false;
  }

  bool readParam(uint32_t uid, uint32_t length, std::vector<uint8_t>& value, int timeout_ms = 500,
                 int attempts = 2) {
    const std::vector<uint8_t> uid_bytes = le32(uid);
    const std::vector<uint8_t> length_bytes = le32(length);
    uint8_t payload[8];
    std::memcpy(payload, uid_bytes.data(), 4);
    std::memcpy(payload + 4, length_bytes.data(), 4);
    std::vector<uint8_t> reply;
    if (!request(kCmdReadParam, payload, sizeof(payload), reply, timeout_ms, attempts)) {
      return false;
    }
    return parseReadParamReply(reply, uid, length, value);
  }

  bool writeParam(uint32_t uid, const std::vector<uint8_t>& value, int timeout_ms = 500, int attempts = 2) {
    std::vector<uint8_t> payload = le32(uid);
    const std::vector<uint8_t> length_bytes = le32(static_cast<uint32_t>(value.size()));
    payload.insert(payload.end(), length_bytes.begin(), length_bytes.end());
    payload.insert(payload.end(), value.begin(), value.end());
    std::vector<uint8_t> reply;
    if (!request(kCmdWriteParam, payload.data(), payload.size(), reply, timeout_ms, attempts)) {
      return false;
    }
    return !reply.empty() && reply[0] == 0;
  }

  bool writeVerified(uint32_t uid, const std::vector<uint8_t>& value) {
    if (!writeParam(uid, value)) {
      return false;
    }
    std::vector<uint8_t> actual;
    return readParam(uid, static_cast<uint32_t>(value.size()), actual) && actual == value;
  }

  bool readState(uint8_t& state, int timeout_ms = 500, int attempts = 2) {
    const uint8_t payload[6] = {1, 0, 3, 0, 0, 0};
    std::vector<uint8_t> reply;
    if (!request(kCmdReadState, payload, sizeof(payload), reply, timeout_ms, attempts)) {
      return false;
    }
    return parseStateReply(reply, state);
  }

  bool setState(uint8_t state, int timeout_ms = 500, int attempts = 2) {
    const uint8_t payload[9] = {1, 0, 3, 0, 0, 0, 1, 0, state};
    std::vector<uint8_t> reply;
    if (!request(kCmdSetState, payload, sizeof(payload), reply, timeout_ms, attempts)) {
      return false;
    }
    return !reply.empty() && reply[0] == 0;
  }

  bool enableSampling(bool enable, int timeout_ms = 500, int attempts = 2) {
    const uint8_t payload[1] = {static_cast<uint8_t>(enable ? 1 : 0)};
    std::vector<uint8_t> reply;
    if (!request(kCmdSample, payload, sizeof(payload), reply, timeout_ms, attempts)) {
      return false;
    }
    return !reply.empty() && reply[0] == 0;
  }

  bool applyFov(std::string& note) {
    struct Backup {
      uint32_t uid;
      std::vector<uint8_t> value;
    };
    std::vector<Backup> backups;
    const auto remember = [&](uint32_t uid, uint32_t length) {
      std::vector<uint8_t> value;
      if (!readParam(uid, length, value)) {
        return false;
      }
      backups.push_back(Backup{uid, value});
      return true;
    };
    if (!remember(kUidRoiEnable, 1)) {
      note = "读取视场使能失败";
      return false;
    }
    std::vector<uint8_t> pitch0_start;
    std::vector<uint8_t> pitch0_stop;
    if (!fov.full) {
      if (!remember(kUidYaw0Start, 4) || !remember(kUidYaw0Stop, 4)) {
        note = "读取窗口 0 航向失败";
        return false;
      }
      if (fov.wrap) {
        if (!readParam(kUidPitch0Start, 4, pitch0_start) || !readParam(kUidPitch0Stop, 4, pitch0_stop) ||
            !remember(kUidYaw1Start, 4) || !remember(kUidYaw1Stop, 4) || !remember(kUidPitch1Start, 4) ||
            !remember(kUidPitch1Stop, 4)) {
          note = "读取跨 0° 窗口失败";
          return false;
        }
      }
    }

    bool wrote = false;
    const auto restore = [&]() {
      writeParam(kUidRoiEnable, std::vector<uint8_t>{0}, 300, 1);
      for (auto it = backups.rbegin(); it != backups.rend(); ++it) {
        if (it->uid != kUidRoiEnable) {
          writeParam(it->uid, it->value, 300, 1);
        }
      }
      for (const Backup& item : backups) {
        if (item.uid == kUidRoiEnable) {
          writeParam(item.uid, item.value, 300, 1);
        }
      }
    };
    const auto fail = [&](const std::string& why) {
      if (wrote) {
        restore();
      }
      note = why + (wrote ? "，已尝试恢复原视场" : "");
      return false;
    };

    wrote = true;
    if (!writeVerified(kUidRoiEnable, std::vector<uint8_t>{0})) {
      return fail("关闭视场使能失败");
    }
    if (!fov.full) {
      const std::vector<uint8_t> yaw0_start = le32(static_cast<uint32_t>(fov.yaw0_start));
      const std::vector<uint8_t> yaw0_stop = le32(static_cast<uint32_t>(fov.yaw0_stop));
      if (!writeVerified(kUidYaw0Start, yaw0_start) || !writeVerified(kUidYaw0Stop, yaw0_stop)) {
        return fail("写入窗口 0 航向失败");
      }
      if (fov.wrap) {
        if (!writeVerified(kUidYaw1Start, le32(static_cast<uint32_t>(fov.yaw1_start))) ||
            !writeVerified(kUidYaw1Stop, le32(static_cast<uint32_t>(fov.yaw1_stop))) ||
            !writeVerified(kUidPitch1Start, pitch0_start) || !writeVerified(kUidPitch1Stop, pitch0_stop)) {
          return fail("写入跨 0° 窗口失败");
        }
      }
      if (!writeVerified(kUidRoiEnable, std::vector<uint8_t>{static_cast<uint8_t>(fov.enable)})) {
        return fail("打开视场窗口失败");
      }
    }

    std::vector<uint8_t> enable;
    std::vector<uint8_t> yaw_start;
    std::vector<uint8_t> yaw_stop;
    std::vector<uint8_t> pitch_start;
    std::vector<uint8_t> pitch_stop;
    if (!readParam(kUidRoiEnable, 1, enable) || enable[0] != static_cast<uint8_t>(fov.enable)) {
      return fail("视场使能回读不一致");
    }
    note = fov.text;
    if (readParam(kUidYaw0Start, 4, yaw_start) && readParam(kUidYaw0Stop, 4, yaw_stop) &&
        readParam(kUidPitch0Start, 4, pitch_start) && readParam(kUidPitch0Stop, 4, pitch_stop)) {
      note += "，回读 enable=" + std::to_string(enable[0]) + " 航向 " +
              std::to_string(readLeI32(yaw_start.data())) + "°–" + std::to_string(readLeI32(yaw_stop.data())) +
              "° 俯仰 " + std::to_string(readLeI32(pitch_start.data())) + "°–" +
              std::to_string(readLeI32(pitch_stop.data())) + "°";
    }
    return true;
  }

  void logDestination() {
    std::vector<uint8_t> point_ip;
    std::vector<uint8_t> point_port;
    std::vector<uint8_t> imu_ip;
    std::vector<uint8_t> imu_port;
    if (!readParam(kUidPointIp, 4, point_ip) || !readParam(kUidPointPort, 2, point_port)) {
      logLine("没有读到点云目的地址");
      return;
    }
    std::string text = "点云目的 " + describeIpv4(point_ip.data()) + " 端口 " +
                       std::to_string(readLe16(point_port.data()));
    if (readParam(kUidImuIp, 4, imu_ip) && readParam(kUidImuPort, 2, imu_port)) {
      text += "；IMU 目的 " + describeIpv4(imu_ip.data()) + " 端口 " + std::to_string(readLe16(imu_port.data()));
    }
    logLine(text);

    if (!options.configure_dest) {
      return;
    }
    if (options.host_ip.empty() || options.host_ip == "0.0.0.0" || !ipIsLocal(options.host_ip)) {
      logLine("configure_dest 已打开，但 host_ip 不是本机地址，未改目的 IP");
      return;
    }
    uint8_t octets[4] = {};
    if (!parseIpv4(options.host_ip, octets)) {
      logLine("host_ip 无法解析，未改目的 IP");
      return;
    }
    bool little_endian = true;
    const bool wire = looksLikeWireOrder(point_ip.data());
    const bool little = looksLikeLittleEndian(point_ip.data());
    if (wire && !little) {
      little_endian = false;
    } else if (little && !wire) {
      little_endian = true;
    } else {
      logLine("无法从当前目的地址判断 IP 字节序，按协议写成小端 u32");
    }
    uint8_t raw[4];
    storeIpv4(raw, octets, little_endian);
    const std::vector<uint8_t> ip_bytes(raw, raw + 4);
    if (!writeVerified(kUidPointIp, ip_bytes) || !writeVerified(kUidPointPort, le16(options.point_port)) ||
        !writeVerified(kUidImuIp, ip_bytes) || !writeVerified(kUidImuPort, le16(options.imu_port))) {
      logLine("写入点云/IMU 目的地址失败");
      return;
    }
    logLine(std::string("已把点云和 IMU 目的改到 ") + options.host_ip + ":" + std::to_string(options.point_port) +
            " / " + std::to_string(options.imu_port));
  }

  bool startSampling(std::string& note) {
    if (!setState(1) || !enableSampling(true)) {
      note = "打开采样失败";
      return false;
    }
    uint8_t state = 0;
    if (!readState(state) || state != 1) {
      note = "采样状态回读不是采样";
      return false;
    }
    sampling_started.store(true);
    note = "采样已打开";
    return true;
  }

  bool standby() {
    const bool sample_off = enableSampling(false, 300, 1);
    const bool state_off = setState(2, 300, 1);
    uint8_t state = 0;
    const bool read_ok = readState(state, 300, 1);
    sampling_started.store(false);
    return sample_off && state_off && read_ok && state == 2;
  }

  void discover() {
    if (options.discover_sec <= 0) {
      return;
    }
    std::string error;
    Socket socket = bindUdp("0.0.0.0", options.discover_port, error);
    if (!socket.valid()) {
      logLine("设备发现跳过: " + error);
      return;
    }
    const auto deadline =
        Clock::now() + std::chrono::milliseconds(static_cast<int>(options.discover_sec * 1000.0));
    std::set<std::string> seen;
    while (Clock::now() < deadline) {
      pollfd pfd{};
      pfd.fd = socket.get();
      pfd.events = POLLIN;
      const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
      if (::poll(&pfd, 1, static_cast<int>(left < 0 ? 0 : left)) <= 0) {
        continue;
      }
      uint8_t buffer[2048];
      sockaddr_in source{};
      socklen_t source_size = sizeof(source);
      const ssize_t got = ::recvfrom(socket.get(), buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr*>(&source),
                                    &source_size);
      if (got < 0) {
        continue;
      }
      char ip_text[INET_ADDRSTRLEN] = {};
      if (::inet_ntop(AF_INET, &source.sin_addr, ip_text, sizeof(ip_text)) == nullptr) {
        continue;
      }
      for (const Frame& frame : decodeFrames(buffer, static_cast<size_t>(got))) {
        if (frame.sender != kDeviceAddress || frame.set != kCmdSet || frame.id != kCmdDiscover) {
          continue;
        }
        if (!seen.insert(ip_text).second) {
          continue;
        }
        discovered_sn = asciiPrefix(frame.data, 14);
        logLine(std::string("发现 ") + ip_text + " SN " + discovered_sn);
        if (options.lidar_ip != ip_text) {
          logLine("发现地址和 lidar_ip 不同，仍连接 " + options.lidar_ip);
        }
      }
    }
  }

  void configure() {
    std::string fov_note = fov.valid ? fov.text : "视场参数无效，发布端不裁剪";
    std::string sample_note = "未改采样";
    if (!cmd_fd.valid()) {
      summary = "被动接收 " + options.lidar_ip + "，" + fov_note;
      return;
    }
    std::vector<uint8_t> probe;
    if (!readParam(kUidRoiEnable, 1, probe)) {
      logLine("控制口无应答，改为只接收。确认雷达 IP、网线，以及 UDP " + std::to_string(options.cmd_port));
      summary = "被动接收 " + options.lidar_ip + "，" + fov_note;
      return;
    }
    if (fov.valid) {
      std::string note;
      if (!applyFov(note)) {
        logLine(note);
        fov_note += "（设备写入失败，发布端仍按该角度裁剪）";
      } else {
        fov_note = note;
        logLine("视场 " + note);
      }
    }
    logDestination();
    if (options.auto_sampling) {
      if (!startSampling(sample_note)) {
        logLine(sample_note);
      } else {
        logLine(sample_note);
      }
    }
    std::ostringstream text;
    text << "雷达 " << options.lidar_ip << "  " << fov_note << "  " << sample_note;
    if (!discovered_sn.empty()) {
      text << "  SN " << discovered_sn;
    }
    summary = text.str();
  }

  void noteLoss(uint16_t count, uint16_t& last, bool& have, std::atomic<uint64_t>& lost) {
    if (have) {
      const uint16_t delta = static_cast<uint16_t>(count - last);
      if (delta > 1 && delta < 1000) {
        lost += static_cast<uint64_t>(delta - 1);
      }
    }
    last = count;
    have = true;
  }

  void handleDatagram(const uint8_t* data, size_t size, uint64_t host_ns, Clock::time_point received) {
    const DataPacket packet = parseDataPacket(data, size);
    if (!packet.ok) {
      return;
    }
    const bool compatible = options.timestamp_source != TimestampSource::Native;
    const bool dual = packet.data_type == 0x11;
    const uint32_t interval = packet.dot_num == 0 ? 0 : static_cast<uint32_t>(packet.time_interval) * 100u / packet.dot_num;
    const uint64_t span = packet.points.empty() ? 0 :
        (packet.points.size() - 1) / (dual ? 2 : 1) * static_cast<uint64_t>(interval);
    uint64_t base = 0;
    if (compatible) {
      std::lock_guard<std::mutex> lock(mutex);
      base = compatibility.packetTime(packet, host_ns, received, span);
      if (!compatibility.stats.clock_fault.empty()) {
        compatibility.stats.dropped_imu_samples += imu.size();
        imu.clear();
      }
    } else {
      base = packetTimeNs(packet);
    }
    if (packet.has_imu) {
      noteLoss(packet.udp_cnt, last_imu_udp, have_imu_udp, lost_imu_packets);
      ++imu_packets;
      if (!logged_imu) {
        logged_imu = true;
        logLine("IMU 首包 " + std::to_string(size) + " B");
      }
      ImuSample sample;
      sample.time_ns = base;
      sample.gyro_x = packet.imu.gyro[0];
      sample.gyro_y = packet.imu.gyro[1];
      sample.gyro_z = packet.imu.gyro[2];
      sample.acc_x = packet.imu.acc[0];
      sample.acc_y = packet.imu.acc[1];
      sample.acc_z = packet.imu.acc[2];
      std::lock_guard<std::mutex> lock(mutex);
      if (compatible) {
        const bool finite = compatibility.acceptImu(sample);
        if (!finite || base == 0) {
          return;
        }
      }
      if (imu.size() >= CompatibilityStream::kImuCapacity) {
        imu.pop_front();
        if (compatible) {
          ++compatibility.stats.dropped_imu_samples;
        }
      }
      imu.push_back(sample);
      return;
    }

    noteLoss(packet.udp_cnt, last_point_udp, have_point_udp, lost_point_packets);
    ++point_packets;
    if (!logged_point) {
      logged_point = true;
      logLine("点云首包 " + std::to_string(size) + " B，类型 " + std::to_string(packet.data_type) + "，点数 " +
              std::to_string(packet.points.size()));
    }
    if (compatible && base == 0) {
      return;
    }
    const double max_range = options.max_range_m;
    std::vector<CloudPoint> accepted;
    accepted.reserve(packet.points.size());
    for (size_t i = 0; i < packet.points.size(); ++i) {
      const RawPoint& raw = packet.points[i];
      if (raw.zero && !options.publish_fill) {
        ++points_fill;
        continue;
      }
      CloudPoint point;
      point.x = static_cast<float>(raw.x_mm) / 1000.f;
      point.y = static_cast<float>(raw.y_mm) / 1000.f;
      point.z = static_cast<float>(raw.z_mm) / 1000.f;
      point.reflectivity = raw.reflectivity;
      point.tag = raw.tag;
      point.line = static_cast<uint8_t>(i % static_cast<size_t>(options.line_num));
      const size_t dot = dual ? i / 2 : i;
      point.time_ns = base + static_cast<uint64_t>(dot) * interval;
      if (max_range > 0.0) {
        const float range2 = point.x * point.x + point.y * point.y + point.z * point.z;
        if (range2 > static_cast<float>(max_range * max_range)) {
          ++points_range;
          continue;
        }
      }
      if (!yawInPlan(point.x, point.y, fov)) {
        ++points_fov;
        continue;
      }
      accepted.push_back(point);
    }
    if (accepted.empty() && !compatible) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex);
    if (compatible) {
      points_kept += accepted.size();
      compatibility.addPoints(accepted, base + span, received);
      return;
    }
    for (const CloudPoint& point : accepted) {
      if (points.size() >= kPointCap) {
        points.pop_front();
        ++points_queue_drop;
      }
      points.push_back(point);
      ++points_kept;
    }
  }

  void drain(int fd) {
    uint8_t buffer[2048];
    while (running.load()) {
      sockaddr_in source{};
      socklen_t source_size = sizeof(source);
      const ssize_t got = ::recvfrom(fd, buffer, sizeof(buffer), MSG_DONTWAIT, reinterpret_cast<sockaddr*>(&source),
                                    &source_size);
      if (got < 0) {
        if (errno == EINTR) {
          continue;
        }
        return;
      }
      if (got == 0 || source.sin_addr.s_addr != lidar_addr) {
        if (got == 0) {
          return;
        }
        continue;
      }
      if (options.timestamp_source == TimestampSource::Native) {
        handleDatagram(buffer, static_cast<size_t>(got), 0, {});
      } else {
        const auto received = Clock::now();
        handleDatagram(buffer, static_cast<size_t>(got), unixNowNs(), received);
      }
    }
  }

  void loop() {
    while (running.load()) {
      pollfd fds[2] = {};
      fds[0].fd = point_fd.get();
      fds[0].events = POLLIN;
      fds[1].fd = imu_fd.get();
      fds[1].events = POLLIN;
      const int waited = ::poll(fds, 2, 200);
      if (waited < 0) {
        if (errno == EINTR) {
          continue;
        }
        return;
      }
      if (waited == 0) {
        continue;
      }
      if ((fds[0].revents & POLLIN) != 0) {
        drain(point_fd.get());
      }
      if ((fds[1].revents & POLLIN) != 0) {
        drain(imu_fd.get());
      }
    }
  }
};

LidarSession::LidarSession() : impl_(std::make_unique<Impl>()) {}

LidarSession::~LidarSession() { stop(); }

bool LidarSession::start(const Options& options, std::string& error) {
  stop();
  impl_->options = options;
  if (impl_->options.line_num < 1) {
    impl_->options.line_num = 1;
  } else if (impl_->options.line_num > 64) {
    impl_->options.line_num = 64;
  }
  if (impl_->options.max_range_m < 0.0) {
    impl_->options.max_range_m = 0.0;
  }
  if (impl_->options.discover_sec > 10.0) {
    impl_->options.discover_sec = 10.0;
  }
  uint8_t lidar_octets[4] = {};
  if (!parseIpv4(impl_->options.lidar_ip, lidar_octets) ||
      ::inet_pton(AF_INET, impl_->options.lidar_ip.c_str(), &impl_->lidar_addr) != 1) {
    error = "雷达 IP 无效: " + impl_->options.lidar_ip;
    return false;
  }

  std::string fov_error;
  if (!planFov(impl_->options.fov_start_deg, impl_->options.fov_stop_deg, impl_->fov, fov_error)) {
    logLine(fov_error);
    impl_->fov = FovPlan();
  }

  std::string bind_ip = impl_->options.host_ip;
  if (bind_ip == "0.0.0.0") {
    bind_ip.clear();
  }
  if (!bind_ip.empty() && !ipIsLocal(bind_ip)) {
    logLine("本机没有 " + bind_ip + "，改为监听 0.0.0.0");
    bind_ip.clear();
  }

  impl_->point_fd = bindUdp(bind_ip, impl_->options.point_port, error);
  if (!impl_->point_fd.valid()) {
    return false;
  }
  impl_->imu_fd = bindUdp(bind_ip, impl_->options.imu_port, error);
  if (!impl_->imu_fd.valid()) {
    impl_->point_fd.reset();
    return false;
  }
  std::string cmd_error;
  impl_->cmd_fd = bindUdp(bind_ip, 0, cmd_error);
  if (!impl_->cmd_fd.valid()) {
    logLine("控制口绑定失败，只接收数据: " + cmd_error);
  }
  logLine("监听 " + (bind_ip.empty() ? std::string("0.0.0.0") : bind_ip) + " 点云 UDP " +
          std::to_string(impl_->options.point_port) + "，IMU UDP " + std::to_string(impl_->options.imu_port) +
          "，雷达 " + impl_->options.lidar_ip + ":" + std::to_string(impl_->options.cmd_port));

  impl_->resetCounters();
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->points.clear();
    impl_->imu.clear();
    impl_->compatibility = CompatibilityStream(options.timestamp_source, options.cloud_period_ns);
  }
  impl_->running.store(true);
  impl_->thread = std::thread([this]() { impl_->loop(); });
  impl_->discover();
  impl_->configure();
  logLine(impl_->summary);
  return true;
}

void LidarSession::stop() {
  if (!impl_) {
    return;
  }
  impl_->running.store(false);
  if (impl_->thread.joinable()) {
    impl_->thread.join();
  }
  if (impl_->sampling_started.load() && impl_->options.standby_on_exit) {
    if (impl_->standby()) {
      logLine("已切换到待机");
    } else if (impl_->cmd_fd.valid()) {
      logLine("退出时切换待机失败");
    }
  }
  impl_->point_fd.reset();
  impl_->imu_fd.reset();
  impl_->cmd_fd.reset();
}

void LidarSession::takePoints(std::vector<CloudPoint>& points) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  points.assign(impl_->points.begin(), impl_->points.end());
  impl_->points.clear();
}

void LidarSession::takeImu(std::vector<ImuSample>& samples) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  samples.assign(impl_->imu.begin(), impl_->imu.end());
  impl_->imu.clear();
}

void LidarSession::takeFrames(std::vector<std::vector<CloudPoint>>& frames) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->compatibility.takeFrames(frames);
}

CompatibilityStats LidarSession::compatibilityStats() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->compatibility.stats;
}

Stats LidarSession::stats() const {
  Stats stats;
  stats.point_packets = impl_->point_packets.load();
  stats.imu_packets = impl_->imu_packets.load();
  stats.points_kept = impl_->points_kept.load();
  stats.points_fill = impl_->points_fill.load();
  stats.points_fov = impl_->points_fov.load();
  stats.points_range = impl_->points_range.load();
  stats.points_queue_drop = impl_->points_queue_drop.load();
  stats.lost_point_packets = impl_->lost_point_packets.load();
  stats.lost_imu_packets = impl_->lost_imu_packets.load();
  return stats;
}

std::string LidarSession::summary() const { return impl_->summary; }

}  // namespace mid360_agras
