#include "mid360_agras/protocol.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>

namespace mid360_agras {
namespace {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

uint16_t reflectedCrc(const uint8_t* data, size_t size, uint16_t init, uint16_t poly) {
  uint16_t value = init;
  for (size_t i = 0; i < size; ++i) {
    value = static_cast<uint16_t>(value ^ data[i]);
    for (int bit = 0; bit < 8; ++bit) {
      if (value & 1) {
        value = static_cast<uint16_t>((value >> 1) ^ poly);
      } else {
        value = static_cast<uint16_t>(value >> 1);
      }
    }
  }
  return value;
}

uint8_t crc8(const uint8_t* data, size_t size) {
  return static_cast<uint8_t>(reflectedCrc(data, size, 0x77, 0x8C));
}

uint16_t crc16(const uint8_t* data, size_t size) {
  return reflectedCrc(data, size, 0x3692, 0x8408);
}

uint64_t readLe64(const uint8_t* data) {
  uint64_t value = 0;
  for (int i = 7; i >= 0; --i) {
    value = (value << 8) | data[i];
  }
  return value;
}

float readF32(const uint8_t* data) {
  const uint32_t word = readLe32(data);
  float value = 0.f;
  std::memcpy(&value, &word, sizeof(value));
  return value;
}

int16_t readLeI16(const uint8_t* data) {
  return static_cast<int16_t>(readLe16(data));
}

RawPoint cartesian(const uint8_t* sample, int width) {
  RawPoint point;
  if (width >= 14) {
    point.x_mm = readLeI32(sample);
    point.y_mm = readLeI32(sample + 4);
    point.z_mm = readLeI32(sample + 8);
    point.reflectivity = sample[12];
    point.tag = sample[13];
  } else {
    point.x_mm = readLeI16(sample) * 10;
    point.y_mm = readLeI16(sample + 2) * 10;
    point.z_mm = readLeI16(sample + 4) * 10;
    point.reflectivity = sample[6];
    point.tag = sample[7];
  }
  point.zero = point.x_mm == 0 && point.y_mm == 0 && point.z_mm == 0;
  return point;
}

RawPoint spherical(const uint8_t* sample) {
  const uint32_t depth = readLe32(sample);
  const double theta = readLe16(sample + 4) * 0.01 * M_PI / 180.0;
  const double phi = readLe16(sample + 6) * 0.01 * M_PI / 180.0;
  RawPoint point;
  point.x_mm = static_cast<int32_t>(std::lround(depth * std::sin(theta) * std::cos(phi)));
  point.y_mm = static_cast<int32_t>(std::lround(depth * std::sin(theta) * std::sin(phi)));
  point.z_mm = static_cast<int32_t>(std::lround(depth * std::cos(theta)));
  point.reflectivity = sample[8];
  point.tag = sample[9];
  point.zero = depth == 0;
  return point;
}

}  // namespace

std::vector<uint8_t> le32(uint32_t value) {
  return {static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8),
          static_cast<uint8_t>(value >> 16), static_cast<uint8_t>(value >> 24)};
}

std::vector<uint8_t> le16(uint16_t value) {
  return {static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8)};
}

uint16_t readLe16(const uint8_t* data) {
  return static_cast<uint16_t>(data[0] | (static_cast<uint16_t>(data[1]) << 8));
}

uint32_t readLe32(const uint8_t* data) {
  return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
}

int32_t readLeI32(const uint8_t* data) {
  return static_cast<int32_t>(readLe32(data));
}

std::vector<uint8_t> encodeFrame(uint8_t id, const uint8_t* data, size_t size, uint16_t seq,
                                 uint8_t set, uint8_t flags, uint8_t sender, uint8_t receiver) {
  if (size > 1023 - 13) {
    return {};
  }
  std::vector<uint8_t> frame(13 + size, 0);
  frame[0] = 0x55;
  const uint16_t prefix = static_cast<uint16_t>(0x400 | frame.size());
  frame[1] = static_cast<uint8_t>(prefix);
  frame[2] = static_cast<uint8_t>(prefix >> 8);
  frame[3] = crc8(frame.data(), 3);
  frame[4] = sender;
  frame[5] = receiver;
  frame[6] = static_cast<uint8_t>(seq);
  frame[7] = static_cast<uint8_t>(seq >> 8);
  frame[8] = flags;
  frame[9] = set;
  frame[10] = id;
  if (size > 0 && data != nullptr) {
    std::memcpy(frame.data() + 11, data, size);
  }
  const uint16_t sum = crc16(frame.data(), frame.size() - 2);
  frame[frame.size() - 2] = static_cast<uint8_t>(sum);
  frame[frame.size() - 1] = static_cast<uint8_t>(sum >> 8);
  return frame;
}

std::vector<Frame> decodeFrames(const uint8_t* data, size_t size) {
  std::vector<Frame> frames;
  if (data == nullptr) {
    return frames;
  }
  for (size_t offset = 0; offset + 13 <= size;) {
    const uint16_t prefix = readLe16(data + offset + 1);
    const uint16_t length = static_cast<uint16_t>(prefix & 0x3FF);
    if (data[offset] != 0x55 || (prefix >> 10) != 1 || length < 13 || offset + length > size) {
      ++offset;
      continue;
    }
    if (crc8(data + offset, 3) != data[offset + 3] ||
        crc16(data + offset, length - 2) != readLe16(data + offset + length - 2)) {
      ++offset;
      continue;
    }
    Frame frame;
    frame.sender = data[offset + 4];
    frame.receiver = data[offset + 5];
    frame.seq = readLe16(data + offset + 6);
    frame.ack = (data[offset + 8] & 0x80) != 0;
    frame.set = data[offset + 9];
    frame.id = data[offset + 10];
    frame.data.assign(data + offset + 11, data + offset + length - 2);
    frames.push_back(std::move(frame));
    offset += length;
  }
  return frames;
}

DataPacket parseDataPacket(const uint8_t* data, size_t size) {
  DataPacket packet;
  if (data == nullptr || size < 36 || data[0] == 0xAA || data[0] == 0x55 || data[0] > 1) {
    return packet;
  }
  const uint16_t length = readLe16(data + 1);
  if (length != size || length < 36) {
    return packet;
  }
  packet.version = data[0];
  packet.length = length;
  packet.time_interval = readLe16(data + 3);
  packet.dot_num = readLe16(data + 5);
  packet.udp_cnt = readLe16(data + 7);
  packet.frame_cnt = data[9];
  packet.data_type = data[10];
  packet.time_type = data[11];
  packet.timestamp = readLe64(data + 28);

  const uint8_t* body = data + 36;
  const size_t body_size = static_cast<size_t>(length - 36);
  if (packet.data_type == 0) {
    if (body_size < 24) {
      return packet;
    }
    packet.has_imu = true;
    for (int i = 0; i < 3; ++i) {
      packet.imu.gyro[i] = readF32(body + 4 * i);
      packet.imu.acc[i] = readF32(body + 12 + 4 * i);
    }
    packet.ok = true;
    return packet;
  }

  int stride = 0;
  if (packet.data_type == 1) {
    stride = 14;
  } else if (packet.data_type == 2) {
    stride = 8;
  } else if (packet.data_type == 3) {
    stride = 10;
  } else if (packet.data_type == 0x11) {
    stride = 28;
  } else {
    return packet;
  }
  size_t count = stride == 0 ? 0 : body_size / static_cast<size_t>(stride);
  if (packet.dot_num < count) {
    count = packet.dot_num;
  }
  packet.points.reserve(count * (packet.data_type == 0x11 ? 2 : 1));
  for (size_t i = 0; i < count; ++i) {
    const uint8_t* sample = body + i * static_cast<size_t>(stride);
    if (packet.data_type == 0x11) {
      packet.points.push_back(cartesian(sample, 14));
      packet.points.push_back(cartesian(sample + 14, 14));
    } else if (packet.data_type == 3) {
      packet.points.push_back(spherical(sample));
    } else {
      packet.points.push_back(cartesian(sample, stride));
    }
  }
  packet.ok = true;
  return packet;
}

bool parseReadParamReply(const std::vector<uint8_t>& reply, uint32_t uid, uint32_t length,
                         std::vector<uint8_t>& value) {
  if (length > 256 || reply.size() != 9u + length || reply[0] != 0) {
    return false;
  }
  if (readLe32(reply.data() + 1) != uid || readLe16(reply.data() + 5) != length) {
    return false;
  }
  value.assign(reply.begin() + 9, reply.end());
  return true;
}

bool parseStateReply(const std::vector<uint8_t>& reply, uint8_t& state) {
  if (reply.size() != 14 || reply[0] != 0) {
    return false;
  }
  if (readLe32(reply.data() + 1) != 0 || readLe16(reply.data() + 5) != 1 ||
      readLe32(reply.data() + 7) != 3 || readLe16(reply.data() + 11) != 1) {
    return false;
  }
  state = reply[13];
  return true;
}

bool planFov(int start_deg, int stop_deg, FovPlan& plan, std::string& error) {
  plan = FovPlan();
  if (start_deg < 0 || start_deg > 359 || stop_deg < 0 || stop_deg > 360 || start_deg == stop_deg) {
    error = "视场角无效：起始角须为 0–359，结束角须为 0–360，且两者不能相同。全周请填 0 和 360。";
    return false;
  }
  if (start_deg == 0 && stop_deg == 360) {
    plan.valid = true;
    plan.full = true;
    plan.enable = 0;
    plan.text = "全周（0°–360°）";
    return true;
  }
  plan.valid = true;
  plan.wrap = start_deg > stop_deg && stop_deg != 0;
  plan.yaw0_start = start_deg;
  plan.yaw0_stop = (plan.wrap || stop_deg == 0) ? 360 : stop_deg;
  plan.enable = plan.wrap ? 3 : 1;
  if (plan.wrap) {
    plan.yaw1_start = 0;
    plan.yaw1_stop = stop_deg;
    plan.text = "航向 " + std::to_string(start_deg) + "°–" + std::to_string(stop_deg) + "°（跨过 0°）";
  } else if (stop_deg == 0) {
    plan.text = "航向 " + std::to_string(start_deg) + "°–360°";
  } else {
    plan.text = "航向 " + std::to_string(start_deg) + "°–" + std::to_string(stop_deg) + "°";
  }
  return true;
}

bool yawInPlan(double x, double y, const FovPlan& plan) {
  if (!plan.valid || plan.full || plan.enable == 0) {
    return true;
  }
  double deg = std::atan2(y, x) * 180.0 / M_PI;
  if (deg < 0.0) {
    deg += 360.0;
  }
  const auto inside = [&](int begin, int end) {
    if (end >= 360) {
      return deg + 1e-9 >= begin;
    }
    return deg + 1e-9 >= begin && deg <= static_cast<double>(end) + 1e-9;
  };
  if (plan.enable == 1) {
    return inside(plan.yaw0_start, plan.yaw0_stop);
  }
  if (plan.enable == 3) {
    return inside(plan.yaw0_start, plan.yaw0_stop) || inside(plan.yaw1_start, plan.yaw1_stop);
  }
  return true;
}

bool parseIpv4(const std::string& text, uint8_t octets[4]) {
  unsigned parts[4] = {0, 0, 0, 0};
  char extra = 0;
  if (std::sscanf(text.c_str(), "%u.%u.%u.%u%c", &parts[0], &parts[1], &parts[2], &parts[3], &extra) != 4) {
    return false;
  }
  for (unsigned part : parts) {
    if (part > 255) {
      return false;
    }
  }
  for (int i = 0; i < 4; ++i) {
    octets[i] = static_cast<uint8_t>(parts[i]);
  }
  return true;
}

void storeIpv4(uint8_t out[4], const uint8_t octets[4], bool little_endian_u32) {
  if (!little_endian_u32) {
    std::memcpy(out, octets, 4);
    return;
  }
  const uint32_t value = (static_cast<uint32_t>(octets[0]) << 24) | (static_cast<uint32_t>(octets[1]) << 16) |
                         (static_cast<uint32_t>(octets[2]) << 8) | octets[3];
  out[0] = static_cast<uint8_t>(value);
  out[1] = static_cast<uint8_t>(value >> 8);
  out[2] = static_cast<uint8_t>(value >> 16);
  out[3] = static_cast<uint8_t>(value >> 24);
}

std::string describeIpv4(const uint8_t raw[4]) {
  std::ostringstream text;
  text << std::hex;
  for (int i = 0; i < 4; ++i) {
    if (i) {
      text << ' ';
    }
    if (raw[i] < 16) {
      text << '0';
    }
    text << static_cast<int>(raw[i]);
  }
  text << std::dec << "  小端整数 " << static_cast<int>(raw[3]) << '.' << static_cast<int>(raw[2]) << '.'
       << static_cast<int>(raw[1]) << '.' << static_cast<int>(raw[0]) << "  字节序 " << static_cast<int>(raw[0])
       << '.' << static_cast<int>(raw[1]) << '.' << static_cast<int>(raw[2]) << '.' << static_cast<int>(raw[3]);
  return text.str();
}

}  // namespace mid360_agras
