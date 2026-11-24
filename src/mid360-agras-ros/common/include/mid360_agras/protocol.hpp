#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace mid360_agras {

// Agras MID360 control frame. This is not the retail Livox 0xAA protocol.
constexpr uint8_t kHostAddress = 0x28;
constexpr uint8_t kDeviceAddress = 0xAD;
constexpr uint8_t kCmdSet = 0x32;
constexpr uint8_t kCmdReadParam = 0x12;
constexpr uint8_t kCmdWriteParam = 0x13;
constexpr uint8_t kCmdSample = 0x14;
constexpr uint8_t kCmdDiscover = 0x21;
constexpr uint8_t kCmdSetState = 0x50;
constexpr uint8_t kCmdReadState = 0x51;

constexpr uint32_t kUidRoiEnable = 0x00010023;
constexpr uint32_t kUidYaw0Start = 0x00010024;
constexpr uint32_t kUidYaw0Stop = 0x00010025;
constexpr uint32_t kUidPitch0Start = 0x00010026;
constexpr uint32_t kUidPitch0Stop = 0x00010027;
constexpr uint32_t kUidYaw1Start = 0x00010029;
constexpr uint32_t kUidYaw1Stop = 0x0001002A;
constexpr uint32_t kUidPitch1Start = 0x0001002B;
constexpr uint32_t kUidPitch1Stop = 0x0001002C;
constexpr uint32_t kUidPointIp = 0x0001001B;
constexpr uint32_t kUidPointPort = 0x0001001C;
constexpr uint32_t kUidImuIp = 0x0001001D;
constexpr uint32_t kUidImuPort = 0x0001001E;

constexpr int kLineNumberMid360 = 4;

struct Frame {
  uint8_t sender = 0;
  uint8_t receiver = 0;
  uint16_t seq = 0;
  bool ack = false;
  uint8_t set = 0;
  uint8_t id = 0;
  std::vector<uint8_t> data;
};

std::vector<uint8_t> encodeFrame(uint8_t id, const uint8_t* data, size_t size, uint16_t seq,
                                 uint8_t set = kCmdSet, uint8_t flags = 0x40,
                                 uint8_t sender = kHostAddress, uint8_t receiver = kDeviceAddress);

std::vector<Frame> decodeFrames(const uint8_t* data, size_t size);

struct RawPoint {
  int32_t x_mm = 0;
  int32_t y_mm = 0;
  int32_t z_mm = 0;
  uint8_t reflectivity = 0;
  uint8_t tag = 0;
  bool zero = false;
};

struct ImuRaw {
  float gyro[3] = {0.f, 0.f, 0.f};
  float acc[3] = {0.f, 0.f, 0.f};
};

struct DataPacket {
  bool ok = false;
  uint8_t version = 0;
  uint16_t length = 0;
  uint16_t time_interval = 0;
  uint16_t dot_num = 0;
  uint16_t udp_cnt = 0;
  uint8_t frame_cnt = 0;
  uint8_t data_type = 0;
  uint8_t time_type = 0;
  uint64_t timestamp = 0;
  std::vector<RawPoint> points;
  bool has_imu = false;
  ImuRaw imu;
};

DataPacket parseDataPacket(const uint8_t* data, size_t size);

bool parseReadParamReply(const std::vector<uint8_t>& reply, uint32_t uid, uint32_t length,
                         std::vector<uint8_t>& value);
bool parseStateReply(const std::vector<uint8_t>& reply, uint8_t& state);

struct FovPlan {
  bool valid = false;
  bool full = false;
  bool wrap = false;
  int enable = 0;
  int yaw0_start = 0;
  int yaw0_stop = 0;
  int yaw1_start = 0;
  int yaw1_stop = 0;
  std::string text;
};

// 0 and 360 is a full turn (ROI enable = 0). Any other window is written to the radar.
bool planFov(int start_deg, int stop_deg, FovPlan& plan, std::string& error);
bool yawInPlan(double x, double y, const FovPlan& plan);

bool parseIpv4(const std::string& text, uint8_t octets[4]);
void storeIpv4(uint8_t out[4], const uint8_t octets[4], bool little_endian_u32);
std::string describeIpv4(const uint8_t raw[4]);

std::vector<uint8_t> le32(uint32_t value);
std::vector<uint8_t> le16(uint16_t value);
uint16_t readLe16(const uint8_t* data);
uint32_t readLe32(const uint8_t* data);
int32_t readLeI32(const uint8_t* data);

}  // namespace mid360_agras
