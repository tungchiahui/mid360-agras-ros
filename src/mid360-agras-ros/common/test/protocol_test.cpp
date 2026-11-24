#include "mid360_agras/device.hpp"
#include "mid360_agras/pack.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace {

int failures = 0;

void expect(bool condition, const std::string& text) {
  if (!condition) {
    std::cerr << "FAIL " << text << "\n";
    ++failures;
  }
}

std::string hexOf(const std::vector<uint8_t>& data) {
  static const char* digits = "0123456789abcdef";
  std::string text;
  text.reserve(data.size() * 2);
  for (uint8_t byte : data) {
    text.push_back(digits[byte >> 4]);
    text.push_back(digits[byte & 0x0f]);
  }
  return text;
}

std::vector<uint8_t> unhex(const std::string& text) {
  std::vector<uint8_t> data;
  for (size_t i = 0; i + 1 < text.size(); i += 2) {
    data.push_back(static_cast<uint8_t>(std::stoi(text.substr(i, 2), nullptr, 16)));
  }
  return data;
}

void putLe16(std::vector<uint8_t>& data, size_t offset, uint16_t value) {
  data[offset] = static_cast<uint8_t>(value);
  data[offset + 1] = static_cast<uint8_t>(value >> 8);
}

void putLe32(std::vector<uint8_t>& data, size_t offset, uint32_t value) {
  for (int i = 0; i < 4; ++i) {
    data[offset + i] = static_cast<uint8_t>(value >> (8 * i));
  }
}

void putF32(std::vector<uint8_t>& data, size_t offset, float value) {
  uint32_t word = 0;
  std::memcpy(&word, &value, sizeof(word));
  putLe32(data, offset, word);
}

}  // namespace

int main() {
  using namespace mid360_agras;
  const std::vector<uint8_t> query = le32(kUidRoiEnable);
  const std::vector<uint8_t> length = le32(1);
  std::vector<uint8_t> payload = query;
  payload.insert(payload.end(), length.begin(), length.end());
  const std::vector<uint8_t> frame = encodeFrame(kCmdReadParam, payload.data(), payload.size(), 0x0111);
  expect(hexOf(frame) == "551504a928ad11014032122300010001000000e9d9", "Agras UID query wire format");

  const std::vector<uint8_t> ack = unhex("55170438ad281101803212002300010001000000005f99");
  const std::vector<Frame> replies = decodeFrames(ack.data(), ack.size());
  expect(replies.size() == 1 && replies[0].ack && replies[0].seq == 0x0111 && replies[0].id == kCmdReadParam, "ack");
  std::vector<uint8_t> value;
  expect(parseReadParamReply(replies[0].data, kUidRoiEnable, 1, value) && value.size() == 1 && value[0] == 0,
         "ROI readback value");

  std::vector<uint8_t> corrupt = ack;
  corrupt.back() ^= 1;
  expect(decodeFrames(corrupt.data(), corrupt.size()).empty(), "invalid CRC");
  std::vector<uint8_t> prefixed = {0x00};
  prefixed.insert(prefixed.end(), frame.begin(), frame.end());
  expect(decodeFrames(prefixed.data(), prefixed.size()).size() == 1, "frame after padding");

  std::vector<uint8_t> cloud(36 + 28, 0);
  putLe16(cloud, 1, static_cast<uint16_t>(cloud.size()));
  putLe16(cloud, 5, 2);
  cloud[10] = 1;
  putLe32(cloud, 36, 1000);
  putLe32(cloud, 44, 200);
  cloud[48] = 40;
  cloud[62] = 60;
  const DataPacket points = parseDataPacket(cloud.data(), cloud.size());
  expect(points.ok && points.points.size() == 2, "point count");
  expect(points.points[0].x_mm == 1000 && points.points[0].z_mm == 200 && !points.points[0].zero, "valid point");
  expect(points.points[1].zero && points.points[1].reflectivity == 60, "cropped point");

  std::vector<uint8_t> imu(60, 0);
  putLe16(imu, 1, 60);
  for (int i = 0; i < 6; ++i) {
    putF32(imu, 36 + 4 * i, static_cast<float>(i + 1));
  }
  const DataPacket imu_packet = parseDataPacket(imu.data(), imu.size());
  expect(imu_packet.ok && imu_packet.has_imu && imu_packet.imu.gyro[0] == 1.f && imu_packet.imu.acc[2] == 6.f, "imu");

  std::vector<uint8_t> dual(36 + 28, 0);
  putLe16(dual, 1, static_cast<uint16_t>(dual.size()));
  putLe16(dual, 5, 1);
  dual[10] = 0x11;
  putLe32(dual, 36, 1000);
  putLe32(dual, 50, 2000);
  const DataPacket dual_packet = parseDataPacket(dual.data(), dual.size());
  expect(dual_packet.ok && dual_packet.points.size() == 2 && dual_packet.points[0].x_mm == 1000 &&
             dual_packet.points[1].x_mm == 2000,
         "dual echo");

  FovPlan plan;
  std::string error;
  expect(planFov(0, 360, plan, error) && plan.full && plan.enable == 0, "full fov");
  expect(yawInPlan(-1.0, 0.0, plan) && yawInPlan(0.0, 1.0, plan), "full accepts every yaw");
  expect(planFov(10, 200, plan, error) && !plan.wrap && plan.enable == 1 && plan.yaw0_stop == 200, "window");
  expect(yawInPlan(1.0, 0.0, plan) == false && yawInPlan(0.0, 1.0, plan), "window membership");
  expect(planFov(350, 10, plan, error) && plan.wrap && plan.enable == 3 && plan.yaw0_stop == 360 &&
             plan.yaw1_stop == 10,
         "wrap window");
  expect(yawInPlan(1.0, 0.0, plan) && yawInPlan(-1.0, 0.0, plan) == false, "wrap membership");
  const double rad = 355.0 * M_PI / 180.0;
  expect(yawInPlan(std::cos(rad), std::sin(rad), plan), "355 deg inside wrap");
  expect(planFov(300, 0, plan, error) && !plan.wrap && plan.yaw0_stop == 360 && plan.enable == 1, "stop zero");
  expect(!planFov(10, 10, plan, error) && !plan.valid, "equal angles rejected");
  expect(!planFov(-1, 20, plan, error), "negative angle rejected");

  uint8_t octets[4] = {};
  uint8_t stored[4] = {};
  expect(parseIpv4("192.168.1.20", octets), "parse ip");
  storeIpv4(stored, octets, true);
  expect(stored[0] == 20 && stored[1] == 1 && stored[2] == 168 && stored[3] == 192, "little-endian ipv4");
  storeIpv4(stored, octets, false);
  expect(stored[0] == 192 && stored[3] == 20, "wire-order ipv4");

  std::vector<CloudPoint> cloud_points(1);
  cloud_points[0].x = 1.f;
  cloud_points[0].reflectivity = 7;
  cloud_points[0].line = 3;
  cloud_points[0].time_ns = 42;
  std::vector<LivoxPointXyzrtlt> packed;
  expect(packLivoxPoints(cloud_points, packed) == 42 && packed.size() == 1 && packed[0].intensity == 7.f, "pack");

  const std::vector<uint8_t> state_ok = {0, 0, 0, 0, 0, 1, 0, 3, 0, 0, 0, 1, 0, 1};
  uint8_t state = 0;
  expect(parseStateReply(state_ok, state) && state == 1, "state reply");

  LidarSession session;
  expect(session.summary() == "未连接", "session starts idle");

  if (failures != 0) {
    std::cerr << failures << " failed\n";
    return 1;
  }
  std::cout << "protocol selftest ok\n";
  return 0;
}
