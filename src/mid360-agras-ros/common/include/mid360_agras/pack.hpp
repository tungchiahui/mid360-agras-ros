#pragma once

#include "mid360_agras/device.hpp"

#include <cstdint>
#include <vector>

namespace mid360_agras {

#pragma pack(push, 1)
struct LivoxPointXyzrtlt {
  float x;
  float y;
  float z;
  float intensity;
  uint8_t tag;
  uint8_t line;
  double timestamp;
};
#pragma pack(pop)

static_assert(sizeof(LivoxPointXyzrtlt) == 26, "PointCloud2 point_step must match livox_ros_driver2");

struct CustomPointRaw {
  uint32_t offset_time = 0;
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;
  uint8_t reflectivity = 0;
  uint8_t tag = 0;
  uint8_t line = 0;
};

inline uint64_t packLivoxPoints(const std::vector<CloudPoint>& input, std::vector<LivoxPointXyzrtlt>& output) {
  output.resize(input.size());
  if (input.empty()) {
    return 0;
  }
  const uint64_t base = input.front().time_ns;
  for (size_t i = 0; i < input.size(); ++i) {
    LivoxPointXyzrtlt& dst = output[i];
    const CloudPoint& src = input[i];
    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;
    dst.intensity = static_cast<float>(src.reflectivity);
    dst.tag = src.tag;
    dst.line = src.line;
    dst.timestamp = static_cast<double>(src.time_ns);
  }
  return base;
}

inline uint64_t packCustomPoints(const std::vector<CloudPoint>& input, std::vector<CustomPointRaw>& output) {
  output.resize(input.size());
  if (input.empty()) {
    return 0;
  }
  const uint64_t base = input.front().time_ns;
  for (size_t i = 0; i < input.size(); ++i) {
    uint64_t relative = input[i].time_ns >= base ? input[i].time_ns - base : 0;
    if (relative > 0xffffffffu) {
      relative = 0xffffffffu;
    }
    CustomPointRaw& dst = output[i];
    dst.offset_time = static_cast<uint32_t>(relative);
    dst.x = input[i].x;
    dst.y = input[i].y;
    dst.z = input[i].z;
    dst.reflectivity = input[i].reflectivity;
    dst.tag = input[i].tag;
    dst.line = input[i].line;
  }
  return base;
}

}  // namespace mid360_agras
