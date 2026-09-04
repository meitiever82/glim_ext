#pragma once
#include <cstdint>
#include <Eigen/Core>

namespace gnss_core {

enum class Quality : uint8_t { NONE = 0, SINGLE = 1, DGPS = 2, FLOAT = 3, FIXED = 4 };

// 源无关的一条 GNSS 定位样本(与 gnss_msgs/RtkFix 对应,但不依赖 ROS 类型)
struct RtkFixSample {
  double stamp = 0.0;            // unix seconds
  Quality quality = Quality::NONE;
  double lat = 0.0, lon = 0.0, alt = 0.0;   // WGS-84, deg/deg/m
  Eigen::Vector3d sigma_enu = Eigen::Vector3d::Zero();  // m
  double diff_age = 0.0;         // s
  int sats_used = 0;
  double heading = 0.0;          // deg
  bool heading_valid = false;
};

struct EnuPoint {
  double stamp = 0.0;
  Eigen::Vector3d enu = Eigen::Vector3d::Zero();
  Quality quality = Quality::NONE;
};

}  // namespace gnss_core
