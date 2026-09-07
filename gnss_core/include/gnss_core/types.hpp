#pragma once
#include <cstdint>
#include <Eigen/Core>

namespace gnss_core {

enum class Quality : uint8_t { NONE = 0, SINGLE = 1, DGPS = 2, FLOAT = 3, FIXED = 4 };

// 样本有效时间的来源:Header = ROS header.stamp(到达时间),GnssTime = 板卡自报时间
enum class StampSource { Header, GnssTime };

// 选择样本的有效时间:GnssTime 且 gnss_time>0 → gnss_time,否则 header_stamp;再加 offset
inline double effective_stamp(double header_stamp, double gnss_time, StampSource src, double offset) {
  const double base = (src == StampSource::GnssTime && gnss_time > 0.0) ? gnss_time : header_stamp;
  return base + offset;
}

// 源无关的一条 GNSS 定位样本(与 gnss_msgs/RtkFix 对应,但不依赖 ROS 类型)
struct RtkFixSample {
  double stamp = 0.0;            // 有效时间(unix s):壳侧用 effective_stamp() 填
  double header_stamp = 0.0;     // 原始 header.stamp
  double gnss_time = 0.0;        // 原始板卡时间(0 = 无)
  Quality quality = Quality::NONE;
  double lat = 0.0, lon = 0.0, alt = 0.0;   // WGS-84, deg/deg/m
  Eigen::Vector3d sigma_enu = Eigen::Vector3d::Zero();  // m, E/N/U
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
