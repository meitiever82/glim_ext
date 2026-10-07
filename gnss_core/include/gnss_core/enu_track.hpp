#pragma once
#include <istream>
#include <ostream>
#include <vector>
#include <Eigen/Core>
#include "gnss_core/pos_io.hpp"
#include "gnss_core/types.hpp"

namespace gnss_core {

struct EnuSample {
  double stamp = 0.0;                                 // UTC unix 秒
  Eigen::Vector3d enu = Eigen::Vector3d::Zero();      // [E, N, U] m
  Eigen::Vector3d sigma_enu = Eigen::Vector3d::Zero();// [sde, sdn, sdu] m
  Quality quality = Quality::NONE;
};

struct EnuTrack {
  Eigen::Vector3d origin_lla = Eigen::Vector3d::Zero();  // [lat, lon, alt] deg/deg/m
  std::vector<EnuSample> samples;
};

struct EnuTrackOptions {
  Quality min_quality = Quality::FLOAT;
  bool has_origin = false;
  Eigen::Vector3d origin_lla = Eigen::Vector3d::Zero();
};

EnuTrack make_enu_track(const std::vector<PosRecord>& recs, const EnuTrackOptions& opt);
void write_enu_track(std::ostream& out, const EnuTrack& track);
EnuTrack read_enu_track(std::istream& in);   // 供测试与 Python 侧格式对齐用

}  // namespace gnss_core
