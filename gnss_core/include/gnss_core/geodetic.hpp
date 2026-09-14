#pragma once
#include <Eigen/Core>
#include <memory>

namespace GeographicLib { class LocalCartesian; }

namespace gnss_core {

// 经纬高 ↔ 局部 ENU(米)。原点固定于构造时给定的 lat0/lon0/alt0。
class LlaToEnu {
public:
  LlaToEnu(double lat0, double lon0, double alt0);
  ~LlaToEnu();
  Eigen::Vector3d forward(double lat, double lon, double alt) const;  // 返回 [E,N,U]
  Eigen::Vector3d reverse(const Eigen::Vector3d& enu) const;          // [E,N,U] → [lat,lon,alt](deg/deg/m)
private:
  std::unique_ptr<GeographicLib::LocalCartesian> impl_;
};

// 两点间 WGS-84 椭球测地线距离(m),只看水平位置。
double geodesic_distance_m(double lat1, double lon1, double lat2, double lon2);

}  // namespace gnss_core
