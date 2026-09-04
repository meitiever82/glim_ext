#pragma once
#include <Eigen/Core>
#include <memory>

namespace GeographicLib { class LocalCartesian; }

namespace gnss_core {

// 经纬高 → 局部 ENU(米)。原点固定于构造时给定的 lat0/lon0/alt0。
class LlaToEnu {
public:
  LlaToEnu(double lat0, double lon0, double alt0);
  ~LlaToEnu();
  Eigen::Vector3d forward(double lat, double lon, double alt) const;  // 返回 [E,N,U]
private:
  std::unique_ptr<GeographicLib::LocalCartesian> impl_;
};

}  // namespace gnss_core
