#include "gnss_core/geodetic.hpp"
#include <GeographicLib/LocalCartesian.hpp>

namespace gnss_core {

LlaToEnu::LlaToEnu(double lat0, double lon0, double alt0)
  : impl_(std::make_unique<GeographicLib::LocalCartesian>(lat0, lon0, alt0)) {}

LlaToEnu::~LlaToEnu() = default;

Eigen::Vector3d LlaToEnu::forward(double lat, double lon, double alt) const {
  double e, n, u;
  impl_->Forward(lat, lon, alt, e, n, u);
  return {e, n, u};
}

}  // namespace gnss_core
