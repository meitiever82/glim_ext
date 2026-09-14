#include "gnss_core/geodetic.hpp"
#include <GeographicLib/Geodesic.hpp>
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

Eigen::Vector3d LlaToEnu::reverse(const Eigen::Vector3d& enu) const {
  double lat, lon, alt;
  impl_->Reverse(enu.x(), enu.y(), enu.z(), lat, lon, alt);
  return {lat, lon, alt};
}

double geodesic_distance_m(double lat1, double lon1, double lat2, double lon2) {
  double s12 = 0.0;
  GeographicLib::Geodesic::WGS84().Inverse(lat1, lon1, lat2, lon2, s12);
  return s12;
}

}  // namespace gnss_core
