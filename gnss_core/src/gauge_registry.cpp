#include "gnss_core/gauge_registry.hpp"

namespace gnss_core {

GaugeRegistry& GaugeRegistry::instance() {
  static GaugeRegistry g;   // Meyers singleton：无静态初始化顺序依赖
  return g;
}

bool GaugeRegistry::initialized() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return initialized_;
}

Eigen::Isometry3d GaugeRegistry::T_world_enu() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return initialized_ ? T_ : Eigen::Isometry3d::Identity();
}

std::string GaugeRegistry::owner() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return owner_;
}

bool GaugeRegistry::try_publish(const Eigen::Isometry3d& T, const std::string& owner) {
  if (!T.matrix().allFinite()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (initialized_) {
    return false;
  }
  T_ = T;
  owner_ = owner;
  initialized_ = true;
  return true;
}

void GaugeRegistry::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  initialized_ = false;
  T_ = Eigen::Isometry3d::Identity();
  owner_.clear();
}

}  // namespace gnss_core
