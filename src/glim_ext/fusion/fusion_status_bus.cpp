#include <glim_ext/fusion/fusion_status_bus.hpp>

namespace glim {

FusionStatusBus& FusionStatusBus::instance() {
  static FusionStatusBus bus;
  return bus;
}

void FusionStatusBus::publish(const FusionResult& result) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = result;
  has_value_ = true;
  ++sequence_;
}

std::optional<FusionResult> FusionStatusBus::latest() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_value_) {
    return std::nullopt;
  }
  return latest_;
}

std::optional<FusionStatus> FusionStatusBus::latest_status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_value_) {
    return std::nullopt;
  }
  return latest_.status;
}

uint64_t FusionStatusBus::sequence() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return sequence_;
}

}  // namespace glim
