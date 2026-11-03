// In-process publication of the stage-2 FusionResult/FusionStatus (spec 2.1 §8).
//
// GLIM extension modules cannot create publishers through ExtensionModuleROS2 (it only
// exposes subscriptions), so the localization module publishes its latest result here and
// the 2.2 semantic map (and later the perception stage) read it in the same process.
// Latest-value semantics: readers do not queue, they always get the newest snapshot.

#pragma once

#include <cstdint>
#include <mutex>
#include <optional>

#include <glim_ext/fusion/fusion_types.hpp>

namespace glim {

class FusionStatusBus {
public:
  static FusionStatusBus& instance();

  void publish(const FusionResult& result);

  std::optional<FusionResult> latest() const;
  std::optional<FusionStatus> latest_status() const;

  // Monotonic counter incremented on every publish; lets readers detect new data.
  uint64_t sequence() const;

private:
  FusionStatusBus() = default;

  mutable std::mutex mutex_;
  FusionResult latest_;
  bool has_value_ = false;
  uint64_t sequence_ = 0;
};

}  // namespace glim
