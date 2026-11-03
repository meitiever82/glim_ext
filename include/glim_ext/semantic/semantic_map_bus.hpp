#pragma once
// In-process publication of the active stage-2 semantic submap (spec 2.2 §5/§6.2).
//
// The semantic map module owns a SubmapManager and publishes it here. Stage 4 perception
// and stage 5 consume the latest active submap in the same process, avoiding a ROS
// publisher (ExtensionModuleROS2 only exposes subscriptions).

#include <memory>
#include <mutex>

#include "glim_ext/semantic/submap_manager.hpp"

namespace glim {

class SemanticMapBus {
public:
  static SemanticMapBus& instance();

  void set_manager(std::shared_ptr<SubmapManager> manager);
  std::shared_ptr<SubmapManager> manager() const;

private:
  SemanticMapBus() = default;

  mutable std::mutex mutex_;
  std::shared_ptr<SubmapManager> manager_;
};

}  // namespace glim
