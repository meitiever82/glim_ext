#include "glim_ext/semantic/semantic_map_bus.hpp"

namespace glim {

SemanticMapBus& SemanticMapBus::instance() {
  static SemanticMapBus bus;
  return bus;
}

void SemanticMapBus::set_manager(std::shared_ptr<SubmapManager> manager) {
  std::lock_guard<std::mutex> lock(mutex_);
  manager_ = std::move(manager);
}

std::shared_ptr<SubmapManager> SemanticMapBus::manager() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return manager_;
}

}  // namespace glim
