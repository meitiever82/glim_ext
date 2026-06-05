#include <glim_ext/prior_map_localization.hpp>

#include <glim/util/logging.hpp>

namespace glim {

PriorMapLocalization::PriorMapLocalization() {
  auto logger = create_module_logger("prior_loc");
  logger->info("prior_map_localization stub loaded");
}

PriorMapLocalization::~PriorMapLocalization() {}

std::vector<GenericTopicSubscription::Ptr> PriorMapLocalization::create_subscriptions(rclcpp::Node& node) {
  return {};
}

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::PriorMapLocalization();
}
