#pragma once

#include <glim/util/extension_module_ros2.hpp>

namespace glim {

class PriorMapLocalization : public ExtensionModuleROS2 {
public:
  PriorMapLocalization();
  ~PriorMapLocalization() override;

  std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override;
};

}  // namespace glim
