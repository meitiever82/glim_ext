// huace_adapter_node: 华测 610 驱动 RtkFix → 归一化 gnss_msgs/RtkFix
//
// 职责：把 610 驱动发布的 gnss_msgs/RtkFix 透传到统一话题 /gnss/rtkfix，
// 供下游 localization_node 处理。当前 610 驱动已输出标准 RtkFix，
// 本节点主要做话题归一化与字段防御性填充。
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "gnss_msgs/msg/rtk_fix.hpp"

namespace gnss_bringup {

using gnss_msgs::msg::RtkFix;

class HuaceAdapterNode : public rclcpp::Node {
 public:
  HuaceAdapterNode() : rclcpp::Node("huace_adapter") {
    LoadParams();
    CreatePublishers();
    CreateSubscriptions();
    RCLCPP_INFO(get_logger(), "huace_adapter: %s -> %s",
                input_topic_.c_str(), output_topic_.c_str());
  }

 private:
  void LoadParams() {
    input_topic_ =
        GetParam<std::string>("topics.rtkfix_huace", "/gnss/rtkfix_huace");
    output_topic_ = GetParam<std::string>("topics.rtkfix", "/gnss/rtkfix");
  }

  template <typename T>
  auto GetParam(const std::string& name, const T& default_value) -> T {
    if (!has_parameter(name)) {
      declare_parameter<T>(name, default_value);
    }
    return get_parameter(name).get_value<T>();
  }

  void CreatePublishers() {
    pub_ = create_publisher<RtkFix>(
        output_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
  }

  void CreateSubscriptions() {
    sub_ = create_subscription<RtkFix>(
        input_topic_, rclcpp::SensorDataQoS(),
        [this](const RtkFix& msg) { OnRtkFix(Normalize(msg)); });
  }

  static auto Normalize(const RtkFix& in) -> RtkFix {
    RtkFix out = in;
    // 防御：如果驱动没填 gnss_time，用 header.stamp 兜底
    if (out.gnss_time <= 0.0 && out.header.stamp.sec > 0) {
      out.gnss_time = static_cast<double>(out.header.stamp.sec) +
                      static_cast<double>(out.header.stamp.nanosec) * 1e-9;
    }
    // 防御：ins_status 非法时标为 unknown
    if (out.ins_status > 10 && out.ins_status != RtkFix::INS_UNKNOWN) {
      out.ins_status = RtkFix::INS_UNKNOWN;
    }
    // 防御：heading_type 非法时标为 none
    if (out.heading_type > RtkFix::HEADING_TYPE_OTHER) {
      out.heading_type = RtkFix::HEADING_TYPE_NONE;
    }
    return out;
  }

  void OnRtkFix(const RtkFix& msg) { pub_->publish(msg); }

  std::string input_topic_;
  std::string output_topic_;

  rclcpp::Publisher<RtkFix>::SharedPtr pub_;
  rclcpp::Subscription<RtkFix>::SharedPtr sub_;
};

}  // namespace gnss_bringup

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_bringup::HuaceAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
