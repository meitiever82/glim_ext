// metrics_node: 订阅 LocalizationData，计算并发布慢变定位质量指标。
//
// 当前实现（Phase 2 骨架）：
// - RTK 固定率：按有效时长加权，session 累计
// - 零速检测：speed < threshold（无 VCU/IMU 接入，后续可扩展）
// - 静态标准差：零速段内用 Welford 在线估计水平 σ，段结束时清零
//
// 发布：robominer_msgs/LocalizationMetrics @ 1 Hz
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "gnss_bringup/localization_types.hpp"
#include "robominer_msgs/msg/localization_data.hpp"
#include "robominer_msgs/msg/localization_metrics.hpp"

namespace gnss_bringup {
namespace {

using robominer_msgs::msg::LocalizationData;
using robominer_msgs::msg::LocalizationMetrics;

struct Welford2D {
  size_t n = 0;
  double mean_x = 0.0, mean_y = 0.0;
  double m2_x = 0.0, m2_y = 0.0;
  double max_offset = 0.0;

  void Reset() {
    n = 0;
    mean_x = mean_y = 0.0;
    m2_x = m2_y = 0.0;
    max_offset = 0.0;
  }

  void Add(double x, double y) {
    ++n;
    const double dx = x - mean_x;
    const double dy = y - mean_y;
    mean_x += dx / static_cast<double>(n);
    mean_y += dy / static_cast<double>(n);
    m2_x += dx * (x - mean_x);
    m2_y += dy * (y - mean_y);
    const double off = std::hypot(x - mean_x, y - mean_y);
    if (off > max_offset) max_offset = off;
  }

  double StdHorizontal() const {
    if (n < 2) return -1.0;
    const double vx = m2_x / static_cast<double>(n - 1);
    const double vy = m2_y / static_cast<double>(n - 1);
    return std::sqrt(vx + vy);
  }

  double MaxOffset() const { return n < 1 ? -1.0 : max_offset; }
};

}  // namespace

class MetricsNode : public rclcpp::Node {
 public:
  MetricsNode() : rclcpp::Node("gnss_metrics") {
    LoadParams();
    CreatePublishers();
    CreateSubscriptions();
    CreateTimers();
    RCLCPP_INFO(get_logger(),
                "gnss_metrics: zero_vel_thresh=%.2f m/s publish_hz=%.1f",
                zero_velocity_threshold_mps_, publish_rate_hz_);
  }

 private:
  void LoadParams() {
    input_topic_ = GetParam<std::string>("topics.localization_data",
                                         "/localization/localization_data");
    output_topic_ = GetParam<std::string>("topics.metrics",
                                          "/localization/metrics");
    zero_velocity_threshold_mps_ =
        GetParam<double>("metrics.zero_velocity_threshold_mps", 0.2);
    min_valid_duration_s_ =
        GetParam<double>("metrics.min_valid_duration_s", 1.0);
    publish_rate_hz_ = GetParam<double>("metrics.publish_rate_hz", 1.0);
    frame_id_ = GetParam<std::string>("metrics.frame_id", "world");
  }

  template <typename T>
  auto GetParam(const std::string& name, const T& default_value) -> T {
    if (!has_parameter(name)) {
      declare_parameter<T>(name, default_value);
    }
    return get_parameter(name).get_value<T>();
  }

  void CreatePublishers() {
    pub_ = create_publisher<LocalizationMetrics>(
        output_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
  }

  void CreateSubscriptions() {
    sub_ = create_subscription<LocalizationData>(
        input_topic_, rclcpp::SensorDataQoS(),
        [this](const LocalizationData& msg) { OnData(msg); });
  }

  void CreateTimers() {
    const auto period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / std::max(publish_rate_hz_, 0.1)));
    timer_ = create_wall_timer(period, [this]() { Publish(); });
  }

  void OnData(const LocalizationData& msg) {
    const rclcpp::Time t = msg.header.stamp;
    if (last_time_.nanoseconds() > 0) {
      const double dt = (t - last_time_).seconds();
      if (dt > 0.0 && dt < 5.0) {  // 忽略跳变/异常间隔
        UpdateFixRatio(msg, dt);
        UpdateStaticStd(msg, dt);
      }
    }
    last_time_ = t;
    last_data_ = msg;
    has_data_ = true;
  }

  void UpdateFixRatio(const LocalizationData& msg, double dt) {
    if (!msg.is_valid) return;
    total_valid_s_ += dt;
    if (msg.rtk_status.status == static_cast<uint8_t>(loc::RtkStatus::RtkFix)) {
      fixed_s_ += dt;
    }
  }

  void UpdateStaticStd(const LocalizationData& msg, double /*dt*/) {
    const bool zero_vel =
        std::isfinite(msg.speed) &&
        std::abs(msg.speed) < zero_velocity_threshold_mps_;

    if (!zero_vel) {
      if (static_.n > 0) {
        // 段结束：记录最后值后重置
        last_static_std_ = static_.StdHorizontal();
        last_static_max_offset_ = static_.MaxOffset();
        ++segment_count_;
      }
      static_.Reset();
      return;
    }

    if (std::isfinite(msg.x) && std::isfinite(msg.y)) {
      static_.Add(msg.x, msg.y);
    }
  }

  void Publish() {
    LocalizationMetrics msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    if (total_valid_s_ >= min_valid_duration_s_) {
      msg.rtk_fix_ratio = fixed_s_ / total_valid_s_;
    } else {
      msg.rtk_fix_ratio = -1.0;
    }
    msg.fix_time_s = fixed_s_;
    msg.total_time_s = total_valid_s_;

    if (static_.n > 0) {
      msg.static_std_horizontal = static_.StdHorizontal();
      msg.static_std_vertical =
          -1.0;  // LocalizationData 只有水平位置，无垂直静止 σ
      msg.static_max_deviation = static_.MaxOffset();
    } else {
      msg.static_std_horizontal = last_static_std_;
      msg.static_std_vertical = -1.0;
      msg.static_max_deviation = last_static_max_offset_;
    }
    msg.segment_count = segment_count_;

    pub_->publish(msg);
  }

  bool has_data_ = false;
  LocalizationData last_data_;
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double zero_velocity_threshold_mps_ = 0.2;
  double min_valid_duration_s_ = 1.0;
  double publish_rate_hz_ = 1.0;

  double fixed_s_ = 0.0;
  double total_valid_s_ = 0.0;
  Welford2D static_;
  double last_static_std_ = -1.0;
  double last_static_max_offset_ = -1.0;
  uint32_t segment_count_ = 0;

  rclcpp::Publisher<LocalizationMetrics>::SharedPtr pub_;
  rclcpp::Subscription<LocalizationData>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace gnss_bringup

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_bringup::MetricsNode>());
  rclcpp::shutdown();
  return 0;
}
