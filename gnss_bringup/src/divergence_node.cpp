// divergence_node: 双路 RtkFix 水平偏差监测节点。
//
// 输入：
//   /gnss/rtkfix_device       — 板卡/610 融合解（或北云驱动直接输出的 RtkFix）
//   /gnss/rtkfix_independent  — rtkrcv 独立解算输出（或其他独立参考源）
//
// 处理：
//   - 按 gnss_time 或 header.stamp 配对最近两帧
//   - 统一原点做 BLH→ENU 水平距离
//   - 喂给 gnss_core::DivergenceMonitor（经验 σ 学习、持续超限计时）
//
// 输出：
//   /diagnostics — diagnostic_msgs/DiagnosticArray，名称 "gnss: device_divergence"
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include "gnss_core/diagnosis.hpp"
#include "gnss_core/divergence_monitor.hpp"
#include "gnss_core/geodetic.hpp"
#include "gnss_msgs/msg/rtk_fix.hpp"

namespace gnss_bringup {

using gnss_msgs::msg::RtkFix;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using gnss_core::DivergenceMonitor;
using gnss_core::DivergenceState;
using gnss_core::DiagnosisConfig;
using gnss_core::Quality;

struct Sample {
  rclcpp::Time recv_time;
  double epoch_t;  // 配对键：gnss_time > 0 时用 gnss_time，否则用 stamp
  double lat;
  double lon;
  double sdn;
  double sde;
  Quality quality;
};

class DivergenceNode : public rclcpp::Node {
 public:
  DivergenceNode() : rclcpp::Node("gnss_divergence") {
    LoadParams();
    converter_ = std::make_unique<gnss_core::LlaToEnu>(origin_lat_, origin_lon_,
                                                        origin_height_);
    monitor_ = std::make_unique<DivergenceMonitor>(diag_cfg_);
    CreatePublishers();
    CreateSubscriptions();
    CreateTimers();
    RCLCPP_INFO(get_logger(),
                "gnss_divergence: device=%s independent=%s diag=%s",
                device_topic_.c_str(), independent_topic_.c_str(),
                diag_topic_.c_str());
  }

 private:
  void LoadParams() {
    device_topic_ = GetParam<std::string>("topics.rtkfix_device",
                                          "/gnss/rtkfix_device");
    independent_topic_ =
        GetParam<std::string>("topics.rtkfix_independent",
                              "/gnss/rtkfix_independent");
    diag_topic_ = GetParam<std::string>("topics.diagnostics", "/diagnostics");

    origin_lat_ = GetParam<double>("origin_lat", 0.0);
    origin_lon_ = GetParam<double>("origin_lon", 0.0);
    origin_height_ = GetParam<double>("origin_height", 0.0);

    diag_cfg_.divergence_sigma =
        GetParam<double>("divergence.sigma", 3.0);
    diag_cfg_.divergence_window_s =
        GetParam<double>("divergence.window_s", 600.0);
    diag_cfg_.divergence_min_samples =
        GetParam<int>("divergence.min_samples", 60);
    diag_cfg_.divergence_sigma_floor_m =
        GetParam<double>("divergence.sigma_floor_m", 0.05);
    diag_cfg_.divergence_sigma_max_m =
        GetParam<double>("divergence.sigma_max_m", 0.10);
    diag_cfg_.divergence_pair_max_dt_s =
        GetParam<double>("divergence.pair_max_dt_s", 2.0);
    diag_cfg_.divergence_epoch_max_dt_s =
        GetParam<double>("divergence.epoch_max_dt_s", 0.1);

    publish_rate_hz_ = GetParam<double>("divergence.publish_rate_hz", 1.0);
    min_quality_for_learn_ =
        GetParam<int>("divergence.min_quality_for_learn",
                      static_cast<int>(Quality::FIXED));
  }

  template <typename T>
  auto GetParam(const std::string& name, const T& default_value) -> T {
    if (!has_parameter(name)) {
      declare_parameter<T>(name, default_value);
    }
    return get_parameter(name).get_value<T>();
  }

  void CreatePublishers() {
    diag_pub_ = create_publisher<DiagnosticArray>(diag_topic_, 1);
  }

  void CreateSubscriptions() {
    device_sub_ = create_subscription<RtkFix>(
        device_topic_, rclcpp::SensorDataQoS(),
        [this](const RtkFix& msg) { OnDevice(msg); });
    independent_sub_ = create_subscription<RtkFix>(
        independent_topic_, rclcpp::SensorDataQoS(),
        [this](const RtkFix& msg) { OnIndependent(msg); });
  }

  void CreateTimers() {
    const auto period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / std::max(publish_rate_hz_, 0.1)));
    timer_ = create_wall_timer(period, [this]() { PublishDiagnostics(); });
  }

  static Sample ToSample(const RtkFix& msg, const rclcpp::Time& recv) {
    Sample s;
    s.recv_time = recv;
    s.epoch_t = msg.gnss_time > 0.0 ? msg.gnss_time : msg.header.stamp.sec +
                                                            msg.header.stamp.nanosec * 1e-9;
    s.lat = msg.latitude;
    s.lon = msg.longitude;
    s.sdn = msg.sigma_enu[1];  // N
    s.sde = msg.sigma_enu[0];  // E
    s.quality = static_cast<Quality>(msg.quality);
    return s;
  }

  void OnDevice(const RtkFix& msg) {
    Prune(device_q_);
    device_q_.push_back(ToSample(msg, now()));
    TryEvaluate();
  }

  void OnIndependent(const RtkFix& msg) {
    Prune(independent_q_);
    independent_q_.push_back(ToSample(msg, now()));
    TryEvaluate();
  }

  void Prune(std::deque<Sample>& q) {
    const rclcpp::Time now_t = now();
    while (!q.empty() &&
           (now_t - q.front().recv_time).seconds() > 5.0) {
      q.pop_front();
    }
  }

  std::optional<std::pair<Sample, Sample>> FindPair() {
    if (device_q_.empty() || independent_q_.empty()) return std::nullopt;

    // 找 epoch_t 最接近的一对
    double best_dt = std::numeric_limits<double>::max();
    auto best_d = device_q_.end();
    auto best_i = independent_q_.end();

    for (auto d = device_q_.begin(); d != device_q_.end(); ++d) {
      for (auto i = independent_q_.begin(); i != independent_q_.end(); ++i) {
        const double dt = std::abs(d->epoch_t - i->epoch_t);
        if (dt < best_dt) {
          best_dt = dt;
          best_d = d;
          best_i = i;
        }
      }
    }

    if (best_d == device_q_.end()) return std::nullopt;

    // 用 epoch_t 配对容差：如果两帧都带 gnss_time 用严格容差，否则用宽松容差
    const bool both_gnss_time =
        best_d->epoch_t != best_d->recv_time.seconds() &&
        best_i->epoch_t != best_i->recv_time.seconds();
    const double max_dt = both_gnss_time
                              ? diag_cfg_.divergence_epoch_max_dt_s
                              : diag_cfg_.divergence_pair_max_dt_s;
    if (best_dt > max_dt) return std::nullopt;

    return std::make_pair(*best_d, *best_i);
  }

  void TryEvaluate() {
    auto pair = FindPair();
    if (!pair) {
      last_state_ = monitor_->update(now().seconds(), std::nullopt, 0.0, false);
      return;
    }

    const auto& d = pair->first;
    const auto& i = pair->second;

    const Eigen::Vector3d enu_d = converter_->forward(d.lat, d.lon, 0.0);
    const Eigen::Vector3d enu_i = converter_->forward(i.lat, i.lon, 0.0);
    const double diff_m = std::hypot(enu_d.x() - enu_i.x(),
                                     enu_d.y() - enu_i.y());

    const double current_sigma = std::hypot(i.sde, i.sdn);
    const bool learnable =
        static_cast<int>(d.quality) >= min_quality_for_learn_ &&
        static_cast<int>(i.quality) >= min_quality_for_learn_;

    last_state_ =
        monitor_->update(now().seconds(), diff_m, current_sigma, learnable);
  }

  void PublishDiagnostics() {
    DiagnosticArray array;
    array.header.stamp = now();

    DiagnosticStatus status;
    status.name = "gnss: device_divergence";
    status.hardware_id = "gnss";

    const bool alarming =
        last_state_.since.has_value() &&
        (now().seconds() - *last_state_.since) > diag_cfg_.divergence_hold_s;

    status.level = alarming ? DiagnosticStatus::WARN : DiagnosticStatus::OK;
    status.message = alarming ? "divergence exceeding learned threshold"
                              : "divergence within threshold";

    auto add_kv = [&](const std::string& key, const std::string& value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value;
      status.values.push_back(kv);
    };

    add_kv("divergence_m",
           last_state_.divergence_m ? std::to_string(*last_state_.divergence_m)
                                    : "none");
    add_kv("threshold_m", std::to_string(last_state_.threshold_m));
    add_kv("empirical", last_state_.empirical ? "true" : "false");
    add_kv("since_s",
           last_state_.since ? std::to_string(now().seconds() - *last_state_.since)
                             : "none");

    array.status.push_back(status);
    diag_pub_->publish(array);
  }

  std::string device_topic_;
  std::string independent_topic_;
  std::string diag_topic_;

  double origin_lat_ = 0.0;
  double origin_lon_ = 0.0;
  double origin_height_ = 0.0;

  DiagnosisConfig diag_cfg_;
  double publish_rate_hz_ = 1.0;
  int min_quality_for_learn_ = static_cast<int>(Quality::FIXED);

  std::unique_ptr<gnss_core::LlaToEnu> converter_;
  std::unique_ptr<DivergenceMonitor> monitor_;
  DivergenceState last_state_;

  std::deque<Sample> device_q_;
  std::deque<Sample> independent_q_;

  rclcpp::Subscription<RtkFix>::SharedPtr device_sub_;
  rclcpp::Subscription<RtkFix>::SharedPtr independent_sub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace gnss_bringup

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_bringup::DivergenceNode>());
  rclcpp::shutdown();
  return 0;
}
