// bynav_adapter_node: 北云 OEM7 驱动话题 → 扩展 gnss_msgs/RtkFix
//
// 职责：把 OEM7 专有消息（INSPVAX/BESTPOS/HEADING2/CORRIMU）归一化成
// 与 610 等价的 gnss_msgs/RtkFix，供下游 localization_node 统一处理。
// 本身不做 LocalizationGate 判决、不输出 LocalizationData。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <novatel_oem7_msgs/msg/bestpos.hpp>
#include <novatel_oem7_msgs/msg/corrimu.hpp>
#include <novatel_oem7_msgs/msg/heading2.hpp>
#include <novatel_oem7_msgs/msg/inspvax.hpp>

#include "gnss_bringup/localization_mapping.hpp"
#include "gnss_bringup/localization_types.hpp"
#include "gnss_msgs/msg/rtk_fix.hpp"

namespace gnss_bringup {

using gnss_msgs::msg::RtkFix;
using loc::Blh;
using loc::EnuConverter;
using loc::ImuInput;
using loc::PosType;
using loc::corrimudata_imu_scale;
using loc::convert_imu;
using loc::heading_narrow_int_from_oem7;
using loc::ins_status_from_oem7;
using loc::pos_type_from_oem7;

using Inspvax = novatel_oem7_msgs::msg::INSPVAX;
using Bestpos = novatel_oem7_msgs::msg::BESTPOS;
using CorrImu = novatel_oem7_msgs::msg::CORRIMU;
using Heading2 = novatel_oem7_msgs::msg::HEADING2;

class ByNavAdapterNode : public rclcpp::Node {
 public:
  ByNavAdapterNode() : rclcpp::Node("bynav_adapter") {
    LoadParams();
    CreatePublishers();
    CreateSubscriptions();
    RCLCPP_INFO(get_logger(),
                "bynav_adapter: inspvax=%s bestpos=%s corrimu=%s heading2=%s -> %s",
                inspvax_topic_.c_str(), bestpos_topic_.c_str(),
                corrimudata_topic_.c_str(), heading2_topic_.c_str(),
                output_topic_.c_str());
  }

 private:
  void LoadParams() {
    output_topic_ = GetParam<std::string>("topics.rtkfix", "/gnss/rtkfix");
    world_frame_id_ = GetParam<std::string>("world_frame_id", "world");
    leap_seconds_ = GetParam<int>("leap_seconds", 18);
    imu_rate_hz_ = GetParam<double>("imu_rate_hz", 100.0);
    if (!(imu_rate_hz_ > 0.0)) {
      RCLCPP_WARN(get_logger(), "Invalid imu_rate_hz=%.3f, fallback to 100 Hz",
                  imu_rate_hz_);
      imu_rate_hz_ = 100.0;
    }
    imu_scale_ = corrimudata_imu_scale(imu_rate_hz_);

    inspvax_topic_ = GetParam<std::string>("topics.inspvax", "/bynav/inspvax");
    bestpos_topic_ = GetParam<std::string>("topics.bestpos", "/bynav/bestpos");
    corrimudata_topic_ =
        GetParam<std::string>("topics.corrimudata", "/bynav/corrimudata");
    heading2_topic_ = GetParam<std::string>("topics.heading2", "/bynav/heading2");
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
    auto qos = rclcpp::SensorDataQoS();
    inspvax_sub_ = create_subscription<Inspvax>(
        inspvax_topic_, qos,
        [this](const Inspvax& msg) { OnInspvax(msg); });
    bestpos_sub_ = create_subscription<Bestpos>(
        bestpos_topic_, qos,
        [this](const Bestpos& msg) { OnBestpos(msg); });
    corrimu_sub_ = create_subscription<CorrImu>(
        corrimudata_topic_, qos,
        [this](const CorrImu& msg) { OnCorrImu(msg); });
    heading2_sub_ = create_subscription<Heading2>(
        heading2_topic_, qos,
        [this](const Heading2& msg) { OnHeading2(msg); });
  }

  static auto PosTypeToQuality(PosType pt) -> uint8_t {
    return static_cast<uint8_t>(pt);
  }

  static auto GpsToUnixTime(uint16_t gps_week, uint32_t week_ms,
                            int leap_seconds) -> double {
    static constexpr int64_t kGpsEpochOffset = 315964800;
    const double gps_total = static_cast<double>(gps_week) * 604800.0 +
                             static_cast<double>(week_ms) / 1000.0;
    return gps_total - static_cast<double>(leap_seconds) +
           static_cast<double>(kGpsEpochOffset);
  }

  void OnBestpos(const Bestpos& msg) {
    bestpos_ = msg;
    has_bestpos_ = true;
  }

  void OnCorrImu(const CorrImu& msg) {
    corrimu_ = msg;
    has_corrimu_ = true;
  }

  void OnHeading2(const Heading2& msg) {
    heading2_ = msg;
    has_heading2_ = true;
  }

  void OnInspvax(const Inspvax& msg) {
    inspvax_ = msg;
    has_inspvax_ = true;

    RtkFix out;
    out.header.stamp = now();
    out.header.frame_id = world_frame_id_;
    out.gnss_time = GpsToUnixTime(msg.nov_header.gps_week_number,
                                   msg.nov_header.gps_week_milliseconds,
                                   leap_seconds_);

    const PosType pos_type = pos_type_from_oem7(msg.pos_type.type);
    out.quality = PosTypeToQuality(pos_type);
    out.raw_status = static_cast<uint8_t>(msg.pos_type.type);

    out.latitude = msg.latitude;
    out.longitude = msg.longitude;
    out.altitude = msg.height;

    // 标准差：优先用 BESTPOS；BESTPOS 未到时用 INSPVAX 的 std
    if (has_bestpos_) {
      out.sigma_enu[0] = bestpos_.lon_stdev;  // E
      out.sigma_enu[1] = bestpos_.lat_stdev;  // N
      out.sigma_enu[2] = bestpos_.hgt_stdev;  // U
    } else {
      out.sigma_enu[0] = msg.longitude_stdev;
      out.sigma_enu[1] = msg.latitude_stdev;
      out.sigma_enu[2] = msg.height_stdev;
    }

    out.diff_age = has_bestpos_ ? bestpos_.diff_age : 0.0f;
    out.sats_used = has_bestpos_ ? bestpos_.num_sol_svs : 0;
    out.sats_main = has_bestpos_ ? bestpos_.num_sol_svs : 0;
    out.sats_aux = has_heading2_ ? heading2_.num_sv_obs : 0;

    // 航向与旧 localization_parser 一致：主 yaw 用 INSPVAX azimuth，
    // HEADING2 仅用于基线长度/双天线固定解标志/辅助星数。
    out.heading = msg.azimuth;
    if (has_heading2_) {
      out.heading_sigma = heading2_.heading_stdev;
      out.heading_valid = heading_narrow_int_from_oem7(heading2_.pos_type.type);
      out.baseline_length = heading2_.length;
      out.heading_type = out.heading_valid ? RtkFix::HEADING_TYPE_NARROW_INT
                                           : RtkFix::HEADING_TYPE_OTHER;
    } else {
      out.heading_sigma = 0.0f;
      out.heading_valid = false;
      out.heading_type = RtkFix::HEADING_TYPE_NONE;
    }

    out.ratio = 0.0f;  // RTKLIB 独立解才有，OEM7 不直接提供

    out.vel_east = msg.east_velocity;
    out.vel_north = msg.north_velocity;
    out.vel_up = msg.up_velocity;

    out.roll = msg.roll;
    out.pitch = msg.pitch;  // localization_node 统一做符号转换

    const auto imu = ImuNow();
    out.gyro_x = imu.wx;
    out.gyro_y = imu.wy;
    out.gyro_z = imu.wz;
    out.acc_x = imu.ax;
    out.acc_y = imu.ay;
    out.acc_z = imu.az;

    const auto ins = ins_status_from_oem7(msg.ins_status.status);
    out.ins_status = (ins == loc::InsStatus::Unknown)
                         ? RtkFix::INS_UNKNOWN
                         : static_cast<uint8_t>(ins);

    out.undulation = msg.undulation;

    pub_->publish(out);
  }

  auto ImuNow() const -> loc::ImuOutput {
    if (!has_corrimu_) return loc::ImuOutput{};
    ImuInput in;
    in.rate_x = corrimu_.pitch_rate;
    in.rate_y = corrimu_.roll_rate;
    in.rate_z = corrimu_.yaw_rate;
    in.acc_x = corrimu_.lateral_acc;
    in.acc_y = corrimu_.longitudinal_acc;
    in.acc_z = corrimu_.vertical_acc;
    return convert_imu(in, imu_scale_);
  }

  std::string output_topic_;
  std::string world_frame_id_;
  int leap_seconds_ = 18;
  double imu_rate_hz_ = 100.0;
  loc::ImuScale imu_scale_;

  std::string inspvax_topic_;
  std::string bestpos_topic_;
  std::string corrimudata_topic_;
  std::string heading2_topic_;

  Inspvax inspvax_;
  Bestpos bestpos_;
  CorrImu corrimu_;
  Heading2 heading2_;
  bool has_inspvax_ = false;
  bool has_bestpos_ = false;
  bool has_corrimu_ = false;
  bool has_heading2_ = false;

  rclcpp::Publisher<RtkFix>::SharedPtr pub_;
  rclcpp::Subscription<Inspvax>::SharedPtr inspvax_sub_;
  rclcpp::Subscription<Bestpos>::SharedPtr bestpos_sub_;
  rclcpp::Subscription<CorrImu>::SharedPtr corrimu_sub_;
  rclcpp::Subscription<Heading2>::SharedPtr heading2_sub_;
};

}  // namespace gnss_bringup

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_bringup::ByNavAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
