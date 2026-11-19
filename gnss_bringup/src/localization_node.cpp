// localization_node: 统一 RtkFix → LocalizationData 判决节点。
//
// 覆盖场景：
//   - bynav_adapter 输出的扩展 RtkFix
//   - huace_adapter 输出的归一化 RtkFix
//   - 任何其他已发布 gnss_msgs/RtkFix 的 GNSS 源
//
// 职责：字段映射 + LocalizationGate + 202012 告警；
// 不处理接收机专有话题（由上游 adapter 完成）。
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "gnss_bringup/localization_gate.hpp"
#include "gnss_bringup/localization_mapping.hpp"
#include "gnss_bringup/localization_types.hpp"
#include "gnss_bringup/loc_alarm_state.hpp"
#include "gnss_msgs/msg/rtk_fix.hpp"
#include "robominer_msgs/msg/fault_code.hpp"
#include "robominer_msgs/msg/fault_codes.hpp"
#include "robominer_msgs/msg/localization_data.hpp"
#include "robominer_msgs/msg/localization_data_aux.hpp"

namespace gnss_bringup {

using gnss_msgs::msg::RtkFix;
using robominer_msgs::msg::LocalizationData;
using robominer_msgs::msg::LocalizationDataAux;
using std_msgs::msg::Float64;
using namespace std::chrono_literals;
using loc::Blh;
using loc::Enu;
using loc::EnuConverter;
using loc::FaultCode;
using loc::GateInput;
using loc::GateOutput;
using loc::GateParams;
using loc::ImuInput;
using loc::InsStatus;
using loc::LocAlarmAction;
using loc::LocAlarmInput;
using loc::LocAlarmState;
using loc::LocalizationGate;
using loc::PosType;
using loc::RtkStatus;
using loc::compute_heading;
using loc::compute_speed;
using loc::convert_imu;
using loc::corrimudata_imu_scale;
using loc::driver_imu_scale;
using loc::normalize_angle;
using loc::reorder_std;

auto ClampToU8(double v) -> uint8_t {
  if (!std::isfinite(v)) return 0;
  return static_cast<uint8_t>(std::clamp(v, 0.0, 255.0));
}

class LocalizationNode : public rclcpp::Node {
 public:
  LocalizationNode() : rclcpp::Node("localization_node") {
    LoadParams();
    converter_ = std::make_unique<EnuConverter>(Blh{origin_lat_, origin_lon_, origin_height_});
    gate_ = std::make_unique<LocalizationGate>(gate_params_);
    alarm_ = std::make_unique<LocAlarmState>(alarm_params_);
    CreatePublishers();
    CreateSubscriptions();
    CreateTimers();
    RCLCPP_INFO(get_logger(),
                "localization_node: input=%s output=%s (unified RtkFix -> LocalizationData)",
                input_topic_.c_str(), output_topic_.c_str());
  }

 private:
  void LoadParams() {
    input_topic_ =
        GetParam<std::string>("topics.rtkfix", "/gnss/rtkfix");
    output_topic_ = GetParam<std::string>("topics.localization_data",
                                          "/localization/localization_data");
    output_topic_aux_ = GetParam<std::string>("topics.localization_data_aux",
                                              "/localization/localization_data_aux");
    fault_topic_ = GetParam<std::string>("topics.fault", "fault_code/localization_fault");
    heartbeat_topic_ =
        GetParam<std::string>("topics.heartbeat",
                              "/localization/localization_parser/heartbeat");

    origin_lat_ = GetParam<double>("origin_lat", 0.0);
    origin_lon_ = GetParam<double>("origin_lon", 0.0);
    origin_height_ = GetParam<double>("origin_height", 0.0);
    world_frame_id_ = GetParam<std::string>("world_frame_id", "world");
    rtk_offset_yaw_ = GetParam<double>("rtk_offset_yaw", 0.0);

    data_timeout_sec_ = GetParam<double>("data_timeout_sec", 0.8);
    publish_rate_hz_ = GetParam<double>("publish_rate_hz", 50.0);
    aux_rate_hz_ = GetParam<double>("aux_rate_hz", 50.0);
    heartbeat_hz_ = GetParam<int>("heartbeat_hz", 1);

    gate_params_.sticky_grace_time =
        GetParam<double>("gate.sticky_grace_time", 30.0);
    gate_params_.single_point_age_threshold =
        GetParam<double>("gate.single_point_age_threshold", 10.0);
    gate_params_.xyz_threshold = GetParam<double>("gate.xyz_threshold", 10000.0);
    gate_params_.stars_threshold =
        static_cast<uint16_t>(GetParam<int>("gate.stars_threshold", 10));
    gate_params_.heading_length = GetParam<double>("gate.heading_length", 1.6);
    gate_params_.heading_length_threshold =
        GetParam<double>("gate.heading_length_threshold", 0.2);

    alarm_params_.startup_grace_s =
        GetParam<double>("alarm_startup_grace_s", 10.0);
    alarm_params_.recover_stable_s =
        GetParam<double>("alarm_recover_stable_s", 2.0);
    alarm_params_.min_interval_s =
        GetParam<double>("alarm_min_interval_s", 10.0);
    alarm_params_.cooldown_s = GetParam<double>("alarm_cooldown_s", 300.0);
    fault_code_number_ = GetParam<int>("fault_code", 12);
  }

  template <typename T>
  auto GetParam(const std::string& name, const T& default_value) -> T {
    if (!has_parameter(name)) {
      declare_parameter<T>(name, default_value);
    }
    return get_parameter(name).get_value<T>();
  }

  void CreatePublishers() {
    main_pub_ = create_publisher<LocalizationData>(
        output_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    aux_pub_ = create_publisher<LocalizationDataAux>(
        output_topic_aux_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    fault_pub_ = create_publisher<robominer_msgs::msg::FaultCodes>(
        fault_topic_, rclcpp::QoS(1).transient_local());
    heartbeat_pub_ = create_publisher<Float64>(
        heartbeat_topic_, rclcpp::QoS(1).transient_local());
  }

  void CreateSubscriptions() {
    sub_ = create_subscription<RtkFix>(
        input_topic_, rclcpp::SensorDataQoS(),
        [this](const RtkFix& msg) { OnRtkFix(msg); });
  }

  void CreateTimers() {
    const auto main_period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_));
    main_timer_ = create_wall_timer(main_period, [this]() { PublishMain(); });

    const auto aux_period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / aux_rate_hz_));
    aux_timer_ = create_wall_timer(aux_period, [this]() { PublishAux(); });

    int hb_hz = heartbeat_hz_;
    if (!(hb_hz >= 1 && hb_hz <= 1000)) {
      RCLCPP_WARN(get_logger(), "Invalid heartbeat_hz=%d, fallback to 1", hb_hz);
      hb_hz = 1;
    }
    heartbeat_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / hb_hz)), [this]() {
          Float64 msg;
          msg.data = now().seconds();
          heartbeat_pub_->publish(msg);
        });
  }

  static auto QualityToPosType(uint8_t q) -> PosType {
    switch (q) {
      case 1: return PosType::InsPsrsp;
      case 2: return PosType::InsPsrdiff;
      case 3: return PosType::InsRtkfloat;
      case 4: return PosType::InsRtkfixed;
      default: return PosType::None;
    }
  }

  static auto InsStatusFromMsg(uint8_t s) -> InsStatus {
    if (s <= 10) return static_cast<InsStatus>(s);
    return InsStatus::Unknown;
  }

  void OnRtkFix(const RtkFix& msg) {
    last_rtkfix_ = msg;
    has_rtkfix_ = true;
    last_rtkfix_time_ = std::chrono::steady_clock::now();

    const PosType pos_type = QualityToPosType(msg.quality);
    const InsStatus ins_status = InsStatusFromMsg(msg.ins_status);

    const Blh blh{msg.latitude, msg.longitude, msg.altitude};
    enu_ = converter_->blh_to_enu(blh);

    GateInput in;
    in.pos_type = pos_type;
    in.ins_status = ins_status;
    in.age = msg.diff_age;
    in.svs = msg.sats_main;
    in.svs2 = msg.sats_aux;
    in.heading_narrow_int = (msg.heading_type == RtkFix::HEADING_TYPE_NARROW_INT);
    in.heading_length = msg.baseline_length;
    in.x = enu_.e;
    in.y = enu_.n;
    in.z = enu_.u;
    gate_out_ = gate_->update(in, last_rtkfix_time_);

    // 告警状态机
    LocAlarmInput alarm_in;
    alarm_in.client_connected = true;
    alarm_in.is_valid = gate_out_.is_valid;
    alarm_in.fault_code = gate_out_.fault_code;
    alarm_in.rtk_status = gate_out_.rtk_status;
    alarm_in.svs = msg.sats_main;
    alarm_in.svs2 = msg.sats_aux;
    alarm_in.age = msg.diff_age;

    auto action = alarm_->update(!gate_out_.is_valid,
                                  LocAlarmState::build_fault_msg(alarm_in),
                                  last_rtkfix_time_);
    if (action) {
      PublishFault(action->add, action->msg);
    }

    PublishAuxDirectly(msg, pos_type, ins_status);
  }

  enum class Snapshot { kOk, kNoData, kTimeout };

  auto CurrentSnapshot() const -> Snapshot {
    if (!has_rtkfix_) return Snapshot::kNoData;
    const double stale =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      last_rtkfix_time_)
            .count();
    if (stale > data_timeout_sec_) return Snapshot::kTimeout;
    return Snapshot::kOk;
  }

  void PublishMain() {
    const Snapshot snap = CurrentSnapshot();
    LocalizationData msg;
    msg.header.stamp = now();
    msg.header.frame_id = world_frame_id_;

    if (snap == Snapshot::kOk) {
      const auto& r = last_rtkfix_;
      msg.is_valid = gate_out_.is_valid;
      msg.latitude = r.latitude;
      msg.longitude = r.longitude;
      msg.altitude = r.altitude;
      msg.x = enu_.e;
      msg.y = enu_.n;
      msg.z = enu_.u;
      msg.vx = r.vel_east;
      msg.vy = r.vel_north;
      msg.vz = r.vel_up;
      msg.speed = compute_speed(r.vel_north, r.vel_east, r.vel_up);
      msg.ax = r.acc_x;
      msg.ay = r.acc_y;
      msg.az = r.acc_z;
      msg.roll = r.roll;
      msg.pitch = -r.pitch;
      msg.yaw = compute_heading(r.heading, rtk_offset_yaw_);
      msg.wx = r.gyro_x;
      msg.wy = r.gyro_y;
      msg.wz = r.gyro_z;
      msg.solnsvs = r.sats_main;
      msg.solnsvs2 = r.sats_aux;
      msg.age = ClampToU8(r.diff_age);
      msg.gga_age = 0;
      msg.rtk_status.status = static_cast<uint8_t>(gate_out_.rtk_status);
      FillStaticStd(msg, r.sigma_enu);
    } else {
      const bool timeout = (snap == Snapshot::kTimeout);
      msg.is_valid = false;
      msg.rtk_status.status =
          static_cast<uint8_t>(timeout ? RtkStatus::DataTimeout : RtkStatus::RtkInit);
      msg.static_std_horizontal = -1.0;
      msg.static_std_vertical = -1.0;
    }

    main_pub_->publish(msg);
  }

  void PublishAuxDirectly(const RtkFix& r, PosType pos_type,
                          InsStatus ins_status) {
    LocalizationDataAux aux;
    aux.header.stamp = r.header.stamp;
    aux.header.frame_id = world_frame_id_;
    aux.ins_status = static_cast<uint8_t>(ins_status);
    aux.pos_type = static_cast<uint8_t>(pos_type);
    aux.heading_length = r.baseline_length;
    aux.angle_difference = 0.0;  // RtkFix 未提供
    aux.undulation = r.undulation;
    aux.gps_time = std::to_string(r.gnss_time > 0.0 ? r.gnss_time
                                                    : r.header.stamp.sec);
    aux.cors_age = 0;  // RtkFix 未提供该语义
    aux.part_solnsvs = r.sats_main;
    aux.part_solnsvs2 = r.sats_aux;
    aux.fault_code = static_cast<uint8_t>(gate_out_.fault_code);
    aux_pub_->publish(aux);
  }

  static void FillStaticStd(LocalizationData& msg,
                            const std::array<double, 3>& sigma_enu) {
    const double sde = std::isfinite(sigma_enu[0]) ? sigma_enu[0] : 0.0;
    const double sdn = std::isfinite(sigma_enu[1]) ? sigma_enu[1] : 0.0;
    const double sdu = std::isfinite(sigma_enu[2]) ? sigma_enu[2] : 0.0;
    msg.static_std_horizontal = std::hypot(sde, sdn);
    msg.static_std_vertical = sdu;
  }

  void PublishAux() {
    // 主回调已经直接发布 aux；定时器兜底：无新数据时按上一帧重发
    if (!has_rtkfix_) return;
    // 复用上一次的 gate_out_，因为字段未变
  }

  void PublishFault(bool add, const std::string& message) {
    robominer_msgs::msg::FaultCodes codes;
    robominer_msgs::msg::FaultCode fc;
    fc.header.stamp = now();
    fc.fault_code = fault_code_number_;
    fc.fault_action = add ? robominer_msgs::msg::FaultCode::FAULT_ADD
                          : robominer_msgs::msg::FaultCode::FAULT_RESET;
    fc.fault_trigger_module.trigger_module =
        robominer_msgs::msg::FaultTriggerModule::LOCALIZATION;
    fc.fault_level.level = robominer_msgs::msg::FaultLevel::CRITICAL;
    fc.fault_msg = message;
    codes.fault_codes.push_back(fc);
    fault_pub_->publish(codes);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_topic_aux_;
  std::string fault_topic_;
  std::string heartbeat_topic_;

  double origin_lat_ = 0.0;
  double origin_lon_ = 0.0;
  double origin_height_ = 0.0;
  std::string world_frame_id_;
  double rtk_offset_yaw_ = 0.0;

  double data_timeout_sec_ = 0.8;
  double publish_rate_hz_ = 50.0;
  double aux_rate_hz_ = 50.0;
  int heartbeat_hz_ = 1;
  int fault_code_number_ = 12;

  GateParams gate_params_;
  LocAlarmState::Params alarm_params_;

  std::unique_ptr<EnuConverter> converter_;
  std::unique_ptr<LocalizationGate> gate_;
  std::unique_ptr<LocAlarmState> alarm_;

  RtkFix last_rtkfix_;
  bool has_rtkfix_ = false;
  std::chrono::steady_clock::time_point last_rtkfix_time_;
  Enu enu_;
  GateOutput gate_out_;

  rclcpp::Publisher<LocalizationData>::SharedPtr main_pub_;
  rclcpp::Publisher<LocalizationDataAux>::SharedPtr aux_pub_;
  rclcpp::Publisher<robominer_msgs::msg::FaultCodes>::SharedPtr fault_pub_;
  rclcpp::Publisher<Float64>::SharedPtr heartbeat_pub_;
  rclcpp::Subscription<RtkFix>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::TimerBase::SharedPtr aux_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace gnss_bringup

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_bringup::LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
