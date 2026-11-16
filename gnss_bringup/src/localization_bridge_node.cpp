// 定位桥接节点：消费北云驱动已发布的 OEM7 话题，输出与旧
// localization_parser 兼容的 LocalizationData / LocalizationDataAux、
// 202012 故障码与进程心跳。
//
// 与旧实现的关系（见 docs/gnss/plans/2026-11-16-gnss_bringup-replace-localization.md）：
// - 数据源从设备 TCP 原始流换成驱动话题，字段语义与门控逐条复刻
//   （门控/告警纯逻辑见 localization_gate.hpp / loc_alarm_state.hpp）；
// - 不发动态 TF：动态 map->base_link 由北云驱动独占（方案 A），
//   launch 只补一条静态 world->map；
// - 时间戳用节点系统时间（与旧实现一致，不用 GPS 数据时间）。
//
// 字段来源：
//   INSPVAX  位置/姿态/速度/ins_status/pos_type/undulation/GPS 时间
//   BESTPOS  diff_age / num_sol_svs
//   HEADING2 基线长度 / num_sv_obs / 解类型（仅 NARROW_INT 参与基线段门控）
//   CORRIMU  每个 IMU 采样间隔的角速度/速度增量
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <novatel_oem7_msgs/msg/bestpos.hpp>
#include <novatel_oem7_msgs/msg/corrimu.hpp>
#include <novatel_oem7_msgs/msg/heading2.hpp>
#include <novatel_oem7_msgs/msg/inspvax.hpp>
#include <robominer_msgs/msg/fault_codes.hpp>
#include <robominer_msgs/msg/localization_data.hpp>
#include <robominer_msgs/msg/localization_data_aux.hpp>
#include <std_msgs/msg/float64.hpp>

#include "gnss_bringup/loc_alarm_state.hpp"
#include "gnss_bringup/localization_gate.hpp"
#include "gnss_bringup/localization_mapping.hpp"
#include "gnss_bringup/localization_types.hpp"

namespace gnss_bringup {
namespace {

using loc::EnuConverter;
using loc::FaultCode;
using loc::GateInput;
using loc::GateOutput;
using loc::GateParams;
using loc::ImuInput;
using loc::ImuScale;
using loc::LocalizationGate;
using loc::LocAlarmAction;
using loc::LocAlarmInput;
using loc::LocAlarmState;
using loc::RtkStatus;

using Inspvax = novatel_oem7_msgs::msg::INSPVAX;
using Bestpos = novatel_oem7_msgs::msg::BESTPOS;
using CorrImu = novatel_oem7_msgs::msg::CORRIMU;
using Heading2 = novatel_oem7_msgs::msg::HEADING2;

constexpr double kMinRateHz = 0.001;
constexpr double kMaxRateHz = 1000.0;

/// 频率参数防御：非法值（<=0 / NaN / 越界）回退默认，避免 1.0/rate 溢出或定时器周期 0
auto SanitizeRateHz(double rate_hz, double default_hz, rclcpp::Logger logger,
                    const char* name) -> double {
  if (!(rate_hz >= kMinRateHz && rate_hz <= kMaxRateHz)) {
    RCLCPP_WARN(logger, "Invalid %s=%.4f, fallback to %.1f Hz", name, rate_hz,
                default_hz);
    return default_hz;
  }
  return rate_hz;
}

auto ClampToU8(double v) -> uint8_t {
  if (!std::isfinite(v)) return 0;
  return static_cast<uint8_t>(std::clamp(v, 0.0, 255.0));
}

}  // namespace

class LocalizationBridge : public rclcpp::Node {
 public:
  LocalizationBridge() : rclcpp::Node("localization_bridge") {
    LoadParams();

    enu_converter_ = std::make_unique<EnuConverter>(
        loc::Blh{origin_lat_, origin_lon_, origin_height_});
    gate_ = std::make_unique<LocalizationGate>(gate_params_);
    alarm_ = std::make_unique<LocAlarmState>(alarm_params_);

    CreatePublishers();
    CreateSubscriptions();
    CreateTimers();

    RCLCPP_INFO(get_logger(),
                "localization_bridge: origin=(%.8f, %.8f, %.3f) imu_rate=%.1fHz "
                "main=%s aux=%s heartbeat=%s",
                origin_lat_, origin_lon_, origin_height_, imu_rate_hz_,
                main_topic_.c_str(), aux_topic_.c_str(),
                heartbeat_topic_.c_str());
  }

 private:
  // === 参数 ===
  void LoadParams() {
    origin_lat_ = GetParam<double>("origin_lat", 0.0);
    origin_lon_ = GetParam<double>("origin_lon", 0.0);
    origin_height_ = GetParam<double>("origin_height", 0.0);
    rtk_offset_yaw_ = GetParam<double>("rtk_offset_yaw", 0.0);
    leap_seconds_ = GetParam<int>("leap_seconds", 18);
    world_frame_id_ = GetParam<std::string>("world_frame_id", "world");
    data_timeout_sec_ = GetParam<double>("data_timeout_sec", 0.8);
    imu_rate_hz_ = GetParam<double>("imu_rate_hz", 100.0);
    if (!(imu_rate_hz_ > 0.0)) {
      RCLCPP_WARN(get_logger(), "Invalid imu_rate_hz=%.3f, fallback to 100 Hz",
                  imu_rate_hz_);
      imu_rate_hz_ = 100.0;
    }
    imu_scale_ = loc::corrimudata_imu_scale(imu_rate_hz_);

    publish_rate_hz_ = SanitizeRateHz(
        GetParam<double>("publish_rate_hz", 50.0), 50.0, get_logger(),
        "publish_rate_hz");
    aux_rate_hz_ =
        SanitizeRateHz(GetParam<double>("aux_rate_hz", 50.0), 50.0,
                       get_logger(), "aux_rate_hz");

    main_topic_ = GetParam<std::string>("localization_topic",
                                        "/localization/localization_data");
    aux_topic_ = GetParam<std::string>(
        "localization_topic_aux", "/localization/localization_data_aux");
    fault_topic_ =
        GetParam<std::string>("fault_topic", "fault_code/localization_fault");
    heartbeat_topic_ = GetParam<std::string>(
        "heartbeat_topic", "/localization/localization_parser/heartbeat");
    heartbeat_hz_ = GetParam<int>("heartbeat_hz", 1);

    inspvax_topic_ = GetParam<std::string>("topics.inspvax", "/bynav/inspvax");
    bestpos_topic_ = GetParam<std::string>("topics.bestpos", "/bynav/bestpos");
    corrimudata_topic_ =
        GetParam<std::string>("topics.corrimudata", "/bynav/corrimudata");
    heading2_topic_ = GetParam<std::string>("topics.heading2", "/bynav/heading2");

    GateParams gp;
    gp.sticky_grace_time = GetParam<double>("gate.sticky_grace_time", 30.0);
    gp.single_point_age_threshold =
        GetParam<double>("gate.single_point_age_threshold", 10.0);
    gp.xyz_threshold = GetParam<double>("gate.xyz_threshold", 10000.0);
    gp.stars_threshold =
        static_cast<uint16_t>(GetParam<int>("gate.stars_threshold", 10));
    gp.heading_length = GetParam<double>("gate.heading_length", 1.276);
    gp.heading_length_threshold =
        GetParam<double>("gate.heading_length_threshold", 0.2);
    gate_params_ = gp;

    LocAlarmState::Params ap;
    ap.startup_grace_s = GetParam<double>("alarm_startup_grace_s", 10.0);
    ap.recover_stable_s = GetParam<double>("alarm_recover_stable_s", 2.0);
    ap.min_interval_s = GetParam<double>("alarm_min_interval_s", 10.0);
    ap.cooldown_s = GetParam<double>("alarm_cooldown_s", 300.0);
    alarm_params_ = ap;

    const int fault_code = GetParam<int>("fault_code", 12);
    fault_code_ = (fault_code >= 1 && fault_code <= 255)
                      ? static_cast<uint8_t>(fault_code)
                      : static_cast<uint8_t>(12);
    if (fault_code != fault_code_) {
      RCLCPP_WARN(get_logger(),
                  "Invalid fault_code=%d (valid range [1,255]), fallback to 12",
                  fault_code);
    }
  }

  template <typename T>
  auto GetParam(const std::string& name, const T& default_value) -> T {
    if (!has_parameter(name)) {
      declare_parameter<T>(name, default_value);
    }
    return get_parameter(name).get_value<T>();
  }

  // === 发布器 ===
  void CreatePublishers() {
    rclcpp::QoS data_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    main_pub_ =
        create_publisher<robominer_msgs::msg::LocalizationData>(main_topic_,
                                                                data_qos);
    aux_pub_ = create_publisher<robominer_msgs::msg::LocalizationDataAux>(
        aux_topic_, data_qos);

    // transient_local：晚启动的 guardian 也能收到最后一条故障状态
    fault_pub_ = create_publisher<robominer_msgs::msg::FaultCodes>(
        fault_topic_, rclcpp::QoS(5).transient_local());

    // 心跳与业务解耦：进程存活即发布，内容为当前时间戳（秒）
    heartbeat_pub_ = create_publisher<std_msgs::msg::Float64>(
        heartbeat_topic_, rclcpp::QoS(1).transient_local());
  }

  // === 订阅 ===
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

  void CreateTimers() {
    const auto main_period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_));
    main_timer_ = create_wall_timer(main_period, [this]() { PublishMain(); });

    const auto aux_period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / aux_rate_hz_));
    aux_timer_ = create_wall_timer(aux_period, [this]() { PublishAux(); });

    int hb_hz = heartbeat_hz_;
    if (!(hb_hz >= 1 && hb_hz <= 1000)) {
      RCLCPP_WARN(get_logger(),
                  "Invalid heartbeat_hz=%d (valid range [1,1000]), fallback to 1",
                  hb_hz);
      hb_hz = 1;
    }
    heartbeat_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / hb_hz)), [this]() {
          std_msgs::msg::Float64 msg;
          msg.data = now().seconds();
          heartbeat_pub_->publish(msg);
        });
  }

  // === 输入回调 ===
  void OnInspvax(const Inspvax& msg) {
    inspvax_ = msg;
    has_inspvax_ = true;
    last_inspvax_ = std::chrono::steady_clock::now();

    // ins_status 未匹配时沿用上一帧（旧实现同语义，首帧回落 InsInactive）
    const auto ins = loc::ins_status_from_oem7(msg.ins_status.status);
    if (ins != loc::InsStatus::Unknown) {
      aux_ins_status_ = ins;
    } else if (!has_aux_ins_status_) {
      aux_ins_status_ = loc::InsStatus::InsInactive;
    }
    has_aux_ins_status_ = true;

    aux_pos_type_ = loc::pos_type_from_oem7(msg.pos_type.type);
    aux_gps_time_ =
        loc::gps_to_beijing_time(msg.nov_header.gps_week_number,
                                 msg.nov_header.gps_week_milliseconds,
                                 leap_seconds_);

    GateInput in;
    in.pos_type = aux_pos_type_;
    in.ins_status = aux_ins_status_;
    in.age = has_bestpos_ ? static_cast<double>(bestpos_.diff_age) : 0.0;
    in.svs = has_bestpos_ ? bestpos_.num_sol_svs : 0;
    in.svs2 = has_heading2_ ? heading2_.num_sv_obs : 0;
    in.heading_narrow_int =
        has_heading2_ && loc::heading_narrow_int_from_oem7(heading2_.pos_type.type);
    in.heading_length = has_heading2_ ? heading2_.length : 0.0;

    enu_ = enu_converter_->blh_to_enu(loc::Blh{msg.latitude, msg.longitude,
                                               msg.height});
    in.x = enu_.e;
    in.y = enu_.n;
    in.z = enu_.u;
    gate_out_ = gate_->update(in, last_inspvax_);
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

  // === 状态快照 ===
  enum class Snapshot { kOk, kNoData, kTimeout };

  auto CurrentSnapshot() const -> Snapshot {
    if (!has_inspvax_) return Snapshot::kNoData;
    const double stale =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      last_inspvax_)
            .count();
    if (stale > data_timeout_sec_) return Snapshot::kTimeout;
    return Snapshot::kOk;
  }

  auto CachedAge() const -> double {
    return has_bestpos_ ? static_cast<double>(bestpos_.diff_age) : 0.0;
  }

  auto CachedSvs() const -> uint16_t {
    return has_bestpos_ ? bestpos_.num_sol_svs : 0;
  }

  auto CachedSvs2() const -> uint16_t {
    return has_heading2_ ? heading2_.num_sv_obs : 0;
  }

  auto ImuNow() const -> loc::ImuOutput {
    if (!has_corrimu_) return loc::ImuOutput{};
    ImuInput in;
    // CORRIMU 字段顺序：pitch/roll/yaw + lateral/longitudinal/vertical
    in.rate_x = corrimu_.pitch_rate;
    in.rate_y = corrimu_.roll_rate;
    in.rate_z = corrimu_.yaw_rate;
    in.acc_x = corrimu_.lateral_acc;
    in.acc_y = corrimu_.longitudinal_acc;
    in.acc_z = corrimu_.vertical_acc;
    return loc::convert_imu(in, imu_scale_);
  }

  void PublishMain() {
    const Snapshot snap = CurrentSnapshot();
    robominer_msgs::msg::LocalizationData msg;
    msg.header.stamp = now();
    msg.header.frame_id = world_frame_id_;

    LocAlarmInput alarm_in;
    alarm_in.client_connected = has_inspvax_;

    if (snap == Snapshot::kOk) {
      const auto imu = ImuNow();
      msg.is_valid = gate_out_.is_valid;
      msg.latitude = inspvax_.latitude;
      msg.longitude = inspvax_.longitude;
      msg.altitude = inspvax_.height;
      msg.x = enu_.e;
      msg.y = enu_.n;
      msg.z = enu_.u;
      msg.vx = inspvax_.east_velocity;
      msg.vy = inspvax_.north_velocity;
      msg.vz = inspvax_.up_velocity;
      msg.speed = loc::compute_speed(inspvax_.north_velocity,
                                     inspvax_.east_velocity,
                                     inspvax_.up_velocity);
      msg.ax = imu.ax;
      msg.ay = imu.ay;
      msg.az = imu.az;
      msg.roll = inspvax_.roll;
      msg.pitch = -inspvax_.pitch;
      msg.yaw = loc::compute_heading(inspvax_.azimuth, rtk_offset_yaw_);
      msg.wx = imu.wx;
      msg.wy = imu.wy;
      msg.wz = imu.wz;
      msg.solnsvs = static_cast<uint8_t>(CachedSvs());
      msg.solnsvs2 = static_cast<uint8_t>(CachedSvs2());
      msg.age = ClampToU8(CachedAge());
      msg.gga_age = 0;  // 驱动未发布 GGA；旧实现该字段同样恒 0
      msg.rtk_status.status = static_cast<uint8_t>(gate_out_.rtk_status);

      alarm_in.is_valid = gate_out_.is_valid;
      alarm_in.fault_code = gate_out_.fault_code;
      alarm_in.rtk_status = gate_out_.rtk_status;
      alarm_in.svs = CachedSvs();
      alarm_in.svs2 = CachedSvs2();
      alarm_in.age = CachedAge();
    } else {
      const bool timeout = (snap == Snapshot::kTimeout);
      msg.is_valid = false;
      msg.rtk_status.status =
          static_cast<uint8_t>(timeout ? RtkStatus::DataTimeout
                                       : RtkStatus::RtkInit);

      alarm_in.is_valid = false;
      alarm_in.fault_code =
          timeout ? FaultCode::DataTimeout : FaultCode::NoData;
      alarm_in.rtk_status = timeout ? RtkStatus::DataTimeout : RtkStatus::RtkInit;
      alarm_in.svs = CachedSvs();
      alarm_in.svs2 = CachedSvs2();
      alarm_in.age = CachedAge();
    }

    LogTransitions(msg, alarm_in);
    CheckAlarm(alarm_in);
    main_pub_->publish(msg);
  }

  void PublishAux() {
    const Snapshot snap = CurrentSnapshot();
    robominer_msgs::msg::LocalizationDataAux msg;
    msg.header.stamp = now();
    msg.header.frame_id = world_frame_id_;
    msg.ins_status = static_cast<uint8_t>(aux_ins_status_);
    msg.pos_type = static_cast<uint8_t>(aux_pos_type_);
    msg.heading_length = has_heading2_ ? heading2_.length : 0.0;
    msg.angle_difference = 0.0;  // 驱动未提供航迹角差
    msg.undulation = has_inspvax_ ? inspvax_.undulation : 0.0;
    msg.gps_time = aux_gps_time_;
    msg.part_solnsvs = static_cast<uint8_t>(CachedSvs());
    // 旧实现 part_solnsvs2 取 HEADINGA.solnsvs（参与解算数），与 solnsvs2 的
    // HEADINGA.obs（观测数）不同，对应 HEADING2 的 num_sv_in_sol / num_sv_obs
    msg.part_solnsvs2 =
        has_heading2_ ? heading2_.num_sv_in_sol : static_cast<uint8_t>(0);
    msg.cors_age = 0;  // soft_dtu 已停用，与旧实现一致

    if (snap == Snapshot::kOk) {
      msg.fault_code = static_cast<uint8_t>(gate_out_.fault_code);
    } else {
      msg.fault_code = static_cast<uint8_t>(
          snap == Snapshot::kTimeout ? FaultCode::DataTimeout
                                     : FaultCode::NoData);
    }
    aux_pub_->publish(msg);
  }

  /// RTK / fault 跃迁日志（稳态不刷屏），携带触发上下文便于现场判断
  void LogTransitions(const robominer_msgs::msg::LocalizationData& msg,
                      const LocAlarmInput& in) {
    const auto rtk = static_cast<RtkStatus>(msg.rtk_status.status);
    if (!has_logged_ || rtk != last_logged_rtk_) {
      RCLCPP_INFO(get_logger(),
                  "[RTK] %s -> %s | valid=%d age=%.1f svs=%u/%u pos_type=%u "
                  "ins=%u enu=(%.1f,%.1f,%.1f)",
                  loc::rtk_status_to_string(last_logged_rtk_),
                  loc::rtk_status_to_string(rtk), msg.is_valid ? 1 : 0,
                  in.age, static_cast<unsigned>(in.svs),
                  static_cast<unsigned>(in.svs2),
                  static_cast<unsigned>(aux_pos_type_),
                  static_cast<unsigned>(aux_ins_status_), enu_.e, enu_.n, enu_.u);
      last_logged_rtk_ = rtk;
    }
    if (!has_logged_ || in.fault_code != last_logged_fault_) {
      if (in.fault_code == FaultCode::None) {
        RCLCPP_INFO(get_logger(), "[FAULT] %s -> None(0) | valid=%d rtk=%s",
                    loc::fault_code_to_string(last_logged_fault_),
                    msg.is_valid ? 1 : 0,
                    loc::rtk_status_to_string(rtk));
      } else {
        RCLCPP_WARN(get_logger(),
                    "[FAULT] %s -> %s | valid=%d rtk=%s svs=%u/%u age=%.1f "
                    "enu=(%.1f,%.1f,%.1f) pos_type=%u ins=%u",
                    loc::fault_code_to_string(last_logged_fault_),
                    loc::fault_code_to_string(in.fault_code),
                    msg.is_valid ? 1 : 0, loc::rtk_status_to_string(rtk),
                    static_cast<unsigned>(in.svs),
                    static_cast<unsigned>(in.svs2), in.age, enu_.e, enu_.n,
                    enu_.u, static_cast<unsigned>(aux_pos_type_),
                    static_cast<unsigned>(aux_ins_status_));
      }
      last_logged_fault_ = in.fault_code;
    }
    has_logged_ = true;
  }

  /// 202012 状态机：is_valid=false 即时 ADD，恢复稳定 2s 后 RESET
  void CheckAlarm(const LocAlarmInput& in) {
    const auto now_tp = std::chrono::steady_clock::now();
    std::optional<LocAlarmAction> act;
    if (!in.is_valid) {
      act = alarm_->update(true, LocAlarmState::build_fault_msg(in), now_tp);
    } else {
      act = alarm_->update(false, "", now_tp);
    }
    if (!act.has_value()) return;

    robominer_msgs::msg::FaultCodes codes;
    robominer_msgs::msg::FaultCode fc;
    fc.header.stamp = now();
    fc.fault_code = fault_code_;
    fc.fault_action = act->add ? robominer_msgs::msg::FaultCode::FAULT_ADD
                               : robominer_msgs::msg::FaultCode::FAULT_RESET;
    fc.fault_trigger_module.trigger_module =
        robominer_msgs::msg::FaultTriggerModule::LOCALIZATION;
    fc.fault_level.level = robominer_msgs::msg::FaultLevel::CRITICAL;
    fc.fault_msg = act->msg;
    codes.fault_codes.push_back(fc);
    fault_pub_->publish(codes);
    RCLCPP_WARN(get_logger(), "[localization fault] code=2020%02d action=%s msg=%s",
                static_cast<int>(fault_code_), act->add ? "ADD" : "RESET",
                act->msg.c_str());
  }

  // === 参数 ===
  double origin_lat_{0.0};
  double origin_lon_{0.0};
  double origin_height_{0.0};
  double rtk_offset_yaw_{0.0};
  int leap_seconds_{18};
  std::string world_frame_id_{"world"};
  double data_timeout_sec_{0.8};
  double imu_rate_hz_{100.0};
  double publish_rate_hz_{50.0};
  double aux_rate_hz_{50.0};
  int heartbeat_hz_{1};
  uint8_t fault_code_{12};

  std::string main_topic_;
  std::string aux_topic_;
  std::string fault_topic_;
  std::string heartbeat_topic_;
  std::string inspvax_topic_;
  std::string bestpos_topic_;
  std::string corrimudata_topic_;
  std::string heading2_topic_;

  GateParams gate_params_;
  ImuScale imu_scale_{};
  LocAlarmState::Params alarm_params_;

  // === 纯逻辑 ===
  std::unique_ptr<EnuConverter> enu_converter_;
  std::unique_ptr<LocalizationGate> gate_;
  std::unique_ptr<LocAlarmState> alarm_;
  GateOutput gate_out_{};
  loc::Enu enu_{};

  // === 缓存（单线程 executor：回调与定时器串行，无锁） ===
  bool has_inspvax_{false};
  bool has_bestpos_{false};
  bool has_heading2_{false};
  bool has_corrimu_{false};
  std::chrono::steady_clock::time_point last_inspvax_{};
  Inspvax inspvax_{};
  Bestpos bestpos_{};
  Heading2 heading2_{};
  CorrImu corrimu_{};

  loc::InsStatus aux_ins_status_{loc::InsStatus::InsInactive};
  loc::PosType aux_pos_type_{loc::PosType::None};
  bool has_aux_ins_status_{false};
  std::string aux_gps_time_;

  // 跃迁日志状态
  bool has_logged_{false};
  RtkStatus last_logged_rtk_{RtkStatus::RtkInit};
  FaultCode last_logged_fault_{FaultCode::None};

  // === ROS 句柄 ===
  rclcpp::Publisher<robominer_msgs::msg::LocalizationData>::SharedPtr main_pub_;
  rclcpp::Publisher<robominer_msgs::msg::LocalizationDataAux>::SharedPtr aux_pub_;
  rclcpp::Publisher<robominer_msgs::msg::FaultCodes>::SharedPtr fault_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heartbeat_pub_;
  rclcpp::Subscription<Inspvax>::SharedPtr inspvax_sub_;
  rclcpp::Subscription<Bestpos>::SharedPtr bestpos_sub_;
  rclcpp::Subscription<Heading2>::SharedPtr heading2_sub_;
  rclcpp::Subscription<CorrImu>::SharedPtr corrimu_sub_;
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::TimerBase::SharedPtr aux_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace gnss_bringup

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_bringup::LocalizationBridge>());
  rclcpp::shutdown();
  return 0;
}
