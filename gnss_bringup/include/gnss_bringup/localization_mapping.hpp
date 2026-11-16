// 移植说明：本文件移植自 boonray_ws/src/localization/localization_parser
// （include/localization_parser/core/common/coordinate.hpp 与
// src/core/processor/beiyun_processor.cpp），Copyright (c) Boonray，
// 2026-11-16 移植到 gnss_bringup，去掉 Eigen / ROS 依赖，公式与常量保持不变。
//
// 纯逻辑：不依赖 ROS、不依赖 Eigen，可直接单测。
// 职责：设备量 → LocalizationData 字段的换算（BLH→ENU、姿态符号、标准差重排、
// IMU 缩放/轴序、速度合成）。
#pragma once
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <ctime>
#include <string>

#include "gnss_bringup/localization_types.hpp"

namespace gnss_bringup::loc {

// === WGS84 椭球参数（与旧实现一致） ===
constexpr double kSemiMajorAxis = 6378137.0;
constexpr double kFlattening = 1.0 / 298.257223563;
constexpr double kEccentricity2 = kFlattening * (2.0 - kFlattening);
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;

struct Blh {
  double lat_deg{0.0};   ///< 纬度（度）
  double lon_deg{0.0};   ///< 经度（度）
  double height_m{0.0};  ///< 椭球高（米）
};

struct Enu {
  double e{0.0};  ///< 东（米）
  double n{0.0};  ///< 北（米）
  double u{0.0};  ///< 天（米）
};

/// BLH → ECEF。非有限输入返回零向量（沿用旧实现的防御行为，调用方随后会
/// 因 ENU 越界门控判无效）。
inline auto blh_to_ecef(const Blh& blh) -> Enu {
  if (!std::isfinite(blh.lat_deg) || !std::isfinite(blh.lon_deg) ||
      !std::isfinite(blh.height_m)) {
    return Enu{0.0, 0.0, 0.0};
  }
  const double lat = blh.lat_deg * kDeg2Rad;
  const double lon = blh.lon_deg * kDeg2Rad;
  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double sin_lon = std::sin(lon);
  const double cos_lon = std::cos(lon);
  const double n_rad = kSemiMajorAxis /
                       std::sqrt(1.0 - kEccentricity2 * sin_lat * sin_lat);
  Enu ecef;
  ecef.e = (n_rad + blh.height_m) * cos_lat * cos_lon;
  ecef.n = (n_rad + blh.height_m) * cos_lat * sin_lon;
  ecef.u = (n_rad * (1.0 - kEccentricity2) + blh.height_m) * sin_lat;
  return ecef;
}

/// 以配置原点为站心的 BLH→ENU 转换器。构造时预计算原点 ECEF 与三角函数，
/// 与旧 common::Coordinate 等价（旧类用 Eigen::Vector3d，这里用 Enu 承载）。
class EnuConverter {
 public:
  explicit EnuConverter(const Blh& origin) : origin_(origin) {
    origin_ecef_ = blh_to_ecef(origin);
    const double lat = origin.lat_deg * kDeg2Rad;
    const double lon = origin.lon_deg * kDeg2Rad;
    sin_lat_ = std::sin(lat);
    cos_lat_ = std::cos(lat);
    sin_lon_ = std::sin(lon);
    cos_lon_ = std::cos(lon);
  }

  const Blh& origin() const { return origin_; }

  auto blh_to_enu(const Blh& blh) const -> Enu {
    const Enu ecef = blh_to_ecef(blh);
    const double dx = ecef.e - origin_ecef_.e;
    const double dy = ecef.n - origin_ecef_.n;
    const double dz = ecef.u - origin_ecef_.u;
    Enu enu;
    enu.e = -sin_lon_ * dx + cos_lon_ * dy;
    enu.n = -sin_lat_ * cos_lon_ * dx - sin_lat_ * sin_lon_ * dy + cos_lat_ * dz;
    enu.u = cos_lat_ * cos_lon_ * dx + cos_lat_ * sin_lon_ * dy + sin_lat_ * dz;
    return enu;
  }

 private:
  Blh origin_;
  Enu origin_ecef_;
  double sin_lat_{0.0};
  double cos_lat_{0.0};
  double sin_lon_{0.0};
  double cos_lon_{0.0};
};

/// 归一化到 [-180, 180]；NaN/Inf 原样返回（避免 Inf 进入 fmod 后的死循环）
inline auto normalize_angle(double angle) -> double {
  if (!std::isfinite(angle)) return angle;
  angle = std::fmod(angle, 360.0);
  if (angle > 180.0) angle -= 360.0;
  if (angle < -180.0) angle += 360.0;
  return angle;
}

/// 设备方位角 → ENU 航向：heading = normalize(90 - azimuth + offset)
/// 旧实现的 Data.pitch 取反、yaw 用本公式；符号约定不得改，否则下游控制反向。
inline auto compute_heading(double azimuth_deg, double offset_deg) -> double {
  return normalize_angle(90.0 - azimuth_deg + offset_deg);
}

/// 速度大小（m/s），保留到 mm/s（旧实现对 3 位小数做四舍五入）
inline auto compute_speed(double north, double east, double up) -> double {
  return std::round(std::sqrt(north * north + east * east + up * up) * 1000.0) /
         1000.0;
}

// === IMU 量纲/轴序 ===
// 旧实现直接消费设备原始 CORRIMUDATAA：角速度 ×125 再转弧度、加速度 ×125，
// 且做 X/Y 轴交换。北云 ros_driver 发布的 corrimudata 大概率已完成单位换算
// （deg/s、m/s²）并可能已按车体轴输出——两者不能无条件照搬，因此把三个
// 自由度都做成参数，由 bridge 按实测（T0 比对）配置。
struct ImuInput {
  double rate_x{0.0};
  double rate_y{0.0};
  double rate_z{0.0};
  double acc_x{0.0};
  double acc_y{0.0};
  double acc_z{0.0};
};

struct ImuOutput {
  double wx{0.0};  ///< 角速度（deg/s）
  double wy{0.0};
  double wz{0.0};
  double ax{0.0};  ///< 加速度（m/s²）
  double ay{0.0};
  double az{0.0};
};

struct ImuScale {
  double rate_scale{1.0};  ///< 角速度系数（原始报文用 125 * deg→rad，驱动输出用 1）
  double acc_scale{1.0};   ///< 加速度系数（原始报文用 125，驱动输出用 1）
  bool swap_xy{false};     ///< 是否交换 X/Y 轴（旧实现对原始报文置 true）
};

/// CORRIMUDATA（原始报文与北云驱动话题一致）的换算参数：
/// 字段是每个采样间隔的增量（角速度 rad/sample、速度 m/s/sample），
/// 乘以 IMU 频率得到 rad/s、m/s²，角速度再 rad→deg 得到消息契约的 deg/s；
/// 同时做 X/Y 交换（旧实现同款，字段顺序 roll/pitch → x/y 相反）。
inline auto corrimudata_imu_scale(double imu_rate_hz) -> ImuScale {
  if (!std::isfinite(imu_rate_hz) || imu_rate_hz <= 0.0) {
    imu_rate_hz = 100.0;  // 实车 IMU 100 Hz
  }
  return ImuScale{imu_rate_hz * kRad2Deg, imu_rate_hz, true};
}

/// 假设驱动已输出 deg/s、m/s² 且轴序正确的默认参数（华测等已换算的源）
constexpr auto driver_imu_scale() -> ImuScale { return ImuScale{}; }

inline auto convert_imu(const ImuInput& in, const ImuScale& scale) -> ImuOutput {
  ImuOutput out;
  out.wx = (scale.swap_xy ? in.rate_y : in.rate_x) * scale.rate_scale;
  out.wy = (scale.swap_xy ? in.rate_x : in.rate_y) * scale.rate_scale;
  out.wz = in.rate_z * scale.rate_scale;
  out.ax = (scale.swap_xy ? in.acc_y : in.acc_x) * scale.acc_scale;
  out.ay = (scale.swap_xy ? in.acc_x : in.acc_y) * scale.acc_scale;
  out.az = in.acc_z * scale.acc_scale;
  return out;
}

// === 定位不确定度：设备 BLH/ENU 序 → 消息 ENU 序 ===
struct InsStdInput {
  bool has_std{false};
  double lon_std{0.0};         ///< 经度标准差 → 东
  double lat_std{0.0};         ///< 纬度标准差 → 北
  double height_std{0.0};      ///< 高度标准差 → 天
  double east_vel_std{0.0};
  double north_vel_std{0.0};
  double up_vel_std{0.0};
  double roll_std{0.0};
  double pitch_std{0.0};
  double azimuth_std{0.0};
};

struct PosStdEnu {
  bool has_std{false};
  double pos_std_x{0.0};    ///< 东
  double pos_std_y{0.0};    ///< 北
  double pos_std_z{0.0};    ///< 天
  double vel_std_x{0.0};
  double vel_std_y{0.0};
  double vel_std_z{0.0};
  double att_std_roll{0.0};
  double att_std_pitch{0.0};
  double att_std_yaw{0.0};
};

inline auto reorder_std(const InsStdInput& in) -> PosStdEnu {
  PosStdEnu out;
  out.has_std = in.has_std;
  out.pos_std_x = in.lon_std;
  out.pos_std_y = in.lat_std;
  out.pos_std_z = in.height_std;
  out.vel_std_x = in.east_vel_std;
  out.vel_std_y = in.north_vel_std;
  out.vel_std_z = in.up_vel_std;
  out.att_std_roll = in.roll_std;
  out.att_std_pitch = in.pitch_std;
  out.att_std_yaw = in.azimuth_std;
  return out;
}

// === novatel_oem7_msgs / 北云驱动的枚举 → 本包契约枚举 ===
// 驱动的 INSPVAX.ins_status 是 NovAtel 原始枚举（INS_SOLUTION_GOOD=3），
// 而 robominer_msgs 的 LocalizationDataAux.ins_status 用另一套编号
// （INS_SOLUTION_GOOD=8），不能直接透传。
inline auto ins_status_from_oem7(uint32_t v) -> InsStatus {
  switch (v) {
    case 0:  return InsStatus::InsInactive;
    case 1:  return InsStatus::InsAligning;
    case 2:  return InsStatus::InsHighVariance;
    case 3:  return InsStatus::InsSolutionGood;   // OEM7 INS_SOLUTION_GOOD
    case 6:  return InsStatus::InsSolutionFree;   // OEM7 INS_SOLUTION_FREE
    case 7:  return InsStatus::InsAlignmentComplete;
    case 8:  return InsStatus::DeterminingOrientation;
    case 9:  return InsStatus::WaitingInitialPos;
    case 10: return InsStatus::WaitingAzimuth;
    case 11: return InsStatus::InitializingBiases;
    case 12: return InsStatus::MotionDetect;
    default: return InsStatus::Unknown;
  }
}

/// INSPVAX.pos_type（NovAtel PositionOrVelocityType）→ 本包 PosType
inline auto pos_type_from_oem7(uint32_t v) -> PosType {
  switch (v) {
    case 53: return PosType::InsPsrsp;     // INS_PSRSP
    case 54: return PosType::InsPsrdiff;   // INS_PSRDIFF
    case 55: return PosType::InsRtkfloat;  // INS_RTKFLOAT
    case 56: return PosType::InsRtkfixed;  // INS_RTKFIXED
    default: return PosType::None;
  }
}

/// HEADING2.pos_type == NARROW_INT(50)：双天线航向为窄巷固定解，
/// 只有该解类型参与基线长度门控（与旧实现一致）。
constexpr auto heading_narrow_int_from_oem7(uint32_t v) -> bool { return v == 50; }

/// GPS 周 + 周内毫秒 → 北京时间字符串（YYYY-MM-DD HH:MM:SS.mmm）
/// 与旧 TimeUtils::GpsToBeijingTime 同一公式：gps_total - leap + 8h + GPS 历元偏移。
inline auto gps_to_beijing_time(uint16_t gps_week, uint32_t week_ms,
                                int leap_seconds = 18) -> std::string {
  static constexpr int64_t kGpsEpochOffset = 315964800;
  const double gps_total = static_cast<double>(gps_week) * 604800.0 +
                           static_cast<double>(week_ms) / 1000.0;
  const double unix_ts = gps_total - static_cast<double>(leap_seconds) +
                         8.0 * 3600.0 + static_cast<double>(kGpsEpochOffset);
  if (!std::isfinite(unix_ts)) return {};
  const auto whole = std::floor(unix_ts);
  const time_t ts = static_cast<time_t>(whole);
  const int ms = static_cast<int>((unix_ts - whole) * 1000.0);
  struct tm tm_buf {};
  if (gmtime_r(&ts, &tm_buf) == nullptr) return {};
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d.%03d",
                tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec, ms);
  return buf;
}

}  // namespace gnss_bringup::loc
