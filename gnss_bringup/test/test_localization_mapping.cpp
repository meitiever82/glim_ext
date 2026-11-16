#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <string>
#include "gnss_bringup/localization_mapping.hpp"

using namespace gnss_bringup::loc;

namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

// ---------- 角度：归一化与航向 ----------

TEST(LocNormalizeAngle, WrapsIntoMinus180To180) {
  EXPECT_DOUBLE_EQ(normalize_angle(0.0), 0.0);
  EXPECT_DOUBLE_EQ(normalize_angle(190.0), -170.0);
  EXPECT_DOUBLE_EQ(normalize_angle(-190.0), 170.0);
  EXPECT_DOUBLE_EQ(normalize_angle(540.0), 180.0);
  EXPECT_DOUBLE_EQ(normalize_angle(-540.0), -180.0);
  // 端点不翻转（旧实现用 fmod，180/-180 都原样保留）
  EXPECT_DOUBLE_EQ(normalize_angle(180.0), 180.0);
  EXPECT_DOUBLE_EQ(normalize_angle(-180.0), -180.0);
}

TEST(LocNormalizeAngle, NonFinitePassesThrough) {
  EXPECT_TRUE(std::isnan(normalize_angle(std::numeric_limits<double>::quiet_NaN())));
  EXPECT_EQ(normalize_angle(std::numeric_limits<double>::infinity()),
            std::numeric_limits<double>::infinity());
}

TEST(LocComputeHeading, MatchesLegacyFormula) {
  // 旧公式：normalize(90 - azimuth + offset)
  EXPECT_DOUBLE_EQ(compute_heading(0.0, 0.0), 90.0);
  EXPECT_DOUBLE_EQ(compute_heading(90.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(compute_heading(180.0, 0.0), -90.0);
  EXPECT_DOUBLE_EQ(compute_heading(270.0, 0.0), -180.0);
  // 安装偏差角叠加后仍归一化
  EXPECT_DOUBLE_EQ(compute_heading(0.0, 30.0), 120.0);
  EXPECT_DOUBLE_EQ(compute_heading(350.0, 0.0), 100.0);
}

// ---------- BLH → ENU ----------

TEST(EnuConverter, OriginMapsToZero) {
  const Blh origin{31.0988603033, 119.72496128, 77.669};
  const EnuConverter conv(origin);
  const Enu enu = conv.blh_to_enu(origin);
  EXPECT_NEAR(enu.e, 0.0, 1e-6);
  EXPECT_NEAR(enu.n, 0.0, 1e-6);
  EXPECT_NEAR(enu.u, 0.0, 1e-6);
}

TEST(EnuConverter, NorthEastUpAxes) {
  const Blh origin{31.0988603033, 119.72496128, 77.669};
  const EnuConverter conv(origin);

  // 纬度 +0.0001° ≈ 北向 11.13 m，东向应近似 0
  const Enu north = conv.blh_to_enu(Blh{origin.lat_deg + 0.0001, origin.lon_deg,
                                       origin.height_m});
  EXPECT_NEAR(north.n, 11.13, 0.05);
  EXPECT_NEAR(north.e, 0.0, 0.01);
  EXPECT_NEAR(north.u, 0.0, 0.05);

  // 经度 +0.0001° 在该纬度 ≈ 东向 9.5 m
  const Enu east = conv.blh_to_enu(Blh{origin.lat_deg, origin.lon_deg + 0.0001,
                                      origin.height_m});
  EXPECT_NEAR(east.e, 9.53, 0.05);
  EXPECT_NEAR(east.n, 0.0, 0.01);

  // 高度 +100 m 基本全部落在天向
  const Enu up = conv.blh_to_enu(
      Blh{origin.lat_deg, origin.lon_deg, origin.height_m + 100.0});
  EXPECT_NEAR(up.u, 100.0, 1e-6);
  EXPECT_NEAR(up.e, 0.0, 1e-6);
  EXPECT_NEAR(up.n, 0.0, 1e-6);
}

TEST(EnuConverter, NonFiniteBlhProducesHugeEnuNotNan) {
  // 沿用旧实现：非有限 BLH → ECEF 零向量 → 减去原点后得到一个巨大的有限 ENU，
  // 由后续 ENU 越界门控判无效（而不是在这里抛异常或返回 NaN）。
  const Blh origin{31.0988603033, 119.72496128, 77.669};
  const EnuConverter conv(origin);
  const Enu enu = conv.blh_to_enu(
      Blh{std::numeric_limits<double>::quiet_NaN(), origin.lon_deg, origin.height_m});
  EXPECT_TRUE(std::isfinite(enu.e));
  EXPECT_GT(std::abs(enu.e) + std::abs(enu.n) + std::abs(enu.u), 1e6);
}

// ---------- 标准差重排 ----------

TEST(LocReorderStd, LonToEastLatToNorthHeightToUp) {
  InsStdInput in;
  in.has_std = true;
  in.lon_std = 1.0;
  in.lat_std = 2.0;
  in.height_std = 3.0;
  in.east_vel_std = 4.0;
  in.north_vel_std = 5.0;
  in.up_vel_std = 6.0;
  in.roll_std = 7.0;
  in.pitch_std = 8.0;
  in.azimuth_std = 9.0;

  const PosStdEnu out = reorder_std(in);
  EXPECT_TRUE(out.has_std);
  EXPECT_DOUBLE_EQ(out.pos_std_x, 1.0);
  EXPECT_DOUBLE_EQ(out.pos_std_y, 2.0);
  EXPECT_DOUBLE_EQ(out.pos_std_z, 3.0);
  EXPECT_DOUBLE_EQ(out.vel_std_x, 4.0);
  EXPECT_DOUBLE_EQ(out.vel_std_y, 5.0);
  EXPECT_DOUBLE_EQ(out.vel_std_z, 6.0);
  EXPECT_DOUBLE_EQ(out.att_std_roll, 7.0);
  EXPECT_DOUBLE_EQ(out.att_std_pitch, 8.0);
  EXPECT_DOUBLE_EQ(out.att_std_yaw, 9.0);
}

// ---------- IMU 量纲 / 轴序 ----------

TEST(LocConvertImu, DriverDefaultIsIdentity) {
  ImuInput in;
  in.rate_x = 1.5;
  in.rate_y = -2.5;
  in.rate_z = 3.5;
  in.acc_x = 0.1;
  in.acc_y = 0.2;
  in.acc_z = 9.8;
  const ImuOutput out = convert_imu(in, driver_imu_scale());
  EXPECT_DOUBLE_EQ(out.wx, 1.5);
  EXPECT_DOUBLE_EQ(out.wy, -2.5);
  EXPECT_DOUBLE_EQ(out.wz, 3.5);
  EXPECT_DOUBLE_EQ(out.ax, 0.1);
  EXPECT_DOUBLE_EQ(out.ay, 0.2);
  EXPECT_DOUBLE_EQ(out.az, 9.8);
}

TEST(LocConvertImu, RawCorrimudataScaleAndSwap) {
  ImuInput in;
  in.rate_x = 2.0;
  in.rate_y = 3.0;
  in.rate_z = 1.0;
  in.acc_x = 4.0;
  in.acc_y = 5.0;
  in.acc_z = 6.0;
  const ImuOutput out = convert_imu(in, corrimudata_imu_scale(100.0));

  // X/Y 交换 + 100 Hz + 角速度 rad→deg
  EXPECT_NEAR(out.wx, 3.0 * 100.0 * 180.0 / kPi, 1e-9);
  EXPECT_NEAR(out.wy, 2.0 * 100.0 * 180.0 / kPi, 1e-9);
  EXPECT_NEAR(out.wz, 1.0 * 100.0 * 180.0 / kPi, 1e-9);
  EXPECT_NEAR(out.ax, 5.0 * 100.0, 1e-9);
  EXPECT_NEAR(out.ay, 4.0 * 100.0, 1e-9);
  EXPECT_NEAR(out.az, 6.0 * 100.0, 1e-9);
}

TEST(LocConvertImu, CorrimudataScaleFromImuRate) {
  ImuInput in;
  in.rate_x = 2.0;
  in.rate_y = 3.0;
  const ImuOutput out = convert_imu(in, corrimudata_imu_scale(100.0));
  EXPECT_NEAR(out.wx, 3.0 * 100.0 * 180.0 / kPi, 1e-9);
  EXPECT_NEAR(out.wy, 2.0 * 100.0 * 180.0 / kPi, 1e-9);
}

TEST(LocConvertImu, CorrimudataScaleFallbackOnInvalidRate) {
  const ImuScale s = corrimudata_imu_scale(0.0);
  EXPECT_DOUBLE_EQ(s.rate_scale, 100.0 * 180.0 / kPi);
  EXPECT_DOUBLE_EQ(s.acc_scale, 100.0);
  EXPECT_TRUE(s.swap_xy);
}

// ---------- OEM7 枚举映射 / GPS 时间 ----------

TEST(LocOem7Mapping, InsStatus) {
  EXPECT_EQ(ins_status_from_oem7(3), InsStatus::InsSolutionGood);
  EXPECT_EQ(ins_status_from_oem7(10), InsStatus::WaitingAzimuth);
  EXPECT_EQ(ins_status_from_oem7(12), InsStatus::MotionDetect);
  EXPECT_EQ(ins_status_from_oem7(99), InsStatus::Unknown);
}

TEST(LocOem7Mapping, PosType) {
  EXPECT_EQ(pos_type_from_oem7(53), PosType::InsPsrsp);
  EXPECT_EQ(pos_type_from_oem7(56), PosType::InsRtkfixed);
  EXPECT_EQ(pos_type_from_oem7(0), PosType::None);
  EXPECT_TRUE(heading_narrow_int_from_oem7(50));
  EXPECT_FALSE(heading_narrow_int_from_oem7(34));
}

TEST(LocOem7Mapping, GpsToBeijingTime) {
  // 2026-01-01 00:00:00 GPS（week 2399, tow 345618s）→ 北京时间 08:00:00.000
  const std::string t = gps_to_beijing_time(2399, 345618000, 18);
  EXPECT_EQ(t, "2026-01-01 08:00:00.000");
}

// ---------- 速度 ----------

TEST(LocComputeSpeed, MagnitudeRoundedToMillimeter) {
  EXPECT_DOUBLE_EQ(compute_speed(3.0, 4.0, 0.0), 5.0);
  EXPECT_DOUBLE_EQ(compute_speed(0.0, 0.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(compute_speed(1.0, 1.0, 0.0), 1.414);  // sqrt(2)=1.41421...
  EXPECT_DOUBLE_EQ(compute_speed(-3.0, -4.0, 0.0), 5.0);
}

// ---------- 字符串/枚举映射 ----------

TEST(LocTypes, PosTypeAndInsStatusParsing) {
  EXPECT_EQ(pos_type_from_string("INS_RTKFIXED"), PosType::InsRtkfixed);
  EXPECT_EQ(pos_type_from_string("INS_RTKFLOAT"), PosType::InsRtkfloat);
  EXPECT_EQ(pos_type_from_string("INS_PSRSP"), PosType::InsPsrsp);
  EXPECT_EQ(pos_type_from_string("INS_PSRDIFF"), PosType::InsPsrdiff);
  EXPECT_EQ(pos_type_from_string("INS_NONE"), PosType::None);
  EXPECT_EQ(pos_type_from_string(""), PosType::None);

  EXPECT_EQ(ins_status_from_string("INS_SOLUTION_GOOD"), InsStatus::InsSolutionGood);
  EXPECT_EQ(ins_status_from_string("INS_INACTIVE"), InsStatus::InsInactive);
  EXPECT_EQ(ins_status_from_string("MOTION_DETECT"), InsStatus::MotionDetect);
  EXPECT_EQ(ins_status_from_string("WHATEVER"), InsStatus::Unknown);
}

TEST(LocTypes, RtkStatusFromPosTypeIsDeviceQuality) {
  EXPECT_EQ(rtk_status_from_pos_type(PosType::InsRtkfixed), RtkStatus::RtkFix);
  EXPECT_EQ(rtk_status_from_pos_type(PosType::InsRtkfloat), RtkStatus::RtkFloat);
  EXPECT_EQ(rtk_status_from_pos_type(PosType::InsPsrsp), RtkStatus::Aposit);
  EXPECT_EQ(rtk_status_from_pos_type(PosType::InsPsrdiff), RtkStatus::RtcmDiff);
  EXPECT_EQ(rtk_status_from_pos_type(PosType::None), RtkStatus::RtkInit);
}

TEST(LocTypes, RaiseFaultCodeKeepsHigherPriority) {
  FaultCode cur = FaultCode::None;
  raise_fault_code(cur, FaultCode::SatelliteInsufficient);
  EXPECT_EQ(cur, FaultCode::SatelliteInsufficient);
  // 低优先级不覆盖
  raise_fault_code(cur, FaultCode::GraceTimeout);
  EXPECT_EQ(cur, FaultCode::SatelliteInsufficient);
  // 更高优先级覆盖
  raise_fault_code(cur, FaultCode::EnuOutOfBounds);
  EXPECT_EQ(cur, FaultCode::EnuOutOfBounds);
}
