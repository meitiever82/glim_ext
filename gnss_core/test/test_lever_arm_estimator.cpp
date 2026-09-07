#include <gtest/gtest.h>

#include <cmath>

#include "gnss_core/geodetic.hpp"
#include "gnss_core/lever_arm_estimator.hpp"
#include "gnss_core/synth.hpp"
#include "synth_fixtures.hpp"

using namespace gnss_core;
using gnss_core::test_fixtures::straight_then_turn;
using gnss_core::test_fixtures::straight_then_turn_with_ramp;

namespace {

double yaw_of(const Eigen::Isometry3d& T) { return std::atan2(T.linear()(1, 0), T.linear()(0, 0)); }

Eigen::Isometry3d rotated_frame(double yaw, const Eigen::Vector3d& t) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() = t;
  return T;
}

}  // namespace

// PLAN Task 14 测试 1。轨迹是纯平面运动(无俯仰/横滚),杆臂 z 分量与 T_world_enu 的 z 平移
// 不可分离,因此断言水平分量 < 5 cm、z 标记为不可观(见 lever_z_observable),完整 3D 恢复见下一条。
TEST(LeverArm, RecoversLeverAndOffsetFromSyntheticData) {
  SynthConfig cfg;
  cfg.lever_imu = {0.3, 1.2, -0.5};
  cfg.sigma_fixed = {0.01, 0.01, 0.03};
  cfg.T_enu_world = rotated_frame(0.7, {50, -20, 3});
  auto traj = straight_then_turn();
  auto res = synthesize(traj, cfg, {});
  // 模拟 RTK 时间戳整体早 80 ms(即需要 +0.08 的 time_offset)
  for (auto& s : res.samples) s.stamp -= 0.08;
  LlaToEnu conv(cfg.lat0, cfg.lon0, cfg.alt0);
  auto est = estimate_lever_arm(traj, res.samples, conv);
  EXPECT_TRUE(est.lever_observable);
  EXPECT_FALSE(est.lever_z_observable);
  EXPECT_TRUE(est.time_offset_observable);
  EXPECT_NEAR(est.time_offset, 0.08, 0.011);
  EXPECT_LT((est.lever_imu.head<2>() - cfg.lever_imu.head<2>()).norm(), 0.05);
  EXPECT_DOUBLE_EQ(est.lever_imu.z(), 0.0);   // 不可观时钉在 0(config 默认值)
  EXPECT_LT(est.rms_residual, 0.05);
  EXPECT_GT(est.n_pairs, 400);
  // T_world_enu ≈ T_enu_world⁻¹(z 平移吸收了 lever_z,只比 xy)
  const Eigen::Isometry3d T_true = cfg.T_enu_world.inverse();
  EXPECT_NEAR(yaw_of(est.T_world_enu), yaw_of(T_true), 2e-3);
  EXPECT_LT((est.T_world_enu.translation().head<2>() - T_true.translation().head<2>()).norm(), 0.05);
}

// 有坡道(俯仰)时 z 杆臂可观:完整 3D 杆臂 < 5 cm
TEST(LeverArm, RecoversFullLeverWithPitch) {
  SynthConfig cfg;
  cfg.lever_imu = {0.3, 1.2, -0.5};
  cfg.sigma_fixed = {0.01, 0.01, 0.03};
  cfg.T_enu_world = rotated_frame(-1.3, {-10, 40, 3});
  auto traj = straight_then_turn_with_ramp();
  auto res = synthesize(traj, cfg, {});
  for (auto& s : res.samples) s.stamp += 0.03;   // RTK 时间戳晚 30 ms → time_offset = −0.03
  LlaToEnu conv(cfg.lat0, cfg.lon0, cfg.alt0);
  auto est = estimate_lever_arm(traj, res.samples, conv);
  EXPECT_TRUE(est.lever_observable);
  EXPECT_TRUE(est.lever_z_observable);
  EXPECT_NEAR(est.time_offset, -0.03, 0.011);
  EXPECT_LT((est.lever_imu - cfg.lever_imu).norm(), 0.05);
  EXPECT_LT(est.rms_residual, 0.05);
  const Eigen::Isometry3d T_true = cfg.T_enu_world.inverse();
  EXPECT_NEAR(yaw_of(est.T_world_enu), yaw_of(T_true), 2e-3);
  EXPECT_LT((est.T_world_enu.translation() - T_true.translation()).norm(), 0.05);
}

TEST(LeverArm, StraightLineIsNotObservable) {
  std::vector<TrajPose> tr;
  for (int k = 0; k <= 200; ++k) {
    TrajPose p;
    p.stamp = k * 0.1;
    p.T_world_imu = Eigen::Isometry3d::Identity();
    p.T_world_imu.translation() = Eigen::Vector3d(5 * p.stamp, 0, 0);
    tr.push_back(p);
  }
  SynthConfig cfg;
  cfg.lever_imu = {0, 1, 0};
  cfg.sigma_fixed.setZero();
  auto res = synthesize(tr, cfg, {});
  LlaToEnu conv(cfg.lat0, cfg.lon0, cfg.alt0);
  auto est = estimate_lever_arm(tr, res.samples, conv);
  EXPECT_FALSE(est.lever_observable);
  EXPECT_LT(est.yaw_range_deg, 30.0);
  EXPECT_GT(est.n_pairs, 100);   // 仍返回结果(T_world_enu 可用)
  // 匀速直线上时移等价于平移 → Δt 也不可观;平局取最靠近 0 的候选
  EXPECT_FALSE(est.time_offset_observable);
  EXPECT_NEAR(est.time_offset, 0.0, 0.011);
}

TEST(LeverArm, ZeroNoiseZeroLeverZeroOffsetGivesTinyResidual) {
  SynthConfig cfg;
  cfg.sigma_fixed.setZero();
  auto traj = straight_then_turn();
  auto res = synthesize(traj, cfg, {});
  LlaToEnu conv(cfg.lat0, cfg.lon0, cfg.alt0);
  auto est = estimate_lever_arm(traj, res.samples, conv);
  EXPECT_LT(est.rms_residual, 1e-3);
  EXPECT_NEAR(est.time_offset, 0.0, 1e-3);
  EXPECT_LT(est.lever_imu.norm(), 1e-3);
  EXPECT_LT(est.T_world_enu.translation().norm(), 1e-3);
  EXPECT_NEAR(yaw_of(est.T_world_enu), 0.0, 1e-6);
}

TEST(LeverArm, QualityGateAndEmptyInput) {
  SynthConfig cfg;
  cfg.sigma_fixed.setZero();
  Injection inj;
  inj.float_from = 0;
  inj.float_to = 100;   // 全程 FLOAT
  auto traj = straight_then_turn();
  auto res = synthesize(traj, cfg, inj);
  LlaToEnu conv(cfg.lat0, cfg.lon0, cfg.alt0);
  auto est = estimate_lever_arm(traj, res.samples, conv);   // 默认 min_quality=FIXED → 无配对
  EXPECT_EQ(est.n_pairs, 0);
  EXPECT_FALSE(est.lever_observable);
  LeverArmOptions opt;
  opt.min_quality = Quality::FLOAT;
  auto est2 = estimate_lever_arm(traj, res.samples, conv, opt);
  EXPECT_GT(est2.n_pairs, 400);
  auto est3 = estimate_lever_arm({}, res.samples, conv);
  EXPECT_EQ(est3.n_pairs, 0);
}
