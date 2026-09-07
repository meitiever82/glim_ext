#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>

#include "gnss_core/geodetic.hpp"
#include "gnss_core/pos_io.hpp"
#include "gnss_core/synth.hpp"
#include "synth_fixtures.hpp"

using namespace gnss_core;
using gnss_core::test_fixtures::straight_then_turn;

TEST(Synth, ZeroNoiseZeroLeverReproducesTrajectory) {
  SynthConfig cfg;
  cfg.sigma_fixed.setZero();
  cfg.sigma_float.setZero();
  auto res = synthesize(straight_then_turn(), cfg, {});
  const auto& s = res.samples;
  ASSERT_GT(s.size(), 400u);
  ASSERT_EQ(res.truth_enu.size(), s.size());
  LlaToEnu conv(cfg.lat0, cfg.lon0, cfg.alt0);
  const auto& last = s.back();
  const auto enu = conv.forward(last.lat, last.lon, last.alt);
  EXPECT_NEAR(enu.x(), 100.0, 1e-3);
  EXPECT_NEAR(enu.y(), 100.0, 1e-3);
  EXPECT_EQ(last.quality, Quality::FIXED);
  EXPECT_NEAR(last.header_stamp - last.gnss_time, 0.05, 1e-9);
  EXPECT_NEAR(last.stamp, last.gnss_time, 1e-12);
  EXPECT_EQ(last.sats_used, 20);
  EXPECT_NEAR(last.diff_age, 1.0, 1e-9);
  // 样本间隔 = 1/rate_hz
  EXPECT_NEAR(s[1].stamp - s[0].stamp, 0.1, 1e-9);
}

TEST(Synth, LeverArmShowsUpOnlyAfterTurn) {
  SynthConfig cfg;
  cfg.sigma_fixed.setZero();
  cfg.lever_imu = {0, 1.0, 0};   // 天线在 IMU 左侧 1 m
  auto res = synthesize(straight_then_turn(), cfg, {});
  const auto& s = res.samples;
  LlaToEnu conv(cfg.lat0, cfg.lon0, cfg.alt0);
  auto at = [&](double t) {
    for (auto& x : s)
      if (std::abs(x.stamp - t) < 1e-6) return conv.forward(x.lat, x.lon, x.alt);
    throw std::runtime_error("no sample");
  };
  EXPECT_NEAR(at(10.0).y(), 1.0, 1e-3);    // 直行段:天线在 +y 1 m
  EXPECT_NEAR(at(40.0).x(), 99.0, 1e-3);   // 转 90° 后:天线在 −x 1 m
  // 转弯中点(25 s,yaw=45°):杆臂 (0,1,0) → (−sin45, cos45)
  EXPECT_NEAR(at(25.0).x(), 100.0 - M_SQRT1_2, 1e-3);
  EXPECT_NEAR(at(25.0).y(), M_SQRT1_2, 1e-3);
}

TEST(Synth, WrongFixInjectionKeepsFixedLabelAndSmallSigma) {
  SynthConfig cfg;
  cfg.sigma_fixed.setZero();
  Injection inj;
  inj.wrong_fix_ratio = 0.05;
  auto res = synthesize(straight_then_turn(), cfg, inj);
  const auto& s = res.samples;
  ASSERT_EQ(res.truth_enu.size(), s.size());
  LlaToEnu conv(cfg.lat0, cfg.lon0, cfg.alt0);
  int n_bad = 0;
  for (size_t i = 0; i < s.size(); ++i) {
    // 错误固定历元 quality 仍为 FIXED、sigma 仍报 sigma_fixed(此处 0)
    EXPECT_EQ(s[i].quality, Quality::FIXED);
    EXPECT_TRUE(s[i].sigma_enu.isZero());
    const double dev = (conv.forward(s[i].lat, s[i].lon, s[i].alt) - res.truth_enu[i]).norm();
    if (dev > 0.5) {
      ++n_bad;
      EXPECT_NEAR(dev, 1.0, 1e-3);   // 偏 1 m
    }
  }
  const double ratio = static_cast<double>(n_bad) / static_cast<double>(s.size());
  EXPECT_NEAR(ratio, 0.05, 0.02);

  // 同 seed 可复现,不同 seed 结果不同
  auto res2 = synthesize(straight_then_turn(), cfg, inj);
  for (size_t i = 0; i < s.size(); ++i) EXPECT_DOUBLE_EQ(res2.samples[i].lat, s[i].lat);
  SynthConfig cfg3 = cfg;
  cfg3.seed = 7;
  auto res3 = synthesize(straight_then_turn(), cfg3, inj);
  int n_diff = 0;
  for (size_t i = 0; i < s.size(); ++i) n_diff += (res3.samples[i].lat != s[i].lat);
  EXPECT_GT(n_diff, 0);
}

TEST(Synth, StaleSegmentRaisesDiffAge) {
  SynthConfig cfg;
  Injection inj;
  inj.stale_from = 10;
  inj.stale_to = 20;
  auto res = synthesize(straight_then_turn(), cfg, inj);
  for (auto& x : res.samples) {
    if (x.stamp > 19.5 && x.stamp < 20.0) { EXPECT_GT(x.diff_age, 50.0); }
  }
  for (auto& x : res.samples) {
    if (x.stamp < 9.5) { EXPECT_LT(x.diff_age, 2.0); }
  }
}

TEST(Synth, FloatSegmentLabelsFloat) {
  SynthConfig cfg;
  Injection inj;
  inj.float_from = 30;
  inj.float_to = 40;
  auto res = synthesize(straight_then_turn(), cfg, inj);
  for (auto& x : res.samples) {
    if (x.stamp > 30 && x.stamp < 40) {
      EXPECT_EQ(x.quality, Quality::FLOAT);
      EXPECT_GT(x.sigma_enu.x(), 0.1);
    } else if (x.stamp < 29.95 || x.stamp > 40.05) {   // 段边界(含端点)不做断言
      EXPECT_EQ(x.quality, Quality::FIXED);
      EXPECT_LT(x.sigma_enu.x(), 0.1);
    }
  }
}

TEST(Synth, TEnuWorldIsApplied) {
  SynthConfig cfg;
  cfg.sigma_fixed.setZero();
  cfg.T_enu_world.linear() = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  cfg.T_enu_world.translation() = Eigen::Vector3d(10, 20, 3);
  auto res = synthesize(straight_then_turn(), cfg, {});
  LlaToEnu conv(cfg.lat0, cfg.lon0, cfg.alt0);
  // world (100,0,0) @ t=20 → enu (10, 120, 3)
  for (size_t i = 0; i < res.samples.size(); ++i) {
    if (std::abs(res.samples[i].stamp - 20.0) < 1e-6) {
      const auto enu = conv.forward(res.samples[i].lat, res.samples[i].lon, res.samples[i].alt);
      EXPECT_NEAR(enu.x(), 10.0, 1e-3);
      EXPECT_NEAR(enu.y(), 120.0, 1e-3);
      EXPECT_NEAR(enu.z(), 3.0, 1e-3);
      EXPECT_NEAR((res.truth_enu[i] - enu).norm(), 0.0, 1e-6);
    }
  }
}

TEST(Synth, ReadGlimTrajTumFormat) {
  const std::string path = std::string(::testing::TempDir()) + "gnss_core_traj.txt";
  {
    std::ofstream f(path);
    f << "# timestamp tx ty tz qx qy qz qw\n";
    f << "100.5 1 2 3 0 0 0 1\n";
    f << "\n";
    f << "101.5 4 5 6 0 0 0.7071067811865476 0.7071067811865476\n";
  }
  auto tr = read_glim_traj(path);
  ASSERT_EQ(tr.size(), 2u);
  EXPECT_DOUBLE_EQ(tr[0].stamp, 100.5);
  EXPECT_NEAR(tr[0].T_world_imu.translation().z(), 3.0, 1e-12);
  const Eigen::Vector3d ex = tr[1].T_world_imu.linear() * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(ex.y(), 1.0, 1e-9);   // yaw 90°
  std::remove(path.c_str());
  EXPECT_THROW(read_glim_traj("/nonexistent/traj.txt"), std::runtime_error);
}

TEST(Synth, InterpolatePoseSlerpAndLinear) {
  auto tr = straight_then_turn();
  TrajPose p;
  ASSERT_TRUE(interpolate_pose(tr, 25.05, p));   // 转弯中点附近:yaw 应为 45.45°
  const double yaw = std::atan2(p.T_world_imu.linear()(1, 0), p.T_world_imu.linear()(0, 0));
  EXPECT_NEAR(yaw, (25.05 - 20) / 10 * M_PI_2, 1e-6);
  ASSERT_TRUE(interpolate_pose(tr, 10.05, p));
  EXPECT_NEAR(p.T_world_imu.translation().x(), 50.25, 1e-9);
  EXPECT_FALSE(interpolate_pose(tr, -1.0, p));
  EXPECT_FALSE(interpolate_pose(tr, 50.2, p));
}

TEST(Synth, SampleToPosRecordRoundTrip) {
  SynthConfig cfg;
  Injection inj;
  inj.float_from = 30;
  inj.float_to = 40;
  auto res = synthesize(straight_then_turn(), cfg, inj);
  for (const auto& s : res.samples) {
    const PosRecord r = sample_to_pos_record(s);
    EXPECT_DOUBLE_EQ(r.stamp, s.stamp);
    EXPECT_EQ(q_to_quality(r.q), s.quality);
    EXPECT_DOUBLE_EQ(r.sdne(0), s.sigma_enu.y());   // sdn ↔ N
    EXPECT_DOUBLE_EQ(r.sdne(1), s.sigma_enu.x());   // sde ↔ E
    const RtkFixSample b = pos_record_to_sample(r);
    EXPECT_DOUBLE_EQ(b.lat, s.lat);
    EXPECT_EQ(b.quality, s.quality);
    EXPECT_DOUBLE_EQ(b.sigma_enu.x(), s.sigma_enu.x());
    EXPECT_DOUBLE_EQ(b.diff_age, s.diff_age);
    EXPECT_EQ(b.sats_used, s.sats_used);
    EXPECT_DOUBLE_EQ(b.gnss_time, s.stamp);
  }
}
