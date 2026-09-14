#include <gtest/gtest.h>

#include <cmath>

#include "gnss_core/divergence_monitor.hpp"
using namespace gnss_core;

namespace {
DiagnosisConfig cfg_small() {
  DiagnosisConfig c;
  c.divergence_sigma = 3.0;
  c.divergence_window_s = 100.0;
  c.divergence_min_samples = 10;
  c.divergence_sigma_floor_m = 0.05;
  return c;
}
}  // namespace

TEST(DivergenceMonitor, FallsBackToSolverSigmaUntilTheWindowHasEnoughSamples) {
  DivergenceMonitor m(cfg_small());
  const auto s = m.update(0.0, 0.02, std::hypot(0.011, 0.012));
  EXPECT_FALSE(s.empirical);
  EXPECT_NEAR(s.threshold_m, 3.0 * std::hypot(0.011, 0.012), 1e-12);
  EXPECT_FALSE(s.since.has_value());
}

TEST(DivergenceMonitor, FallbackSigmaHasAMillimetreFloor) {
  DivergenceMonitor m(cfg_small());
  EXPECT_NEAR(m.update(0.0, std::nullopt, 0.0).threshold_m, 3.0 * 1e-3, 1e-12);
}

TEST(DivergenceMonitor, UsesRmsOfTheWindowOnceFull) {
  DivergenceMonitor m(cfg_small());
  // 10 个 0.2 m 样本(610 与独立解长期稳定相差 0.2 m)→ RMS 0.2,阈值 0.6
  for (int i = 0; i < 10; ++i) m.update(i, 0.2, 0.001);
  const auto s = m.update(10.0, 0.25, 0.001);
  EXPECT_TRUE(s.empirical);
  EXPECT_NEAR(s.threshold_m, 0.6, 1e-9);
  EXPECT_FALSE(s.since.has_value()) << "0.25 < 0.6,不算超限";
}

TEST(DivergenceMonitor, EmpiricalSigmaHasAFloor) {
  DivergenceMonitor m(cfg_small());
  for (int i = 0; i < 10; ++i) m.update(i, 0.001, 0.001);   // 两路几乎重合
  EXPECT_NEAR(m.update(10.0, 0.001, 0.001).threshold_m, 3.0 * 0.05, 1e-9);
}

TEST(DivergenceMonitor, ExceedingSamplesStartTheClockAndStayOutOfTheWindow) {
  DivergenceMonitor m(cfg_small());
  for (int i = 0; i < 10; ++i) m.update(i, 0.1, 0.001);   // 阈值 0.3
  const size_t before = m.window_size();
  auto s = m.update(10.0, 1.0, 0.001);
  ASSERT_TRUE(s.since.has_value());
  EXPECT_DOUBLE_EQ(*s.since, 10.0);
  s = m.update(11.0, 1.2, 0.001);
  EXPECT_DOUBLE_EQ(*s.since, 10.0) << "持续超限时起始时刻保持";
  EXPECT_EQ(m.window_size(), before) << "超限段不能拉高自己的阈值";
  EXPECT_NEAR(s.threshold_m, 0.3, 1e-9);
}

TEST(DivergenceMonitor, RecoveryOrLostPairingClearsTheClock) {
  DivergenceMonitor m(cfg_small());
  m.update(0.0, 1.0, 0.01);
  EXPECT_FALSE(m.update(1.0, 0.0, 0.01).since.has_value());
  m.update(2.0, 1.0, 0.01);
  const auto s = m.update(3.0, std::nullopt, 0.01);
  EXPECT_FALSE(s.since.has_value());
  EXPECT_FALSE(s.divergence_m.has_value());
}

TEST(DivergenceMonitor, OldSamplesAgeOutOfTheWindow) {
  DivergenceMonitor m(cfg_small());
  for (int i = 0; i < 10; ++i) m.update(i, 0.2, 0.001);
  EXPECT_TRUE(m.update(50.0, 0.2, 0.001).empirical);
  const auto s = m.update(150.0, std::nullopt, 0.02);   // 窗口 100 s,之前的样本全部过期
  EXPECT_FALSE(s.empirical);
  EXPECT_EQ(m.window_size(), 0u);
}
