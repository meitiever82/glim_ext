#include <gtest/gtest.h>

#include <cmath>
#include <limits>

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

// 控制者裁定(round3a 修正):样本不足、还在用回退 σ 判定时,依然要照 design
// decision 2 对回退阈值做判定(而不是像 empirical 模式那样把超限样本排除在
// 窗口外)——否则 Task 7 的"5 s 内 0.5 m 偏差、样本不足 60 个也要报"用例过不了。
TEST(DivergenceMonitor, FallbackRegimeStillJudgesAndAdmitsSamples) {
  DivergenceMonitor m(cfg_small());
  const auto s1 = m.update(0.0, 0.5, std::hypot(0.011, 0.012));
  ASSERT_TRUE(s1.since.has_value());
  EXPECT_DOUBLE_EQ(*s1.since, 0.0);
  EXPECT_EQ(m.window_size(), 1u);
  const auto s2 = m.update(5.0, 0.5, std::hypot(0.011, 0.012));
  ASSERT_TRUE(s2.since.has_value());
  EXPECT_DOUBLE_EQ(*s2.since, 0.0) << "持续超限时起始时刻保持";
  EXPECT_EQ(m.window_size(), 2u) << "回退阶段超限样本也要入窗口,否则经验基线永远建立不起来";
}

TEST(DivergenceMonitor, RegimeChangeRestartsTheClock) {
  DivergenceMonitor m(cfg_small());
  for (int i = 0; i < 9; ++i) m.update(i, 0.1, 0.001);   // 9 个,均超回退阈值 0.003
  m.update(9.0, 0.1, 0.001);                              // 第 10 个,窗口刚好填满,转入 empirical
  const auto s = m.update(10.0, 1.0, 0.001);
  EXPECT_TRUE(s.empirical);
  ASSERT_TRUE(s.since.has_value());
  EXPECT_DOUBLE_EQ(*s.since, 10.0)
      << "由回退模式的旧计时切到 empirical 模式,应重新起算,不能沿用回退阶段的 0.0";
}

// round3a fix1:上一版"回退/经验切换即清零计时"的规则有一个漏洞——一次持续
// 时长接近 divergence_window_s 的偏差,会在 empirical 模式下被排除在窗口外,
// 窗口被剪枝耗尽后掉回回退模式,重置计时,然后回退模式又把故障样本收进窗口,
// 攒出一份等于故障本身的"经验基线",故障就此被吸收、不再报警。
// 本用例复现该场景并断言它不会发生:0.5 m 的偏差从 t=20 一直报警到 t=400,
// 阈值全程远小于 0.5 m(不会被故障样本自己的 RMS 抬上去)。
TEST(DivergenceMonitor, LongDivergenceIsNotAbsorbedWhenTheWindowStarves) {
  DivergenceMonitor m(cfg_small());
  for (int t = 0; t < 20; ++t) m.update(t, 0.02, 0.001);   // 攒出一份 0.02 m 的基线
  for (int t = 20; t <= 400; ++t) {
    const auto s = m.update(t, 0.5, 0.001);
    ASSERT_TRUE(s.since.has_value()) << "t=" << t;
    EXPECT_DOUBLE_EQ(*s.since, 20.0) << "t=" << t;
    EXPECT_LT(s.threshold_m, 0.5) << "t=" << t;
  }
}

// round3a fix1:只有真正的数据缺口(配对样本间隔 >= divergence_window_s)才应该
// 重新预热;一次真正的数据缺口之后,重新预热期间依然按 design decision 2 判定
// (回退阈值),直到窗口重新攒够样本才转回经验模式。
TEST(DivergenceMonitor, LongPairingGapRestartsWarmUp) {
  DivergenceMonitor m(cfg_small());
  for (int t = 0; t < 20; ++t) m.update(t, 0.02, 0.001);
  m.update(200.0, std::nullopt, 0.001);   // 隧道/信号中断:200-19=181 >= window_s(100)

  const auto s1 = m.update(250.0, 0.1, 0.001);   // 250-200 一段时间没配对了,重新预热
  ASSERT_TRUE(s1.since.has_value());
  EXPECT_DOUBLE_EQ(*s1.since, 250.0);
  EXPECT_EQ(m.window_size(), 1u)
      << "重新预热:按回退阈值 0.003 判定(0.1 超限),但样本仍要入窗口";

  for (int t = 251; t <= 259; ++t) m.update(t, 0.1, 0.001);   // 窗口攒到 10(含 t=250 的那个)
  const auto s2 = m.update(260.0, 0.1, 0.001);
  EXPECT_TRUE(s2.empirical);
  EXPECT_FALSE(s2.since.has_value()) << "预热在这一拍开始时结束,重新起算计时";
  EXPECT_NEAR(s2.threshold_m, 0.3, 1e-9);   // 基线 0.1 → 阈值 0.3
}

TEST(DivergenceMonitor, NonFiniteDivergenceIsIgnored) {
  {
    DivergenceMonitor m(cfg_small());
    m.update(0.0, 0.02, 0.001);
    const auto s = m.update(1.0, std::numeric_limits<double>::quiet_NaN(), 0.001);
    EXPECT_FALSE(s.divergence_m.has_value());
    EXPECT_FALSE(s.since.has_value());
    EXPECT_EQ(m.window_size(), 1u);
  }
  {
    DivergenceMonitor m(cfg_small());
    m.update(0.0, 0.02, 0.001);
    const auto s = m.update(1.0, std::numeric_limits<double>::infinity(), 0.001);
    EXPECT_FALSE(s.divergence_m.has_value());
    EXPECT_FALSE(s.since.has_value());
    EXPECT_EQ(m.window_size(), 1u);
  }
}
