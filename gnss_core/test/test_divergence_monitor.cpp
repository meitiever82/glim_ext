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

// 窗口样本不足(还没有经验基线)时,独立解自报 σ 高于 5 cm 下限就由它决定阈值
TEST(DivergenceMonitor, CurrentSolverSigmaAboveTheFloorSetsTheThresholdBeforeWarmUp) {
  DivergenceMonitor m(cfg_small());
  const auto s = m.update(0.0, 0.2, 0.1);
  EXPECT_FALSE(s.empirical);
  EXPECT_NEAR(s.threshold_m, 3.0 * 0.1, 1e-12);
  EXPECT_FALSE(s.since.has_value()) << "0.2 < 0.3,不算超限(只看下限的话阈值是 0.15,会误判超限)";
}

// 既没有经验基线、也没有独立解(自报 σ 传 0)时,阈值就是 5 cm 下限
TEST(DivergenceMonitor, WithoutBaseOrSolverSigmaTheThresholdIsTheFloor) {
  DivergenceMonitor m(cfg_small());
  const auto s = m.update(0.0, std::nullopt, 0.0);
  EXPECT_FALSE(s.empirical);
  EXPECT_NEAR(s.threshold_m, 3.0 * 0.05, 1e-12);
}

// final fix F1:启动/重新预热阶段也有 5 cm 下限——rtkrcv 自报的毫米级 σ 不能把阈值压到
// 几毫米,否则每次启动都会误报 device_divergence。
TEST(DivergenceMonitor, FallbackUsesTheFiveCentimetreFloor) {
  DivergenceMonitor m(cfg_small());
  const auto s = m.update(0.0, 0.1, 0.001);
  EXPECT_NEAR(s.threshold_m, 0.15, 1e-9);
  EXPECT_FALSE(s.since.has_value()) << "0.1 < 0.15,不算超限";
}

// final fix F1:预热结束后,独立解自报 σ 变大(rtkrcv 掉到 FLOAT/SINGLE)依然要抬高阈值,
// 不能一直按 FIXED 时学到的经验阈值判定。
TEST(DivergenceMonitor, CurrentSolverSigmaRaisesTheThresholdEvenAfterWarmUp) {
  DivergenceMonitor m(cfg_small());
  for (int t = 0; t < 20; ++t) m.update(t, 0.02, 0.005);   // 经验基线 0.05(下限)→ 阈值 0.15
  auto s = m.update(20.0, 0.3, 0.2);
  EXPECT_NEAR(s.threshold_m, 0.6, 1e-9);
  EXPECT_FALSE(s.since.has_value()) << "0.3 < 0.6,独立解自己只有 20 cm 精度,不算超限";

  // t=20 的 0.3 m 样本在抬高后的阈值 0.6 下不算超限,所以按规则入了窗口;
  // t=21 的窗口是 20 个 0.02 加 1 个 0.3,RMS = sqrt(0.098/21)。
  s = m.update(21.0, 0.3, 0.005);
  EXPECT_NEAR(s.threshold_m, 3.0 * std::sqrt(0.098 / 21.0), 1e-9);
  ASSERT_TRUE(s.since.has_value());
  EXPECT_DOUBLE_EQ(*s.since, 21.0);
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

// 控制者裁定(round3a 修正):预热期(样本不足、没有经验基线)依然要按本拍阈值
// (final fix F1 起是 5 cm 下限与当前自报 σ 取大)做判定,且超限样本照样入窗口(而不是
// 像 empirical 模式那样把超限样本排除在窗口外)——否则 Task 7 的"5 s 内 0.5 m 偏差、样本不足 60 个也要报"用例过不了。
TEST(DivergenceMonitor, FallbackRegimeStillJudgesAndAdmitsSamples) {
  DivergenceMonitor m(cfg_small());
  const auto s1 = m.update(0.0, 0.5, std::hypot(0.011, 0.012));
  ASSERT_TRUE(s1.since.has_value());
  EXPECT_DOUBLE_EQ(*s1.since, 0.0);
  EXPECT_EQ(m.window_size(), 1u);
  const auto s2 = m.update(5.0, 0.5, std::hypot(0.011, 0.012));
  ASSERT_TRUE(s2.since.has_value());
  EXPECT_DOUBLE_EQ(*s2.since, 0.0) << "持续超限时起始时刻保持";
  EXPECT_EQ(m.window_size(), 2u) << "预热期超限样本也要入窗口,否则经验基线永远建立不起来";
}

TEST(DivergenceMonitor, RegimeChangeRestartsTheClock) {
  DivergenceMonitor m(cfg_small());
  for (int i = 0; i < 9; ++i) m.update(i, 0.2, 0.001);   // 9 个,均超预热期阈值 0.15(下限),t=0 起计时
  m.update(9.0, 0.2, 0.001);                              // 第 10 个,窗口刚好填满,转入 empirical
  const auto s = m.update(10.0, 1.0, 0.001);              // 经验 σ 0.2 → 阈值 0.6,1.0 超限
  EXPECT_TRUE(s.empirical);
  EXPECT_NEAR(s.threshold_m, 0.6, 1e-9);
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
// (没有经验基线,阈值 = 5 cm 下限与当前自报 σ 取大),直到窗口重新攒够样本才转回经验模式。
TEST(DivergenceMonitor, LongPairingGapRestartsWarmUp) {
  DivergenceMonitor m(cfg_small());
  for (int t = 0; t < 20; ++t) m.update(t, 0.02, 0.001);
  m.update(200.0, std::nullopt, 0.001);   // 隧道/信号中断:200-19=181 >= window_s(100)

  const auto s1 = m.update(250.0, 0.2, 0.001);   // 250-200 一段时间没配对了,重新预热
  EXPECT_FALSE(s1.empirical) << "重新预热时 held 的经验基线已清掉";
  ASSERT_TRUE(s1.since.has_value());
  EXPECT_DOUBLE_EQ(*s1.since, 250.0);
  EXPECT_EQ(m.window_size(), 1u)
      << "重新预热:按下限阈值 0.15 判定(0.2 超限),但样本仍要入窗口";

  for (int t = 251; t <= 259; ++t) m.update(t, 0.2, 0.001);   // 窗口攒到 10(含 t=250 的那个)
  const auto s2 = m.update(260.0, 0.2, 0.001);
  EXPECT_TRUE(s2.empirical);
  EXPECT_FALSE(s2.since.has_value()) << "预热在这一拍开始时结束,重新起算计时";
  EXPECT_NEAR(s2.threshold_m, 0.6, 1e-9);   // 基线 0.2 → 阈值 0.6
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

// round3a fix2(N1):fix1 的规则 3 让预热结束后、窗口被剪枝耗尽时掉回 rtkrcv 自报
// 的回退 σ——生产环境里那通常只有几毫米,连正常的 0.02 m 偏差都会被判"超限",
// 样本永远进不了窗口,since 永远清不掉,一次早已恢复的故障会被永远误报下去。
// 本用例:0.5 m 的故障持续到 t=250(足够让窗口被剪枝耗尽),然后恢复到 0.02 m;
// 断言恢复后 since 会清零,而不是继续误报到 t=600。
TEST(DivergenceMonitor, RecoversAfterALongFaultInsteadOfLatching) {
  DivergenceMonitor m(cfg_small());
  for (int t = 0; t < 20; ++t) m.update(t, 0.02, 0.001);          // 攒出 0.02 m 的基线
  for (int t = 20; t <= 250; ++t) {
    const auto s = m.update(t, 0.5, 0.001);
    ASSERT_TRUE(s.since.has_value()) << "t=" << t;
    EXPECT_DOUBLE_EQ(*s.since, 20.0) << "t=" << t;
  }
  for (int t = 251; t <= 600; ++t) {
    const auto s = m.update(t, 0.02, 0.001);
    EXPECT_FALSE(s.since.has_value()) << "t=" << t << ":故障已恢复,不该继续误报";
  }
  const auto s = m.update(600.0, 0.02, 0.001);
  EXPECT_FALSE(s.since.has_value());
  EXPECT_TRUE(s.empirical);
  EXPECT_GE(m.window_size(), 10u);
  EXPECT_NEAR(s.threshold_m, 0.15, 1e-9);
}

// round3a fix2:窗口被剪枝掉到不够 min_samples,但配对间隔还远小于
// divergence_window_s(不是真正的数据缺口)时,应该沿用上一次学到的经验基线,
// 而不是只剩 5 cm 下限 / 当前自报 σ。
TEST(DivergenceMonitor, ShortPairingGapKeepsTheLearnedBaseline) {
  DivergenceMonitor m(cfg_small());
  // 基线要高于 5 cm 下限,才能和"没有 held 基线、只剩下限"区分开
  for (int t = 0; t < 20; ++t) m.update(t, 0.2, 0.001);   // 基线 0.2 → 阈值 0.6
  const auto s = m.update(110.0, 0.5, 0.001);   // 窗口被剪到 9 个样本,但 110-19=91 < 100
  EXPECT_FALSE(s.since.has_value()) << "0.5 < 0.6;若丢了 held 基线,阈值只剩 0.15,会误判超限";
  EXPECT_TRUE(s.empirical);
  EXPECT_NEAR(s.threshold_m, 0.6, 1e-9);
}
