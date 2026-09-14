#include <gtest/gtest.h>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include "diag_test_fixtures.hpp"
#include "gnss_core/diagnosis_engine.hpp"
using namespace gnss_core;
using gnss_core::test_fixtures::make_1005_frame;
using gnss_core::test_fixtures::sat_line;

namespace {
const double X = -2148744.1, Y = 4426641.2, Z = 4044655.9;

SolutionSample fixed(double lat = 44.5, double lon = 90.28) {
  SolutionSample s;
  s.quality = Quality::FIXED;
  s.lat = lat;
  s.lon = lon;
  s.ns = 20;
  s.sdn = 0.011;
  s.sde = 0.012;
  s.age = 0.8;
  s.ratio = 25.0;
  return s;
}

void corrections(DiagnosisEngine& e, double t) {
  const auto f = make_1005_frame(1, X, Y, Z);
  e.on_corrections(t, f.data(), f.size());
}

bool has_code(const TickResult& r, const std::string& code) {
  return std::any_of(r.result.verdicts.begin(), r.result.verdicts.end(),
                     [&](const Verdict& v) { return v.code == code; });
}

bool opened(const TickResult& r, const std::string& code) {
  return std::any_of(r.transitions.begin(), r.transitions.end(), [&](const EventTransition& t) {
    return t.kind == EventKind::Open && t.code == code;
  });
}

DiagnosisEngine make_engine(DiagnosisConfig cfg = {}) {
  return DiagnosisEngine(cfg, {}, true, std::nullopt, std::nullopt);
}
}  // namespace

TEST(DiagTestFixtures, Make1005FrameRoundTripsThroughTheParser) {
  const auto frame = make_1005_frame(1234, -22458.1234, 4548123.4567, 4451234.8901);
  RtcmFramer framer;
  const auto msgs = framer.feed(frame);
  ASSERT_EQ(msgs.size(), 1u);
  BaseStationCoords c;
  ASSERT_TRUE(parse_base_station(msgs[0], c));
  EXPECT_EQ(c.station_id, 1234);
  EXPECT_NEAR(c.x, -22458.1234, 1e-4);
  EXPECT_NEAR(c.y, 4548123.4567, 1e-4);
  EXPECT_NEAR(c.z, 4451234.8901, 1e-4);
}

TEST(DiagnosisEngine, RejectsAnInvalidConfig) {
  DiagnosisConfig cfg;
  cfg.corr_gap_s = 0.0;
  EXPECT_THROW(make_engine(cfg), std::invalid_argument);
}

TEST(DiagnosisEngine, CorrectionsGapOpensCorrOutage) {   // 移植 test_epochs_and_corr_outage_event
  auto e = make_engine();
  corrections(e, 100.0);
  e.on_solution(100.0, fixed());
  auto r = e.tick(101.0);
  EXPECT_EQ(r.result.status().code, "rtk_fixed");
  EXPECT_TRUE(r.transitions.empty());
  e.on_solution(103.5, fixed());
  r = e.tick(104.0);
  EXPECT_EQ(r.result.status().code, "corr_outage");
  EXPECT_TRUE(opened(r, "corr_outage"));
}

TEST(DiagnosisEngine, AnyCorrectionBytesCountAsLinkAlive) {
  auto e = make_engine();
  corrections(e, 90.0);   // 真实 1005 帧先立一次链路时刻
  e.on_solution(100.0, fixed());
  const std::vector<uint8_t> garbage = {0x01, 0x02, 0x03};
  EXPECT_TRUE(e.on_corrections(100.0, garbage.data(), garbage.size()).empty());
  EXPECT_FALSE(has_code(e.tick(102.0), "corr_outage")) << "垃圾字节也要刷新 corr_last_t_,gap 应为 2s 而非 12s";
}

TEST(DiagnosisEngine, StaleSolutionDegradesToNoSolution) {   // 移植 test_stale_solution_degrades_to_no_solution
  auto e = make_engine();
  corrections(e, 100.0);
  e.on_solution(100.0, fixed());
  EXPECT_FALSE(has_code(e.tick(102.0), "no_solution"));
  corrections(e, 105.5);
  const auto r = e.tick(106.0);   // 独立解已 6 s 没更新
  EXPECT_TRUE(has_code(r, "no_solution"));
  EXPECT_TRUE(opened(r, "no_solution"));
  EXPECT_FALSE(has_code(r, "corr_outage"));
}

TEST(DiagnosisEngine, StaleDeviceFixDoesNotFeedTheCorrAgeFallback) {   // 移植 test_stale_can_epoch_gated_from_fallbacks
  auto e = make_engine();
  SolutionSample dev = fixed();
  dev.age = 99.0;
  e.on_device_solution(40.0, dev);
  corrections(e, 100.0);
  EXPECT_FALSE(has_code(e.tick(100.5), "corr_outage")) << "60 s 前的 610 龄期不能拿来判差分中断";
  e.on_device_solution(100.4, dev);
  EXPECT_TRUE(has_code(e.tick(100.6), "corr_outage")) << "新鲜的 610 龄期作为回退";
}

TEST(DiagnosisEngine, DivergenceNeedsHoldAndItsClockResetsWhenPairingIsLost) {   // 移植 test_div_since_cleared_when_inputs_vanish
  auto e = make_engine();
  const double far_lat = 44.5 + 0.5 / 111000.0;   // 610 解偏北约 0.5 m
  TickResult last;
  for (int i = 0; i <= 6; ++i) {
    const double t = 100.0 + i;
    corrections(e, t);
    e.on_solution(t, fixed());
    e.on_device_solution(t, fixed(far_lat));
    last = e.tick(t + 0.1);
    if (i == 4) EXPECT_FALSE(has_code(last, "device_divergence")) << "只持续了 4 s";
  }
  EXPECT_TRUE(has_code(last, "device_divergence"));

  for (int i = 7; i <= 12; ++i) {   // 独立解断流:两路到达时刻差 >= 2 s 后不再配对
    const double t = 100.0 + i;
    corrections(e, t);
    e.on_device_solution(t, fixed(far_lat));
    last = e.tick(t + 0.1);
    if (i == 8) EXPECT_FALSE(last.divergence.since.has_value()) << "两路到达时刻相差 2 s,不再配对";
  }
  EXPECT_FALSE(last.divergence.since.has_value());

  corrections(e, 113.0);
  e.on_solution(113.0, fixed());
  e.on_device_solution(113.0, fixed(far_lat));
  const auto r = e.tick(113.1);
  EXPECT_FALSE(has_code(r, "device_divergence")) << "恢复配对后必须重新累计持续时间";
  ASSERT_TRUE(r.divergence.since.has_value());
  EXPECT_DOUBLE_EQ(*r.divergence.since, 113.1);
}

// final fix F2:两路按历元时刻配对,不按到达时刻。车以 2 m/s 向北行驶,610 每 0.1 s 出一个
// 历元、到达无延迟;rtkrcv 每秒一个历元、比 610 晚到 0.4 s。按到达时刻配对时,rtkrcv 的
// 历元 t-0.4 会被拿去跟 610 的最新历元比,车速 × 时间差就成了 ~1 m 的"偏差"。
TEST(DiagnosisEngine, PairsOnEpochTimeSoMotionIsNotDivergence) {
  auto e = make_engine();
  const auto lat_at = [](double epoch) { return 44.5 + (2.0 * (epoch - 100.0)) / 111132.0; };
  int ticks = 0;
  for (int k = 0; k <= 700; ++k) {   // t = 100.0 .. 170.0,步长 0.1 s(用整数步避免累积误差)
    const double t = 100.0 + 0.1 * k;
    SolutionSample dev = fixed(lat_at(t));
    dev.epoch_t = t;
    e.on_device_solution(t, dev);
    if (k % 10 == 0) corrections(e, t);
    if (k % 10 == 4) {
      SolutionSample sol = fixed(lat_at(t - 0.4));
      sol.epoch_t = t - 0.4;
      e.on_solution(t, sol);
    }
    if (k % 10 == 5) {
      const auto r = e.tick(t);
      ++ticks;
      EXPECT_FALSE(has_code(r, "device_divergence")) << "t=" << t;
      ASSERT_TRUE(r.divergence.divergence_m.has_value()) << "t=" << t << ":同历元的两路必须配上";
      EXPECT_LT(*r.divergence.divergence_m, 0.01) << "t=" << t;
    }
  }
  EXPECT_EQ(ticks, 70);
}

// final fix F2:两路都带历元时刻、但最近的也差 >= 0.5 s 时不配对(也不退回按到达时刻配对)
TEST(DiagnosisEngine, EpochTimesTooFarApartDoNotPair) {
  auto e = make_engine();
  corrections(e, 100.0);
  SolutionSample dev = fixed(44.5 + 0.5 / 111000.0);
  dev.epoch_t = 99.5;
  e.on_device_solution(99.9, dev);
  dev.epoch_t = 100.5;
  e.on_device_solution(100.1, dev);
  SolutionSample sol = fixed();
  sol.epoch_t = 100.0;
  e.on_solution(100.0, sol);
  const auto r = e.tick(100.2);   // 到达时刻只差 0.1 s,按到达时刻本来会配上
  EXPECT_FALSE(r.divergence.divergence_m.has_value());
  EXPECT_FALSE(r.divergence.since.has_value());
}

TEST(DiagnosisEngine, BaseStationShiftFromRtcm1005) {
  DiagnosisConfig cfg;
  cfg.base_warmup_s = 10.0;
  auto e = make_engine(cfg);
  const auto base = make_1005_frame(7, X, Y, Z);
  auto u = e.on_corrections(0.0, base.data(), base.size());
  ASSERT_EQ(u.size(), 1u);
  EXPECT_EQ(u[0].coords.station_id, 7);
  EXPECT_TRUE(u[0].feed.history_changed);
  EXPECT_FALSE(u[0].feed.offset_m.has_value());

  u = e.on_corrections(10.0, base.data(), base.size());
  ASSERT_EQ(u.size(), 1u);
  EXPECT_TRUE(u[0].feed.baseline_learned);
  ASSERT_TRUE(e.baseline().has_value());

  const auto moved = make_1005_frame(7, X + 0.8, Y, Z);
  u = e.on_corrections(11.0, moved.data(), moved.size());
  ASSERT_EQ(u.size(), 1u);
  EXPECT_TRUE(u[0].feed.history_changed);
  ASSERT_TRUE(u[0].feed.offset_m.has_value());
  EXPECT_NEAR(*u[0].feed.offset_m, 0.8, 1e-3);

  e.on_solution(11.0, fixed());
  const auto r = e.tick(11.5);
  EXPECT_EQ(r.result.status().code, "base_shift");
  EXPECT_TRUE(opened(r, "base_shift"));
}

// final fix F3:运维确认基站确实搬迁后,通过引擎以最后收到的 1005/1006 坐标重置基线
TEST(DiagnosisEngine, ResetBaseBaselineClearsBaseShift) {
  DiagnosisConfig cfg;
  cfg.base_warmup_s = 10.0;
  auto e = make_engine(cfg);
  const std::vector<uint8_t> garbage = {0x01, 0x02, 0x03};
  e.on_corrections(0.0, garbage.data(), garbage.size());
  EXPECT_FALSE(e.reset_base_baseline(0.0).has_value()) << "还没收到过 1005/1006,没有坐标可用";

  const auto base = make_1005_frame(7, X, Y, Z);
  e.on_corrections(0.0, base.data(), base.size());
  ASSERT_TRUE(e.on_corrections(10.0, base.data(), base.size())[0].feed.baseline_learned);
  const auto moved = make_1005_frame(7, X + 0.8, Y, Z);
  e.on_corrections(11.0, moved.data(), moved.size());
  e.on_solution(11.0, fixed());
  ASSERT_TRUE(has_code(e.tick(11.5), "base_shift"));

  const auto u = e.reset_base_baseline(11.5);
  ASSERT_TRUE(u.has_value());
  EXPECT_DOUBLE_EQ(u->t, 11.5);
  EXPECT_EQ(u->coords.station_id, 7);
  EXPECT_NEAR(u->coords.x, X + 0.8, 1e-3);
  ASSERT_TRUE(u->feed.offset_m.has_value());
  EXPECT_DOUBLE_EQ(*u->feed.offset_m, 0.0);
  EXPECT_TRUE(u->feed.baseline_learned);
  ASSERT_TRUE(e.baseline().has_value());
  EXPECT_NEAR(e.baseline()->x, X + 0.8, 1e-3);

  e.on_solution(12.0, fixed());
  EXPECT_FALSE(has_code(e.tick(12.0), "base_shift")) << "重置后 held 的基站位移必须同时清零,不能等下一条 1005";
}

TEST(DiagnosisEngine, PersistedBaselineIsUsedImmediately) {
  auto e = DiagnosisEngine(DiagnosisConfig{}, {}, true, Ecef{X, Y, Z}, Ecef{X, Y, Z});
  const auto moved = make_1005_frame(7, X, Y + 0.3, Z);
  const auto u = e.on_corrections(0.0, moved.data(), moved.size());
  ASSERT_EQ(u.size(), 1u);
  ASSERT_TRUE(u[0].feed.offset_m.has_value());
  EXPECT_NEAR(*u[0].feed.offset_m, 0.3, 1e-3);
}

TEST(DiagnosisEngine, MultipathFromStatLinesExpiresWhenTheStreamStops) {
  auto e = make_engine();
  corrections(e, 100.0);
  e.on_solution(100.0, fixed());
  e.on_stat_line(100.0, sat_line("C08", 1000.0, 1, 15.0, 3.5, 30.0, 1, 0, 0));
  e.on_stat_line(100.2, sat_line("G17", 1000.0, 1, 12.0, -2.8, 33.0, 1, 0, 0));
  EXPECT_TRUE(has_code(e.tick(100.5), "multipath"));

  corrections(e, 106.0);
  e.on_solution(106.0, fixed());
  EXPECT_FALSE(has_code(e.tick(106.0), "multipath")) << "$SAT 流 5.8 s 没更新,多路径结论不能一直挂着";
}

TEST(DiagnosisEngine, CycleSlipsCountedOnArrivalTime) {
  auto e = make_engine();
  corrections(e, 100.0);
  e.on_solution(100.0, fixed());
  e.on_stat_line(100.0, sat_line("G05", 1000.0, 1, 60.0, 0.1, 45.0, 1, 0, 0));
  e.on_stat_line(101.0, sat_line("G05", 1001.0, 1, 60.0, 0.1, 45.0, 1, 9, 0));
  EXPECT_TRUE(has_code(e.tick(101.5), "cycle_slip"));
  corrections(e, 140.0);
  e.on_solution(140.0, fixed());
  EXPECT_FALSE(has_code(e.tick(140.0), "cycle_slip")) << "30 s 窗口过后不再计数";
}

TEST(DiagnosisEngine, DualFrequencySlipCountersDoNotAccumulateSpuriousSlips) {   // finding I1
  auto e = make_engine();
  for (int t = 100; t <= 140; ++t) {
    corrections(e, static_cast<double>(t));
    e.on_solution(static_cast<double>(t), fixed());
    const double tow = 1000.0 + (t - 100);
    e.on_stat_line(static_cast<double>(t), sat_line("G05", tow, 1, 60.0, 0.1, 45.0, 1, 1, 0));
    e.on_stat_line(static_cast<double>(t), sat_line("G05", tow, 2, 60.0, 0.1, 45.0, 1, 0, 0));
    EXPECT_FALSE(has_code(e.tick(t + 0.5), "cycle_slip"))
        << "t=" << t << ":L1 的 slipc 从第二个历元起恒为 1,没有真实周跳;"
        << "L2 是独立频点的独立计数器,不该被 SlipWindow 当成同一颗星的另一次观测";
  }
}

TEST(DiagnosisEngine, SatsMinMetricOnlyWhenASolutionExists) {
  DiagnosisConfig cfg;
  cfg.close_hysteresis_s = 0.0;
  auto e = make_engine(cfg);
  corrections(e, 100.0);
  e.tick(104.0);                     // 无解 + 差分中断:开 corr_outage 与 no_solution
  corrections(e, 105.0);
  e.on_solution(105.0, fixed());
  e.tick(105.1);                     // 两者恢复计时开始
  corrections(e, 105.2);
  const auto r = e.tick(105.2);
  const auto it = std::find_if(r.transitions.begin(), r.transitions.end(),
                               [](const EventTransition& t) { return t.code == "corr_outage"; });
  ASSERT_NE(it, r.transitions.end());
  EXPECT_EQ(it->kind, EventKind::Close);
  EXPECT_EQ(it->peak.count("sats_min"), 0u) << "无解时不报 sats_min,不能把峰值拉成 0";
  EXPECT_GT(it->peak.at("corr_gap_s"), 3.0);
}

TEST(DiagnosisEngine, ShutdownClosesEverythingOpen) {
  auto e = make_engine();
  corrections(e, 100.0);
  e.tick(104.0);
  const auto closed = e.shutdown(105.0);
  ASSERT_FALSE(closed.empty());
  for (const auto& t : closed) {
    EXPECT_EQ(t.kind, EventKind::Close);
    EXPECT_EQ(t.reason, CloseReason::Shutdown);
  }
}
