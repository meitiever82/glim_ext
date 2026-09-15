#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "gnss_core/report_stats.hpp"
using namespace gnss_core;

namespace {
const double kBase = 3600.0 * 100;   // 整点
const double kLat = 44.5, kLon = 90.28;

double north(double metres) { return kLat + metres / 111132.0; }

PosRecord rec(double t, int q = 1, double lat = kLat, double lon = kLon) {
  PosRecord r;
  r.stamp = t;
  r.q = q;
  r.lat = lat;
  r.lon = lon;
  r.height = 600.0;
  return r;
}

ReportParams params(double t0, double t1) {
  ReportParams p;
  p.window = ReportWindow{t0, t1};
  return p;
}

ReportEvent event(const std::string& code, double t_open, std::optional<double> t_close,
                  std::optional<LatLon> pos = std::nullopt) {
  ReportEvent e;
  e.code = code;
  e.level = "serious";
  e.message = code;
  e.t_open = t_open;
  e.t_close = t_close;
  if (t_close) e.close_reason = "recovered";
  e.pos = pos;
  return e;
}
}  // namespace

// 移植 rtk-monitor test_report_stats:固定率 9/11、分小时 0.8
TEST(ReportStats, FixRatioQualityCountsAndHourlyPerSource) {
  ReportInputs in;
  for (int i = 0; i < 10; ++i) in.sources["rtkrcv"].push_back(rec(kBase + i, i < 8 ? 1 : 2));
  in.sources["rtkrcv"].push_back(rec(kBase + 3700.0, 1));
  in.sources["can"] = {rec(kBase, 1), rec(kBase + 1, 1), rec(kBase + 2, 2), rec(kBase + 3, 5)};

  const auto s = compute_report(in, params(kBase, kBase + 7200.0));
  ASSERT_EQ(s.sources.size(), 2u);
  EXPECT_EQ(s.sources[0].name, "can");
  EXPECT_EQ(s.sources[1].name, "rtkrcv");

  const auto& rtk = s.sources[1];
  EXPECT_EQ(rtk.epochs, 11);
  ASSERT_TRUE(rtk.fix_ratio.has_value());
  EXPECT_NEAR(*rtk.fix_ratio, 9.0 / 11.0, 1e-12);
  ASSERT_EQ(rtk.hourly.size(), 2u);
  EXPECT_DOUBLE_EQ(rtk.hourly[0].t_start, kBase);
  EXPECT_EQ(rtk.hourly[0].epochs, 10);
  EXPECT_NEAR(*rtk.hourly[0].fix_ratio, 0.8, 1e-12);
  EXPECT_EQ(rtk.hourly[1].epochs, 1);
  EXPECT_NEAR(*rtk.hourly[1].fix_ratio, 1.0, 1e-12);

  const auto& can = s.sources[0];
  EXPECT_NEAR(*can.fix_ratio, 0.5, 1e-12) << "固定解一律按 RTKLIB Q==1,不沿用 rtk-monitor 的 can q==4";
  EXPECT_EQ(can.counts.fixed, 2);
  EXPECT_EQ(can.counts.floating, 1);
  EXPECT_EQ(can.counts.dgps, 0);
  EXPECT_EQ(can.counts.single, 1);
  EXPECT_EQ(can.counts.other, 0);
}

TEST(ReportStats, HourlyBucketsCoverTheWholeWindowAndEmptyHoursHaveNoRatio) {
  ReportInputs in;
  in.sources["rtkrcv"] = {rec(kBase + 10.0)};
  auto s = compute_report(in, params(kBase, kBase + 3 * 3600.0));
  ASSERT_EQ(s.sources[0].hourly.size(), 3u);
  EXPECT_EQ(s.sources[0].hourly[1].epochs, 0);
  EXPECT_FALSE(s.sources[0].hourly[1].fix_ratio.has_value());

  s = compute_report(in, params(kBase + 1800.0, kBase + 5400.0));   // 不在整点开始的窗口
  ASSERT_EQ(s.sources[0].hourly.size(), 2u);
  EXPECT_DOUBLE_EQ(s.sources[0].hourly[0].t_start, kBase);
  EXPECT_DOUBLE_EQ(s.sources[0].hourly[1].t_start, kBase + 3600.0);
}

TEST(ReportStats, SourcesAreOrderedCanGpchcRtkrcvRefThenOthersByName) {
  ReportInputs in;
  for (const char* n : {"zeta", "ref", "can", "rtkrcv", "alpha"}) in.sources[n] = {rec(kBase)};
  const std::vector<std::string> want{"can", "rtkrcv", "ref", "alpha", "zeta"};
  EXPECT_EQ(ordered_source_names(in.sources), want);
  const auto s = compute_report(in, params(kBase, kBase + 10.0));
  ASSERT_EQ(s.sources.size(), want.size());
  for (size_t i = 0; i < want.size(); ++i) EXPECT_EQ(s.sources[i].name, want[i]);
}

// 移植 rtk-monitor test_report_can_rtk_deviation
TEST(ReportStats, DeviceVsRtkrcvPairsTheNearestEpochWithinTolerance) {
  ReportInputs in;
  in.sources["rtkrcv"] = {rec(kBase + 1.0), rec(kBase + 2.0)};
  in.sources["can"] = {rec(kBase + 1.2, 1, north(1.0))};
  const auto s = compute_report(in, params(kBase, kBase + 10.0));
  ASSERT_EQ(s.divergence.size(), 1u);
  EXPECT_EQ(s.divergence[0].device, "can");
  EXPECT_EQ(s.divergence[0].reference, "rtkrcv");
  EXPECT_EQ(s.divergence[0].n, 1) << "kBase+2 的独立解离 can 历元 0.8 s,不配对";
  EXPECT_NEAR(*s.divergence[0].max_m, 1.0, 0.01);
  EXPECT_NEAR(*s.divergence[0].mean_m, 1.0, 0.01);
}

// 移植 rtk-monitor test_report_can_rtk_deviation_skips_far_apart_timestamps,并覆盖 gpchc 与容差边界
TEST(ReportStats, FarApartEpochsAreNotPairedAndEachDeviceIsReported) {
  ReportInputs in;
  in.sources["rtkrcv"] = {rec(kBase + 1.0), rec(kBase + 5.0)};
  in.sources["can"] = {rec(kBase + 1.9, 1, north(5.0))};
  in.sources["gpchc"] = {rec(kBase + 5.5, 1, north(2.0))};   // 恰好 0.5 s,容差含边界
  auto s = compute_report(in, params(kBase, kBase + 10.0));
  ASSERT_EQ(s.divergence.size(), 2u);
  EXPECT_EQ(s.divergence[0].device, "can");
  EXPECT_EQ(s.divergence[0].n, 0);
  EXPECT_FALSE(s.divergence[0].max_m.has_value());
  EXPECT_EQ(s.divergence[1].device, "gpchc");
  EXPECT_EQ(s.divergence[1].n, 1);
  EXPECT_NEAR(*s.divergence[1].max_m, 2.0, 0.01);

  ReportInputs no_ref;
  no_ref.sources["can"] = {rec(kBase + 1.0)};
  EXPECT_TRUE(compute_report(no_ref, params(kBase, kBase + 10.0)).divergence.empty()) << "没有 rtkrcv 就无法比较";
}

// 移植 test_report_abs_ref_control_point_deviation / ignores_non_fixed_epochs,改为只取最近控制点
TEST(ReportStats, AbsRefUsesFixedEpochsOfEverySourceAndTheNearestControlPoint) {
  ReportInputs in;
  in.sources["rtkrcv"] = {rec(kBase + 1.0, 1, north(0.5)), rec(kBase + 2.0, 2, north(0.3)), rec(kBase + 3.0, 1, 44.6)};
  in.sources["can"] = {rec(kBase + 4.0, 1, north(1.8))};
  auto p = params(kBase, kBase + 10.0);
  p.control_points = {ControlPoint{"CP1", kLat, kLon}, ControlPoint{"CP2", north(2.0), kLon}};

  auto s = compute_report(in, p);
  ASSERT_EQ(s.abs_ref.samples.size(), 2u);
  EXPECT_EQ(s.abs_ref.samples[0].source, "rtkrcv");
  EXPECT_EQ(s.abs_ref.samples[0].control_point, "CP1") << "离 CP1 0.5 m、CP2 1.5 m,只算最近的";
  EXPECT_NEAR(s.abs_ref.samples[0].dev_m, 0.5, 0.01);
  EXPECT_EQ(s.abs_ref.samples[1].source, "can");
  EXPECT_EQ(s.abs_ref.samples[1].control_point, "CP2");
  EXPECT_NEAR(s.abs_ref.samples[1].dev_m, 0.2, 0.01);
  EXPECT_NEAR(*s.abs_ref.max_m, 0.5, 0.01);
  EXPECT_TRUE(s.abs_ref.exceeded);

  p.abs_ref_max_m = 0.6;
  EXPECT_FALSE(compute_report(in, p).abs_ref.exceeded);
  p.control_points.clear();
  s = compute_report(in, p);
  EXPECT_TRUE(s.abs_ref.samples.empty());
  EXPECT_FALSE(s.abs_ref.max_m.has_value());
  EXPECT_FALSE(s.abs_ref.exceeded);
}

// 移植 test_report_base_series_curve
TEST(ReportStats, BaseSeriesIsTheOffsetFromTheFirstSample) {
  ReportInputs in;
  in.base_history = {BaseSample{kBase, Ecef{-2148744.0, 4426641.0, 4044655.0}},
                     BaseSample{kBase + 10.0, Ecef{-2148744.3, 4426641.0, 4044655.0}}};
  auto p = params(kBase, kBase + 100.0);
  auto s = compute_report(in, p);
  ASSERT_EQ(s.base.series.size(), 2u);
  EXPECT_DOUBLE_EQ(s.base.series[0].offset_m, 0.0);
  EXPECT_NEAR(s.base.series[1].offset_m, 0.3, 1e-6);
  EXPECT_NEAR(*s.base.max_m, 0.3, 1e-6);
  EXPECT_TRUE(s.base.exceeded) << "超过 base_shift_m 0.1";
  p.base_shift_m = 0.5;
  EXPECT_FALSE(compute_report(in, p).base.exceeded);
  EXPECT_FALSE(compute_report(ReportInputs{}, p).base.max_m.has_value());
}

TEST(ReportStats, EventsAreCopiedAndSummarisedByCode) {
  ReportInputs in;
  in.events = {event("corr_outage", kBase, kBase + 28.0), event("low_sats", kBase + 50.0, std::nullopt),
               event("corr_outage", kBase + 100.0, kBase + 112.0)};
  in.warnings = {"某个警告"};
  const auto s = compute_report(in, params(kBase, kBase + 200.0));
  EXPECT_EQ(s.events.size(), 3u);
  ASSERT_EQ(s.event_summary.size(), 2u);
  EXPECT_EQ(s.event_summary[0].code, "corr_outage");
  EXPECT_EQ(s.event_summary[0].count, 2);
  EXPECT_NEAR(s.event_summary[0].closed_duration_s, 40.0, 1e-9);
  EXPECT_EQ(s.event_summary[0].unclosed, 0);
  EXPECT_EQ(s.event_summary[1].code, "low_sats");
  EXPECT_EQ(s.event_summary[1].count, 1);
  EXPECT_EQ(s.event_summary[1].unclosed, 1);
  EXPECT_EQ(s.warnings, in.warnings) << "装载阶段的警告要带进报告";
}

TEST(ReportStats, TracksShareOneOriginAndSplitOnTimeGaps) {
  ReportInputs in;
  for (int i = 0; i < 10; ++i) in.sources["can"].push_back(rec(kBase + i, 1, north(i)));
  for (int i = 0; i < 5; ++i) in.sources["can"].push_back(rec(kBase + 30 + i, 2, north(20 + i)));
  in.sources["rtkrcv"] = {rec(kBase + 5.0, 1, north(3.0))};
  const auto s = compute_report(in, params(kBase, kBase + 100.0));
  ASSERT_TRUE(s.track_origin.has_value());
  EXPECT_EQ(*s.track_origin, "can");
  ASSERT_EQ(s.tracks.size(), 2u);
  EXPECT_EQ(s.tracks[0].source, "can");
  ASSERT_EQ(s.tracks[0].segments.size(), 2u) << "中间断了 21 s > 5 s";
  ASSERT_EQ(s.tracks[0].segments[0].size(), 10u);
  EXPECT_NEAR(s.tracks[0].segments[0][0].e, 0.0, 1e-6);
  EXPECT_NEAR(s.tracks[0].segments[0][0].n, 0.0, 1e-6);
  EXPECT_NEAR(s.tracks[0].segments[0][9].n, 9.0, 0.02);
  EXPECT_EQ(s.tracks[0].segments[1].size(), 5u);
  EXPECT_EQ(s.tracks[0].segments[1][0].q, 2);
  ASSERT_EQ(s.tracks[1].segments.size(), 1u);
  EXPECT_NEAR(s.tracks[1].segments[0][0].n, 3.0, 0.02) << "所有源用同一个原点";
}

// 10001 点、stride=ceil(10001/4000)=3:10000 % 3 == 1,段尾索引不会被 stride
// 恰好整除(不像 round4a task-3 首版用 10000 点时 9999 % 3 == 0 的巧合,那份
// 数据测不出去掉 `i != b` 的变异——见 task-3-report.md「Fix round 1」)。
TEST(ReportStats, LongTracksAreDownsampledKeepingSegmentEnds) {
  ReportInputs in;
  for (int i = 0; i < 10001; ++i) in.sources["can"].push_back(rec(kBase + i, 1, north(i * 0.1)));
  auto p = params(kBase, kBase + 20000.0);
  p.max_track_points = 4000;
  const auto s = compute_report(in, p);
  ASSERT_EQ(s.tracks.size(), 1u);
  ASSERT_EQ(s.tracks[0].segments.size(), 1u);
  const auto& seg = s.tracks[0].segments[0];
  EXPECT_LE(seg.size(), 4002u);
  EXPECT_GE(seg.size(), 2500u);
  EXPECT_DOUBLE_EQ(seg.front().t, kBase);
  EXPECT_DOUBLE_EQ(seg.back().t, kBase + 10000.0) << "段尾必须保留";
}

TEST(ReportStats, EventMarkersUseTheEventIndexAndSkipEventsWithoutPosition) {
  ReportInputs in;
  in.events = {event("a", kBase, kBase + 1.0, LatLon{kLat, kLon}), event("b", kBase + 2.0, std::nullopt),
               event("c", kBase + 3.0, std::nullopt, LatLon{north(10.0), kLon})};
  const auto s = compute_report(in, params(kBase, kBase + 10.0));
  ASSERT_TRUE(s.track_origin.has_value());
  EXPECT_EQ(*s.track_origin, "事件") << "没有 .pos 时用第一个带位置的事件作原点";
  ASSERT_EQ(s.event_markers.size(), 2u);
  EXPECT_EQ(s.event_markers[0].index, 1);
  EXPECT_NEAR(s.event_markers[0].n, 0.0, 1e-6);
  EXPECT_EQ(s.event_markers[1].index, 3);
  EXPECT_NEAR(s.event_markers[1].n, 10.0, 0.05);
  EXPECT_TRUE(s.tracks.empty());
}

// design 决定 11:原点取"第一个非空源"的首条记录;某个源恰好一条记录都没有
// (比如那份 .pos 存在但窗口内没有落在时间窗里)不能让 recs.front() 越界。
TEST(ReportStats, TrackOriginSkipsEmptySourcesInOrder) {
  ReportInputs in;
  in.sources["can"] = {};
  in.sources["rtkrcv"] = {rec(kBase + 1.0, 1, north(2.0))};
  const auto s = compute_report(in, params(kBase, kBase + 10.0));
  ASSERT_TRUE(s.track_origin.has_value());
  EXPECT_EQ(*s.track_origin, "rtkrcv") << "can 排在前面但是空的,原点应跳到 rtkrcv";
  ASSERT_EQ(s.sources.size(), 2u);
  EXPECT_EQ(s.sources[0].name, "can");
  EXPECT_EQ(s.sources[0].epochs, 0);
  EXPECT_FALSE(s.sources[0].fix_ratio.has_value());
  ASSERT_EQ(s.tracks.size(), 1u) << "空源不产生轨迹条目";
  EXPECT_EQ(s.tracks[0].source, "rtkrcv");
}

// 事件汇总次数相同时按代码升序(设计里没写明的隐含顺序,由 std::map 聚合的
// 遍历顺序 + stable_sort 一起保证;不依赖 events 输入顺序)
TEST(ReportStats, EventSummaryTiesAreOrderedByCode) {
  ReportInputs in;
  in.events = {event("zeta_code", kBase, kBase + 1.0), event("alpha_code", kBase + 10.0, kBase + 11.0)};
  const auto s = compute_report(in, params(kBase, kBase + 100.0));
  ASSERT_EQ(s.event_summary.size(), 2u);
  EXPECT_EQ(s.event_summary[0].code, "alpha_code") << "次数都是 1,按代码升序,不按输入顺序(zeta 先出现)";
  EXPECT_EQ(s.event_summary[1].code, "zeta_code");
}

TEST(ReportStats, EmptyInputsProduceEmptyStats) {
  const auto s = compute_report(ReportInputs{}, params(kBase, kBase + 100.0));
  EXPECT_TRUE(s.sources.empty());
  EXPECT_TRUE(s.divergence.empty());
  EXPECT_TRUE(s.events.empty());
  EXPECT_TRUE(s.tracks.empty());
  EXPECT_FALSE(s.track_origin.has_value());
  EXPECT_DOUBLE_EQ(s.params.window.t1, kBase + 100.0);
}
