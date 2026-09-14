#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "gnss_core/event_book.hpp"
using namespace gnss_core;

namespace {
const Verdict OK{Level::Ok, "rtk_fixed", "RTK 固定"};
const Verdict OUT{Level::Serious, "corr_outage", "差分中断 5s——5G 链路或平台转发问题"};
const Verdict FLOAT_{Level::Warning, "ambiguity", "模糊度无法固定(ratio=1.5)——遮挡过渡区常见"};
const Verdict INFO{Level::Info, "not_fixed", "非固定解(FLOAT)"};

std::vector<Verdict> v(std::initializer_list<Verdict> l) { return l; }

std::vector<std::string> describe(const std::vector<EventTransition>& ts) {
  std::vector<std::string> out;
  for (const auto& t : ts) {
    out.push_back(std::string(t.kind == EventKind::Open ? "open " : "close ") + t.code + "@" +
                  std::to_string(static_cast<int>(t.t)));
  }
  return out;
}
}  // namespace

TEST(EventBook, OpenCloseWithHysteresis) {   // 移植 test_open_close_with_hysteresis
  EventBook b(10.0);
  EXPECT_TRUE(b.update(100, v({OK}), std::nullopt, {}).empty());
  auto t = b.update(101, v({OUT, OK}), LatLon{44.5, 90.2}, {});
  ASSERT_EQ(describe(t), (std::vector<std::string>{"open corr_outage@101"}));
  EXPECT_EQ(t[0].level, Level::Serious);
  ASSERT_TRUE(t[0].pos.has_value());
  EXPECT_DOUBLE_EQ(t[0].pos->lat, 44.5);
  EXPECT_TRUE(b.update(105, v({OUT, OK}), std::nullopt, {}).empty());
  EXPECT_TRUE(b.update(106, v({OK}), std::nullopt, {}).empty());
  EXPECT_TRUE(b.update(110, v({OK}), std::nullopt, {}).empty()) << "恢复 4 s,未满 10 s";
  t = b.update(117, v({OK}), std::nullopt, {});
  ASSERT_EQ(describe(t), (std::vector<std::string>{"close corr_outage@117"}));
  EXPECT_DOUBLE_EQ(t[0].t_open, 101.0);
  EXPECT_EQ(t[0].reason, CloseReason::Recovered);
  EXPECT_EQ(t[0].message, OUT.message);
  EXPECT_TRUE(b.open_codes().empty());
}

TEST(EventBook, DifferentCodesAreTrackedIndependently) {
  // rtk-monitor 在这里会"关旧开新";按码独立后两个事件并存,旧的按自己的迟滞关闭
  EventBook b(10.0);
  b.update(100, v({OUT}), std::nullopt, {});
  auto t = b.update(101, v({FLOAT_}), std::nullopt, {});
  EXPECT_EQ(describe(t), (std::vector<std::string>{"open ambiguity@101"}));
  EXPECT_EQ(b.open_codes(), (std::vector<std::string>{"ambiguity", "corr_outage"}));
  t = b.update(111, v({FLOAT_}), std::nullopt, {});
  EXPECT_EQ(describe(t), (std::vector<std::string>{"close corr_outage@111"}));
  EXPECT_EQ(b.open_codes(), (std::vector<std::string>{"ambiguity"}));
}

TEST(EventBook, SimultaneousHitsOpenSimultaneousEventsInVerdictOrder) {
  EventBook b(10.0);
  const auto t = b.update(100, v({OUT, FLOAT_, INFO}), std::nullopt, {});
  EXPECT_EQ(describe(t), (std::vector<std::string>{"open corr_outage@100", "open ambiguity@100"}));
}

TEST(EventBook, ClosesComeBeforeOpensWithinATick) {
  EventBook b(0.0);
  b.update(100, v({OUT}), std::nullopt, {});
  b.update(101, v({OK}), std::nullopt, {});    // 恢复计时开始
  const auto t = b.update(102, v({FLOAT_}), std::nullopt, {});
  EXPECT_EQ(describe(t), (std::vector<std::string>{"close corr_outage@102", "open ambiguity@102"}));
}

TEST(EventBook, RelapseResetsHysteresis) {   // 移植 test_relapse_resets_hysteresis
  EventBook b(10.0);
  b.update(100, v({OUT}), std::nullopt, {});
  b.update(101, v({OK}), std::nullopt, {});
  b.update(105, v({OUT}), std::nullopt, {});
  EXPECT_TRUE(b.update(120, v({OK}), std::nullopt, {}).empty()) << "复发后恢复计时从 120 重新开始";
  EXPECT_EQ(describe(b.update(131, v({OK}), std::nullopt, {})),
            (std::vector<std::string>{"close corr_outage@131"}));
}

TEST(EventBook, InfoAndOkNeverOpen) {   // 移植 test_info_does_not_open
  EventBook b(10.0);
  EXPECT_TRUE(b.update(100, v({INFO, OK}), std::nullopt, {}).empty());
  EXPECT_TRUE(b.open_codes().empty());
}

TEST(EventBook, SameCodeReopensAfterClosing) {
  EventBook b(1.0);
  b.update(100, v({OUT}), std::nullopt, {});
  b.update(101, v({OK}), std::nullopt, {});
  b.update(102, v({OK}), std::nullopt, {});
  EXPECT_EQ(describe(b.update(114, v({OUT}), std::nullopt, {})),
            (std::vector<std::string>{"open corr_outage@114"}));
}

TEST(EventBook, PeakMetricsAccumulateAndCloseUsesTheLastPosition) {   // 移植 test_peak_metrics_accumulate_and_persist
  EventBook b(1.0);
  b.update(100, v({OUT}), LatLon{44.0, 90.0}, {{"corr_gap_s", 5.0}});
  b.update(101, v({OUT}), LatLon{44.1, 90.1}, {{"corr_gap_s", 12.0}});
  b.update(102, v({OK}), LatLon{44.2, 90.2}, {{"corr_gap_s", 0.0}});   // 不活跃的 tick 不更新峰值与位置
  const auto t = b.update(104, v({OK}), std::nullopt, {});
  ASSERT_EQ(t.size(), 1u);
  EXPECT_DOUBLE_EQ(t[0].peak.at("corr_gap_s"), 12.0);
  ASSERT_TRUE(t[0].pos.has_value());
  EXPECT_DOUBLE_EQ(t[0].pos->lat, 44.1);
  EXPECT_DOUBLE_EQ(t[0].pos->lon, 90.1);
}

TEST(EventBook, MinSuffixMetricsAggregateMinOthersByAbsoluteValue) {   // 移植 test_min_suffix_metrics_aggregate_min
  EventBook b(1.0);
  b.update(100, v({OUT}), std::nullopt, {{"sats_min", 12.0}, {"corr_gap_s", 3.0}, {"divergence_m", 0.0}});
  b.update(101, v({OUT}), std::nullopt, {{"sats_min", 4.0}, {"corr_gap_s", -9.0}});
  b.update(102, v({OUT}), std::nullopt, {{"sats_min", 8.0}, {"corr_gap_s", 5.0}});
  b.update(103, v({OK}), std::nullopt, {});
  const auto t = b.update(105, v({OK}), std::nullopt, {});
  ASSERT_EQ(t.size(), 1u);
  EXPECT_DOUBLE_EQ(t[0].peak.at("sats_min"), 4.0);
  EXPECT_DOUBLE_EQ(t[0].peak.at("corr_gap_s"), -9.0) << "取绝对值最大者并保留符号";
  EXPECT_EQ(t[0].peak.count("divergence_m"), 0u) << "值为 0 的非 _min 指标从未写入";
}

TEST(EventBook, CloseAllAtShutdown) {
  EventBook b(10.0);
  b.update(100, v({OUT, FLOAT_}), LatLon{1.0, 2.0}, {{"corr_gap_s", 4.0}});
  const auto t = b.close_all(130);
  EXPECT_EQ(describe(t), (std::vector<std::string>{"close ambiguity@130", "close corr_outage@130"}));
  for (const auto& e : t) EXPECT_EQ(e.reason, CloseReason::Shutdown);
  EXPECT_TRUE(b.open_codes().empty());
  EXPECT_TRUE(b.close_all(131).empty());
  EXPECT_STREQ(close_reason_name(CloseReason::Shutdown), "shutdown");
}
