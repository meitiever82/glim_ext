#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "gnss_bringup/diag_node_support.hpp"
#include "node_process_harness.hpp"   // make_temp_dir、TempDirGuard、read_file、count_occurrences
using namespace gnss_bringup;
using gnss_bringup_test::count_occurrences;
using gnss_bringup_test::make_temp_dir;
using gnss_bringup_test::read_file;
using gnss_bringup_test::TempDirGuard;
namespace fs = std::filesystem;

namespace {
gnss_msgs::msg::RtkFix fix() {
  gnss_msgs::msg::RtkFix m;
  m.quality = gnss_msgs::msg::RtkFix::QUALITY_FIXED;
  m.latitude = 44.5;
  m.longitude = 90.28;
  m.sigma_enu = {0.022, 0.011, 0.033};   // E, N, U
  m.diff_age = 0.8f;
  m.sats_used = 20;
  return m;
}

std::string value_of(const diagnostic_msgs::msg::DiagnosticStatus& s, const std::string& key) {
  for (const auto& kv : s.values) {
    if (kv.key == key) return kv.value;
  }
  return "<missing>";
}

gnss_core::TickResult tick_with(std::vector<gnss_core::Verdict> verdicts) {
  gnss_core::TickResult r;
  r.result.verdicts = std::move(verdicts);
  r.divergence.threshold_m = 0.15;
  return r;
}
}  // namespace

TEST(ToSolutionSample, MapsFieldsAndSwapsSigmaOrder) {
  const auto s = to_solution_sample(fix());
  EXPECT_EQ(s.quality, gnss_core::Quality::FIXED);
  EXPECT_DOUBLE_EQ(s.lat, 44.5);
  EXPECT_DOUBLE_EQ(s.lon, 90.28);
  EXPECT_EQ(s.ns, 20);
  EXPECT_DOUBLE_EQ(s.sdn, 0.011) << "N 取 sigma_enu[1]";
  EXPECT_DOUBLE_EQ(s.sde, 0.022) << "E 取 sigma_enu[0]";
  EXPECT_NEAR(s.age, 0.8, 1e-6);
}

TEST(ToSolutionSample, ZeroRatioAndZeroGnssTimeMeanNotProvided) {
  auto m = fix();
  auto s = to_solution_sample(m);
  EXPECT_FALSE(s.ratio.has_value()) << "ratio 0 = 源不提供,ambiguity 规则不能拿 0 去判";
  EXPECT_FALSE(s.epoch_t.has_value()) << "gnss_time 0 = 源不提供,只能按到达时刻配对";
  m.ratio = 7.5f;
  m.gnss_time = 1789208625.25;
  s = to_solution_sample(m);
  ASSERT_TRUE(s.ratio.has_value());
  EXPECT_DOUBLE_EQ(*s.ratio, 7.5);
  ASSERT_TRUE(s.epoch_t.has_value());
  EXPECT_DOUBLE_EQ(*s.epoch_t, 1789208625.25);
}

TEST(ToSolutionSample, UnknownQualityIsNone) {
  auto m = fix();
  m.quality = 9;
  EXPECT_EQ(to_solution_sample(m).quality, gnss_core::Quality::NONE);
}

TEST(DiagnosticStatus, LevelIsTheWorstVerdictAndMessageIsTheStatusVerdict) {
  using S = diagnostic_msgs::msg::DiagnosticStatus;
  const auto r = tick_with({{gnss_core::Level::Warning, "multipath", "G01 残差异常——疑似多路径"},
                            {gnss_core::Level::Serious, "device_divergence", "610 输出与独立解算偏差 0.40m"},
                            {gnss_core::Level::Ok, "rtk_fixed", "RTK 固定"}});
  const auto st = make_diagnostic_status(r, {"device_divergence", "multipath"});
  EXPECT_EQ(st.name, "gnss_diag");
  EXPECT_EQ(st.level, S::ERROR) << "级别取全部结论里最严重的,不是第一条";
  EXPECT_EQ(st.message, "G01 残差异常——疑似多路径");
  EXPECT_EQ(value_of(st, "status_code"), "multipath");
  EXPECT_EQ(value_of(st, "open_events"), "device_divergence,multipath");
  EXPECT_EQ(value_of(st, "divergence_m"), "-");
  EXPECT_EQ(value_of(st, "divergence_threshold_m"), "0.150");
  EXPECT_EQ(value_of(st, "verdict.device_divergence"), "serious 610 输出与独立解算偏差 0.40m");
}

TEST(DiagnosticStatus, OkAndInfoAreOkWarningIsWarn) {
  using S = diagnostic_msgs::msg::DiagnosticStatus;
  EXPECT_EQ(to_diagnostic_level(gnss_core::Level::Ok), S::OK);
  EXPECT_EQ(to_diagnostic_level(gnss_core::Level::Info), S::OK);
  EXPECT_EQ(to_diagnostic_level(gnss_core::Level::Warning), S::WARN);
  EXPECT_EQ(to_diagnostic_level(gnss_core::Level::Critical), S::ERROR);
  auto r = tick_with({{gnss_core::Level::Info, "no_solution", "独立解算未启用"}});
  r.divergence.divergence_m = 0.0234;
  const auto st = make_diagnostic_status(r, {});
  EXPECT_EQ(st.level, S::OK);
  EXPECT_EQ(value_of(st, "open_events"), "-");
  EXPECT_EQ(value_of(st, "divergence_m"), "0.023");
}

TEST(DiagnosticStatus, EmptyVerdictsAreAnErrorNotACrash) {
  const auto st = make_diagnostic_status(tick_with({}), {});
  EXPECT_EQ(st.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST(DiagnosticStatus, StartupGraceIsOk) {
  const auto st = make_startup_grace_status(42.4);
  EXPECT_EQ(st.name, "gnss_diag");
  EXPECT_EQ(st.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_NE(st.message.find("启动宽限期"), std::string::npos) << st.message;
  EXPECT_NE(st.message.find("42"), std::string::npos) << st.message;
}

TEST(DayFileAppender, WritesTheHeaderOnceAndRollsOverAtUtcMidnight) {
  const auto root = make_temp_dir("day_appender_");
  ASSERT_FALSE(root.empty());
  TempDirGuard guard(root);
  {
    DayFileAppender out(root, "events.log", "% header\n");
    EXPECT_TRUE(out.append(1789257599.0, "a"));   // 2026-09-12 23:59:59 UTC
    EXPECT_TRUE(out.append(1789257599.5, "a2"));
    EXPECT_TRUE(out.append(1789257600.0, "b"));   // 2026-09-13 00:00:00 UTC
    EXPECT_EQ(out.current_path(), root + "/20260913/events.log");
  }
  EXPECT_EQ(read_file(root + "/20260912/events.log"), "% header\na\na2\n");
  EXPECT_EQ(read_file(root + "/20260913/events.log"), "% header\nb\n");
  {
    DayFileAppender again(root, "events.log", "% header\n");   // 重启后追加,不重复写头
    EXPECT_TRUE(again.append(1789257601.0, "c"));
  }
  EXPECT_EQ(count_occurrences(read_file(root + "/20260913/events.log"), "% header"), 1u);
}

TEST(DayFileAppender, FailureIsReportedAndTheNextLineRetries) {
  const auto base = make_temp_dir("day_appender_fail_");
  ASSERT_FALSE(base.empty());
  TempDirGuard guard(base);
  const std::string root = base + "/root";
  std::ofstream(root) << "a regular file where the root directory should be";
  DayFileAppender out(root, "events.log", "% header\n");
  EXPECT_FALSE(out.append(1789208625.0, "lost"));
  fs::remove(root);
  fs::create_directories(root);
  EXPECT_TRUE(out.append(1789208626.0, "kept"));
  EXPECT_EQ(read_file(root + "/20260912/events.log"), "% header\nkept\n");
}

TEST(BaseHistory, ParsesDataLinesOnly) {
  const auto p = parse_base_history_line("2026/09/12 10:23:45.000  -2148744.1000 4426641.2000 4044655.9000");
  ASSERT_TRUE(p.has_value());
  EXPECT_DOUBLE_EQ(p->x, -2148744.1);
  EXPECT_DOUBLE_EQ(p->z, 4044655.9);
  EXPECT_FALSE(parse_base_history_line("% time=UTC").has_value());
  EXPECT_FALSE(parse_base_history_line("2026/09/12 10:23:45.000  -2148744.1").has_value()) << "掉电留下的半行";
  EXPECT_FALSE(parse_base_history_line("").has_value());
}

TEST(BaseHistory, ReadsTheLastValidLineOfTheNewestDayThatHasOne) {
  const auto root = make_temp_dir("base_history_");
  ASSERT_FALSE(root.empty());
  TempDirGuard guard(root);
  const auto write = [&root](const std::string& rel, const std::string& text) {
    fs::create_directories(fs::path(root + "/" + rel).parent_path());
    std::ofstream(root + "/" + rel) << text;
  };
  write("20260910/base.pos", "% h\n2026/09/10 01:00:00.000  1.0000 2.0000 3.0000\n");
  write("20260912/base.pos", "% h\n2026/09/12 01:00:00.000  4.0000 5.0000 6.0000\n"
                             "2026/09/12 02:00:00.000  7.0000 8.0000 9.0000\n"
                             "2026/09/12 03:00:00.000  10.0\n");
  write("20260913/base.pos", "% only a header\n");
  fs::create_directories(root + "/20260914");                     // 没有 base.pos
  write("notadate/base.pos", "2026/09/20 01:00:00.000  0.0000 0.0000 0.0000\n");

  const auto last = read_last_base_history(root);
  ASSERT_TRUE(last.has_value());
  EXPECT_DOUBLE_EQ(last->x, 7.0);
  EXPECT_DOUBLE_EQ(last->z, 9.0);
  EXPECT_FALSE(read_last_base_history(root + "/absent").has_value());
}

TEST(MonotonicClockGuard, ClampsSmallJitterAndReportsLargeBackwardJumps) {
  MonotonicClockGuard g(1.0);
  EXPECT_FALSE(g.last().has_value());
  auto s = g.step(100.0);
  EXPECT_FALSE(s.jumped);
  EXPECT_DOUBLE_EQ(s.t, 100.0);
  s = g.step(99.5);                     // 回退 0.5 s:夹住
  EXPECT_FALSE(s.jumped);
  EXPECT_DOUBLE_EQ(s.t, 100.0);
  s = g.step(101.0);
  EXPECT_DOUBLE_EQ(s.t, 101.0);
  s = g.step(50.0);                     // 回退 51 s:回跳,新起点
  EXPECT_TRUE(s.jumped);
  EXPECT_DOUBLE_EQ(s.t, 50.0);
  s = g.step(50.5);
  EXPECT_FALSE(s.jumped);
  EXPECT_DOUBLE_EQ(*g.last(), 50.5);
  EXPECT_FALSE(g.step(5000.0).jumped) << "向前跳不算回跳";
}

TEST(ControlPoints, ParsesParallelArrays) {
  const auto cps = control_points_from_params({"K1", "K2"}, {44.5, 44.6}, {90.1, 90.2});
  ASSERT_EQ(cps.size(), 2u);
  EXPECT_EQ(cps[1].name, "K2");
  EXPECT_DOUBLE_EQ(cps[1].lat, 44.6);
  EXPECT_DOUBLE_EQ(cps[1].lon, 90.2);
  EXPECT_TRUE(control_points_from_params({}, {}, {}).empty());
}

TEST(ControlPoints, RejectsMismatchedEmptyOrOutOfRange) {
  const auto rejected = [](std::vector<std::string> n, std::vector<double> la, std::vector<double> lo) {
    try {
      control_points_from_params(n, la, lo);
    } catch (const std::invalid_argument& e) {
      return std::string(e.what()).rfind("control_points", 0) == 0;
    }
    return false;
  };
  EXPECT_TRUE(rejected({"K1"}, {}, {90.0}));
  EXPECT_TRUE(rejected({""}, {44.0}, {90.0}));
  EXPECT_TRUE(rejected({"K1"}, {91.0}, {90.0}));
  EXPECT_TRUE(rejected({"K1"}, {44.0}, {-181.0}));
  EXPECT_TRUE(rejected({"K1"}, {std::numeric_limits<double>::quiet_NaN()}, {90.0}));
}

TEST(UnpairedWatch, WarnsOnceAfterTheStreakAndResetsWhenPaired) {
  UnpairedWatch w(3);
  EXPECT_FALSE(w.update(true, false));
  EXPECT_FALSE(w.update(true, false));
  EXPECT_TRUE(w.update(true, false)) << "连续 3 拍两路都在却没配上";
  EXPECT_FALSE(w.update(true, false)) << "同一段只提醒一次";
  EXPECT_FALSE(w.update(true, true));
  EXPECT_FALSE(w.update(true, false));
  EXPECT_FALSE(w.update(false, false)) << "有一路没到不算配对失败,计数清零";
  EXPECT_FALSE(w.update(true, false));
  EXPECT_FALSE(w.update(true, false));
  EXPECT_TRUE(w.update(true, false));
}
