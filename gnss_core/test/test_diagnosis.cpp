#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "gnss_core/diagnosis.hpp"
using namespace gnss_core;

namespace {
// 移植 rtk-monitor tests/test_rules.py 的 _sol / _inp
SolutionSample sol(Quality q = Quality::FIXED, int ns = 38, std::optional<double> ratio = 25.0) {
  SolutionSample s;
  s.quality = q;
  s.lat = 44.5;
  s.lon = 90.28;
  s.ns = ns;
  s.sdn = 0.011;
  s.sde = 0.012;
  s.age = 0.8;
  s.ratio = ratio;
  return s;
}

DiagnosisInput inp() {
  DiagnosisInput in;
  in.now = 1000.0;
  in.corr_last_t = 999.5;
  in.corr_age = 0.8;
  in.base_offset_m = 0.0;
  in.sol = sol();
  in.divergence_threshold_m = 3.0 * std::hypot(0.011, 0.012);
  return in;
}

SatStat sat(const char* name, double el, double resp, double snr) {
  SatStat s;
  s.sat = name;
  s.el = el;
  s.resp = resp;
  s.snr = snr;
  s.valid = true;
  return s;
}

bool has_code(const DiagnosisResult& r, const std::string& code) {
  return std::any_of(r.verdicts.begin(), r.verdicts.end(),
                     [&](const Verdict& v) { return v.code == code; });
}

const DiagnosisConfig kCfg{};
}  // namespace

TEST(Rules, AllGoodIsFixed) {
  const auto r = evaluate_rules(inp(), kCfg);
  EXPECT_EQ(r.status().code, "rtk_fixed");
  EXPECT_EQ(r.status().level, Level::Ok);
  EXPECT_EQ(r.status().message, "RTK 固定");
  EXPECT_EQ(r.verdicts.size(), 1u);
}

TEST(Rules, CorrOutageWinsStatusButAmbiguityIsStillReported) {
  auto in = inp();
  in.corr_last_t = 990.0;               // gap 10 s > 3 s
  in.sol = sol(Quality::FLOAT, 38, 1.5);
  const auto r = evaluate_rules(in, kCfg);
  EXPECT_EQ(r.status().code, "corr_outage");
  EXPECT_EQ(r.status().level, Level::Serious);
  EXPECT_EQ(r.status().message, "差分中断 10s——5G 链路或平台转发问题");
  EXPECT_TRUE(has_code(r, "ambiguity")) << "全命中:优先级低的并发故障也必须报告";
  EXPECT_EQ(r.verdicts.back().code, "not_fixed");
}

TEST(Rules, CorrOutageByAgeOverrun) {
  auto in = inp();
  in.corr_age = 15.0;
  const auto r = evaluate_rules(in, kCfg);
  EXPECT_EQ(r.status().code, "corr_outage");
  EXPECT_EQ(r.status().message, "差分中断 15s——5G 链路或平台转发问题");
}

TEST(Rules, BaseShift) {
  auto in = inp();
  in.base_offset_m = 0.8;
  const auto r = evaluate_rules(in, kCfg);
  EXPECT_EQ(r.status().code, "base_shift");
  EXPECT_EQ(r.status().level, Level::Critical);
  EXPECT_EQ(r.status().message, "⚠ 基站坐标变动 0.80m——所有定位结果将整体平移");
}

TEST(Rules, LowSats) {
  auto in = inp();
  in.sol = sol(Quality::FIXED, 4);
  const auto r = evaluate_rules(in, kCfg);
  EXPECT_EQ(r.status().code, "low_sats");
  EXPECT_EQ(r.status().message, "卫星数不足（4 颗）——疑似遮挡");
}

TEST(Rules, MultipathNeedsTwoBadSatellitesAndNamesThem) {
  auto in = inp();
  in.sats = {sat("C08", 15, 3.5, 30), sat("G17", 12, 2.8, 33)};
  in.sol = sol(Quality::FLOAT, 38, 1.5);
  auto r = evaluate_rules(in, kCfg);
  EXPECT_EQ(r.status().code, "multipath");
  EXPECT_EQ(r.status().message, "C08、G17 残差异常——疑似多路径");
  EXPECT_TRUE(has_code(r, "ambiguity"));

  in.sats.pop_back();
  r = evaluate_rules(in, kCfg);
  EXPECT_FALSE(has_code(r, "multipath")) << "只有一颗坏星不报多路径";
}

TEST(Rules, MultipathUsesAbsoluteResidual) {
  auto in = inp();
  in.sats = {sat("C08", 15, -3.5, 30), sat("G17", 12, -2.8, 33)};
  EXPECT_EQ(evaluate_rules(in, kCfg).status().code, "multipath")
      << "RTKLIB 残差带符号,大的负残差同样是异常";
}

TEST(Rules, MultipathNamesAtMostFourSatellites) {
  auto in = inp();
  in.sats = {sat("G01", 10, 3, 30), sat("G02", 10, 3, 30), sat("G03", 10, 3, 30),
             sat("G04", 10, 3, 30), sat("G05", 10, 3, 30)};
  EXPECT_EQ(evaluate_rules(in, kCfg).status().message, "G01、G02、G03、G04 残差异常——疑似多路径");
}

TEST(Rules, AbsRefShiftWhenNearControlPointAndDeviated) {
  auto in = inp();
  in.control_points = {{"CP1", 44.5 + 0.5 / 111000.0, 90.28}};
  const auto r = evaluate_rules(in, kCfg);
  EXPECT_EQ(r.status().code, "abs_ref_shift");
  EXPECT_EQ(r.status().level, Level::Critical);
  EXPECT_EQ(r.status().message.rfind("⚠ 绝对基准偏差 0.50m@控制点 CP1", 0), 0u) << r.status().message;
}

TEST(Rules, NoAbsRefAlarmOnControlPointFarAwayWithoutPointsOrOnFloat) {
  auto in = inp();
  in.control_points = {{"CP1", 44.5, 90.28}};
  EXPECT_EQ(evaluate_rules(in, kCfg).status().code, "rtk_fixed");
  in.control_points = {{"CP1", 44.6, 90.28}};
  EXPECT_EQ(evaluate_rules(in, kCfg).status().code, "rtk_fixed");
  in.control_points = {};
  EXPECT_EQ(evaluate_rules(in, kCfg).status().code, "rtk_fixed");
  in.control_points = {{"CP1", 44.5 + 0.5 / 111000.0, 90.28}};
  in.sol = sol(Quality::FLOAT, 38, 25.0);
  EXPECT_FALSE(has_code(evaluate_rules(in, kCfg), "abs_ref_shift"));
}

TEST(Rules, AbsRefJudgesOnlyTheNearestControlPoint) {
  auto in = inp();
  // 最近点在 0.05 m(未超限),另一点 0.5 m(超限但不是最近点)→ 不报
  in.control_points = {{"FAR", 44.5 + 0.5 / 111000.0, 90.28}, {"NEAR", 44.5 + 0.05 / 111000.0, 90.28}};
  EXPECT_FALSE(has_code(evaluate_rules(in, kCfg), "abs_ref_shift"));
}

TEST(Rules, AmbiguityOnFloatLowRatio) {
  auto in = inp();
  in.sol = sol(Quality::FLOAT, 38, 1.8);
  const auto r = evaluate_rules(in, kCfg);
  EXPECT_EQ(r.status().code, "ambiguity");
  EXPECT_EQ(r.status().message, "模糊度无法固定（ratio=1.8）——遮挡过渡区常见");
}

TEST(Rules, AmbiguityNeedsARatio) {
  auto in = inp();
  in.sol = sol(Quality::FLOAT, 38, std::nullopt);
  const auto r = evaluate_rules(in, kCfg);
  EXPECT_FALSE(has_code(r, "ambiguity"));
  EXPECT_EQ(r.status().code, "not_fixed");
  EXPECT_EQ(r.status().message, "非固定解（FLOAT）");
}

TEST(Rules, CycleSlips) {
  auto in = inp();
  in.slip_count_30s = 9;
  EXPECT_EQ(evaluate_rules(in, kCfg).status().code, "cycle_slip");
  in.slip_count_30s = 5;
  EXPECT_EQ(evaluate_rules(in, kCfg).status().code, "rtk_fixed") << "等于门限不报";
}

TEST(Rules, DivergenceNeedsHold) {
  auto in = inp();
  in.divergence_m = 0.5;
  in.divergence_since = 998.0;   // 已持续 2 s < 5 s
  EXPECT_EQ(evaluate_rules(in, kCfg).status().code, "rtk_fixed");
  in.divergence_since = 990.0;   // 10 s
  const auto r = evaluate_rules(in, kCfg);
  EXPECT_EQ(r.status().code, "device_divergence");
  EXPECT_EQ(r.status().message, "610 输出与独立解算偏差 0.50m——疑似 610 融合问题");
}

TEST(Rules, DivergenceUsesTheSuppliedThreshold) {
  auto in = inp();
  in.divergence_m = 0.5;
  in.divergence_since = 990.0;
  in.divergence_threshold_m = 0.6;
  EXPECT_FALSE(has_code(evaluate_rules(in, kCfg), "device_divergence"));
}

TEST(Rules, NoDataAtAllShortCircuits) {
  auto in = inp();
  in.sol.reset();
  in.corr_last_t.reset();
  in.corr_age.reset();
  in.base_offset_m = 0.8;   // 即便有基站偏移,也只报 no_data
  const auto r = evaluate_rules(in, kCfg);
  ASSERT_EQ(r.verdicts.size(), 1u);
  EXPECT_EQ(r.status().code, "no_data");
  EXPECT_EQ(r.status().level, Level::Warning);
  EXPECT_EQ(r.status().message, "无数据——检查采集链路与设备连接");
}

TEST(Rules, SolverDeadIsNotReportedFixed) {
  auto in = inp();
  in.sol.reset();
  const auto r = evaluate_rules(in, kCfg);
  EXPECT_EQ(r.status().code, "no_solution");
  EXPECT_EQ(r.status().level, Level::Warning);
  EXPECT_EQ(r.status().message, "独立解算无输出——rtkrcv 未运行或未收敛");
}

TEST(Rules, SolverDisabledIsInfo) {
  auto in = inp();
  in.sol.reset();
  in.solver_enabled = false;
  const auto r = evaluate_rules(in, kCfg);
  EXPECT_EQ(r.status().code, "no_solution");
  EXPECT_EQ(r.status().level, Level::Info);
  EXPECT_EQ(r.status().message, "独立解算未启用");
}

TEST(Rules, VerdictsAreInPriorityOrderWhenManyFire) {
  auto in = inp();
  in.corr_last_t = 990.0;
  in.base_offset_m = 0.8;
  in.sol = sol(Quality::FLOAT, 4, 1.0);
  in.slip_count_30s = 9;
  std::vector<std::string> codes;
  for (const auto& v : evaluate_rules(in, kCfg).verdicts) codes.push_back(v.code);
  EXPECT_EQ(codes, (std::vector<std::string>{"corr_outage", "base_shift", "low_sats", "ambiguity",
                                             "cycle_slip", "not_fixed"}));
}

TEST(Rules, LevelHelpers) {
  EXPECT_STREQ(level_name(Level::Serious), "serious");
  EXPECT_TRUE(level_opens_event(Level::Warning));
  EXPECT_FALSE(level_opens_event(Level::Info));
  EXPECT_FALSE(level_opens_event(Level::Ok));
  EXPECT_STREQ(quality_name(Quality::DGPS), "DGPS");
}

TEST(DiagnosisConfigValidation, DefaultsAreValidAndMatchRtkMonitor) {
  const DiagnosisConfig c;
  EXPECT_NO_THROW(validate_diagnosis_config(c));
  EXPECT_DOUBLE_EQ(c.corr_gap_s, 3.0);
  EXPECT_DOUBLE_EQ(c.age_max_s, 10.0);
  EXPECT_DOUBLE_EQ(c.base_shift_m, 0.1);
  EXPECT_EQ(c.min_sats, 6);
  EXPECT_DOUBLE_EQ(c.close_hysteresis_s, 10.0);
  EXPECT_DOUBLE_EQ(c.abs_ref_radius_m, 3.0);
}

TEST(DiagnosisConfigValidation, RejectsNonsenseWithTheFieldName) {
  const auto expect_rejected = [](DiagnosisConfig c, const std::string& field) {
    try {
      validate_diagnosis_config(c);
      ADD_FAILURE() << field << " 没有被拒绝";
    } catch (const std::invalid_argument& e) {
      EXPECT_EQ(std::string(e.what()).rfind(field, 0), 0u) << e.what();
    }
  };
  DiagnosisConfig c;
  c.corr_gap_s = 0.0;           expect_rejected(c, "corr_gap_s");       c = {};
  c.sol_stale_s = -1.0;         expect_rejected(c, "sol_stale_s");      c = {};
  c.min_sats = -1;              expect_rejected(c, "min_sats");         c = {};
  c.low_el_deg = 91.0;          expect_rejected(c, "low_el_deg");       c = {};
  c.abs_ref_radius_m = 0.1;     expect_rejected(c, "abs_ref_radius_m"); c = {};   // 必须大于 abs_ref_max_m
  c.divergence_min_samples = 1; expect_rejected(c, "divergence_min_samples");
}
