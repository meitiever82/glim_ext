#include <gtest/gtest.h>
#include <string>
#include "gnss_core/rtkstat.hpp"
using namespace gnss_core;

namespace {
// RTKLIB 列序:$SAT,week,tow,sat,frq,az,el,resp,resc,vsat,snr,fix,slip,lock,outc,slipc,rejc
std::string sat_line(const char* sat, double tow, int frq, double el, double resp,
                     double snr, int vsat, int slipc, int rejc) {
  return "$SAT," + std::to_string(2380) + "," + std::to_string(tow) + "," + sat + "," +
         std::to_string(frq) + ",123.4," + std::to_string(el) + "," + std::to_string(resp) +
         ",0.001," + std::to_string(vsat) + "," + std::to_string(snr) + ",1,0,100,0," +
         std::to_string(slipc) + "," + std::to_string(rejc);
}
}  // namespace

// ---------- parse_sat_line ----------

TEST(ParseSatLine, ParsesAllConsumedFields) {
  SatStat s;
  ASSERT_TRUE(parse_sat_line(sat_line("G05", 302400.0, 1, 42.5, -0.35, 44.0, 1, 7, 2), s));
  EXPECT_NEAR(s.tow, 302400.0, 1e-6);
  EXPECT_EQ(s.sat, "G05");
  EXPECT_NEAR(s.el, 42.5, 1e-6);
  EXPECT_NEAR(s.resp, -0.35, 1e-6);
  EXPECT_NEAR(s.snr, 44.0, 1e-6);
  EXPECT_TRUE(s.valid);
  EXPECT_EQ(s.slipc, 7);
  EXPECT_EQ(s.rejc, 2);
}

TEST(ParseSatLine, VsatZeroMeansNotUsedInSolution) {
  SatStat s;
  ASSERT_TRUE(parse_sat_line(sat_line("G07", 302400.0, 1, 10.0, 0.1, 30.0, 0, 0, 0), s));
  EXPECT_FALSE(s.valid);
}

TEST(ParseSatLine, RejectsNonSatLine) {
  SatStat s;
  EXPECT_FALSE(parse_sat_line("$POS,2380,302400.0,1,2.3,4.5", s));
  EXPECT_FALSE(parse_sat_line("", s));
}

TEST(ParseSatLine, RejectsTruncatedLine) {
  SatStat s;
  EXPECT_FALSE(parse_sat_line("$SAT,2380,302400.0,G05,1,123.4", s));
}

TEST(ParseSatLine, RejectsNonNumericField) {
  SatStat s;
  EXPECT_FALSE(parse_sat_line("$SAT,2380,BADTOW,G05,1,123.4,42.5,-0.35,0.001,1,44.0,1,0,100,0,7,2", s));
}

TEST(ParseSatLine, LeavesOutputUntouchedOnFailure) {
  SatStat s;
  s.sat = "PRESET";
  EXPECT_FALSE(parse_sat_line("not a sat line", s));
  EXPECT_EQ(s.sat, "PRESET");
}

// ---------- StatEpochAccumulator ----------

TEST(StatEpochAccumulator, CollectsSatellitesOfOneEpoch) {
  StatEpochAccumulator acc;
  acc.feed(sat_line("G05", 302400.0, 1, 42.5, -0.35, 44.0, 1, 0, 0));
  acc.feed(sat_line("G07", 302400.0, 1, 31.0, 0.12, 41.0, 1, 0, 0));
  ASSERT_EQ(acc.epoch().size(), 2u);
  EXPECT_EQ(acc.epoch()[0].sat, "G05");
  EXPECT_EQ(acc.epoch()[1].sat, "G07");
  EXPECT_NEAR(acc.epoch_tow(), 302400.0, 1e-6);
}

TEST(StatEpochAccumulator, KeepsOnlyFirstFrequencyPerSatellite) {
  StatEpochAccumulator acc;
  acc.feed(sat_line("G05", 302400.0, 1, 42.5, -0.35, 44.0, 1, 0, 0));
  acc.feed(sat_line("G05", 302400.0, 2, 42.5, -0.90, 38.0, 1, 0, 0));   // L2,同一颗星
  ASSERT_EQ(acc.epoch().size(), 1u);
  EXPECT_NEAR(acc.epoch()[0].snr, 44.0, 1e-6) << "应保留第一个频点";
}

TEST(StatEpochAccumulator, HoldsCompletedEpochUntilNextOneStartsFilling) {
  StatEpochAccumulator acc;
  acc.feed(sat_line("G05", 302400.0, 1, 42.5, -0.35, 44.0, 1, 0, 0));
  acc.feed(sat_line("G07", 302400.0, 1, 31.0, 0.12, 41.0, 1, 0, 0));
  ASSERT_EQ(acc.epoch().size(), 2u);

  // 新历元的第一颗星到达后,才切换到新历元(消费者不会看到空列表闪一下)
  acc.feed(sat_line("G05", 302401.0, 1, 42.6, -0.30, 44.0, 1, 0, 0));
  ASSERT_EQ(acc.epoch().size(), 1u);
  EXPECT_NEAR(acc.epoch_tow(), 302401.0, 1e-6);
}

TEST(StatEpochAccumulator, IgnoresNonSatLines) {
  StatEpochAccumulator acc;
  acc.feed(sat_line("G05", 302400.0, 1, 42.5, -0.35, 44.0, 1, 0, 0));
  acc.feed("$POS,2380,302400.0,1,2.3,4.5");
  acc.feed("garbage");
  EXPECT_EQ(acc.epoch().size(), 1u);
}

TEST(StatEpochAccumulator, ResetClearsEverything) {
  StatEpochAccumulator acc;
  acc.feed(sat_line("G05", 302400.0, 1, 42.5, -0.35, 44.0, 1, 0, 0));
  ASSERT_FALSE(acc.epoch().empty());
  acc.reset();
  EXPECT_TRUE(acc.epoch().empty());
}

// ---------- SlipWindow ----------

TEST(SlipWindow, FirstObservationOfSatelliteIsBaselineNotASlip) {
  SlipWindow w(30.0);
  w.feed(100.0, "G05", 7);           // 重连后第一次看到,累计值 7 不应被算成 7 次周跳
  EXPECT_EQ(w.count(100.0), 0);
}

TEST(SlipWindow, CountsIncrementsAcrossSatellites) {
  SlipWindow w(30.0);
  w.feed(100.0, "G05", 0);
  w.feed(100.0, "G07", 0);
  w.feed(101.0, "G05", 2);           // +2
  w.feed(101.0, "G07", 1);           // +1
  EXPECT_EQ(w.count(101.0), 3);
}

TEST(SlipWindow, DropsHitsOlderThanWindow) {
  SlipWindow w(30.0);
  w.feed(100.0, "G05", 0);
  w.feed(101.0, "G05", 3);           // +3 发生在 t=101
  EXPECT_EQ(w.count(120.0), 3);
  EXPECT_EQ(w.count(131.5), 0) << "已滑出 30 s 窗口";
}

TEST(SlipWindow, IgnoresNonIncreasingCounter) {
  SlipWindow w(30.0);
  w.feed(100.0, "G05", 5);
  w.feed(101.0, "G05", 5);
  w.feed(102.0, "G05", 3);           // 计数器回绕/重置,不应算作负增量
  EXPECT_EQ(w.count(102.0), 0);
}

// ---------- parse_llh_solution ----------

TEST(ParseLlhSolution, ParsesRtkrcvLlhLineAsUtc) {
  // rtkrcv llh 解流的列序与 .pos 数据行相同,时间列默认 GPST
  const std::string line =
      "2026/09/03 10:23:45.000 44.50123456 90.28765432 617.1230 1 38 "
      "0.0120 0.0110 0.0300 0.0000 0.0000 0.0000 0.80 20.5";
  PosRecord r;
  ASSERT_TRUE(parse_llh_solution(line, r));
  EXPECT_NEAR(r.lat, 44.50123456, 1e-8);
  EXPECT_NEAR(r.lon, 90.28765432, 1e-8);
  EXPECT_NEAR(r.height, 617.123, 1e-6);
  EXPECT_EQ(r.q, 1);
  EXPECT_EQ(r.ns, 38);
  EXPECT_NEAR(r.sdne(0), 0.012, 1e-9);
  EXPECT_NEAR(r.sdne(2), 0.030, 1e-9);
  EXPECT_NEAR(r.age, 0.80, 1e-9);
  EXPECT_NEAR(r.ratio, 20.5, 1e-9);
}

TEST(ParseLlhSolution, GpstHeaderTimeIsConvertedToUtcByLeapSeconds) {
  const std::string line =
      "2026/09/03 10:23:45.000 44.50123456 90.28765432 617.1230 1 38 "
      "0.0120 0.0110 0.0300 0.0000 0.0000 0.0000 0.80 20.5";
  PosRecord gpst, utc;
  PosReadOptions o_gpst;  o_gpst.default_time_system = PosTimeSystem::GPST;
  PosReadOptions o_utc;   o_utc.default_time_system = PosTimeSystem::UTC;
  ASSERT_TRUE(parse_llh_solution(line, gpst, o_gpst));
  ASSERT_TRUE(parse_llh_solution(line, utc, o_utc));
  EXPECT_NEAR(utc.stamp - gpst.stamp, 18.0, 1e-9);
}

TEST(ParseLlhSolution, RejectsCommentAndEmptyLines) {
  PosRecord r;
  EXPECT_FALSE(parse_llh_solution("% program : RTKLIB", r));
  EXPECT_FALSE(parse_llh_solution("", r));
  EXPECT_FALSE(parse_llh_solution("   ", r));
}

TEST(ParseLlhSolution, RejectsTruncatedLine) {
  PosRecord r;
  EXPECT_FALSE(parse_llh_solution("2026/09/03 10:23:45.000 44.5 90.2", r));
}

TEST(ParseLlhSolution, AcceptsLineWithoutOptionalAgeAndRatio) {
  // 只到 sdu 的 10 列必须能读,age/ratio 缺省为 0
  const std::string line =
      "2026/09/03 10:23:45.000 44.50123456 90.28765432 617.1230 2 20 0.1000 0.1100 0.2500";
  PosRecord r;
  ASSERT_TRUE(parse_llh_solution(line, r));
  EXPECT_EQ(r.q, 2);
  EXPECT_NEAR(r.age, 0.0, 1e-12);
  EXPECT_NEAR(r.ratio, 0.0, 1e-12);
}
