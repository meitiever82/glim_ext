#include <gtest/gtest.h>
#include "gnss_bringup/rtk_fix_mapping.hpp"
using namespace gnss_bringup;
using gnss_core::PosRecord;

namespace {
PosRecord sample() {
  PosRecord r;
  r.stamp = 1789045801.0;
  r.lat = 44.50123456;
  r.lon = 90.28765432;
  r.height = 617.123;
  r.q = 1;
  r.ns = 38;
  r.sdne = Eigen::Vector3d(0.011, 0.022, 0.033);   // sdn, sde, sdu
  r.age = 0.8;
  r.ratio = 20.5;
  return r;
}
}  // namespace

TEST(RtkFixMapping, SigmaIsReorderedFromNeuToEnu) {
  // 这是最容易出的错:RTKLIB 给 N/E/U,RtkFix 要 E/N/U
  const auto m = to_rtk_fix(sample());
  EXPECT_DOUBLE_EQ(m.sigma_enu[0], 0.022) << "E 应取 sde";
  EXPECT_DOUBLE_EQ(m.sigma_enu[1], 0.011) << "N 应取 sdn";
  EXPECT_DOUBLE_EQ(m.sigma_enu[2], 0.033) << "U 应取 sdu";
}

TEST(RtkFixMapping, RtklibQMapsToNormalizedQuality) {
  PosRecord r = sample();
  r.q = 1; EXPECT_EQ(to_rtk_fix(r).quality, gnss_msgs::msg::RtkFix::QUALITY_FIXED);
  r.q = 2; EXPECT_EQ(to_rtk_fix(r).quality, gnss_msgs::msg::RtkFix::QUALITY_FLOAT);
  r.q = 4; EXPECT_EQ(to_rtk_fix(r).quality, gnss_msgs::msg::RtkFix::QUALITY_DGPS);
  r.q = 5; EXPECT_EQ(to_rtk_fix(r).quality, gnss_msgs::msg::RtkFix::QUALITY_SINGLE);
  r.q = 0; EXPECT_EQ(to_rtk_fix(r).quality, gnss_msgs::msg::RtkFix::QUALITY_NONE);
}

TEST(RtkFixMapping, KeepsRawRtklibQForTraceability) {
  PosRecord r = sample();
  r.q = 2;
  EXPECT_EQ(to_rtk_fix(r).raw_status, 2);
}

TEST(RtkFixMapping, GnssTimeComesFromTheSolutionEpoch) {
  const auto m = to_rtk_fix(sample());
  EXPECT_DOUBLE_EQ(m.gnss_time, 1789045801.0);
}

TEST(RtkFixMapping, PositionAndAgeAndSatsAreCopied) {
  const auto m = to_rtk_fix(sample());
  EXPECT_NEAR(m.latitude, 44.50123456, 1e-9);
  EXPECT_NEAR(m.longitude, 90.28765432, 1e-9);
  EXPECT_NEAR(m.altitude, 617.123, 1e-9);
  EXPECT_NEAR(m.diff_age, 0.8f, 1e-6);
  EXPECT_EQ(m.sats_used, 38);
}

TEST(RtkFixMapping, HeadingIsAlwaysInvalidForSingleAntennaSolution) {
  EXPECT_FALSE(to_rtk_fix(sample()).heading_valid);
}

// ---------- LineSplitter ----------
// TcpStream 交付任意切分的字节块,不保证按行到达;这里覆盖“行被切在两个块
// 中间”这个最容易导致静默丢数据/脏数据的场景(brief 明确点名的坑)。

namespace {
std::vector<uint8_t> to_bytes(const std::string& s) {
  return std::vector<uint8_t>(s.begin(), s.end());
}
std::vector<std::string> feed_str(LineSplitter& sp, const std::string& s) {
  const auto b = to_bytes(s);
  return sp.feed(b.data(), b.size());
}
}  // namespace

TEST(LineSplitter, SingleChunkWithTrailingNewlineYieldsAllLines) {
  LineSplitter sp;
  const auto lines = feed_str(sp, "abc\ndef\n");
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_EQ(lines[0], "abc");
  EXPECT_EQ(lines[1], "def");
  EXPECT_EQ(sp.buffered(), 0u);
}

TEST(LineSplitter, PartialLineIsHeldUntilNewlineArrives) {
  LineSplitter sp;
  EXPECT_TRUE(feed_str(sp, "abc").empty());
  EXPECT_EQ(sp.buffered(), 3u);
  const auto lines = feed_str(sp, "def\n");
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "abcdef");
  EXPECT_EQ(sp.buffered(), 0u);
}

TEST(LineSplitter, LineSplitAcrossManyChunksIsNotLost) {
  // 一行被切成三块字节到达,每一块都不含 '\n':必须原样拼回,而不是被
  // 当成三条破损的行。
  LineSplitter sp;
  EXPECT_TRUE(feed_str(sp, "part").empty());
  EXPECT_TRUE(feed_str(sp, "ial-l").empty());
  const auto lines = feed_str(sp, "ine\n");
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "partial-line");
}

TEST(LineSplitter, CarriageReturnBeforeNewlineIsStripped) {
  LineSplitter sp;
  const auto lines = feed_str(sp, "crlf\r\n");
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "crlf");
}

TEST(LineSplitter, OneChunkCanYieldMultipleCompleteLinesPlusAPartial) {
  LineSplitter sp;
  const auto lines = feed_str(sp, "one\ntwo\nthree-partial");
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_EQ(lines[0], "one");
  EXPECT_EQ(lines[1], "two");
  EXPECT_EQ(sp.buffered(), std::string("three-partial").size());
  const auto rest = feed_str(sp, "\n");
  ASSERT_EQ(rest.size(), 1u);
  EXPECT_EQ(rest[0], "three-partial");
}

TEST(LineSplitter, EmptyLinesAreYieldedAsEmptyStrings) {
  LineSplitter sp;
  const auto lines = feed_str(sp, "\n\nafter\n");
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_EQ(lines[0], "");
  EXPECT_EQ(lines[1], "");
  EXPECT_EQ(lines[2], "after");
}
