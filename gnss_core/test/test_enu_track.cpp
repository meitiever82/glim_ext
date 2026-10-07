#include <gtest/gtest.h>
#include <sstream>
#include <vector>
#include "gnss_core/enu_track.hpp"
#include "gnss_core/geodetic.hpp"

using gnss_core::EnuTrack;
using gnss_core::EnuTrackOptions;
using gnss_core::make_enu_track;
using gnss_core::PosRecord;
using gnss_core::Quality;

namespace {

// 红沙泉段 A 的 ENU 原点(gt/enu_origin.txt)
constexpr double kLat0 = 44.470257133;
constexpr double kLon0 = 90.294514984;
constexpr double kAlt0 = 610.6395;

PosRecord rec(double stamp, double lat, double lon, double h, int q) {
  PosRecord r;
  r.stamp = stamp;
  r.lat = lat;
  r.lon = lon;
  r.height = h;
  r.q = q;
  r.ns = 25;
  r.sdne = Eigen::Vector3d(0.11, 0.07, 0.19);  // sdn, sde, sdu —— 注意 .pos 的列序
  return r;
}

}  // namespace

TEST(EnuTrack, ExplicitOriginMapsToZero) {
  const std::vector<PosRecord> recs = {rec(100.0, kLat0, kLon0, kAlt0, 1)};
  EnuTrackOptions opt;
  opt.has_origin = true;
  opt.origin_lla = Eigen::Vector3d(kLat0, kLon0, kAlt0);
  const EnuTrack t = make_enu_track(recs, opt);
  ASSERT_EQ(t.samples.size(), 1u);
  EXPECT_LT(t.samples[0].enu.norm(), 1e-6);
  EXPECT_EQ(t.samples[0].quality, Quality::FIXED);
}

TEST(EnuTrack, DefaultOriginIsFirstQualifyingRecord) {
  // 第一条是 SINGLE(Q=5),被 min_quality=FLOAT 挡掉 → 原点应取第二条
  const std::vector<PosRecord> recs = {rec(100.0, kLat0 + 1e-3, kLon0, kAlt0, 5),
                                       rec(101.0, kLat0, kLon0, kAlt0, 1)};
  const EnuTrack t = make_enu_track(recs, EnuTrackOptions{});
  ASSERT_EQ(t.samples.size(), 1u);
  EXPECT_NEAR(t.origin_lla.x(), kLat0, 1e-12);
  EXPECT_LT(t.samples[0].enu.norm(), 1e-6);
}

TEST(EnuTrack, QualityGateDropsRecordsBelowThreshold) {
  const std::vector<PosRecord> recs = {rec(100.0, kLat0, kLon0, kAlt0, 1),   // FIXED
                                       rec(101.0, kLat0, kLon0, kAlt0, 2),   // FLOAT
                                       rec(102.0, kLat0, kLon0, kAlt0, 4),   // DGPS
                                       rec(103.0, kLat0, kLon0, kAlt0, 0)};  // NONE
  EnuTrackOptions opt;
  opt.has_origin = true;
  opt.origin_lla = Eigen::Vector3d(kLat0, kLon0, kAlt0);
  opt.min_quality = Quality::FLOAT;
  const EnuTrack t = make_enu_track(recs, opt);
  ASSERT_EQ(t.samples.size(), 2u);
  EXPECT_EQ(t.samples[0].quality, Quality::FIXED);
  EXPECT_EQ(t.samples[1].quality, Quality::FLOAT);
}

TEST(EnuTrack, SigmaIsReorderedToEnu) {
  // PosRecord::sdne 的列序是 (sdn, sde, sdu);EnuSample::sigma_enu 必须是 (sde, sdn, sdu)
  const std::vector<PosRecord> recs = {rec(100.0, kLat0, kLon0, kAlt0, 1)};
  EnuTrackOptions opt;
  opt.has_origin = true;
  opt.origin_lla = Eigen::Vector3d(kLat0, kLon0, kAlt0);
  const EnuTrack t = make_enu_track(recs, opt);
  ASSERT_EQ(t.samples.size(), 1u);
  EXPECT_NEAR(t.samples[0].sigma_enu.x(), 0.07, 1e-12);  // sde
  EXPECT_NEAR(t.samples[0].sigma_enu.y(), 0.11, 1e-12);  // sdn
  EXPECT_NEAR(t.samples[0].sigma_enu.z(), 0.19, 1e-12);  // sdu
}

TEST(EnuTrack, MatchesLlaToEnuDirectly) {
  // 与已测过的 LlaToEnu 交叉验证:同一个点,两条路径必须毫米内一致
  const double lat = kLat0 + 2e-4, lon = kLon0 - 3e-4, h = kAlt0 + 1.5;
  const std::vector<PosRecord> recs = {rec(100.0, lat, lon, h, 1)};
  EnuTrackOptions opt;
  opt.has_origin = true;
  opt.origin_lla = Eigen::Vector3d(kLat0, kLon0, kAlt0);
  const EnuTrack t = make_enu_track(recs, opt);
  const gnss_core::LlaToEnu conv(kLat0, kLon0, kAlt0);
  const Eigen::Vector3d expected = conv.forward(lat, lon, h);
  ASSERT_EQ(t.samples.size(), 1u);
  EXPECT_LT((t.samples[0].enu - expected).norm(), 1e-3);
}

TEST(EnuTrack, EmptyInputYieldsEmptyTrack) {
  const EnuTrack t = make_enu_track({}, EnuTrackOptions{});
  EXPECT_TRUE(t.samples.empty());
}

TEST(EnuTrack, NoQualifyingRecordWithoutExplicitOriginYieldsEmptyTrack) {
  const std::vector<PosRecord> recs = {rec(100.0, kLat0, kLon0, kAlt0, 5)};  // SINGLE < FLOAT
  const EnuTrack t = make_enu_track(recs, EnuTrackOptions{});
  EXPECT_TRUE(t.samples.empty());
}

TEST(EnuTrack, WriteReadRoundTrip) {
  const std::vector<PosRecord> recs = {rec(1789462172.0, kLat0 + 1e-4, kLon0, kAlt0, 1),
                                       rec(1789462173.0, kLat0, kLon0 + 1e-4, kAlt0 + 0.5, 2)};
  EnuTrackOptions opt;
  opt.has_origin = true;
  opt.origin_lla = Eigen::Vector3d(kLat0, kLon0, kAlt0);
  const EnuTrack t = make_enu_track(recs, opt);

  std::ostringstream os;
  gnss_core::write_enu_track(os, t);
  std::istringstream is(os.str());
  const EnuTrack back = gnss_core::read_enu_track(is);

  ASSERT_EQ(back.samples.size(), t.samples.size());
  EXPECT_NEAR(back.origin_lla.x(), t.origin_lla.x(), 1e-7);
  EXPECT_NEAR(back.origin_lla.y(), t.origin_lla.y(), 1e-7);
  for (size_t i = 0; i < t.samples.size(); ++i) {
    EXPECT_NEAR(back.samples[i].stamp, t.samples[i].stamp, 1e-3);
    EXPECT_LT((back.samples[i].enu - t.samples[i].enu).norm(), 1e-3);
    EXPECT_LT((back.samples[i].sigma_enu - t.samples[i].sigma_enu).norm(), 1e-3);
    EXPECT_EQ(back.samples[i].quality, t.samples[i].quality);
  }
}

TEST(EnuTrack, ReadSkipsCommentsAndMalformedLines) {
  const std::string text =
    "# origin_lla 44.47025713 90.29451498 610.640\n"
    "# t e n u sde sdn sdu q\n"
    "1789462172.000 1.0 2.0 3.0 0.1 0.2 0.3 4\n"
    "garbage line\n"
    "1789462173.000 4.0 5.0\n"            // 列数不足
    "1789462174.000 7.0 8.0 9.0 0.1 0.2 0.3 3\n";
  std::istringstream is(text);
  const EnuTrack t = gnss_core::read_enu_track(is);
  ASSERT_EQ(t.samples.size(), 2u);
  EXPECT_NEAR(t.samples[0].enu.x(), 1.0, 1e-9);
  EXPECT_EQ(t.samples[1].quality, Quality::FLOAT);
}
