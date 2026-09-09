#include <gtest/gtest.h>
#include "gnss_core/rtk_fix_buffer.hpp"
using namespace gnss_core;

static RtkFixSample mk(double t, Quality q, double lat) {
  RtkFixSample s; s.stamp = t; s.quality = q; s.lat = lat; return s;
}

TEST(RtkFixBuffer, InterpolatesMidpoint) {
  RtkFixBuffer b;
  b.push(mk(100.0, Quality::FIXED, 44.0));
  b.push(mk(102.0, Quality::FIXED, 46.0));
  auto r = b.interpolate(101.0);
  ASSERT_TRUE(r.has_value());
  EXPECT_NEAR(r->lat, 45.0, 1e-9);
}

TEST(RtkFixBuffer, QualityTakesWorseOfEnds) {
  RtkFixBuffer b;
  b.push(mk(100.0, Quality::FIXED, 44.0));
  b.push(mk(102.0, Quality::SINGLE, 46.0));
  auto r = b.interpolate(101.0);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->quality, Quality::SINGLE);   // 较差者
}

TEST(RtkFixBuffer, OutOfRangeReturnsNullopt) {
  RtkFixBuffer b;
  b.push(mk(100.0, Quality::FIXED, 44.0));
  b.push(mk(102.0, Quality::FIXED, 46.0));
  EXPECT_FALSE(b.interpolate(105.0).has_value());
  EXPECT_FALSE(b.interpolate(99.0).has_value());
}

TEST(RtkFixBuffer, PruneDropsOld) {
  RtkFixBuffer b;
  b.push(mk(100.0, Quality::FIXED, 44.0));
  b.push(mk(160.0, Quality::FIXED, 46.0));
  b.prune(30.0, 161.0);                     // 丢弃 < 131
  EXPECT_EQ(b.size(), 1u);
}

TEST(RtkFixBuffer, LatestStampTracksBack) {
  RtkFixBuffer b;
  EXPECT_DOUBLE_EQ(b.latest_stamp(), 0.0);          // 空缓冲
  b.push(mk(100.0, Quality::FIXED, 44.0));
  EXPECT_DOUBLE_EQ(b.latest_stamp(), 100.0);
  b.push(mk(102.0, Quality::FIXED, 46.0));
  EXPECT_DOUBLE_EQ(b.latest_stamp(), 102.0);
  b.push(mk(101.0, Quality::FIXED, 45.0));          // 乱序到达:插入中间,不改 latest
  EXPECT_DOUBLE_EQ(b.latest_stamp(), 102.0);
  b.prune(0.5, b.latest_stamp());                   // 以数据时间 prune,只剩 102
  EXPECT_EQ(b.size(), 1u);
  EXPECT_DOUBLE_EQ(b.latest_stamp(), 102.0);
}

TEST(RtkFixBuffer, InterpolatesRawStamps) {
  RtkFixBuffer b;
  auto a = mk(100.0, Quality::FIXED, 44.0); a.header_stamp = 100.02; a.gnss_time = 100.0;
  auto c = mk(102.0, Quality::FIXED, 46.0); c.header_stamp = 102.04; c.gnss_time = 102.0;
  b.push(a); b.push(c);
  auto r = b.interpolate(101.0);
  ASSERT_TRUE(r.has_value());
  EXPECT_DOUBLE_EQ(r->stamp, 101.0);
  EXPECT_NEAR(r->header_stamp, 101.03, 1e-9);       // 线性插值
  EXPECT_NEAR(r->gnss_time, 101.0, 1e-9);
}

TEST(RtkFixBuffer, MaxGapRejectsWideBracket) {
  RtkFixBuffer b;
  b.push(mk(100.0, Quality::FIXED, 44.0));
  b.push(mk(160.0, Quality::FIXED, 46.0));   // 两端相隔 60 s
  EXPECT_FALSE(b.interpolate(130.0, 2.5).has_value());   // 超过 max_gap_s → nullopt
  EXPECT_TRUE(b.interpolate(130.0, 60.0).has_value());   // 恰好等于 gap 仍可插值
  EXPECT_TRUE(b.interpolate(130.0).has_value());         // 不传 → 无限制,保持旧行为
}

TEST(RtkFixBuffer, OldestStampTracksFront) {
  RtkFixBuffer b;
  EXPECT_DOUBLE_EQ(b.oldest_stamp(), 0.0);          // 空缓冲
  b.push(mk(102.0, Quality::FIXED, 46.0));
  b.push(mk(100.0, Quality::FIXED, 44.0));          // 乱序到达:插到最前
  EXPECT_DOUBLE_EQ(b.oldest_stamp(), 100.0);
  b.prune(1.0, b.latest_stamp());                   // 丢弃 < 101 → 只剩 102
  EXPECT_DOUBLE_EQ(b.oldest_stamp(), 102.0);
}

// ---- 航向回绕插值:角度差归一到 (-180,180] 后插值,结果归一到 [0,360) ----
static double interp_heading(double h0, double h1) {
  RtkFixBuffer b;
  auto a = mk(100.0, Quality::FIXED, 44.0); a.heading = h0; a.heading_valid = true;
  auto c = mk(102.0, Quality::FIXED, 44.0); c.heading = h1; c.heading_valid = true;
  b.push(a); b.push(c);
  auto r = b.interpolate(101.0);
  EXPECT_TRUE(r.has_value());
  EXPECT_TRUE(r->heading_valid);
  return r->heading;
}

TEST(RtkFixBuffer, HeadingWrapsAcrossZero) {
  EXPECT_NEAR(interp_heading(359.0, 1.0), 0.0, 1e-9);
}

TEST(RtkFixBuffer, HeadingWrapsAcrossZeroReverse) {
  EXPECT_NEAR(interp_heading(10.0, 350.0), 0.0, 1e-9);
}

TEST(RtkFixBuffer, HeadingPlainMidpoint) {
  EXPECT_NEAR(interp_heading(90.0, 180.0), 135.0, 1e-9);
}

TEST(RtkFixBuffer, HeadingResultInZeroTo360) {
  double h = interp_heading(350.0, 10.0);   // 中点 0
  EXPECT_GE(h, 0.0); EXPECT_LT(h, 360.0);
  EXPECT_NEAR(h, 0.0, 1e-9);
}
