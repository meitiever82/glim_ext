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
