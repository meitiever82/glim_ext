#include <gtest/gtest.h>
#include "gnss_core/types.hpp"
using namespace gnss_core;

TEST(Types, QualityEnumValues) {
  EXPECT_EQ(static_cast<uint8_t>(Quality::NONE), 0);
  EXPECT_EQ(static_cast<uint8_t>(Quality::FIXED), 4);
}

TEST(Types, RtkFixSampleDefaults) {
  RtkFixSample s;
  EXPECT_EQ(s.quality, Quality::NONE);
  EXPECT_FALSE(s.heading_valid);
  EXPECT_EQ(s.gnss_time, 0.0);
  EXPECT_EQ(s.header_stamp, 0.0);
}

TEST(Types, EffectiveStampPrefersGnssTime) {
  EXPECT_DOUBLE_EQ(effective_stamp(100.0, 99.95, StampSource::GnssTime, 0.0), 99.95);
}

TEST(Types, EffectiveStampFallsBackToHeaderWhenGnssTimeZero) {
  EXPECT_DOUBLE_EQ(effective_stamp(100.0, 0.0, StampSource::GnssTime, 0.0), 100.0);
}

TEST(Types, EffectiveStampHeaderSourceIgnoresGnssTime) {
  EXPECT_DOUBLE_EQ(effective_stamp(100.0, 99.95, StampSource::Header, 0.0), 100.0);
}

TEST(Types, EffectiveStampAppliesOffset) {
  EXPECT_DOUBLE_EQ(effective_stamp(100.0, 0.0, StampSource::Header, -0.03), 99.97);
}
