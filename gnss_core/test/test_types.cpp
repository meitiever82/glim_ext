#include <gtest/gtest.h>
#include "gnss_core/types.hpp"

TEST(Types, QualityEnumValues) {
  EXPECT_EQ(static_cast<uint8_t>(gnss_core::Quality::NONE), 0);
  EXPECT_EQ(static_cast<uint8_t>(gnss_core::Quality::FIXED), 4);
}

TEST(Types, RtkFixSampleDefaults) {
  gnss_core::RtkFixSample s;
  EXPECT_EQ(s.quality, gnss_core::Quality::NONE);
  EXPECT_FALSE(s.heading_valid);
}
