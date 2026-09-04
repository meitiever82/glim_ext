#include <gtest/gtest.h>
#include "gnss_core/geodetic.hpp"

TEST(Geodetic, OriginIsZero) {
  gnss_core::LlaToEnu conv(44.5, 90.28, 617.0);
  const auto p = conv.forward(44.5, 90.28, 617.0);
  EXPECT_NEAR(p.norm(), 0.0, 1e-6);
}

TEST(Geodetic, OneMetreNorth) {
  gnss_core::LlaToEnu conv(44.5, 90.28, 617.0);
  const double dlat = 1.0 / 111320.0;   // ≈ 1 m 纬度
  const auto p = conv.forward(44.5 + dlat, 90.28, 617.0);
  EXPECT_NEAR(p.x(), 0.0, 0.02);        // E
  EXPECT_NEAR(p.y(), 1.0, 0.02);        // N
  EXPECT_NEAR(p.z(), 0.0, 0.02);        // U
}
