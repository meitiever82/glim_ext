#include <gtest/gtest.h>
#include <vector>
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

TEST(Geodetic, ForwardReverseRoundTrip) {
  gnss_core::LlaToEnu conv(44.5, 90.28, 617.0);
  // 100 m 量级的偏移点:forward → reverse → forward,往返误差 < 1e-6 m
  const std::vector<Eigen::Vector3d> enus = {
    {100.0, 0.0, 0.0}, {0.0, -120.0, 0.0}, {70.0, 80.0, 15.0}, {-90.0, 45.0, -8.0}};
  for (const auto& enu : enus) {
    const Eigen::Vector3d lla = conv.reverse(enu);
    const Eigen::Vector3d back = conv.forward(lla.x(), lla.y(), lla.z());
    EXPECT_LT((back - enu).norm(), 1e-6) << "enu=" << enu.transpose();
  }
  // 原点反解应回到构造参数
  const Eigen::Vector3d o = conv.reverse(Eigen::Vector3d::Zero());
  EXPECT_NEAR(o.x(), 44.5, 1e-9);
  EXPECT_NEAR(o.y(), 90.28, 1e-9);
  EXPECT_NEAR(o.z(), 617.0, 1e-6);
}
