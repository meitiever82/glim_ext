#include <gtest/gtest.h>
#include "gnss_core/frame_aligner.hpp"

TEST(FrameAligner, RecoversKnownTransform) {
  // 构造已知:world = Rz(30°) * enu + t
  const double a = 30.0 * M_PI / 180.0;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  R.block<2,2>(0,0) << std::cos(a), -std::sin(a), std::sin(a), std::cos(a);
  Eigen::Vector3d t(10.0, -5.0, 2.0);
  Eigen::Isometry3d T_world_enu_true = Eigen::Isometry3d::Identity();
  T_world_enu_true.linear() = R; T_world_enu_true.translation() = t;

  gnss_core::FrameAligner al(10.0);
  std::vector<Eigen::Vector3d> enus = {
    {0,0,0},{5,0,0},{10,0,0},{15,3,0},{20,6,1}};   // 首尾 > 10m
  for (const auto& e : enus) {
    Eigen::Vector3d world = T_world_enu_true * e;    // submap 位置 = world 真值
    al.add(world, e);
  }
  ASSERT_TRUE(al.initialized());
  const auto T = al.T_world_enu();
  for (const auto& e : enus) {
    EXPECT_LT((T * e - T_world_enu_true * e).norm(), 0.1);
  }
}

TEST(FrameAligner, NotInitializedBelowBaseline) {
  gnss_core::FrameAligner al(10.0);
  al.add({0,0,0},{0,0,0});
  al.add({1,0,0},{1,0,0});          // 基线仅 1m
  EXPECT_FALSE(al.initialized());
}
