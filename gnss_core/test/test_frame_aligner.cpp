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

TEST(FrameAligner, FrozenAfterInitialization) {
  gnss_core::FrameAligner al(10.0);
  al.add({0,0,0},{0,0,0});
  al.add({20,0,0},{20,0,0});        // 恒等变换,基线 20m → 初始化
  ASSERT_TRUE(al.initialized());
  const auto T0 = al.T_world_enu();
  al.add({40,0,0},{0,40,0});        // 与恒等矛盾的点对
  al.add({60,0,0},{0,60,0});
  EXPECT_TRUE(al.initialized());
  EXPECT_TRUE(al.T_world_enu().isApprox(T0));   // 冻结:不受后续点对影响
}

TEST(FrameAligner, NotInitializedWhenEnuStatic) {
  // ENU 静止(RTK 卡死/重复输出)而 est 在走:无法从退化的协方差恢复 yaw,不得初始化
  gnss_core::FrameAligner fa(5.0);
  const Eigen::Vector3d enu_fixed(100.0, 200.0, 5.0);
  for (int i = 0; i < 20; ++i)
    fa.add(Eigen::Vector3d(i * 1.0, 0.0, 0.0), enu_fixed);
  EXPECT_FALSE(fa.initialized());
  EXPECT_TRUE(fa.T_world_enu().isApprox(Eigen::Isometry3d::Identity()));
}
