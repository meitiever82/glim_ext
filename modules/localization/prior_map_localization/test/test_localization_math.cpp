#include <gtest/gtest.h>
#include <glim_ext/localization_math.hpp>

TEST(LocalizationMath, TMapOdomIdentityXt) {
  Eigen::Isometry3d T_map_imu = Eigen::Isometry3d::Identity();
  T_map_imu.translation() = Eigen::Vector3d(10, 5, 1);
  Eigen::Isometry3d T = glim::compute_T_map_odom(T_map_imu, Eigen::Isometry3d::Identity());
  EXPECT_TRUE(T.isApprox(T_map_imu, 1e-9));
}

TEST(LocalizationMath, TMapOdomNonIdentityXt) {
  Eigen::Isometry3d X_t = Eigen::Isometry3d::Identity();
  X_t.translation() = Eigen::Vector3d(3, 4, 0);
  X_t.linear() = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Isometry3d T = glim::compute_T_map_odom(X_t, X_t);
  EXPECT_TRUE(T.isApprox(Eigen::Isometry3d::Identity(), 1e-9));
}

TEST(LocalizationMath, ComposeTakesRollPitchFromXt) {
  Eigen::Isometry3d X_t = Eigen::Isometry3d::Identity();
  X_t.linear() = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  Eigen::Isometry3d T = glim::compose_initial_pose(1.0, 2.0, 0.0, 0.5, X_t);
  EXPECT_NEAR(T.translation().z(), 0.5, 1e-9);
  Eigen::Vector3d up = T.linear() * Eigen::Vector3d::UnitZ();
  EXPECT_NEAR(std::acos(up.z()), 0.2, 1e-6);
}

TEST(LocalizationMath, ComposeStripsExistingYawFromXt) {
  // X_t has a yaw of 1.0 rad. compose_initial_pose with the user's yaw=0 must
  // produce a pose whose yaw is 0 (X_t's yaw is stripped, not added).
  Eigen::Isometry3d X_t = Eigen::Isometry3d::Identity();
  X_t.linear() = Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Isometry3d T = glim::compose_initial_pose(0.0, 0.0, 0.0, 0.0, X_t);
  const double out_yaw = std::atan2(T.linear()(1, 0), T.linear()(0, 0));
  EXPECT_NEAR(out_yaw, 0.0, 1e-9);
}
