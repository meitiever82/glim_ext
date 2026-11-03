#include <gtest/gtest.h>

#include <cmath>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/config.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>

#include "gnss_core/heading_prior_factor.hpp"

using namespace gtsam;
using gnss_core::HeadingPriorFactor;

static Vector eval_with_jacobian(const HeadingPriorFactor& f, const Pose3& X, Matrix& H) {
#if (GTSAM_VERSION_MAJOR > 4) || (GTSAM_VERSION_MAJOR == 4 && GTSAM_VERSION_MINOR >= 3)
  return f.evaluateError(X, &H);
#else
  return f.evaluateError(X, H);
#endif
}

static double wrap_angle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

TEST(HeadingPriorFactor, ZeroResidualWhenAligned) {
  auto model = noiseModel::Isotropic::Sigma(1, 0.01);
  const double yaw = 0.35;
  const HeadingPriorFactor f(Symbol('x', 0).key(), yaw, model);

  const Pose3 X(Rot3::Rz(yaw), Point3(1.0, 2.0, 3.0));
  Vector e = f.evaluateError(X);
  ASSERT_EQ(e.size(), 1);
  EXPECT_NEAR(e(0), 0.0, 1e-12);
}

TEST(HeadingPriorFactor, ResidualIsWrappedYawDifference) {
  auto model = noiseModel::Isotropic::Sigma(1, 0.01);
  const double yaw = 0.1;
  const HeadingPriorFactor f(Symbol('x', 0).key(), yaw, model);

  // 航向 0.4 rad -> 残差 0.3;越过 ±π 也一样(测量 0.1 + 2π 等价)
  const Pose3 X(Rot3::Rz(0.4), Point3(0.0, 0.0, 0.0));
  EXPECT_NEAR(f.evaluateError(X)(0), 0.3, 1e-12);

  const HeadingPriorFactor f_wrapped(Symbol('x', 0).key(), yaw + 2.0 * M_PI, model);
  EXPECT_NEAR(f_wrapped.evaluateError(X)(0), 0.3, 1e-12);

  // 残差始终落在 [-π, π]
  const Pose3 X_opposite(Rot3::Rz(yaw + M_PI), Point3(0.0, 0.0, 0.0));
  EXPECT_NEAR(std::abs(wrap_angle(f.evaluateError(X_opposite)(0))), M_PI, 1e-9);
}

TEST(HeadingPriorFactor, JacobianMatchesNumericalWithRollAndPitch) {
  // 有 roll/pitch 时 body-z 不与 world-z 对齐,不能用平地近似 [0,0,0,0,0,1]。
  auto model = noiseModel::Isotropic::Sigma(1, 0.01);
  const HeadingPriorFactor f(Symbol('x', 0).key(), 0.2, model);

  const Pose3 X(Rot3::RzRyRx(0.15, -0.25, 0.2), Point3(0.0, 0.0, 0.0));
  Matrix H;
  eval_with_jacobian(f, X, H);
  ASSERT_EQ(H.rows(), 1);
  ASSERT_EQ(H.cols(), 6);

  Matrix Hnum = numericalDerivative11<Vector, Pose3>(
    [&](const Pose3& p) { return f.evaluateError(p); }, X);
  EXPECT_TRUE(assert_equal(Hnum, H, 1e-5));

  // 平移块为零(航向不涉及位置)
  EXPECT_LT((H.block<1, 3>(0, 3).norm()), 1e-12);
  // 平地时恰好是绕 z 的 1(tangent 为 (omega, v),绕 z 在列 2)
  const Pose3 flat(Rot3::Rz(0.2), Point3(1.0, 2.0, 3.0));
  Matrix Hflat;
  eval_with_jacobian(f, flat, Hflat);
  Matrix expected = Matrix::Zero(1, 6);
  expected(0, 2) = 1.0;
  EXPECT_TRUE(assert_equal(expected, Hflat, 1e-6));
}

TEST(HeadingPriorFactor, ErrorMatchesQuadraticForm) {
  const Key k = Symbol('x', 0).key();
  const double sigma = 0.01;
  auto model = noiseModel::Isotropic::Sigma(1, sigma);
  const HeadingPriorFactor f(k, 0.0, model);

  Values v;
  v.insert(k, Pose3(Rot3::Rz(0.1), Point3(0.0, 0.0, 0.0)));
  EXPECT_NEAR(f.error(v), 0.5 * 0.1 * 0.1 / (sigma * sigma), 1e-9);
  EXPECT_EQ(f.keys().size(), 1u);
}

TEST(HeadingPriorFactor, CreateKeepsKeyAndMeasurement) {
  auto model = noiseModel::Isotropic::Sigma(1, 0.02);
  gtsam::NonlinearFactor::shared_ptr f = HeadingPriorFactor::create(Symbol('x', 0).key(), 0.5 + 2.0 * M_PI, model);
  ASSERT_NE(f, nullptr);
  const auto* hpf = dynamic_cast<const HeadingPriorFactor*>(f.get());
  ASSERT_NE(hpf, nullptr);
  EXPECT_NEAR(hpf->measured_yaw(), 0.5, 1e-12);
  ASSERT_EQ(f->keys().size(), 1u);
  EXPECT_EQ(f->keys()[0], Symbol('x', 0).key());
}
