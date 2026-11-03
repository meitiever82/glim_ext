#include <gtest/gtest.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/config.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>

#include "gnss_core/relative_translation_factor.hpp"

using namespace gtsam;
using gnss_core::RelativeTranslationFactor;

// 与 test_antenna_prior_factor 相同的版本分支:4.2 的 Jacobian 形参是
// boost::optional<Matrix&>,4.3 是 Matrix*。
static Vector eval_with_jacobian(
  const RelativeTranslationFactor& f,
  const Pose3& X1,
  const Pose3& X2,
  Matrix& H1,
  Matrix& H2) {
#if (GTSAM_VERSION_MAJOR > 4) || (GTSAM_VERSION_MAJOR == 4 && GTSAM_VERSION_MINOR >= 3)
  return f.evaluateError(X1, X2, &H1, &H2);
#else
  return f.evaluateError(X1, X2, H1, H2);
#endif
}

TEST(RelativeTranslationFactor, ZeroResidualWhenDisplacementMatches) {
  const Key k1 = Symbol('x', 0).key();
  const Key k2 = Symbol('x', 1).key();
  auto model = noiseModel::Isotropic::Sigma(2, 0.05);
  // 姿态非单位:残差只看世界系平移差,与旋转无关
  const Pose3 X1(Rot3::Rz(0.7), Point3(0.0, 0.0, 0.0));
  const Pose3 X2(Rot3::Rz(0.7), Point3(2.5, 0.0, 0.0));
  const RelativeTranslationFactor f(k1, k2, Eigen::Vector2d(2.5, 0.0), model);

  Vector e = f.evaluateError(X1, X2);
  ASSERT_EQ(e.size(), 2);
  EXPECT_LT(e.norm(), 1e-12);
}

TEST(RelativeTranslationFactor, ResidualIsPredictedMinusMeasured) {
  auto model = noiseModel::Isotropic::Sigma(2, 0.05);
  const Pose3 X1(Rot3::Identity(), Point3(1.0, 2.0, 3.0));
  const Pose3 X2(Rot3::Identity(), Point3(4.0, 5.0, 3.5));
  const RelativeTranslationFactor f(Symbol('x', 0).key(), Symbol('x', 1).key(),
                                    Eigen::Vector2d(1.0, 0.0), model);

  Vector e = f.evaluateError(X1, X2);
  EXPECT_TRUE(assert_equal(Vector(Eigen::Vector2d(2.0, 3.0)), e, 1e-12));
}

TEST(RelativeTranslationFactor, JacobianMatchesNumericalWithRotation) {
  // 非单位 yaw 是关键:平移扰动在 Pose3 局部系,Jacobian 必须含 R,
  // 仅 ±I 只在两个姿态都为单位旋转时才成立。
  const Pose3 X1(Rot3::RzRyRx(0.2, -0.1, 0.7), Point3(3.0, 1.0, 2.0));
  const Pose3 X2(Rot3::RzRyRx(-0.1, 0.05, -0.2), Point3(6.0, 3.0, 2.2));
  auto model = noiseModel::Isotropic::Sigma(2, 0.05);
  const RelativeTranslationFactor f(Symbol('x', 0).key(), Symbol('x', 1).key(),
                                    Eigen::Vector2d(3.0, 2.0), model);

  Matrix H1, H2;
  eval_with_jacobian(f, X1, X2, H1, H2);
  ASSERT_EQ(H1.rows(), 2);
  ASSERT_EQ(H1.cols(), 6);
  ASSERT_EQ(H2.rows(), 2);
  ASSERT_EQ(H2.cols(), 6);

  Matrix H1num = numericalDerivative21<Vector, Pose3, Pose3>(
    [&](const Pose3& a, const Pose3& b) { return f.evaluateError(a, b); }, X1, X2);
  Matrix H2num = numericalDerivative22<Vector, Pose3, Pose3>(
    [&](const Pose3& a, const Pose3& b) { return f.evaluateError(a, b); }, X1, X2);
  EXPECT_TRUE(assert_equal(H1num, H1, 1e-6));
  EXPECT_TRUE(assert_equal(H2num, H2, 1e-6));

  // 旋转块为零(因子不约束旋转);平移块位于 tangent 的列 3..5
  EXPECT_LT((H1.block<2, 3>(0, 0).norm()), 1e-12);
  EXPECT_LT((H2.block<2, 3>(0, 0).norm()), 1e-12);
}

TEST(RelativeTranslationFactor, ErrorMatchesQuadraticForm) {
  const Key k1 = Symbol('x', 0).key();
  const Key k2 = Symbol('x', 1).key();
  const double sigma = 0.05;
  auto model = noiseModel::Isotropic::Sigma(2, sigma);
  const RelativeTranslationFactor f(k1, k2, Eigen::Vector2d(1.0, 0.0), model);

  Values v;
  v.insert(k1, Pose3(Rot3::Identity(), Point3(0.0, 0.0, 0.0)));
  v.insert(k2, Pose3(Rot3::Identity(), Point3(2.0, 1.0, 0.0)));

  const Vector r = Eigen::Vector2d(1.0, 1.0);
  EXPECT_NEAR(f.error(v), 0.5 * r.squaredNorm() / (sigma * sigma), 1e-9);
  EXPECT_EQ(f.keys().size(), 2u);
}

TEST(RelativeTranslationFactor, CreateKeepsKeysAndMeasurement) {
  auto model = noiseModel::Isotropic::Sigma(2, 0.1);
  gtsam::NonlinearFactor::shared_ptr f = RelativeTranslationFactor::create(
    Symbol('x', 0).key(), Symbol('x', 1).key(), Eigen::Vector2d(1.0, 2.0), model);
  ASSERT_NE(f, nullptr);
  const auto* rtf = dynamic_cast<const RelativeTranslationFactor*>(f.get());
  ASSERT_NE(rtf, nullptr);
  EXPECT_TRUE(rtf->measured().isApprox(Eigen::Vector2d(1.0, 2.0)));
  ASSERT_EQ(f->keys().size(), 2u);
  EXPECT_EQ(f->keys()[0], Symbol('x', 0).key());
  EXPECT_EQ(f->keys()[1], Symbol('x', 1).key());
}
