#include <gtest/gtest.h>
#include <gtsam/config.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Symbol.h>
#include "gnss_core/antenna_prior_factor.hpp"

using namespace gtsam;

// GTSAM 4.2 的 Jacobian 形参是 boost::optional<Matrix&>,4.3 是 Matrix*。
// 测试只在这一处做版本分支,其它代码走两版共有的接口。
static Vector eval_with_jacobian(const gnss_core::AntennaPriorFactor& f, const Pose3& X, Matrix& H) {
#if (GTSAM_VERSION_MAJOR > 4) || (GTSAM_VERSION_MAJOR == 4 && GTSAM_VERSION_MINOR >= 3)
  return f.evaluateError(X, &H);
#else
  return f.evaluateError(X, H);   // 隐式转成 boost::optional<Matrix&>
#endif
}

TEST(AntennaPriorFactor, JacobianMatchesNumerical) {
  Key k = 0;
  Eigen::Vector3d measured(1.0, 2.0, 3.0), lever(0.5, -0.2, 0.1);
  auto model = noiseModel::Isotropic::Sigma(3, 0.05);
  gnss_core::AntennaPriorFactor f(k, measured, lever, model);

  Pose3 X(Rot3::RzRyRx(0.3, -0.1, 0.2), Point3(1.0, 1.0, 1.0));
  Matrix H;
  eval_with_jacobian(f, X, H);
  ASSERT_EQ(H.rows(), 3);
  ASSERT_EQ(H.cols(), 6);
  Matrix Hnum = numericalDerivative11<Vector, Pose3>(
      [&](const Pose3& p) { return f.evaluateError(p); }, X);
  EXPECT_TRUE(assert_equal(Hnum, H, 1e-6));
}

TEST(AntennaPriorFactor, ZeroLeverEqualsTranslationResidual) {
  Key k = 0;
  Eigen::Vector3d measured(1.0, 2.0, 3.0), lever(0, 0, 0);
  auto model = noiseModel::Isotropic::Sigma(3, 0.05);
  gnss_core::AntennaPriorFactor f(k, measured, lever, model);
  // 旋转任意:杆臂为零时旋转不影响残差
  Pose3 X(Rot3::RzRyRx(0.7, 0.2, -0.4), Point3(1.5, 2.5, 3.5));
  Vector e = f.evaluateError(X);
  EXPECT_TRUE(assert_equal(Vector(Point3(0.5, 0.5, 0.5)), e, 1e-9));
}

TEST(AntennaPriorFactor, NonZeroLeverResidualIsRotatedLeverPlusTranslation) {
  Key k = 0;
  Eigen::Vector3d measured(10.0, -3.0, 2.0), lever(2.0, -0.5, 1.2);   // 矿卡量级杆臂
  auto model = noiseModel::Isotropic::Sigma(3, 0.05);
  gnss_core::AntennaPriorFactor f(k, measured, lever, model);

  Rot3 R = Rot3::RzRyRx(0.1, -0.05, 1.3);
  Point3 t(9.0, -2.0, 1.0);
  Pose3 X(R, t);
  Vector e = f.evaluateError(X);
  Vector expected = R.matrix() * lever + Eigen::Vector3d(t) - measured;
  EXPECT_TRUE(assert_equal(expected, e, 1e-12));
  // 与零杆臂不同:残差不等于纯平移差
  EXPECT_GT((e - (Eigen::Vector3d(t) - measured)).norm(), 1.0);
}

TEST(AntennaPriorFactor, CreateReturnsSharedFactor) {
  auto model = noiseModel::Isotropic::Sigma(3, 0.1);
  const Eigen::Vector3d measured(1.0, 2.0, 3.0), lever(0.5, 0.0, 1.0);
  gtsam::NonlinearFactor::shared_ptr f =
      gnss_core::AntennaPriorFactor::create(Symbol('x', 0), measured, lever, model);
  ASSERT_NE(f, nullptr);
  const auto* apf = dynamic_cast<const gnss_core::AntennaPriorFactor*>(f.get());
  ASSERT_NE(apf, nullptr);
  EXPECT_TRUE(apf->measured().isApprox(measured));
  EXPECT_TRUE(apf->body_point().isApprox(lever));
  EXPECT_EQ(f->keys().size(), 1u);
  EXPECT_EQ(f->keys()[0], Symbol('x', 0).key());
}
