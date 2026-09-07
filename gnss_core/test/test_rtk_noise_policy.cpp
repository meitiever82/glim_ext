#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
#include "gnss_core/rtk_noise_policy.hpp"
using namespace gnss_core;

// GTSAM 4.2 的 SharedNoiseModel 是 boost::shared_ptr,4.3 是 std::shared_ptr;
// 用裸指针 dynamic_cast 做类型剥离,两版本通用。
template <class T, class P>
static const T* as(const P& p) { return dynamic_cast<const T*>(p.get()); }

static RtkFixSample good() {
  RtkFixSample s;
  s.quality = Quality::FIXED; s.diff_age = 1.0; s.sats_used = 20;
  s.sigma_enu = {0.01, 0.01, 0.02};
  return s;
}

TEST(NoisePolicy, RejectsBelowMinQuality) {
  RtkNoisePolicy p{NoisePolicyConfig{}};
  auto s = good(); s.quality = Quality::SINGLE;
  EXPECT_EQ(p.evaluate(s), nullptr);
}

TEST(NoisePolicy, RejectsStaleDiffAge) {
  RtkNoisePolicy p{NoisePolicyConfig{}};
  auto s = good(); s.diff_age = 30.0;
  EXPECT_EQ(p.evaluate(s), nullptr);
}

TEST(NoisePolicy, RejectsFewSats) {
  RtkNoisePolicy p{NoisePolicyConfig{}};
  auto s = good(); s.sats_used = 4;
  EXPECT_EQ(p.evaluate(s), nullptr);
}

TEST(NoisePolicy, AcceptsFixedAndAppliesFloor) {
  NoisePolicyConfig cfg; cfg.sigma_floor = {0.02, 0.02, 0.05};   // 标定后的 floor;vertical_scale=3
  RtkNoisePolicy p{cfg};
  auto s = good(); s.sigma_enu = {0.001, 0.001, 0.001};  // 板卡报 1mm
  auto m = p.evaluate(s);
  ASSERT_NE(m, nullptr);
  const auto* robust = as<gtsam::noiseModel::Robust>(m);
  ASSERT_NE(robust, nullptr);
  const auto* diag = as<gtsam::noiseModel::Diagonal>(robust->noise());
  ASSERT_NE(diag, nullptr);
  EXPECT_NEAR(diag->sigmas()(0), 0.02, 1e-9);         // floor(E)
  EXPECT_NEAR(diag->sigmas()(2), 0.05 * 3.0, 1e-9);   // floor(U)*vertical_scale
}

TEST(NoisePolicy, FloatScalesSigmaFiveX) {
  NoisePolicyConfig cfg; cfg.min_quality = 3; cfg.sigma_floor = {0.02, 0.02, 0.05};
  RtkNoisePolicy p{cfg};
  auto s = good(); s.quality = Quality::FLOAT; s.sigma_enu = {0.1, 0.1, 0.1};
  auto m = p.evaluate(s);
  ASSERT_NE(m, nullptr);
  const auto* robust = as<gtsam::noiseModel::Robust>(m);
  ASSERT_NE(robust, nullptr);
  const auto* diag = as<gtsam::noiseModel::Diagonal>(robust->noise());
  ASSERT_NE(diag, nullptr);
  EXPECT_NEAR(diag->sigmas()(0), 0.1 * 5.0, 1e-9);     // FLOAT scale=5
}

TEST(NoisePolicy, DefaultFloorIsOneMetreWhileLeverArmUncalibrated) {
  RtkNoisePolicy p{NoisePolicyConfig{}};    // 默认 sigma_floor = {1,1,1}
  auto s = good(); s.sigma_enu = {0.01, 0.01, 0.02};
  auto m = p.evaluate(s);
  ASSERT_NE(m, nullptr);
  const auto* robust = as<gtsam::noiseModel::Robust>(m);
  ASSERT_NE(robust, nullptr);
  const auto* diag = as<gtsam::noiseModel::Diagonal>(robust->noise());
  ASSERT_NE(diag, nullptr);
  EXPECT_NEAR(diag->sigmas()(0), 1.0, 1e-9);
  EXPECT_NEAR(diag->sigmas()(1), 1.0, 1e-9);
  EXPECT_NEAR(diag->sigmas()(2), 1.0 * 3.0, 1e-9);
}

TEST(NoisePolicy, RejectsInvalidConfig) {
  NoisePolicyConfig cfg; cfg.min_quality = 7;
  EXPECT_THROW(RtkNoisePolicy{cfg}, std::invalid_argument);
  NoisePolicyConfig cfg_neg; cfg_neg.min_quality = -1;
  EXPECT_THROW(RtkNoisePolicy{cfg_neg}, std::invalid_argument);
  NoisePolicyConfig cfg2; cfg2.robust_kernel = "tukey";
  EXPECT_THROW(RtkNoisePolicy{cfg2}, std::invalid_argument);
  NoisePolicyConfig cfg3; cfg3.sigma_floor = {0.02, 0.0, 0.05};
  EXPECT_THROW(RtkNoisePolicy{cfg3}, std::invalid_argument);
  NoisePolicyConfig cfg4; cfg4.sigma_floor = {0.02, 0.02, -0.05};
  EXPECT_THROW(RtkNoisePolicy{cfg4}, std::invalid_argument);
  // 合法边界值不抛
  NoisePolicyConfig ok; ok.min_quality = 0; ok.robust_kernel = "none";
  EXPECT_NO_THROW(RtkNoisePolicy{ok});
  NoisePolicyConfig ok2; ok2.min_quality = 4; ok2.robust_kernel = "cauchy";
  EXPECT_NO_THROW(RtkNoisePolicy{ok2});
}

TEST(NoisePolicy, OutOfRangeQualityReturnsNullptrNotThrow) {
  NoisePolicyConfig cfg; cfg.min_quality = 0;
  RtkNoisePolicy p{cfg};
  auto s = good(); s.quality = static_cast<Quality>(7);
  gtsam::SharedNoiseModel m;
  EXPECT_NO_THROW(m = p.evaluate(s));
  EXPECT_EQ(m, nullptr);
}

TEST(NoisePolicy, NonFiniteSigmaReturnsNullptr) {
  RtkNoisePolicy p{NoisePolicyConfig{}};
  auto s = good(); s.sigma_enu = {std::numeric_limits<double>::quiet_NaN(), 0.01, 0.02};
  EXPECT_EQ(p.evaluate(s), nullptr);
  auto s2 = good(); s2.sigma_enu = {0.01, std::numeric_limits<double>::infinity(), 0.02};
  EXPECT_EQ(p.evaluate(s2), nullptr);
}
