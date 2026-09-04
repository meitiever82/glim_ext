#include <gtest/gtest.h>
#include "gnss_core/rtk_noise_policy.hpp"
using namespace gnss_core;

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
  NoisePolicyConfig cfg;                    // sigma_floor E/N=0.02, U=0.05; vertical_scale=3
  RtkNoisePolicy p{cfg};
  auto s = good(); s.sigma_enu = {0.001, 0.001, 0.001};  // 板卡报 1mm
  auto m = p.evaluate(s);
  ASSERT_NE(m, nullptr);
  auto robust = std::dynamic_pointer_cast<gtsam::noiseModel::Robust>(m);
  ASSERT_NE(robust, nullptr);
  auto diag = std::dynamic_pointer_cast<const gtsam::noiseModel::Diagonal>(robust->noise());
  ASSERT_NE(diag, nullptr);
  EXPECT_NEAR(diag->sigmas()(0), 0.02, 1e-9);         // floor(E)
  EXPECT_NEAR(diag->sigmas()(2), 0.05 * 3.0, 1e-9);   // floor(U)*vertical_scale
}

TEST(NoisePolicy, FloatScalesSigmaFiveX) {
  NoisePolicyConfig cfg; cfg.min_quality = 3;
  RtkNoisePolicy p{cfg};
  auto s = good(); s.quality = Quality::FLOAT; s.sigma_enu = {0.1, 0.1, 0.1};
  auto m = p.evaluate(s);
  ASSERT_NE(m, nullptr);
  auto robust = std::dynamic_pointer_cast<gtsam::noiseModel::Robust>(m);
  auto diag = std::dynamic_pointer_cast<const gtsam::noiseModel::Diagonal>(robust->noise());
  EXPECT_NEAR(diag->sigmas()(0), 0.1 * 5.0, 1e-9);     // FLOAT scale=5
}
