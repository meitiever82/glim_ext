#include <gtest/gtest.h>
#include "gnss_core/trajectory_compare.hpp"
using namespace gnss_core;

static PosRecord rec(double t, double lat, double lon, double h, int q, double sd) {
  PosRecord r{};
  r.stamp = t; r.lat = lat; r.lon = lon; r.height = h; r.q = q;
  r.sdne = Eigen::Vector3d(sd, sd, sd);
  return r;
}

TEST(TrajCompare, PerfectMatchZeroRmse) {
  std::vector<PosRecord> ref = {rec(100, 44.5, 90.28, 617, 1, 0.01)};
  std::vector<PosRecord> test = {rec(100, 44.5, 90.28, 617, 1, 0.01)};
  auto s = compare_by_quality(ref, test);
  ASSERT_EQ(s.count(Quality::FIXED), 1u);
  EXPECT_EQ(s[Quality::FIXED].n, 1);
  EXPECT_NEAR(s[Quality::FIXED].rmse_h, 0.0, 1e-6);
  EXPECT_NEAR(s[Quality::FIXED].rmse_v, 0.0, 1e-6);
}

TEST(TrajCompare, KnownHorizontalOffset) {
  // test 相对 ref 北偏 ~1m。RTKLIB Q=2 才是 float(Q=3 是 SBAS,映射为 NONE)。
  const double dlat = 1.0 / 111320.0;
  std::vector<PosRecord> ref = {rec(100, 44.5, 90.28, 617, 2, 0.1)};
  std::vector<PosRecord> test = {rec(100, 44.5 + dlat, 90.28, 617, 2, 0.1)};
  auto s = compare_by_quality(ref, test);
  ASSERT_EQ(s.count(Quality::FLOAT), 1u);
  EXPECT_NEAR(s[Quality::FLOAT].rmse_h, 1.0, 0.05);
  // 1 m 实际 / σ_h,σ_h = hypot(sdn, sde) = 0.141 → 7.07
  EXPECT_NEAR(s[Quality::FLOAT].sigma_ratio_h_median, 7.07, 0.5);
  EXPECT_NEAR(s[Quality::FLOAT].sigma_ratio_h_mean, 7.07, 0.5);
  EXPECT_NEAR(s[Quality::FLOAT].sigma_ratio_h_rms, 7.07, 0.5);
}

TEST(TrajCompare, MedianRatioRobustToOneWrongFix) {
  // 9 个正常 FIXED 历元(误差 1 cm, σ 1 cm → 比值 ~0.7)+ 1 个错误固定(误差 1 m, σ 3 mm → 比值 ~236)
  const double dlat_1cm = 0.01 / 111320.0, dlat_1m = 1.0 / 111320.0;
  std::vector<PosRecord> ref, test;
  for (int i = 0; i < 9; ++i) {
    ref.push_back(rec(100 + i, 44.5, 90.28, 617, 1, 0.01));
    test.push_back(rec(100 + i, 44.5 + dlat_1cm, 90.28, 617, 1, 0.01));
  }
  ref.push_back(rec(200, 44.5, 90.28, 617, 1, 0.003));
  test.push_back(rec(200, 44.5 + dlat_1m, 90.28, 617, 1, 0.003));
  auto s = compare_by_quality(ref, test);
  ASSERT_EQ(s[Quality::FIXED].n, 10);
  EXPECT_LT(s[Quality::FIXED].sigma_ratio_h_median, 2.0);   // 中位数不受单个外点影响
  EXPECT_GT(s[Quality::FIXED].sigma_ratio_h_mean, 20.0);    // 均值被外点拽走——这就是不用均值的理由
}

TEST(TrajCompare, UnpairedBeyondToleranceSkipped) {
  std::vector<PosRecord> ref = {rec(100, 44.5, 90.28, 617, 1, 0.01)};
  std::vector<PosRecord> test = {rec(102, 44.5, 90.28, 617, 1, 0.01)};  // 2s 差
  auto s = compare_by_quality(ref, test, 0.1);
  EXPECT_TRUE(s.empty());
}

TEST(TrajCompare, ZeroSigmaCountedButExcludedFromRatio) {
  const double dlat = 1.0 / 111320.0;
  std::vector<PosRecord> ref = {rec(100, 44.5, 90.28, 617, 5, 0.0), rec(101, 44.5, 90.28, 617, 5, 0.5)};
  std::vector<PosRecord> test = {rec(100, 44.5 + dlat, 90.28, 617, 5, 0.0), rec(101, 44.5 + dlat, 90.28, 617, 5, 0.5)};
  auto s = compare_by_quality(ref, test);
  ASSERT_EQ(s[Quality::SINGLE].n, 2);
  // 只有第二个历元参与比值:1 / hypot(0.5,0.5) = 1.414
  EXPECT_NEAR(s[Quality::SINGLE].sigma_ratio_h_median, 1.414, 0.05);
  EXPECT_NEAR(s[Quality::SINGLE].sigma_ratio_h_mean, 1.414, 0.05);
  EXPECT_TRUE(std::isfinite(s[Quality::SINGLE].sigma_ratio_h_rms));
}

TEST(TrajCompare, VerticalErrorAndMixedQualities) {
  std::vector<PosRecord> ref = {rec(100, 44.5, 90.28, 617, 1, 0.01), rec(101, 44.5, 90.28, 617, 1, 0.01)};
  std::vector<PosRecord> test = {rec(100.05, 44.5, 90.28, 619, 1, 0.01),   // 50 ms 内配对,垂直偏 2 m
                                 rec(101.0, 44.5, 90.28, 617, 2, 0.1)};    // FLOAT 档
  auto s = compare_by_quality(ref, test, 0.1);
  ASSERT_EQ(s.size(), 2u);
  EXPECT_EQ(s[Quality::FIXED].n, 1);
  EXPECT_NEAR(s[Quality::FIXED].rmse_v, 2.0, 1e-3);
  EXPECT_EQ(s[Quality::FLOAT].n, 1);
  EXPECT_NEAR(s[Quality::FLOAT].rmse_h, 0.0, 1e-6);
}
