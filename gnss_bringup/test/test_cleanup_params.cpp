#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "gnss_bringup/cleanup_params.hpp"
#include "node_process_harness.hpp"   // make_temp_dir, TempDirGuard
using namespace gnss_bringup;
using gnss_bringup_test::make_temp_dir;
using gnss_bringup_test::TempDirGuard;
namespace fs = std::filesystem;

namespace {
const int TODAY = 20260910;

bool rejected(const CleanupParams& p, const std::string& field) {
  try {
    validate_cleanup_params(p);
  } catch (const std::invalid_argument& e) {
    return std::string(e.what()).rfind(field, 0) == 0;
  }
  return false;
}

std::optional<double> low_usage(const std::string&) { return 10.0; }
}  // namespace

TEST(CleanupParams, ValidatesEveryField) {
  CleanupParams ok;
  ok.bag_root = "/b";
  EXPECT_NO_THROW(validate_cleanup_params(ok));
  CleanupParams p = ok;
  p.bag_root.clear();
  EXPECT_TRUE(rejected(p, "bag_root")) << "两个根目录不能都为空";
  p = ok; p.retention_days = 0;      EXPECT_TRUE(rejected(p, "retention_days"));
  p = ok; p.watermark_pct = 0.0;     EXPECT_TRUE(rejected(p, "watermark_pct"));
  p = ok; p.watermark_pct = 100.5;   EXPECT_TRUE(rejected(p, "watermark_pct"));
  p = ok; p.watermark_pct = std::numeric_limits<double>::quiet_NaN(); EXPECT_TRUE(rejected(p, "watermark_pct"));
  p = ok; p.interval_s = 0.5;        EXPECT_TRUE(rejected(p, "interval_s"));
  p = ok; p.interval_s = 1e300;      EXPECT_TRUE(rejected(p, "interval_s")) << "上限 7 天,防止换算毫秒时 int64 溢出";
  p = ok; p.interval_s = 7 * 86400.0; EXPECT_NO_THROW(validate_cleanup_params(p));
}

TEST(CleanupPass, BagRootFirstThenPosRootAndEmptyRootsAreSkipped) {
  const auto base = make_temp_dir("cleanup_pass_");
  ASSERT_FALSE(base.empty());
  TempDirGuard guard(base);
  for (const char* d : {"bags/gnss_20260801_000000", "bags/gnss_20260910_120000", "pos/20260801", "pos/20260910"}) {
    fs::create_directories(base + "/" + d);
  }
  CleanupParams p;
  p.bag_root = base + "/bags";
  p.pos_root = base + "/pos";
  p.watermark_pct = 100.0;

  std::vector<std::string> usage_queries;
  const auto results = run_cleanup_pass(p, TODAY, [&](const std::string& root) {
    usage_queries.push_back(root);
    return std::optional<double>(10.0);
  });
  ASSERT_EQ(results.size(), 2u);
  EXPECT_EQ(results[0].kind, "bag");
  EXPECT_EQ(results[0].report.deleted, (std::vector<std::string>{"gnss_20260801_000000"}));
  EXPECT_EQ(results[1].kind, "pos");
  EXPECT_EQ(results[1].report.deleted, (std::vector<std::string>{"20260801"}));
  EXPECT_EQ(usage_queries, (std::vector<std::string>{p.bag_root, p.pos_root})) << "录包占盘大,先扫录包根目录";
  EXPECT_TRUE(fs::exists(base + "/bags/gnss_20260910_120000"));
  EXPECT_TRUE(fs::exists(base + "/pos/20260910"));

  p.pos_root.clear();
  EXPECT_EQ(run_cleanup_pass(p, TODAY, low_usage).size(), 1u);
}

TEST(CleanupPass, MissingRootIsSkippedButABrokenOneIsAnError) {
  const auto base = make_temp_dir("cleanup_pass_missing_");
  ASSERT_FALSE(base.empty());
  TempDirGuard guard(base);
  fs::create_directories(base + "/pos/20260801");
  fs::create_directories(base + "/pos/20260910");
  std::ofstream(base + "/not_a_dir") << "x";
  CleanupParams p;
  p.bag_root = base + "/never_recorded";
  p.pos_root = base + "/pos";
  p.watermark_pct = 100.0;
  auto results = run_cleanup_pass(p, TODAY, low_usage);
  ASSERT_EQ(results.size(), 2u);
  EXPECT_TRUE(results[0].missing) << "还没录过包时根目录不存在是正常的";
  EXPECT_TRUE(results[0].report.error.empty());
  EXPECT_EQ(results[1].report.deleted, (std::vector<std::string>{"20260801"})) << "一个根目录的问题不影响另一个";

  p.bag_root = base + "/not_a_dir";
  results = run_cleanup_pass(p, TODAY, low_usage);
  EXPECT_FALSE(results[0].missing);
  EXPECT_FALSE(results[0].report.error.empty());
}

TEST(CleanupPass, OverWatermarkUsesTheUsageAfterCleaning) {
  RootCleanupResult r;
  r.used_pct_after = 90.0;
  EXPECT_TRUE(over_watermark(r, 85.0));
  r.used_pct_after = 85.0;
  EXPECT_FALSE(over_watermark(r, 85.0));
  r.used_pct_after.reset();
  EXPECT_FALSE(over_watermark(r, 85.0)) << "查不到用量不报超水位";
}
