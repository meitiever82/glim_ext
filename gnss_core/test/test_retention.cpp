#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "gnss_core/retention.hpp"
using namespace gnss_core;

namespace {
const int TODAY = 20260910;

// 按调用次序返回序列值,用完后一直返回最后一个值(配合"删一个就降到水位下"的用例)
std::function<double()> usage_sequence(std::vector<double> seq) {
  auto next = std::make_shared<size_t>(0);
  return [seq, next]() {
    const double v = seq[std::min(*next, seq.size() - 1)];
    ++*next;
    return v;
  };
}

std::function<bool(const std::string&)> record_removal(std::vector<std::string>& removed) {
  return [&removed](const std::string& name) {
    removed.push_back(name);
    return true;
  };
}
}  // namespace

TEST(Retention, ParsesDayAndBagDirectoryNames) {
  EXPECT_EQ(parse_day_dir_date("20260914"), 20260914);
  EXPECT_FALSE(parse_day_dir_date("2026091").has_value());
  EXPECT_FALSE(parse_day_dir_date("20261301").has_value()) << "13 月";
  EXPECT_FALSE(parse_day_dir_date("20260230").has_value()) << "2 月 30 日";
  EXPECT_FALSE(parse_day_dir_date("gnss_20260914_101010").has_value());
  EXPECT_EQ(parse_bag_dir_date("gnss_20260914_101010"), 20260914);
  EXPECT_FALSE(parse_bag_dir_date("gnss_20260914").has_value());
  EXPECT_FALSE(parse_bag_dir_date("20260914").has_value());
}

TEST(Retention, DateArithmetic) {
  EXPECT_EQ(days_between(20260820, 20260910), 21);
  EXPECT_EQ(days_between(20241231, 20250101), 1);
  EXPECT_EQ(days_between(20240228, 20240301), 2) << "闰年";
  EXPECT_EQ(days_between(20260910, 20260901), -9);
  EXPECT_EQ(utc_yyyymmdd(1789372800.5), 20260914);
}

TEST(Retention, DeletesBeyondRetention) {   // 移植 test_deletes_beyond_retention
  std::vector<std::string> removed;
  const auto deleted = sweep_dated_entries(
      {{"20260910", 20260910}, {"20260820", 20260820}, {"20260901", 20260901}}, TODAY, 14, 85.0,
      usage_sequence({10.0}), record_removal(removed));
  EXPECT_EQ(deleted, (std::vector<std::string>{"20260820"}));
  EXPECT_EQ(removed, deleted);
}

TEST(Retention, DeletesOldestUntilUnderWatermark) {   // 移植 test_deletes_oldest_when_over_watermark
  std::vector<std::string> removed;
  const auto deleted = sweep_dated_entries(
      {{"20260908", 20260908}, {"20260909", 20260909}, {"20260910", 20260910}}, TODAY, 14, 85.0,
      usage_sequence({90.0, 80.0}), record_removal(removed));
  EXPECT_EQ(deleted, (std::vector<std::string>{"20260908"}));
}

TEST(Retention, NeverDeletesToday) {   // 移植 test_never_deletes_today
  std::vector<std::string> removed;
  EXPECT_TRUE(sweep_dated_entries({{"20260910", 20260910}, {"20260911", 20260911}}, TODAY, 0, 0.0,
                                  usage_sequence({100.0}), record_removal(removed))
                  .empty());
}

TEST(Retention, NeverDeletesTheNewestEntryEvenIfOld) {
  // 跨天录包:最新目录名字是昨天的日期,但仍在写
  std::vector<std::string> removed;
  const auto deleted = sweep_dated_entries(
      {{"gnss_20260801_000000", 20260801}, {"gnss_20260802_000000", 20260802}}, TODAY, 14, 85.0,
      usage_sequence({10.0}), record_removal(removed));
  EXPECT_EQ(deleted, (std::vector<std::string>{"gnss_20260801_000000"}));
}

TEST(Retention, StopsWhenARemovalFails) {
  int calls = 0;
  const auto deleted = sweep_dated_entries(
      {{"20260801", 20260801}, {"20260802", 20260802}, {"20260803", 20260803}}, TODAY, 14, 85.0,
      usage_sequence({10.0}), [&calls](const std::string&) {
        ++calls;
        return false;
      });
  EXPECT_TRUE(deleted.empty());
  EXPECT_EQ(calls, 1);
}

TEST(Retention, CleanupDatedRootOnARealDirectory) {
  const char* base = std::getenv("TMPDIR");
  std::string tmpl = std::string(base ? base : "/tmp") + "/retention_XXXXXX";
  std::vector<char> buf(tmpl.begin(), tmpl.end());
  buf.push_back('\0');
  ASSERT_NE(::mkdtemp(buf.data()), nullptr);
  const std::string root = buf.data();
  for (const char* d : {"20260801", "20260909", "20260910", "notadate"}) {
    std::filesystem::create_directories(root + "/" + d);
    std::ofstream(root + "/" + d + "/can.pos") << "x";
  }
  std::ofstream(root + "/20260802") << "a regular file, not a directory";

  const auto report = cleanup_dated_root(root, parse_day_dir_date, TODAY, 14, 100.0);
  EXPECT_TRUE(report.error.empty()) << report.error;
  EXPECT_EQ(report.deleted, (std::vector<std::string>{"20260801"}));
  EXPECT_FALSE(std::filesystem::exists(root + "/20260801"));
  EXPECT_TRUE(std::filesystem::exists(root + "/20260909"));
  EXPECT_TRUE(std::filesystem::exists(root + "/notadate"));
  EXPECT_TRUE(std::filesystem::exists(root + "/20260802")) << "普通文件不参与";

  const auto missing = cleanup_dated_root(root + "/absent", parse_day_dir_date, TODAY, 14, 100.0);
  EXPECT_FALSE(missing.error.empty());
  EXPECT_TRUE(missing.deleted.empty());
  std::filesystem::remove_all(root);
}
