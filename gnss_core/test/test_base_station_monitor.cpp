#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "gnss_core/base_station_monitor.hpp"
using namespace gnss_core;

namespace {
const Ecef XYZ{-2148744.1, 4426641.2, 4044655.9};
Ecef shifted(double dx) { return Ecef{XYZ.x + dx, XYZ.y, XYZ.z}; }

class TempDir {
public:
  TempDir() {
    const char* base = std::getenv("TMPDIR");
    std::string tmpl = std::string(base ? base : "/tmp") + "/base_monitor_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    if (::mkdtemp(buf.data()) != nullptr) path_ = buf.data();
  }
  ~TempDir() {
    if (!path_.empty()) std::filesystem::remove_all(path_);
  }
  const std::string& path() const { return path_; }

private:
  std::string path_;
};
}  // namespace

TEST(BaseStationMonitor, LearnsBaselineThenReportsOffset) {   // 移植 test_learns_baseline_then_reports_offset
  BaseStationMonitor m(100.0, std::nullopt, std::nullopt);
  auto r = m.feed(0, XYZ);
  EXPECT_FALSE(r.offset_m.has_value());
  EXPECT_TRUE(r.history_changed);
  r = m.feed(50, shifted(0.001));
  EXPECT_FALSE(r.offset_m.has_value()) << "仍在预热";
  r = m.feed(101, XYZ);
  ASSERT_TRUE(r.offset_m.has_value());
  EXPECT_TRUE(r.baseline_learned);
  EXPECT_LT(*r.offset_m, 0.002);
  ASSERT_TRUE(m.baseline().has_value());
  EXPECT_NEAR(m.baseline()->x, XYZ.x, 1e-9) << "x 轴三个样本 [x, x+0.001, x] 的中位数是 x";
  r = m.feed(102, shifted(0.5));
  ASSERT_TRUE(r.offset_m.has_value());
  EXPECT_NEAR(*r.offset_m, 0.5, 0.01);
  EXPECT_FALSE(r.baseline_learned);
}

TEST(BaseStationMonitor, EvenSampleCountMedianAveragesTheMiddlePair) {
  BaseStationMonitor m(10.0, std::nullopt, std::nullopt);
  m.feed(0, shifted(0.0));
  m.feed(4, shifted(1.0));
  m.feed(8, shifted(3.0));
  m.feed(10, shifted(10.0));   // 4 个样本 [0, 1, 3, 10] → 中位数 2
  ASSERT_TRUE(m.baseline().has_value());
  EXPECT_NEAR(m.baseline()->x - XYZ.x, 2.0, 1e-6);
}

TEST(BaseStationMonitor, PersistedBaselineSkipsWarmup) {   // 移植 test_baseline_persists_across_restart
  BaseStationMonitor m(600.0, XYZ, XYZ);
  const auto r = m.feed(10, XYZ);
  ASSERT_TRUE(r.offset_m.has_value());
  EXPECT_FALSE(r.history_changed) << "与上次历史坐标相同";
}

TEST(BaseStationMonitor, HistoryRecordsChangesOnly) {   // 移植 test_history_records_changes_only
  BaseStationMonitor m(1.0, std::nullopt, std::nullopt);
  EXPECT_TRUE(m.feed(0, XYZ).history_changed);
  EXPECT_FALSE(m.feed(2, shifted(0.0009)).history_changed) << "1 mm 容差内不算变化";
  EXPECT_TRUE(m.feed(3, shifted(0.5)).history_changed);
}

TEST(BaseStationMonitor, ResetUpdatesBaseline) {   // 移植 test_reset_updates_baseline
  BaseStationMonitor m(1.0, std::nullopt, std::nullopt);
  m.feed(0, XYZ);
  m.feed(2, XYZ);
  const auto rr = m.reset(shifted(0.5));
  EXPECT_TRUE(rr.baseline_learned);
  EXPECT_TRUE(rr.history_changed);
  const auto r = m.feed(6, shifted(0.5));
  ASSERT_TRUE(r.offset_m.has_value());
  EXPECT_LT(*r.offset_m, 0.01);
}

TEST(BaseBaselineFile, RoundTripsAndCreatesParentDirectories) {
  TempDir dir;
  ASSERT_FALSE(dir.path().empty());
  const std::string path = dir.path() + "/sub/base_baseline";
  ASSERT_TRUE(write_base_baseline(path, XYZ));
  const auto back = read_base_baseline(path);
  ASSERT_TRUE(back.has_value());
  EXPECT_NEAR(back->x, XYZ.x, 1e-4);
  EXPECT_NEAR(back->y, XYZ.y, 1e-4);
  EXPECT_NEAR(back->z, XYZ.z, 1e-4);
  std::ifstream in(path);
  std::string line;
  std::getline(in, line);
  EXPECT_EQ(line, "-2148744.1000,4426641.2000,4044655.9000");
}

TEST(BaseBaselineFile, MissingOrCorruptFileMeansNoBaseline) {   // 移植 test_corrupt_kv_falls_back_to_rewarmup
  TempDir dir;
  ASSERT_FALSE(dir.path().empty());
  EXPECT_FALSE(read_base_baseline(dir.path() + "/absent").has_value());
  const std::string path = dir.path() + "/corrupt";
  std::ofstream(path) << "garbage";
  EXPECT_FALSE(read_base_baseline(path).has_value());
  std::ofstream(path) << "1.0,2.0";
  EXPECT_FALSE(read_base_baseline(path).has_value());
  std::ofstream(path) << "1.0,2.0,3.0,4.0";
  EXPECT_FALSE(read_base_baseline(path).has_value());
}

TEST(BaseBaselineFile, WriteFailureReturnsFalse) {
  TempDir dir;
  ASSERT_FALSE(dir.path().empty());
  const std::string blocker = dir.path() + "/file";
  std::ofstream(blocker) << "x";
  EXPECT_FALSE(write_base_baseline(blocker + "/base_baseline", XYZ)) << "父路径是普通文件,写不进去";
}
