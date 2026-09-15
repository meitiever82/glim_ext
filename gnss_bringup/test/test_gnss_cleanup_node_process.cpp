#include <gtest/gtest.h>
#include <unistd.h>

#include <ctime>
#include <filesystem>
#include <string>
#include <vector>

#include "gnss_core/retention.hpp"
#include "node_process_harness.hpp"
using namespace gnss_bringup_test;
namespace fs = std::filesystem;

namespace {
std::vector<std::pair<std::string, std::string>> isolated_env(const std::string& dir) {
  return {{"ROS_DOMAIN_ID", std::to_string(30 + ::getpid() % 10)}, {"ROS_LOG_DIR", dir + "/roslog"}};
}
}  // namespace

TEST(GnssCleanupNodeProcess, RefusesToStartWithoutAnyRoot) {
  const auto dir = make_temp_dir("cleanup_node_bad_");
  ASSERT_FALSE(dir.empty());
  TempDirGuard guard(dir);
  NodeProcess node(GNSS_CLEANUP_NODE_PATH, {"--ros-args"}, dir + "/node.log", isolated_env(dir));
  EXPECT_EQ(node.wait_exit(20.0), 1) << node.log();
  EXPECT_NE(node.log().find("启动失败,配置有误"), std::string::npos) << node.log();
  EXPECT_NE(node.log().find("bag_root"), std::string::npos) << node.log();
}

TEST(GnssCleanupNodeProcess, CleansOnStartupAndKeepsRecentEntries) {
  const auto dir = make_temp_dir("cleanup_node_run_");
  ASSERT_FALSE(dir.empty());
  TempDirGuard guard(dir);
  const std::string today = std::to_string(gnss_core::utc_yyyymmdd(static_cast<double>(std::time(nullptr))));
  for (const auto& d : std::vector<std::string>{"bags/gnss_20200101_000000", "bags/gnss_" + today + "_000000",
                                                "pos/20200101", "pos/" + today}) {
    fs::create_directories(dir + "/" + d);
  }
  NodeProcess node(GNSS_CLEANUP_NODE_PATH,
                   {"--ros-args", "-p", "bag_root:=" + dir + "/bags", "-p", "pos_root:=" + dir + "/pos",
                    "-p", "watermark_pct:=100.0"},
                   dir + "/node.log", isolated_env(dir));
  EXPECT_TRUE(wait_until([&] { return node.log().find("清理完成一轮") != std::string::npos; }, 20.0)) << node.log();
  EXPECT_FALSE(fs::exists(dir + "/bags/gnss_20200101_000000")) << node.log();
  EXPECT_FALSE(fs::exists(dir + "/pos/20200101")) << node.log();
  EXPECT_TRUE(fs::exists(dir + "/bags/gnss_" + today + "_000000"));
  EXPECT_TRUE(fs::exists(dir + "/pos/" + today));
  EXPECT_NE(node.log().find("已删除 " + dir + "/bags/gnss_20200101_000000"), std::string::npos) << node.log();
  node.interrupt();
  EXPECT_EQ(node.wait_exit(20.0), 0) << node.log();
}
