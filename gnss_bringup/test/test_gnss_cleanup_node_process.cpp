#include <gtest/gtest.h>
#include <signal.h>
#include <unistd.h>

#include <chrono>
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

TEST(GnssCleanupNodeProcess, InterruptDuringStartupIsNotAConfigError) {
  // 回归:rclcpp::init 之后、定时器建好之前收到 SIGINT,rclcpp 的信号处理器先让 context
  // 失效——落在节点构造里(参数服务)时 main() 以前打"启动失败,配置有误"并退出 1;落在
  // try 外面的 create_wall_timer 上时异常没人接,直接 std::terminate()/SIGABRT。
  // 这个节点启动期间没有可以当标记的中间日志,改成按启动耗时扫一遍打断时刻:先完整
  // 起一次量出"就绪"要多久,再从 0 到略超过这个时长均匀地挑时刻送 SIGINT。
  // 合法结果只有两种:0(rclcpp 已接管信号,干净退出),或 128+SIGINT(信号到得比
  // rclcpp::init 装处理器还早,按默认动作被杀——那时节点什么都还没做)。
  const auto base = make_temp_dir("cleanup_node_intr_");
  ASSERT_FALSE(base.empty());
  TempDirGuard base_guard(base);
  const auto args = [](const std::string& dir) {
    fs::create_directories(dir + "/bags");
    return std::vector<std::string>{"--ros-args", "-p", "bag_root:=" + dir + "/bags"};
  };

  double ready_s = 0.0;
  {
    const std::string dir = base + "/measure";
    const auto t0 = std::chrono::steady_clock::now();
    NodeProcess node(GNSS_CLEANUP_NODE_PATH, args(dir), dir + ".log", isolated_env(dir));
    ASSERT_TRUE(wait_until([&] { return node.log().find("清理完成一轮") != std::string::npos; }, 20.0,
                           std::chrono::milliseconds(1)))
        << node.log();
    ready_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    node.interrupt();
    ASSERT_EQ(node.wait_exit(20.0), 0) << node.log();
  }

  const int kSteps = 48;
  for (int k = 0; k < kSteps; ++k) {
    const double delay_s = ready_s * 1.25 * k / kSteps;
    SCOPED_TRACE("delay_ms=" + std::to_string(static_cast<int>(delay_s * 1000.0)));
    const std::string dir = base + "/run" + std::to_string(k);
    NodeProcess node(GNSS_CLEANUP_NODE_PATH, args(dir), dir + ".log", isolated_env(dir));
    std::this_thread::sleep_for(std::chrono::duration<double>(delay_s));
    node.interrupt();
    const int rc = node.wait_exit(20.0);
    EXPECT_TRUE(rc == 0 || rc == 128 + SIGINT) << "rc=" << rc << "\n" << node.log();
    EXPECT_EQ(node.log().find("启动失败"), std::string::npos) << node.log();
  }
}
