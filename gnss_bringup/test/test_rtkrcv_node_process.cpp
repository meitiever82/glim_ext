#include <gtest/gtest.h>
#include <signal.h>
#include <unistd.h>
#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>
#include "node_process_harness.hpp"
using namespace gnss_bringup_test;

namespace {
const char* node_exe() { return RTKRCV_NODE_PATH; }
const char* fake() { return FAKE_RTKRCV_PATH; }

std::vector<std::pair<std::string, std::string>> isolated_env(const std::string& dir) {
  return {{"ROS_DOMAIN_ID", std::to_string(40 + ::getpid() % 50)}, {"ROS_LOG_DIR", dir + "/roslog"}};
}

std::vector<std::string> node_args(const std::string& dir, const std::string& binary, int sol_port,
                                   const std::string& mode_arg) {
  return {"--ros-args",
          "-p", "binary:=" + binary,
          "-p", "run_dir:=" + dir + "/run",
          "-p", "sol_port:=" + std::to_string(sol_port),
          "-p", "corr_port:=" + std::to_string(pick_free_port()),
          "-p", "obs_port:=" + std::to_string(pick_free_port()),
          "-p", "restart_delay_s:=0.1",
          "-p", "crash_loop_life_s:=5.0",
          "-p", "max_restart_delay_s:=0.4",
          "-p", "args:=['" + mode_arg + "']"};
}

int pid_after(const std::string& log, const std::string& marker) {
  const auto pos = log.find(marker);
  if (pos == std::string::npos) return -1;
  return std::atoi(log.c_str() + pos + marker.size());
}
}  // namespace

TEST(RtkrcvNodeProcess, BusySolPortRefusesToStartAndWritesNoConf) {
  const std::string dir = make_temp_dir("rtkrcv_node_busy_");
  ASSERT_FALSE(dir.empty());
  std::filesystem::create_directories(dir + "/run");
  ListeningSocket orphan;
  ASSERT_GT(orphan.port(), 0);
  NodeProcess node(node_exe(), node_args(dir, fake(), orphan.port(), "live"), dir + "/node.log", isolated_env(dir));
  EXPECT_EQ(node.wait_exit(20.0), 1) << node.log();
  EXPECT_NE(node.log().find("孤儿"), std::string::npos) << node.log();
  EXPECT_FALSE(std::filesystem::exists(dir + "/run/rtkrcv.conf")) << "端口检查必须先于写 conf";
  std::filesystem::remove_all(dir);
}

TEST(RtkrcvNodeProcess, UnresolvableBinaryRefusesToStartAndWritesNoConf) {
  const std::string dir = make_temp_dir("rtkrcv_node_nobin_");
  ASSERT_FALSE(dir.empty());
  std::filesystem::create_directories(dir + "/run");
  NodeProcess node(node_exe(), node_args(dir, "/nonexistent/rtkrcv", pick_free_port(), "live"),
                   dir + "/node.log", isolated_env(dir));
  EXPECT_EQ(node.wait_exit(20.0), 1) << node.log();
  EXPECT_NE(node.log().find("找不到或不可执行"), std::string::npos) << node.log();
  EXPECT_FALSE(std::filesystem::exists(dir + "/run/rtkrcv.conf"));
  std::filesystem::remove_all(dir);
}

TEST(RtkrcvNodeProcess, BareBinaryNameIsResolvedThroughPathAndStaysUp) {
  // 回归:yaml 默认 binary: "rtkrcv" 是裸名字,以前永远起不来
  const std::string dir = make_temp_dir("rtkrcv_node_path_");
  ASSERT_FALSE(dir.empty());
  std::filesystem::create_directories(dir + "/run");
  const std::string full = fake();
  const std::string fake_dir = full.substr(0, full.rfind('/'));
  const std::string fake_name = full.substr(full.rfind('/') + 1);
  const char* old_path = std::getenv("PATH");
  auto env = isolated_env(dir);
  env.emplace_back("PATH", fake_dir + ":" + (old_path ? old_path : ""));

  NodeProcess node(node_exe(), node_args(dir, fake_name, pick_free_port(), "live"), dir + "/node.log", env);
  ASSERT_TRUE(wait_until([&] { return node.log().find("rtkrcv 已启动 pid=") != std::string::npos; }, 20.0))
      << node.log();
  const int child = pid_after(node.log(), "rtkrcv 已启动 pid=");
  ASSERT_GT(child, 0);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  EXPECT_EQ(::kill(child, 0), 0) << "按 PATH 找到的子进程必须一直活着\n" << node.log();
  EXPECT_EQ(count_occurrences(node.log(), "rtkrcv 退出 pid="), 0u) << node.log();

  node.interrupt();
  EXPECT_EQ(node.wait_exit(20.0), 0) << node.log();
  EXPECT_TRUE(wait_until([&] { return ::kill(child, 0) != 0 && errno == ESRCH; }, 5.0))
      << "节点退出后子进程必须被收掉";
  std::filesystem::remove_all(dir);
}

TEST(RtkrcvNodeProcess, EveryChildExitIsLogged) {
  // 回归:以前崩溃循环完全静默(实测 20 s 重启 6 次,节点日志 0 行)
  const std::string dir = make_temp_dir("rtkrcv_node_die_");
  ASSERT_FALSE(dir.empty());
  std::filesystem::create_directories(dir + "/run");
  NodeProcess node(node_exe(), node_args(dir, fake(), pick_free_port(), "die"), dir + "/node.log", isolated_env(dir));
  EXPECT_TRUE(wait_until([&] { return count_occurrences(node.log(), "rtkrcv 退出 pid=") >= 3; }, 20.0))
      << node.log();
  EXPECT_NE(node.log().find("疑似崩溃循环"), std::string::npos) << node.log();
  node.interrupt();
  EXPECT_EQ(node.wait_exit(20.0), 0) << node.log();
  std::filesystem::remove_all(dir);
}

TEST(RtkrcvNodeProcess, WrittenConfTakesTheBasePositionFromRtcm) {
  const std::string dir = make_temp_dir("rtkrcv_node_conf_");
  ASSERT_FALSE(dir.empty());
  std::filesystem::create_directories(dir + "/run");
  NodeProcess node(node_exe(), node_args(dir, fake(), pick_free_port(), "live"), dir + "/node.log", isolated_env(dir));
  const std::string conf = dir + "/run/rtkrcv.conf";
  ASSERT_TRUE(wait_until([&] { return std::filesystem::exists(conf); }, 20.0)) << node.log();
  ASSERT_TRUE(wait_until([&] { return node.log().find("rtkrcv 已启动 pid=") != std::string::npos; }, 20.0));
  EXPECT_NE(read_file(conf).find("ant2-postype =rtcm\n"), std::string::npos) << read_file(conf);
  node.interrupt();
  EXPECT_EQ(node.wait_exit(20.0), 0) << node.log();
  std::filesystem::remove_all(dir);
}
