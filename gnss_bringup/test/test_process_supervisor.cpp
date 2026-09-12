#include <gtest/gtest.h>
#include <chrono>
#include <string>
#include <thread>
#include "gnss_bringup/process_supervisor.hpp"
using namespace gnss_bringup;

namespace {
// CMake 通过 target_compile_definitions 传入假二进制的绝对路径
const char* fake() { return FAKE_RTKRCV_PATH; }

ProcessSupervisorConfig cfg_for(const std::string& mode, double delay) {
  ProcessSupervisorConfig c;
  c.binary = fake();
  c.args = {mode};
  c.cwd = "/tmp";
  c.restart_delay_s = delay;
  c.crash_loop_life_s = 0.5;
  c.max_restart_delay_s = 0.4;
  return c;
}
}  // namespace

TEST(ProcessSupervisor, SpawnsTheChildOnce) {
  ProcessSupervisor s(cfg_for("live", 0.05));
  s.start();
  for (int i = 0; i < 100 && s.spawn_count() < 1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(s.spawn_count(), 1);
  s.stop();
}

TEST(ProcessSupervisor, RestartsAfterTheChildExits) {
  ProcessSupervisor s(cfg_for("die", 0.05));
  s.start();
  for (int i = 0; i < 200 && s.spawn_count() < 3; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_GE(s.spawn_count(), 3) << "子进程退出后必须重启";
  s.stop();
}

TEST(ProcessSupervisor, BacksOffWhenChildDiesImmediately) {
  ProcessSupervisor s(cfg_for("die", 0.05));
  s.start();
  for (int i = 0; i < 200 && s.spawn_count() < 4; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_GT(s.current_delay_s(), 0.05) << "崩溃循环必须拉长重启间隔";
  EXPECT_LE(s.current_delay_s(), 0.4) << "但不得超过 max_restart_delay_s";
  s.stop();
}

TEST(ProcessSupervisor, StopTerminatesALongLivedChild) {
  ProcessSupervisor s(cfg_for("live", 0.05));
  s.start();
  for (int i = 0; i < 100 && s.spawn_count() < 1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const auto t0 = std::chrono::steady_clock::now();
  s.stop();
  const auto dt = std::chrono::steady_clock::now() - t0;
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(dt).count(), 6000)
      << "SIGTERM 后最多等 5 s 再 SIGKILL";
}

TEST(ProcessSupervisor, StopWithoutStartSpawnsNothing) {
  ProcessSupervisor s(cfg_for("live", 0.05));
  s.stop();
  EXPECT_EQ(s.spawn_count(), 0) << "没 start 过就不该派生任何子进程";
}

TEST(ProcessSupervisor, MissingBinaryKeepsRetryingAndBacksOff) {
  // 二进制不存在时,exec 在子进程里失败 → 子进程立刻退出 → 属于崩溃循环。
  // 要断言的是"确实在重试"且"确实退避了",而不只是"没崩"。
  ProcessSupervisorConfig c = cfg_for("live", 0.05);
  c.binary = "/nonexistent/rtkrcv";
  ProcessSupervisor s(c);
  s.start();
  for (int i = 0; i < 100 && s.spawn_count() < 2; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const int spawns = s.spawn_count();
  const double delay = s.current_delay_s();
  s.stop();
  EXPECT_GE(spawns, 2) << "必须持续重试";
  EXPECT_GT(delay, 0.05) << "崩溃循环必须退避";
  EXPECT_LE(delay, 0.4) << "但不得超过 max_restart_delay_s";
}
