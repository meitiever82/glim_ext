#include <gtest/gtest.h>
#include <signal.h>
#include <cerrno>
#include <chrono>
#include <future>
#include <memory>
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

// 在独立线程上跑 stop(),用 future 给它一个"最多等多久"的上限。
// gtest 本身没有单测超时,如果 stop() 真的卡死(比如 fork()/stop() 竞态
// 回归),裸调用 s.stop() 会让整个测试二进制永远挂起而不是报一个失败;
// 这里改成:stop() 跑在一个 detach 掉的线程上,主线程用
// future::wait_for() 设上限——超时就是一次明确的 FAILED,而不是一次
// 看起来像是"卡住了"的挂起。detach 是必须的:如果 stop() 真的卡死,
// 这个线程永远不会退出,不能指望在测试结束时 join 它。
//
// 参数故意是 shared_ptr 而不是引用:超时意味着 stop() 里那个线程还活着、
// 还在碰这个对象。调用方一旦在 wait_for() 超时后继续往下走(测试函数
// 返回、局部变量析构),一个引用就会变成悬空引用——detach 出去的线程
// 后续对一个已经被析构的对象调用成员函数,是货真价实的 use-after-free,
// 完全可能顺带把同一个二进制里排在后面的、原本无关的测试也拖垮。
// shared_ptr 把对象的生命周期至少延长到这个线程自己退出为止,不管那要
// 等多久。
std::future<void> stop_async(std::shared_ptr<ProcessSupervisor> s) {
  auto promise = std::make_shared<std::promise<void>>();
  std::future<void> done = promise->get_future();
  std::thread([s, promise] {
    s->stop();
    promise->set_value();
  }).detach();
  return done;
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
  auto s = std::make_shared<ProcessSupervisor>(cfg_for("live", 0.05));
  s->start();
  for (int i = 0; i < 100 && s->spawn_count() < 1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const int pid = s->last_child_pid();
  ASSERT_GT(pid, 0) << "子进程应该已经 fork 出来了";

  const auto t0 = std::chrono::steady_clock::now();
  std::future<void> done = stop_async(s);
  // review round 1 的 Critical:stop() 曾经可能在 fork()/child_pid_ 赋值
  // 之间的竞态窗口里卡死在 join() 上,而 gtest 没有单测超时——那种情况下
  // 裸调用 s.stop() 会让整个测试二进制永远挂起。用 wait_for() 给一个
  // 硬性上限,超时就是一次响亮的 FAILED,而不是一次看起来像卡住的挂起。
  const auto status = done.wait_for(std::chrono::milliseconds(6000));
  const auto dt = std::chrono::steady_clock::now() - t0;
  ASSERT_EQ(status, std::future_status::ready)
      << "stop() 在 6 s 内没有返回——很可能卡死了(fork()/stop() 竞态回归)";
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(dt).count(), 6000)
      << "SIGTERM 后最多等 5 s 再 SIGKILL";

  // 只测"stop() 按时返回"是不够的:一个把子进程留成孤儿、自己却提前
  // 返回的 stop() 也能让上面的时间断言通过。kill(pid, 0) 在进程已经不
  // 存在时返回 -1/ESRCH,借此确认子进程真的被回收了,而不只是父进程这边
  // 假装完事了。
  errno = 0;
  const int alive = ::kill(pid, 0);
  EXPECT_NE(alive, 0) << "stop() 之后子进程不应该还存在";
  EXPECT_EQ(errno, ESRCH) << "期望的失败原因是进程已经不存在(ESRCH)";
}

TEST(ProcessSupervisor, StopWithoutStartSpawnsNothing) {
  ProcessSupervisor s(cfg_for("live", 0.05));
  s.stop();
  EXPECT_EQ(s.spawn_count(), 0) << "没 start 过就不该派生任何子进程";
}

TEST(ProcessSupervisor, MissingBinaryKeepsRetryingAndBacksOff) {
  // 二进制不存在时,exec 在子进程里失败 → 子进程立刻退出 → 属于崩溃循环。
  // 要断言的是"确实在重试"且"确实退避了",而不只是"没崩"。
  //
  // review round 1 指出的时序缺口:spawn_count_ 在子进程死活揭晓之前就已
  // 经自增(见 process_supervisor.cpp 里 fork() 成功后的注释),所以
  // spawn_count()>=2 只能保证"已经分类过一次死亡"(current_delay_ 从初始
  // 值刚被设成 restart_delay_s),不能保证"已经分类过两次"——旧版本在这里
  // 只等 spawn_count()>=2 就去读 current_delay_s(),存在一个纯属测试自身
  // 的、和被测代码时序耦合过紧的窗口,实测约 2.7% 概率读到还没来得及翻倍
  // 的 0.05,和"崩溃循环没有退避"这个真正的缺陷长得一模一样。改成直接等
  // 待要断言的那个条件本身成立(current_delay_s() 已经 > restart_delay_s),
  // 不再依赖 spawn_count 和 current_delay_ 更新之间的相对时序。
  ProcessSupervisorConfig c = cfg_for("live", 0.05);
  c.binary = "/nonexistent/rtkrcv";
  ProcessSupervisor s(c);
  s.start();
  for (int i = 0; i < 100 && (s.spawn_count() < 2 || s.current_delay_s() <= 0.05); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const int spawns = s.spawn_count();
  const double delay = s.current_delay_s();
  s.stop();
  EXPECT_GE(spawns, 2) << "必须持续重试";
  EXPECT_GT(delay, 0.05) << "崩溃循环必须退避";
  EXPECT_LE(delay, 0.4) << "但不得超过 max_restart_delay_s";
}

TEST(ProcessSupervisor, StartThenImmediateStopNeverHangs) {
  // 回归测试(review round 1 的 Critical):run() 线程 fork() 成功之后、
  // 还没来得及把 pid 写进 child_pid_ 之前,如果 stop() 恰好在这个窗口里
  // 整个跑完——它当时读到的 child_pid_ 还是 -1,判定"没有子进程要杀",
  // 直接去 join()——子进程就会变成没人管的孤儿,而 run() 线程随后会在
  // 等待一个没有人会去唤醒/信号它的子进程,join() 因此永久卡死。
  //
  // start() 几乎立刻返回(只是起了一个线程),而 run() 线程真正跑到
  // fork() 需要一点点调度延迟,所以"start() 后完全不睡眠就立刻从另一个
  // 线程调 stop()"是命中这个窗口最直接的姿势,不需要猜测具体要睡多少
  // 微秒。重复多次以提高覆盖到该窗口的概率。
  for (int i = 0; i < 200; ++i) {
    auto s = std::make_shared<ProcessSupervisor>(cfg_for("live", 0.01));
    s->start();
    std::future<void> done = stop_async(s);
    ASSERT_EQ(done.wait_for(std::chrono::seconds(3)), std::future_status::ready)
        << "第 " << i << " 次迭代:stop() 没有在 3 s 内返回,"
        << "疑似 fork()/stop() 竞态回归导致的死锁";
  }
}
