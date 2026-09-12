#include <gtest/gtest.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
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

TEST(ProcessSupervisor, ChildDoesNotInheritParentDescriptors) {
  // review round 2 的 Important:子进程关闭继承 fd 那段代码换成了
  // close_range(3, ~0u, 0),失败时才退化到逐个 close()。这里要验证的是
  // 结果本身(子进程手里到底还有没有不该有的 fd),而不是这台机器的
  // RLIMIT_NOFILE 具体是多少——用一个具体、可辨认的 fd 做标记,检查它
  // 有没有被子进程继承下去,不依赖任何和 ulimit 相关的假设。
  //
  // 用 mkstemp() 而不是 open("/dev/null", ...):readlink 出来的目标路径
  // 里带一段进程独有的随机后缀,不会和子进程自己可能打开的任何东西
  // (比如 bash 执行脚本时给脚本本身开的 fd)撞车,不会产生"数字凑巧一样
  // 但根本是两个不相干的东西"这种假阳性/假阴性。mkstemp() 本身不带
  // O_CLOEXEC,正是这里想要的"一个会被 fork() 继承、但不该被子进程留着"
  // 的 fd。
  char tmpl[] = "/tmp/procsup_fdcheck_XXXXXX";
  const int marker_fd = ::mkstemp(tmpl);
  ASSERT_GE(marker_fd, 0) << "mkstemp 失败,没法做这个检查";
  const std::string marker_path = tmpl;
  ::unlink(tmpl);  // fd 还开着就够用,不需要真的留一个文件在磁盘上

  auto s = std::make_shared<ProcessSupervisor>(cfg_for("live", 0.05));
  s->start();
  for (int i = 0; i < 100 && s->spawn_count() < 1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const int pid = s->last_child_pid();
  ASSERT_GT(pid, 0) << "子进程应该已经 fork 出来了";
  // 给 execv 一点时间真正跑起来,再去看它手里的 fd。
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const std::string fd_path =
      "/proc/" + std::to_string(pid) + "/fd/" + std::to_string(marker_fd);
  char link_buf[4096] = {};
  const ssize_t n = ::readlink(fd_path.c_str(), link_buf, sizeof(link_buf) - 1);
  bool leaked = false;
  if (n > 0) {
    link_buf[n] = '\0';
    // 文件已经 unlink 了,如果子进程真的继承了这个 fd,内核仍然认得这个
    // inode,readlink 出来的字符串通常是"<原路径> (deleted)"——只要前缀
    // 命中我们这个独一无二的临时文件名,就能确认是同一个 fd,而不是子
    // 进程自己凑巧用了同一个数字。
    leaked = std::string(link_buf).find(marker_path) != std::string::npos;
  }
  EXPECT_FALSE(leaked) << "子进程不应该继承父进程这边打开的 fd "
                       << marker_fd << "(readlink 目标:" << link_buf << ")";

  ::close(marker_fd);

  // review round 2 的补充意见:这里应该像其它测试一样走 stop_async() +
  // wait_for() 上限,而不是裸调用 s->stop()——不然 stop() 真的卡死时,
  // 拖垮的是整个测试二进制而不是这一个测试用例的失败信息。
  std::future<void> done = stop_async(s);
  ASSERT_EQ(done.wait_for(std::chrono::seconds(6)), std::future_status::ready)
      << "stop() 在 6 s 内没有返回";
}

TEST(ProcessSupervisor, StopSendsBoundedSignalCount) {
  // review round 2:stop() 曾经(以及修这个竞态时新引入的 run() 自救逻辑)
  // 会每 20ms 无条件重发同一个信号,对一个装了信号处理器、想在退出前
  // flush 数据的目标程序(rtkrcv 正是如此)来说,处理函数会被连续不断的
  // 新信号打断,可能永远做不完那次收尾。修复之后应该是"一个信号级别只发
  // 一次"。fake_rtkrcv.sh 的 live-count 模式故意不在第一次 SIGTERM 就
  // 退出(最多撑 2s,或者收满 10 次才主动退出),这样才有机会数出 stop()
  // 期间到底送达了几次——用 live 模式的话一碰就倒,根本没法观察。
  char tmpl[] = "/tmp/procsup_sigcount_XXXXXX";
  const int fd = ::mkstemp(tmpl);
  ASSERT_GE(fd, 0);
  ::close(fd);
  const std::string countfile = tmpl;

  ProcessSupervisorConfig c = cfg_for("live-count", 0.05);
  c.args = {"live-count", countfile};
  auto s = std::make_shared<ProcessSupervisor>(c);
  s->start();
  for (int i = 0; i < 100 && s->spawn_count() < 1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_GT(s->last_child_pid(), 0) << "子进程应该已经 fork 出来了";
  // 给它一点时间真正把 trap 安装好,不然过早发的信号会用默认处置处理掉
  // (直接终止,trap 还没生效,根本数不到)。
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // review round 2 的补充意见:走 stop_async() + wait_for() 上限,而不是
  // 裸调用 s->stop()——理由同上一个测试。
  std::future<void> done = stop_async(s);
  ASSERT_EQ(done.wait_for(std::chrono::seconds(6)), std::future_status::ready)
      << "stop() 在 6 s 内没有返回";

  long count = 0;
  std::ifstream in(countfile);
  if (in) {
    in >> count;
  }
  ::unlink(countfile.c_str());

  // 用来判断"没有被信号轰炸"的门槛:旧的每 20ms 重发一次、撑满 5s 的写法
  // 会打出上百次(5000ms / 20ms ≈ 250);这里给一个数量级远小于那个、但
  // 又给 stop() 自身的信号 + run() 万一触发自救逻辑的信号留了余量的上限。
  EXPECT_GT(count, 0) << "至少应该收到一次 SIGTERM(fake 脚本才有理由继续多活一会儿被数到)";
  EXPECT_LE(count, 5) << "SIGTERM 不应该被连续重发轰炸(实测到 " << count
                       << " 次)——这正是 review round 2 要修的问题";
}

TEST(ProcessSupervisor, StopIsFastEvenWithInheritedParentSignalHandler) {
  // review round 3 的 Important 回归测试:round 2 把"每 20ms 重发同一个
  // 信号"改成了"kill(pid, ...) 直接送达一次",解决了信号轰炸,但重新
  // 打开了 setsid() 竞态的另一半——子进程在完成 setsid()、把信号处置
  // 重置成 SIG_DFL 之前,携带的仍然是从父进程这个线程继承来的处置。
  // round 1/2 的整套测试里没有任何一个进程本身装了 SIGTERM 处理器,这条
  // 回归才一直显示是绿的:真实场景里父进程是 rclcpp 节点,rclcpp 自己会
  // 装一个 SIGTERM 处理器,这才是常态而不是边缘情况。
  //
  // 这里在测试进程(也就是 fork() 的父进程)里故意装一个"什么都不做,
  // 直接返回"的 SIGTERM 处理器,复现这层继承关系。关键是"什么时候调
  // stop()":先等 spawn_count()>=1 再 stop() 不够——哪怕只等一轮 10ms 的
  // 轮询,子进程早就跑完 setsid()/sigaction 重置/execv() 了(这些全是
  // 微秒级的操作),signal_child() 送达的是已经 exec 过、干干净净的目标
  // 程序,根本碰不到这条竞态。真实复现的姿势和
  // StartThenImmediateStopNeverHangs 一样:start() 后完全不等待就立刻从
  // 另一个线程 stop()。这条竞态也不是每次都中——round 3 review 的原始
  // 测量是 20 次里 14 次落进 5s 超时——所以这里循环多次,只要绝大多数
  // 迭代都很快就说明修复生效了,而不是要求一次巧合的时序刚好没踩中。
  struct sigaction old_action {};
  struct sigaction noop_action {};
  noop_action.sa_handler = [](int) {};  // 什么都不做,只是"接住"这个信号
  ::sigemptyset(&noop_action.sa_mask);
  ASSERT_EQ(::sigaction(SIGTERM, &noop_action, &old_action), 0)
      << "装不上测试用的 SIGTERM 处理器,没法复现这个场景";

  int slow_iterations = 0;
  constexpr int kIterations = 20;
  for (int i = 0; i < kIterations; ++i) {
    auto s = std::make_shared<ProcessSupervisor>(cfg_for("live", 0.01));
    const auto t0 = std::chrono::steady_clock::now();
    s->start();
    std::future<void> done = stop_async(s);
    const auto status = done.wait_for(std::chrono::seconds(6));
    const auto dt = std::chrono::steady_clock::now() - t0;
    ASSERT_EQ(status, std::future_status::ready)
        << "第 " << i << " 次迭代:stop() 在 6 s 内没有返回";
    if (std::chrono::duration_cast<std::chrono::milliseconds>(dt).count() >= 2000) {
      ++slow_iterations;
    }
  }

  // 不管上面跑得怎么样,都要先把测试进程自己的信号处置换回去——不能让
  // 这个专门装来复现问题的处理器泄漏到同一个二进制里排在后面的其它测试
  // 用例。
  ::sigaction(SIGTERM, &old_action, nullptr);

  EXPECT_EQ(slow_iterations, 0)
      << kIterations << " 次 start()→stop() 里有 " << slow_iterations
      << " 次被拖到了 2s 以上(修复前的典型表现是卡在 ~5s 的 SIGKILL 超时)"
      << "——说明子进程在完成 setsid()/重置信号处置之前,把父进程继承来的"
      << "处理器当成了自己的,把本该终止它的 SIGTERM 吞掉了";
}
