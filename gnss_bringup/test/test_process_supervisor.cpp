#include <gtest/gtest.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "gnss_bringup/local_reserver.hpp"
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

// 记录监管回调。用 shared_ptr 持有:stop_async() 超时的情况下监管对象会比
// 测试函数活得久,回调捕获裸 this 会变成悬空指针。
struct Recorder {
  std::mutex mu;
  std::vector<int> spawned;
  std::vector<ChildExitInfo> exits;
  std::size_t exit_count() {
    std::lock_guard<std::mutex> lk(mu);
    return exits.size();
  }
};

void attach_recorder(ProcessSupervisorConfig& c, const std::shared_ptr<Recorder>& rec) {
  c.on_spawn = [rec](int pid, const std::string&) {
    std::lock_guard<std::mutex> lk(rec->mu);
    rec->spawned.push_back(pid);
  };
  c.on_exit = [rec](const ChildExitInfo& e) {
    std::lock_guard<std::mutex> lk(rec->mu);
    rec->exits.push_back(e);
  };
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

TEST(ProcessSupervisor, BareBinaryNameIsFoundThroughPath) {
  // 回归:yaml 默认 binary: "rtkrcv" 是裸名字。execv() 不查 PATH,子进程又已经
  // chdir(cwd),以前会在 cwd 下找这个名字、_exit(127)、无限重启。
  const std::string full = fake();
  const std::string dir = full.substr(0, full.rfind('/'));
  const std::string name = full.substr(full.rfind('/') + 1);
  const char* old = std::getenv("PATH");
  const std::string saved = old ? old : "";
  ::setenv("PATH", (dir + ":" + saved).c_str(), 1);

  ProcessSupervisorConfig c = cfg_for("live", 0.05);
  c.binary = name;
  ProcessSupervisor s(c);
  s.start();
  for (int i = 0; i < 100 && s.spawn_count() < 1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const int pid = s.last_child_pid();
  const bool alive = pid > 0 && ::kill(pid, 0) == 0;
  const int spawns = s.spawn_count();
  s.stop();
  ::setenv("PATH", saved.c_str(), 1);

  EXPECT_TRUE(alive) << "裸名字必须按 PATH 找到并真正跑起来";
  EXPECT_EQ(spawns, 1) << "跑起来的 live 子进程不该被反复重启";
}

TEST(ProcessSupervisor, OnExitReportsExitCodeLifetimeAndNextDelay) {
  auto rec = std::make_shared<Recorder>();
  ProcessSupervisorConfig c = cfg_for("die", 0.05);
  attach_recorder(c, rec);
  ProcessSupervisor s(c);
  s.start();
  for (int i = 0; i < 300 && rec->exit_count() < 2; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  s.stop();

  std::lock_guard<std::mutex> lk(rec->mu);
  ASSERT_GE(rec->exits.size(), 2u);
  ASSERT_GE(rec->spawned.size(), 1u);
  const ChildExitInfo& e = rec->exits[0];
  EXPECT_FALSE(e.spawn_failed);
  EXPECT_TRUE(e.exited);
  EXPECT_EQ(e.exit_code, 1) << "fake_rtkrcv.sh die 以 1 退出";
  EXPECT_FALSE(e.signaled);
  EXPECT_EQ(e.pid, rec->spawned[0]);
  EXPECT_GE(e.lifetime_s, 0.0);
  EXPECT_LT(e.lifetime_s, 0.5);
  EXPECT_GT(e.next_delay_s, 0.0);
  EXPECT_TRUE(e.will_restart);
  EXPECT_GE(rec->exits[1].next_delay_s, e.next_delay_s) << "崩溃循环里下一次等待不应缩短";
}

TEST(ProcessSupervisor, OnExitReportsTheKillingSignal) {
  auto rec = std::make_shared<Recorder>();
  ProcessSupervisorConfig c = cfg_for("live", 0.05);
  c.restart_delay_s = 5.0;
  c.max_restart_delay_s = 5.0;   // 被杀之后的重启等待足够长,stop() 负责打断
  attach_recorder(c, rec);
  ProcessSupervisor s(c);
  s.start();
  for (int i = 0; i < 100 && s.spawn_count() < 1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ASSERT_EQ(::kill(s.last_child_pid(), SIGKILL), 0);
  for (int i = 0; i < 300 && rec->exit_count() < 1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  s.stop();

  std::lock_guard<std::mutex> lk(rec->mu);
  ASSERT_GE(rec->exits.size(), 1u);
  EXPECT_TRUE(rec->exits[0].signaled);
  EXPECT_EQ(rec->exits[0].signal, SIGKILL);
  EXPECT_FALSE(rec->exits[0].exited);
  EXPECT_TRUE(rec->exits[0].will_restart);
}

TEST(ProcessSupervisor, MissingBinaryIsReportedWithoutForkingAndBacksOff) {
  auto rec = std::make_shared<Recorder>();
  ProcessSupervisorConfig c = cfg_for("live", 0.05);
  c.binary = "/nonexistent/rtkrcv";
  attach_recorder(c, rec);
  ProcessSupervisor s(c);
  s.start();
  for (int i = 0; i < 100 && (s.start_failure_count() < 2 || s.current_delay_s() <= 0.05); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const int failures = s.start_failure_count();
  const int spawns = s.spawn_count();
  const double delay = s.current_delay_s();
  s.stop();

  EXPECT_GE(failures, 2) << "必须持续重试";
  EXPECT_EQ(spawns, 0) << "解析不到可执行文件时不该 fork 一个注定 _exit(127) 的子进程";
  EXPECT_GT(delay, 0.05) << "必须退避";
  EXPECT_LE(delay, 0.4) << "但不得超过 max_restart_delay_s";
  std::lock_guard<std::mutex> lk(rec->mu);
  ASSERT_FALSE(rec->exits.empty());
  EXPECT_TRUE(rec->exits[0].spawn_failed);
  EXPECT_EQ(rec->exits[0].pid, -1);
  EXPECT_NE(rec->exits[0].detail.find("/nonexistent/rtkrcv"), std::string::npos) << rec->exits[0].detail;
}

TEST(ProcessSupervisor, ExitCausedByStopIsReportedAsNotRestarting) {
  auto rec = std::make_shared<Recorder>();
  ProcessSupervisorConfig c = cfg_for("live", 0.05);
  attach_recorder(c, rec);
  auto s = std::make_shared<ProcessSupervisor>(c);
  s->start();
  for (int i = 0; i < 100 && s->spawn_count() < 1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto done = stop_async(s);
  ASSERT_EQ(done.wait_for(std::chrono::seconds(15)), std::future_status::ready);

  std::lock_guard<std::mutex> lk(rec->mu);
  ASSERT_EQ(rec->exits.size(), 1u);
  EXPECT_FALSE(rec->exits[0].will_restart);
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

// final-fix-wave 第 5 项:这条不变量横跨 LocalReserver 和 ProcessSupervisor
// 两个组件,此前完全没有任何测试覆盖它,但它是整个 rtkrcv_node 设计能成立
// 的前提——rtkrcv 及其子孙进程一旦继承了 LocalReserver 监听 socket 的一份
// fd 拷贝,LocalReserver::stop() 关闭它自己那一份并不会真正释放端口(内核
// 只在所有拷贝都被关闭之后才会真正释放),下一次重启 rtkrcv_node 会在
// start_local_reservers() 里绑定同一个端口时失败。
//
// ProcessSupervisor::run() 已经在 fork() 之后、execv() 之前对继承下来的 fd
// 做了 close_range(3, ~0u, 0)(见 process_supervisor.cpp 里 review round 2
// 的说明,`ChildDoesNotInheritParentDescriptors` 测试已经覆盖了这一半),
// 这条测试要证的是这个通用机制确实也把 LocalReserver 的监听 fd 算在内、
// 结果上端口真的会被释放——而不是重复测同一段代码。
TEST(ProcessSupervisor, SpawnedChildDoesNotInheritLocalReserverListeningSocket) {
  LocalReserver reserver;
  ASSERT_TRUE(reserver.start(0)) << "LocalReserver 没能起来,没法做这个检查";
  const int port = reserver.bound_port();
  ASSERT_GT(port, 0);

  auto supervisor = std::make_shared<ProcessSupervisor>(cfg_for("live", 0.05));
  supervisor->start();
  for (int i = 0; i < 100 && supervisor->spawn_count() < 1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const int child_pid = supervisor->last_child_pid();
  ASSERT_GT(child_pid, 0) << "子进程应该已经 fork 出来了";
  // 给 execv 一点时间真正跑起来,让 close_range()/exec 都已经生效。
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  reserver.stop();

  // 关键:在子进程仍然活着(fake_rtkrcv.sh 的 live 模式,故意还没有
  // stop() supervisor)的情况下检查端口是否已经释放。如果先把 supervisor
  // 也 stop() 掉,子进程被杀死这件事本身就会关闭它持有的所有 fd(不管
  // close_range() 有没有生效),会把"子进程到底有没有继承那份监听 socket
  // 拷贝"这个问题直接掩盖掉——量出来的就不是这条不变量,而是"进程死亡最终
  // 总会回收 fd"这件事,后者任何情况下都成立,验证不出问题。真正要测的是
  // review 描述的场景:长活的子进程还在跑,reserver.stop() 之后端口必须
  // 立刻可以被重新绑定。
  errno = 0;
  ASSERT_EQ(::kill(child_pid, 0), 0) << "子进程在检查端口之前已经不在了,"
                                        "这条测试没有测到它原本要测的东西";

  // 可观测的后果:端口必须能被一个全新的 LocalReserver 立刻重新绑定。如果
  // 长活的子进程手里还攥着监听 socket 的一份拷贝,reserver.stop() 关闭它
  // 自己那一份并不会真正释放端口,这里就会绑不上——这正是 review 描述的
  // "下一次重启 rtkrcv_node 绑不上 sol_port"那类故障的根源。
  LocalReserver again;
  EXPECT_TRUE(again.start(port))
      << "端口没有被真正释放——被监管的子进程很可能继承了 LocalReserver 的"
         "监听 socket";
  again.stop();

  std::future<void> done = stop_async(supervisor);
  ASSERT_EQ(done.wait_for(std::chrono::seconds(6)), std::future_status::ready)
      << "stop() 在 6 s 内没有返回";
}

TEST(ProcessSupervisor, ChildDiesWhenTheForkingThreadGoesAway) {
  // 实现者注意——这条用例的机制和 task-1-brief.md 里给出的原始版本不一样。
  //
  // brief 原版的做法是"泄漏 supervisor 对象、测试进程自己继续跑下去",指望
  // 靠这个来模拟"forking 线程终止"。这个做法测不出真实修复效果,但真正的
  // 原因和这段注释曾经写的不一样,这里把 fix round 1 独立评审用真实测量
  // 纠正过的结论记下来,以免以后又被凭直觉猜错一次:
  //
  //   - 内核层面,"forking 线程退出、但同一线程组里还有活着的兄弟线程"这个
  //     场景**确实会正常送达 PDEATHSIG**——实测验证过:让 forking 线程在
  //     fork() 之后立刻返回(线程函数正常退出),宿主进程的其它线程继续跑,
  //     设了 prctl 的子进程会被送 SIGTERM、死掉、变成僵尸(/proc/<pid>/stat
  //     的 state 是 `Z`);不设 prctl 的对照组则保持 `S`(活着)。这一步和
  //     man page "parent 指创建该进程的线程"的警告是一致的,并不是什么例外。
  //   - 真正让 brief 原版测不出效果的是另外两件事:1) 泄漏出去的 supervisor
  //     的 `running_` 永远是 true,它的 worker 线程会永远阻塞在
  //     `waitpid()+poll()` 那个等待子进程退出的循环里——这个线程从来没有
  //     真正终止过,PDEATHSIG 无从谈起;2) 就算这个线程真的终止了,子进程
  //     被杀死之后也只是变成一个僵尸(`Z`)——僵尸仍然是进程表里一个有效的
  //     pid,`kill(pid, 0)` 对僵尸同样返回 0,原版测试拿它当"进程是否还活着"
  //     的判据,永远不会因为对方变成僵尸而判定为"已经消失"。
  //
  // 因此这里改成 fork() 出一个用完即弃的独立进程来跑 supervisor,再用真正的
  // SIGKILL 终止那个进程——这样被杀的是"整个线程组"(所有线程一起死亡),
  // 内核会把孤儿正常重新托付给 init 并回收,不会停留在僵尸态,和 Step 4
  // 手工复现的 `kill -9 <rtkrcv_node pid>` 在内核语义上完全一致,只是把
  // "手工跑 ros2 run"换成了自动化单测里的一次性子进程。fork() 出来的这个
  // 子进程里绝不能用 gtest 的 ASSERT_*/EXPECT_*(那些宏依赖的 gtest 内部
  // 状态是 fork() 时复制的一份独立副本,子进程里的失败不会传回真正在跑这个
  // TEST 函数的父进程),只用最朴素的返回值检查。
  int pipefd[2];
  ASSERT_EQ(::pipe(pipefd), 0) << "起不了管道,没法从 helper 进程收 pid";

  const pid_t helper = ::fork();
  ASSERT_GE(helper, 0) << "fork() 失败,没法搭这个测试场景";

  if (helper == 0) {
    // ---- helper 子进程:整个测试二进制的一份独立副本,只用来模拟
    // "rtkrcv_node 本体"——起一个 supervisor,从不调用 stop(),把 rtkrcv 的
    // pid 报给父测试进程之后原地等死,靠父测试进程接下来的 SIGKILL 结束
    // 这个进程的生命(不能自己退出,否则测的是"正常退出"而不是"被杀死")。
    ::close(pipefd[0]);
    // fix round 1 的 Important 2:helper 继承了测试二进制的 stdout/stderr。
    // 如果这条用例中途失败,导致下面的清理没能在预期路径上跑到、helper 意
    // 外存活下去,一个还攥着测试进程 stdout 写端的孤儿会让任何读这个测试
    // 输出的东西(ctest 的输出捕获、`| cat` 之类)永远等不到 EOF、白白挂住
    // ——已经在真实 ctest(TIMEOUT 15s)下复现过:gtest 本该打印的失败文本
    // 被整个吞掉,ctest 只报一次语焉不详的 Timeout。这里让 helper 自己的
    // stdout/stderr 都改道 /dev/null,即使它活下来也不会再持有那两个 fd。
    ::freopen("/dev/null", "w", stdout);
    ::freopen("/dev/null", "w", stderr);
    ProcessSupervisorConfig c = cfg_for("live", 0.05);
    ProcessSupervisor sup(c);
    sup.start();
    pid_t grandchild = -1;
    for (int i = 0; i < 200 && grandchild <= 0; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      grandchild = sup.last_child_pid();
    }
    char buf[32];
    const int n = ::snprintf(buf, sizeof(buf), "%d\n", grandchild);
    const ssize_t written = ::write(pipefd[1], buf, static_cast<size_t>(n));
    (void)written;  // 写失败也没有更好的处理方式:父测试进程读不到就会自己报失败
    for (;;) ::pause();  // 只等被 SIGKILL,不做任何清理
  }

  // ---- 父测试进程(真正跑这个 TEST 函数、gtest 断言在这里才有意义)----
  ::close(pipefd[1]);

  // fix round 1 的 Important 2:不管接下来哪一个 ASSERT_* 提前 return,都
  // 必须保证 helper(以及它已经报告出来的 grandchild,如果拿到了的话)被
  // 杀掉、回收掉——不这样做的话,这条用例自己的失败会在 CI 里留下一个孤儿:
  // helper 会带着一个每 50ms 重新 fork 一次 fake 二进制的 supervisor 永远
  // pause() 下去,且不受 ctest 超时的约束(它在自己的会话里,ctest 杀不到
  // 它)。用一个作用域生命周期的清理器覆盖每一条退出路径,而不是只在"一切
  // 顺利"的末尾清理一次。
  struct HelperCleanup {
    pid_t helper_pid = -1;
    pid_t grandchild_pid = -1;
    ~HelperCleanup() {
      if (grandchild_pid > 0) {
        ::kill(grandchild_pid, SIGKILL);
      }
      if (helper_pid > 0) {
        ::kill(helper_pid, SIGKILL);
        int status = 0;
        ::waitpid(helper_pid, &status, 0);  // 避免留下一个僵尸
      }
    }
  } cleanup;
  cleanup.helper_pid = helper;

  // fix round 1 的 Important 2(第二条):原来这里是一次不设上限的 read(),
  // 如果 helper 在 200 次 10ms 轮询里始终没能拿到 grandchild 的 pid(比如
  // fake 二进制路径出了问题),它只会一直卡在上面的轮询循环里,永远不写
  // 这个管道——那样 read() 会永久阻塞,把整个测试二进制挂起,连 ASSERT_*
  // 的机会都没有。改成 poll() 一个有界超时,超时就是一次明确的 FAILED,
  // HelperCleanup 会负责把还卡着的 helper 杀掉。
  pollfd pfd{pipefd[0], POLLIN, 0};
  const int pr = ::poll(&pfd, 1, 3000);
  ASSERT_GT(pr, 0) << "3 秒内没有从 helper 进程收到 rtkrcv 的 pid"
                       "(helper 可能没能成功 fork 出子进程,或者本身卡住了)";

  char buf[32] = {};
  const ssize_t n = ::read(pipefd[0], buf, sizeof(buf) - 1);
  ::close(pipefd[0]);
  ASSERT_GT(n, 0) << "管道另一端已经关闭,却没有收到任何数据"
                     "(helper 可能没能成功 fork 出子进程)";
  const pid_t grandchild = static_cast<pid_t>(std::atoi(buf));
  ASSERT_GT(grandchild, 0);
  cleanup.grandchild_pid = grandchild;
  ASSERT_EQ(::kill(grandchild, 0), 0) << "rtkrcv 子进程应当活着";

  ASSERT_EQ(::kill(helper, SIGKILL), 0) << "杀不掉 helper 进程,没法继续这个测试";
  int status = 0;
  ASSERT_EQ(::waitpid(helper, &status, 0), helper) << "回收 helper 进程失败";
  cleanup.helper_pid = -1;  // 已经在上面正常 waitpid() 回收,不需要 HelperCleanup 再动它

  bool gone = false;
  for (int i = 0; i < 500; ++i) {
    if (::kill(grandchild, 0) != 0 && errno == ESRCH) { gone = true; break; }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (gone) cleanup.grandchild_pid = -1;  // 已经自己死透了,不需要 HelperCleanup 再补刀
  EXPECT_TRUE(gone) << "helper 进程被 SIGKILL 之后 rtkrcv 仍然活着,"
                       "PR_SET_PDEATHSIG 没生效(或者没有跨 execve 保留)";
}
