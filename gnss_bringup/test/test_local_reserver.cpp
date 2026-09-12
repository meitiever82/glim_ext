#include <gtest/gtest.h>
#include <arpa/inet.h>
#include <dlfcn.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include "gnss_bringup/local_reserver.hpp"
using namespace gnss_bringup;

// --- fix round 2: 部署 close() 追踪器,给 double-close 回归测试当探针 ---
//
// fix round 2 review 抓到的 Critical 是一个纯粹靠时序触发的竞态(某个 fd
// 被两条独立路径都判定为死连接、各自摘一次、关两次),vanilla close()
// 对一个已经关闭、还没被复用的 fd 编号再调用一次通常只是安静地返回
// EBADF,不会自己让进程崩溃——真正的破坏("这次 close() 关掉的其实是一个
// 刚被复用、仍然存活的连接")不会在一次简单的"进程有没有崩"检查里露出来。
// 要把这个 bug 变成一个会失败的断言,而不是空对空地看进程存不存活,这里
// 通过 dlsym(RTLD_NEXT, ...) 在测试可执行文件里重新定义 close()/accept4()/
// socket()/pipe2()——LocalReserver 编译进 gnss_bringup_io 这个共享库,但
// 主执行文件里定义的这几个符号在动态符号解析顺序上排在共享库前面,共享库
// 里对这几个 libc 函数的调用会被重定向到这里(用一个独立的最小复现程序
// 验证过这个假设成立),从而不经共享库、不经 LocalReserver 内部感知,就能
// 在进程外部旁观每一个真实的 fd 生命周期事件。
//
// 追踪规则很简单,但需要同时看住"关闭"和"重新分配"两类事件,否则会把
// 合法的 fd 编号复用也误判成 double close():每个 fd 编号维护一个
// "当前是否活着"的布尔值。accept4()/socket()/pipe2() 成功时把对应编号标记
// 为活着(新的一次分配,清空它上一次生命周期遗留的"已关闭"状态)。close()
// 被调用时,如果这个编号已经被标记为"活着"变成"已关闭"过、且中途没有
// 出现新的分配事件把它重新标记为活着,就是一次货真价实的 double close()
// ——计数加一并记下具体的 fd 编号,供测试断言用。
namespace close_tracker {
std::mutex g_mutex;
std::unordered_map<int, bool> g_alive;  // fd -> 当前是否被认为"活着"
std::atomic<int> g_double_close_count{0};
std::atomic<int> g_last_double_close_fd{-1};
// final-fix-wave 第 2/6 项测试用:LocalReserver::start() 每次只会调用一次
// socket()(监听 socket;wake 管道走的是 pipe2()),所以"最近一次成功的
// socket() 调用"就是当前这个 LocalReserver 实例的监听 fd。
std::atomic<int> g_last_socket_fd{-1};

// 最初的设计是:等 start() 返回、拿到监听 fd 之后,从测试线程直接把它
// close() 掉,指望 accept_loop 下一次 poll() 看到 POLLNVAL。用一个独立的
// 最小复现程序验证过这个假设是错的:在 Linux 上,accept_loop 线程一旦已经
// 阻塞在 poll(fd, ..., -1) 里,测试线程 close() 掉同一个 fd **不会**唤醒
// 那次 poll() 调用——会一直阻塞到测试自己的等待超时,变成一次依赖谁先被
// 调度的不稳定竞态。
//
// 改成拦截 poll() 本身注入故障,并且把"武装"这一步放进 socket() 的拦截器
// 里、在返回给调用方(也就是 accept_loop 自己所在的那个线程)之前就同步
// 完成——这样"武装"和"这个线程自己接下来第一次调用 poll()"是同一个线程
// 上的顺序关系,不再是两个线程之间的竞态。
std::atomic<bool> g_arm_poll_fault_on_next_socket{false};
std::atomic<int> g_poll_fault_target_fd{-1};
std::atomic<bool> g_poll_fault_armed{false};

void reset() {
  std::lock_guard<std::mutex> lock(g_mutex);
  g_alive.clear();
  g_double_close_count.store(0);
  g_last_double_close_fd.store(-1);
}

void mark_open(int fd) {
  if (fd < 0) return;
  std::lock_guard<std::mutex> lock(g_mutex);
  g_alive[fd] = true;
}

void note_close(int fd) {
  std::lock_guard<std::mutex> lock(g_mutex);
  const auto it = g_alive.find(fd);
  if (it != g_alive.end() && it->second == false) {
    // 这个编号之前被我们标记过"已关闭",中间没有任何 accept4()/socket()/
    // pipe2() 把它重新标记为"活着"——现在又被关闭一次:double close()。
    g_double_close_count.fetch_add(1);
    g_last_double_close_fd.store(fd);
  }
  g_alive[fd] = false;
}
}  // namespace close_tracker

extern "C" int close(int fd) {
  using CloseFn = int (*)(int);
  static CloseFn real = reinterpret_cast<CloseFn>(::dlsym(RTLD_NEXT, "close"));
  close_tracker::note_close(fd);
  return real(fd);
}

extern "C" int accept4(int sockfd, sockaddr* addr, socklen_t* addrlen, int flags) {
  using Accept4Fn = int (*)(int, sockaddr*, socklen_t*, int);
  static Accept4Fn real = reinterpret_cast<Accept4Fn>(::dlsym(RTLD_NEXT, "accept4"));
  const int fd = real(sockfd, addr, addrlen, flags);
  close_tracker::mark_open(fd);
  return fd;
}

extern "C" int socket(int domain, int type, int protocol) {
  using SocketFn = int (*)(int, int, int);
  static SocketFn real = reinterpret_cast<SocketFn>(::dlsym(RTLD_NEXT, "socket"));
  const int fd = real(domain, type, protocol);
  close_tracker::mark_open(fd);
  if (fd >= 0) {
    close_tracker::g_last_socket_fd.store(fd);
    if (close_tracker::g_arm_poll_fault_on_next_socket.exchange(false)) {
      close_tracker::g_poll_fault_target_fd.store(fd);
      close_tracker::g_poll_fault_armed.store(true);
    }
  }
  return fd;
}

// 拦截 poll():武装标志开着、且这次调用的某个 fd 正是目标 fd 时,不真的
// 调用内核的 poll(),直接伪造出"这个 fd 上 POLLNVAL"的结果并立刻返回——
// 只触发一次(先解除武装,再伪造结果),此后一律转发给真正的 poll(),
// 不影响同一个测试进程里其它 LocalReserver 实例的正常行为。
extern "C" int poll(pollfd* fds, nfds_t nfds, int timeout) {
  using PollFn = int (*)(pollfd*, nfds_t, int);
  static PollFn real = reinterpret_cast<PollFn>(::dlsym(RTLD_NEXT, "poll"));
  if (close_tracker::g_poll_fault_armed.load()) {
    const int target = close_tracker::g_poll_fault_target_fd.load();
    for (nfds_t i = 0; i < nfds; ++i) {
      if (fds[i].fd == target) {
        close_tracker::g_poll_fault_armed.store(false);
        for (nfds_t j = 0; j < nfds; ++j) fds[j].revents = 0;
        fds[i].revents = POLLNVAL;
        return 1;
      }
    }
  }
  return real(fds, nfds, timeout);
}

extern "C" int pipe2(int pipefd[2], int flags) {
  using Pipe2Fn = int (*)(int[2], int);
  static Pipe2Fn real = reinterpret_cast<Pipe2Fn>(::dlsym(RTLD_NEXT, "pipe2"));
  const int rc = real(pipefd, flags);
  if (rc == 0) {
    close_tracker::mark_open(pipefd[0]);
    close_tracker::mark_open(pipefd[1]);
  }
  return rc;
}

namespace {
int connect_to(int port) {
  int c = ::socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
  a.sin_port = ::htons(static_cast<uint16_t>(port));
  if (::connect(c, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0) { ::close(c); return -1; }
  return c;
}
bool wait_clients(const LocalReserver& r, size_t n) {
  for (int i = 0; i < 200; ++i) {
    if (r.client_count() >= n) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}
std::string recv_n(int fd, size_t n) {
  std::string out;
  char buf[256];
  while (out.size() < n) {
    const ssize_t k = ::recv(fd, buf, sizeof(buf), 0);
    if (k <= 0) break;
    out.append(buf, static_cast<size_t>(k));
  }
  return out;
}
}  // namespace

TEST(LocalReserver, BindsAndReportsPort) {
  LocalReserver r;
  ASSERT_TRUE(r.start(0));
  EXPECT_GT(r.bound_port(), 0);
  r.stop();
}

TEST(LocalReserver, BroadcastsToASingleClient) {
  LocalReserver r;
  ASSERT_TRUE(r.start(0));
  const int c = connect_to(r.bound_port());
  ASSERT_GE(c, 0);
  ASSERT_TRUE(wait_clients(r, 1));

  const std::string payload = "RTCM-BYTES";
  r.broadcast(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
  EXPECT_EQ(recv_n(c, payload.size()), payload);
  ::close(c);
  r.stop();
}

TEST(LocalReserver, BroadcastsToEveryConnectedClient) {
  LocalReserver r;
  ASSERT_TRUE(r.start(0));
  const int c1 = connect_to(r.bound_port());
  const int c2 = connect_to(r.bound_port());
  ASSERT_GE(c1, 0);
  ASSERT_GE(c2, 0);
  ASSERT_TRUE(wait_clients(r, 2));

  const std::string payload = "XYZ";
  r.broadcast(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
  EXPECT_EQ(recv_n(c1, 3), payload);
  EXPECT_EQ(recv_n(c2, 3), payload);
  ::close(c1);
  ::close(c2);
  r.stop();
}

TEST(LocalReserver, DropsDisconnectedClientFromCount) {
  LocalReserver r;
  ASSERT_TRUE(r.start(0));
  const int c = connect_to(r.bound_port());
  ASSERT_GE(c, 0);
  ASSERT_TRUE(wait_clients(r, 1));
  ::close(c);

  const std::string payload = "Z";
  // 对端已关闭,broadcast 时写失败 → 客户端应被摘掉
  for (int i = 0; i < 50 && r.client_count() > 0; ++i) {
    r.broadcast(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(r.client_count(), 0u);
  r.stop();
}

TEST(LocalReserver, BroadcastWithNoClientsIsANoop) {
  LocalReserver r;
  ASSERT_TRUE(r.start(0));
  const std::string payload = "nobody-listening";
  r.broadcast(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
  EXPECT_EQ(r.client_count(), 0u);
  r.stop();
}

TEST(LocalReserver, StopReleasesThePortAndIsIdempotent) {
  int port = 0;
  {
    LocalReserver r;
    ASSERT_TRUE(r.start(0));
    port = r.bound_port();
    ASSERT_GT(port, 0);
    r.stop();
    r.stop();                       // 第二次必须无害
    EXPECT_EQ(r.client_count(), 0u);
  }
  // 可观测的后果:同一端口能被重新绑定,说明上一个实例确实释放了它
  LocalReserver again;
  EXPECT_TRUE(again.start(port)) << "stop() 没释放端口,新实例绑不上";
  again.stop();
}

TEST(LocalReserver, StopWithoutStartLeavesNoBoundPort) {
  LocalReserver r;
  r.stop();
  EXPECT_EQ(r.bound_port(), -1);
  EXPECT_EQ(r.client_count(), 0u);
}

// final-fix-wave 第 2 项:accept_loop 因不可恢复错误永久退出("监听 socket
// 坏掉了")之前,LocalReserver 完全没有任何形式的日志/回调——调用方
// (rtkrcv_node 的 corr_reserver_/obs_reserver_)没有办法知道这件事发生过,
// 现场表现为"节点看起来在跑,corrections/raw_obs 却再也传不到 rtkrcv"。
// 用文件顶部的探针注入故障:武装之后,start() 建监听 socket 的那次 socket()
// 调用会同步记下这个 fd 并武装 poll() 拦截器,让 accept_loop 自己第一次
// 调用 poll() 时就看到 POLLNVAL——不依赖任何跨线程的时序假设,验证新加的
// on_fatal_error 回调确实会被调用一次。
TEST(LocalReserver, AcceptLoopFatalErrorInvokesCallback) {
  close_tracker::g_arm_poll_fault_on_next_socket.store(true);

  LocalReserver r;
  std::mutex m;
  std::condition_variable cv;
  bool fired = false;
  std::string detail;
  ASSERT_TRUE(r.start(0, "127.0.0.1", [&](const std::string& d) {
    std::lock_guard<std::mutex> lk(m);
    fired = true;
    detail = d;
    cv.notify_all();
  }));

  {
    std::unique_lock<std::mutex> lk(m);
    ASSERT_TRUE(cv.wait_for(lk, std::chrono::seconds(3), [&] { return fired; }))
        << "accept_loop 遇到不可恢复错误后必须回调通知调用方,而不是悄悄退出";
  }
  EXPECT_FALSE(detail.empty());

  // 回调触发之后,accept_loop 线程已经在退出路上(running_ 已经被置
  // false)——stop() 仍然必须能被安全调用,不能因为这次注入的故障而崩溃
  // 或挂起。
  r.stop();
}

// final-fix-wave 第 6 项(Minor 1):local_reserver.cpp 原来的 bound_port_
// 只在成功路径里被赋值,一次成功的 start() 结束运行之后(不管是正常
// stop() 还是像这里一样自己因为不可恢复错误退出),如果同一个对象后续
// 再 start() 却失败了,bound_port() 会继续报出上一次那个早已不再有任何
// 东西监听的旧端口号——这个包里已经在别处堵住的"说谎的 accessor"这一类
// 问题在这里的一个漏网之鱼。
TEST(LocalReserver, BoundPortResetsToNegativeOneAfterFailedRestart) {
  // 强制 accept_loop 自己因为不可恢复错误退出(running_ 被内部置为
  // false),但不经过 stop()——这样 bound_port_ 才会带着上一次成功的值
  // "残留"下来,才有条件复现这个 bug(stop() 本身已经会正确复位它,不能
  // 用 stop() 来制造前提条件)。用文件顶部的探针注入故障(见
  // AcceptLoopFatalErrorInvokesCallback 上面的说明),不依赖跨线程时序。
  close_tracker::g_arm_poll_fault_on_next_socket.store(true);

  LocalReserver r;
  std::mutex m;
  std::condition_variable cv;
  bool fired = false;
  ASSERT_TRUE(r.start(0, "127.0.0.1", [&](const std::string&) {
    std::lock_guard<std::mutex> lk(m);
    fired = true;
    cv.notify_all();
  }));
  const int old_port = r.bound_port();
  ASSERT_GT(old_port, 0);

  {
    std::unique_lock<std::mutex> lk(m);
    ASSERT_TRUE(cv.wait_for(lk, std::chrono::seconds(3), [&] { return fired; }));
  }
  ASSERT_EQ(r.bound_port(), old_port)
      << "先决条件:accept_loop 自己退出、不经过 stop() 时,bound_port_ 应该"
         "仍然带着上一次成功绑定的值(这是当前设计本身的行为,不是本条要测的"
         "bug)";

  // 用一个占着某个端口的裸 socket,逼这次 start() 在 bind() 阶段失败——
  // 不猜一个可能凑巧空闲的端口号。
  const int blocker = ::socket(AF_INET, SOCK_STREAM, 0);
  ASSERT_GE(blocker, 0);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
  a.sin_port = 0;
  ASSERT_EQ(::bind(blocker, reinterpret_cast<sockaddr*>(&a), sizeof(a)), 0);
  ASSERT_EQ(::listen(blocker, 1), 0);
  socklen_t len = sizeof(a);
  ASSERT_EQ(::getsockname(blocker, reinterpret_cast<sockaddr*>(&a), &len), 0);
  const int blocked_port = ::ntohs(a.sin_port);

  EXPECT_FALSE(r.start(blocked_port)) << "端口已被另一个 socket 占用,这次 start() 应该失败";
  EXPECT_EQ(r.bound_port(), -1)
      << "失败的 start() 之后,bound_port() 不应该继续报出上一次那个早已经"
         "没有任何东西在监听的旧端口号 " << old_port;

  ::close(blocker);
  r.stop();
}

// --- Fix-round-1 additions: direct coverage of broadcast()'s own drop policy ---

TEST(LocalReserver, BroadcastDropsAStalledClientWithoutBlocking) {
  LocalReserver r;
  ASSERT_TRUE(r.start(0));
  const int c = connect_to(r.bound_port());
  ASSERT_GE(c, 0);
  ASSERT_TRUE(wait_clients(r, 1));

  // 故意不读:让内核发送缓冲被灌满,逼 broadcast() 走 EAGAIN/短写的丢弃
  // 路径,而不是像 DropsDisconnectedClientFromCount 那样靠对端整个断开连接
  // 触发 accept_loop 自己的 recv()==0 检测——这里要单独证明 broadcast()
  // 本身的丢弃逻辑确实起作用,而且不会被一个不读数据的客户端拖住。
  const std::string chunk(64 * 1024, 'x');  // 64KiB,远大于典型默认发送缓冲
  const auto t0 = std::chrono::steady_clock::now();
  bool dropped = false;
  for (int i = 0; i < 2000 && !dropped; ++i) {
    r.broadcast(reinterpret_cast<const uint8_t*>(chunk.data()), chunk.size());
    if (r.client_count() == 0) dropped = true;
  }
  const auto elapsed = std::chrono::steady_clock::now() - t0;
  EXPECT_TRUE(dropped) << "写不动的客户端始终没有被摘除";
  // broadcast() 本身绝不阻塞:哪怕客户端从不读、每次都要面对 64KiB 的写,
  // 2000 次调用也应该在几秒内就把它摘掉,不应该顶到测试框架的超时量级。
  EXPECT_LT(std::chrono::duration_cast<std::chrono::seconds>(elapsed).count(), 10);
  ::close(c);
  r.stop();
}

TEST(LocalReserver, BroadcastDeliversMultipleChunksInOrderExactlyOnce) {
  LocalReserver r;
  ASSERT_TRUE(r.start(0));
  const int c = connect_to(r.bound_port());
  ASSERT_GE(c, 0);
  ASSERT_TRUE(wait_clients(r, 1));

  const std::vector<std::string> chunks = {"first-", "second-", "third"};
  std::string expected;
  for (const auto& chunk : chunks) {
    r.broadcast(reinterpret_cast<const uint8_t*>(chunk.data()), chunk.size());
    expected += chunk;
  }
  // 顺序、恰好一次、不被篡改地送达——LocalReserver 存在的唯一理由。
  EXPECT_EQ(recv_n(c, expected.size()), expected);
  ::close(c);
  r.stop();
}

// final-fix-wave 第 4 项:这条测试单独跑 13.3s,在 200-way CPU 超订(和
// 这条 review 用的环境一致)下把整个 test_local_reserver 二进制拖到 75.5s
// ——超过 ctest 默认的 60s TIMEOUT,是一次会在 merge 时把 CI 拖红的真实
// 问题,不是理论推测。而它对被 revert 的那个 bug 的检出率只有约 8%
// (概率性压力测试的通病:样本量再大,单次运行命中窗口的概率也有限)。
//
// 取舍:标成 DISABLED_,不进默认套件(ament_add_gtest 是整个二进制一次
// ctest 调用,二进制内部按 DISABLED_ 跳过比拆一个新的 ctest 目标/改
// TIMEOUT 简单,且不需要碰 CMakeLists.txt 里的 ctest 属性)。需要真的跑
// 一次深度压力回归时,手动执行:
//   ./build/gnss_bringup/test_local_reserver --gtest_also_run_disabled_tests
//     --gtest_filter='*ConcurrentBroadcastAndDisconnectNeverDoubleCloses*'
// 取而代之进入默认套件的是下面的
// StalledClientWithPollhupClosesExactlyOnce——用更小的目标场景(一个
// stall+close 的客户端,而不是 4 客户端 x 800 轮的混战)反复试验,命中率
// 更高、单次运行成本也从两位数秒降到亚秒级,足以在默认 ctest 里每次都跑。
//
// 保留这条测试本身(而不是删除):它仍然是"更大范围内有没有别的、这条
// 更小的定向测试没覆盖到的时序漏洞"这个问题的有效兜底,只是不适合作为
// 每次 merge 都要跑、还带着一个和它不匹配的 60s TIMEOUT 预算的默认用例。
//
// fix round 2: 直接复现 review 报告的场景——多个并发客户端 + 一个疯狂
// broadcast() 的"锤子"线程 + 客户端在广播过程中断线重连——用来确认死连接
// 不会同时被 broadcast() 的 EAGAIN 检测和 accept_loop 自己的
// recv()==0 检测各摘一次、导致同一个 fd 编号被 close() 两次(其中一次可能
// 落在一个刚刚复用了该编号的全新连接上)。
//
// 这是概率性的压力测试,不是单次必然触发的确定性用例——竞态本身就是
// 时序相关的。但它配合上面的 close() 追踪器是可判定失败的:一旦真的发生
// double close(),close_tracker::g_double_close_count 就会变成非零,
// 测试断言会失败,而不是仅仅寄希望于进程崩溃。fix round 1 review 的原始
// 复现方式(4 个客户端 + 锤子线程,报告说"reproduces at 150 clients within
// one round")就是这个形状;这里用更多客户端、更多轮次以提高单次运行的
// 命中概率。
TEST(LocalReserver, DISABLED_ConcurrentBroadcastAndDisconnectNeverDoubleCloses) {
  close_tracker::reset();

  LocalReserver r;
  ASSERT_TRUE(r.start(0));
  const int port = r.bound_port();

  constexpr int kClients = 4;
  constexpr int kRounds = 800;
  std::vector<int> clients(kClients, -1);
  for (int& c : clients) {
    c = connect_to(port);
    ASSERT_GE(c, 0);
  }
  ASSERT_TRUE(wait_clients(r, kClients));

  std::atomic<bool> stop_hammer{false};
  std::thread hammer([&] {
    const std::string chunk(4096, 'y');
    while (!stop_hammer.load()) {
      r.broadcast(reinterpret_cast<const uint8_t*>(chunk.data()), chunk.size());
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
  });

  // 让客户端在 broadcast() 疯狂发送的同时反复断开又重连——这正是复现
  // "同一个 fd 编号被两条独立路径各摘一次、close() 两次,且中间被
  // accept4() 复用给全新连接"的场景。
  for (int round = 0; round < kRounds; ++round) {
    for (int& c : clients) {
      if (c >= 0) {
        ::close(c);
        c = -1;
      }
    }
    // 故意不睡太久:断开和重连之间的间隙越短,越容易和 broadcast() 的
    // 发送窗口重叠,提高命中竞态的概率。
    for (int& c : clients) {
      c = connect_to(port);
    }
  }

  stop_hammer.store(true);
  hammer.join();
  for (const int c : clients) {
    if (c >= 0) ::close(c);
  }
  r.stop();

  EXPECT_EQ(close_tracker::g_double_close_count.load(), 0)
      << "double close() 命中 fd=" << close_tracker::g_last_double_close_fd.load();
}

// final-fix-wave 第 4 项:取代上面 DISABLED_
// ConcurrentBroadcastAndDisconnectNeverDoubleCloses 进入默认套件的定向
// 回归测试。不依赖"4 个客户端 x 800 轮混战"撞出这个时序窗口,而是直接
// 构造 review 描述的那个具体场景——同一个 fd 同时具备(a) broadcast() 会
// 判定"写不动"的条件(发送缓冲仍是满的/连接已经无法写入),和 (b)
// accept_loop 的 poll() 会看到的 POLLHUP(对端已经关闭)——只是仍然依赖
// 两个真实线程(本测试线程里调用的 broadcast() 和后台 accept_loop 线程)
// 在这个窗口上真正交叠,所以用多轮小规模尝试(而不是一次性的强同步)把
// 命中率推到接近 1,单轮成本远低于旧压力测试的 4 客户端/800 轮/独立锤子
// 线程,总耗时是亚秒级。
//
// 断言用的还是文件顶部的 close_tracker:一旦这个 fd 被 close() 了不止
// 一次(不管是被 broadcast() 和 accept_loop 各关一次,还是别的任何路径),
// g_double_close_count 就会非零,測試失败,而不是像"进程有没有崩"那样
// 只能抓最坏情况。
TEST(LocalReserver, StalledClientWithPollhupClosesExactlyOnce) {
  close_tracker::reset();

  LocalReserver r;
  ASSERT_TRUE(r.start(0));
  const int port = r.bound_port();
  ASSERT_GT(port, 0);

  constexpr int kTrials = 40;
  const std::string chunk(64 * 1024, 'z');  // 远大于典型默认发送缓冲

  for (int trial = 0; trial < kTrials; ++trial) {
    int c = connect_to(port);
    ASSERT_GE(c, 0) << "trial " << trial;
    ASSERT_TRUE(wait_clients(r, 1)) << "trial " << trial;

    // 灌满对端的内核接收缓冲(客户端从不 recv()),逼 broadcast() 接下来
    // 对这个 fd 的 send() 走 EAGAIN/短写的"写不动"判定路径。
    for (int i = 0; i < 8; ++i) {
      r.broadcast(reinterpret_cast<const uint8_t*>(chunk.data()), chunk.size());
    }

    // 客户端此刻关闭:服务器这一侧的 fd 立刻同时具备两个条件——
    // (1) 内核发送缓冲仍然是满的,下一次 send() 会失败(ECONNRESET 或
    //     依然 EAGAIN,两者都会让 broadcast() 判定"写不动");
    // (2) accept_loop 后台线程下一次 poll() 会在这个 fd 上看到 POLLHUP。
    ::close(c);

    // 不 sleep,立刻从本测试线程再广播一次——最大化和后台 accept_loop
    // 线程那次独立的 POLLHUP/recv() 检测重叠的概率:两条路径都会尝试
    // 认领同一个"已经死了"的 fd。
    r.broadcast(reinterpret_cast<const uint8_t*>(chunk.data()), chunk.size());

    bool cleaned_up = false;
    for (int i = 0; i < 200 && !cleaned_up; ++i) {
      if (r.client_count() == 0) cleaned_up = true;
      else std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_TRUE(cleaned_up) << "trial " << trial << ":死连接始终没有被摘除";
  }

  r.stop();

  EXPECT_EQ(close_tracker::g_double_close_count.load(), 0)
      << "double close() 命中 fd=" << close_tracker::g_last_double_close_fd.load();
}
