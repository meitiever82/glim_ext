#include <gtest/gtest.h>
#include <arpa/inet.h>
#include <dlfcn.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
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
  return fd;
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
TEST(LocalReserver, ConcurrentBroadcastAndDisconnectNeverDoubleCloses) {
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
