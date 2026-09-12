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
#include <vector>
#include "gnss_bringup/tcp_stream.hpp"
using namespace gnss_bringup;

// final-fix-wave 第 2 项回归测试用的探针。
//
// 最初的设计是:等 run_server() 的监听 socket 建好之后,从测试线程直接把
// 那个 fd close() 掉,指望 accept 循环下一次 poll() 看到 POLLNVAL。用一个
// 独立的最小复现程序验证过这个假设是错的:在 Linux 上,一个线程已经阻塞在
// poll(fd, ..., -1) 里之后,另一个线程 close() 掉同一个 fd **不会**唤醒
// 那次 poll() 调用(POSIX 本身也没有承诺这个行为)——会一直阻塞到超时,
// 测试因此变成了一次不稳定的、依赖谁先被调度的竞态(实测在部分环境下
// 3 秒内根本等不到回调)。
//
// 改成拦截 poll() 本身来注入故障,并且把"武装"这个动作放进 socket() 的
// 拦截器里、在返回给调用方(run_server() 自己所在的那个线程)之前就同步
// 完成——这样"武装"和"这个线程自己接下来第一次调用 poll()"之间是同一个
// 线程上的顺序关系(happens-before),不再是两个线程之间的竞态。run_server()
// 建监听 socket 时唯一会带 SOCK_NONBLOCK 标志调用 socket() 的地方(run_client()
// 的客户端 socket 是先不带这个标志、再用 fcntl() 补非阻塞的,现场就能分辨)。
namespace tcp_stream_test_probe {
std::atomic<int> g_last_nonblock_socket_fd{-1};
// 测试在调用 s.start() 之前把这个设成 true,表示"下一个带 SOCK_NONBLOCK
// 标志的 socket() 调用,就是要注入故障的目标"——由 socket() 的拦截器自己
// 同步完成"记录 fd + 武装 poll 故障"这两步,不依赖任何跨线程的时序假设。
std::atomic<bool> g_arm_poll_fault_on_next_nonblock_socket{false};
std::atomic<int> g_poll_fault_target_fd{-1};
std::atomic<bool> g_poll_fault_armed{false};
}  // namespace tcp_stream_test_probe

extern "C" int socket(int domain, int type, int protocol) {
  using SocketFn = int (*)(int, int, int);
  static SocketFn real = reinterpret_cast<SocketFn>(::dlsym(RTLD_NEXT, "socket"));
  const int fd = real(domain, type, protocol);
  if (fd >= 0 && (type & SOCK_NONBLOCK)) {
    tcp_stream_test_probe::g_last_nonblock_socket_fd.store(fd);
    if (tcp_stream_test_probe::g_arm_poll_fault_on_next_nonblock_socket.exchange(false)) {
      tcp_stream_test_probe::g_poll_fault_target_fd.store(fd);
      tcp_stream_test_probe::g_poll_fault_armed.store(true);
    }
  }
  return fd;
}

// 拦截 poll():当武装标志开着、且这次调用的某个 fd 正是目标 fd 时,不真的
// 调用内核的 poll(),直接伪造出"这个 fd 上 POLLNVAL"的结果并立刻返回——
// 只触发一次(先解除武装,再伪造结果),之后的调用一律照常转发给真正的
// poll(),不影响同一个测试进程里其它 TcpStream 实例的正常行为。
extern "C" int poll(pollfd* fds, nfds_t nfds, int timeout) {
  using PollFn = int (*)(pollfd*, nfds_t, int);
  static PollFn real = reinterpret_cast<PollFn>(::dlsym(RTLD_NEXT, "poll"));
  if (tcp_stream_test_probe::g_poll_fault_armed.load()) {
    const int target = tcp_stream_test_probe::g_poll_fault_target_fd.load();
    for (nfds_t i = 0; i < nfds; ++i) {
      if (fds[i].fd == target) {
        tcp_stream_test_probe::g_poll_fault_armed.store(false);
        for (nfds_t j = 0; j < nfds; ++j) fds[j].revents = 0;
        fds[i].revents = POLLNVAL;
        return 1;
      }
    }
  }
  return real(fds, nfds, timeout);
}

namespace {

// 收集回调数据的小工具,带"等到收够 n 字节"的超时等待
class Sink {
public:
  void push(const uint8_t* d, size_t n) {
    std::lock_guard<std::mutex> lk(m_);
    bytes_.insert(bytes_.end(), d, d + n);
    cv_.notify_all();
  }
  bool wait_for_bytes(size_t n, std::chrono::milliseconds to) {
    std::unique_lock<std::mutex> lk(m_);
    return cv_.wait_for(lk, to, [&] { return bytes_.size() >= n; });
  }
  std::string str() {
    std::lock_guard<std::mutex> lk(m_);
    return std::string(bytes_.begin(), bytes_.end());
  }
private:
  std::mutex m_;
  std::condition_variable cv_;
  std::vector<uint8_t> bytes_;
};

// 一个只接受一个连接、发一段字节然后按需关闭的最小测试服务端
class TestServer {
public:
  int start() {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
    a.sin_port = 0;
    ::bind(fd_, reinterpret_cast<sockaddr*>(&a), sizeof(a));
    ::listen(fd_, 1);
    socklen_t len = sizeof(a);
    ::getsockname(fd_, reinterpret_cast<sockaddr*>(&a), &len);
    return ::ntohs(a.sin_port);
  }
  void accept_and_send(const std::string& payload) {
    // 阻塞 accept() 在客户端从不连接的回归场景下会把整个 gtest 二进制挂起
    // (退化为 CI 里的 "timed out",而不是一条具名失败)。先 poll 一段有限
    // 时间,超时就报一条明确失败并返回。
    pollfd pfd{fd_, POLLIN, 0};
    const int pr = ::poll(&pfd, 1, 3000);
    if (pr <= 0) {
      ADD_FAILURE() << "no incoming connection within 3s (poll returned " << pr << ")";
      return;
    }
    conn_ = ::accept(fd_, nullptr, nullptr);
    ::send(conn_, payload.data(), payload.size(), 0);
  }
  void close_conn() { if (conn_ >= 0) { ::close(conn_); conn_ = -1; } }
  ~TestServer() { close_conn(); if (fd_ >= 0) ::close(fd_); }
private:
  int fd_ = -1, conn_ = -1;
};

// 一直 accept 并立刻关闭每个连接的测试服务端。用于 ClientBackoffGrowsButIsCappedByMax
// ——早期版本连的是一个连不上的端口,状态回调只在跳变时上报一次
// "disconnected" 之后就再也不跳变,断言退化成恒真的 EXPECT_GE(...,1),
// EXPECT_LE 那一半永远不可能失败(参见 task-2-report.md 里 fix-round-1 的
// 记录)。改成"接上又立刻断"的服务端,让每一次重试都产生一对真实的
// connected->disconnected 跳变,EXPECT_LE 才是在真的约束重试频率。
class ImmediateCloseServer {
public:
  int start() {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
    a.sin_port = 0;
    ::bind(fd_, reinterpret_cast<sockaddr*>(&a), sizeof(a));
    ::listen(fd_, 16);
    socklen_t len = sizeof(a);
    ::getsockname(fd_, reinterpret_cast<sockaddr*>(&a), &len);
    running_.store(true);
    worker_ = std::thread([this] {
      while (running_.load()) {
        pollfd pfd{fd_, POLLIN, 0};
        const int pr = ::poll(&pfd, 1, 10);
        if (pr <= 0) continue;
        const int c = ::accept(fd_, nullptr, nullptr);
        if (c >= 0) ::close(c);  // 立刻关闭,制造"连上又断"的跳变
      }
    });
    return ::ntohs(a.sin_port);
  }
  ~ImmediateCloseServer() {
    running_.store(false);
    if (worker_.joinable()) worker_.join();
    if (fd_ >= 0) ::close(fd_);
  }
private:
  int fd_ = -1;
  std::atomic<bool> running_{false};
  std::thread worker_;
};

}  // namespace

TEST(TcpStream, ClientModeDeliversBytesFromServer) {
  TestServer srv;
  const int port = srv.start();

  Sink sink;
  TcpStreamConfig cfg;
  cfg.host = "127.0.0.1";
  cfg.port = port;
  TcpStream s(cfg, [&](const uint8_t* d, size_t n) { sink.push(d, n); });
  s.start();

  srv.accept_and_send("HELLO-RTCM");
  EXPECT_TRUE(sink.wait_for_bytes(10, std::chrono::seconds(3)));
  EXPECT_EQ(sink.str(), "HELLO-RTCM");
  s.stop();
}

TEST(TcpStream, ReportsConnectedThenDisconnected) {
  TestServer srv;
  const int port = srv.start();

  std::mutex m;
  std::condition_variable cv;
  std::vector<bool> states;
  std::vector<bool> terminals;
  TcpStreamConfig cfg;
  cfg.port = port;
  cfg.initial_backoff_s = 0.05;
  TcpStream s(cfg, [](const uint8_t*, size_t) {},
              [&](bool connected, const std::string&, bool terminal) {
                std::lock_guard<std::mutex> lk(m);
                states.push_back(connected);
                terminals.push_back(terminal);
                cv.notify_all();
              });
  s.start();
  srv.accept_and_send("x");
  srv.close_conn();

  std::unique_lock<std::mutex> lk(m);
  ASSERT_TRUE(cv.wait_for(lk, std::chrono::seconds(3),
                          [&] { return states.size() >= 2; }));
  EXPECT_TRUE(states[0]) << "先报 connected";
  EXPECT_FALSE(states[1]) << "对端关闭后报 disconnected";
  // final-fix-wave 第 2 项:普通的断线重连(对端主动关闭连接)不是 worker
  // 永久退出——client 侧还会继续退避重连——两次回调都不应该是 terminal,
  // 调用方据此仍然按 INFO 记录,不能被这次改动误升级成 ERROR。
  EXPECT_FALSE(terminals[0]);
  EXPECT_FALSE(terminals[1]) << "对端关闭连接是可恢复的重连场景,不是终态";
  lk.unlock();
  s.stop();
}

TEST(TcpStream, StopReturnsPromptlyWhenNeverConnected) {
  // 连一个没人监听的端口:start 后线程应当先尝试连接、失败、再进入退避循环。
  // 先等到 worker 真正报告过一次"未连通"(即已经在退避里睡眠),再开始计时
  // 调用 stop() —— 这样测试才能证明 stop() 是靠打断退避睡眠及时返回的,
  // 而不是碰巧在 worker 触达 wait_backoff() 之前就已经 running_==false 退出了
  // (对着空桩这条用例会 0ms 通过,无法区分真实中断和什么都没做)。
  Sink sink;
  std::mutex m;
  std::condition_variable cv;
  bool entered_backoff = false;
  TcpStreamConfig cfg;
  cfg.port = 1;                      // 特权端口,必然连不上
  cfg.initial_backoff_s = 10.0;      // 故意设很长,验证 stop 不是靠等退避结束
  TcpStream s(cfg, [&](const uint8_t* d, size_t n) { sink.push(d, n); },
              [&](bool connected, const std::string&, bool) {
                if (!connected) {
                  std::lock_guard<std::mutex> lk(m);
                  entered_backoff = true;
                  cv.notify_all();
                }
              });
  s.start();
  {
    std::unique_lock<std::mutex> lk(m);
    ASSERT_TRUE(cv.wait_for(lk, std::chrono::seconds(3), [&] { return entered_backoff; }))
        << "worker 从未报告过连接失败,说明它还没进入(或根本没有实现)退避等待";
  }
  const auto t0 = std::chrono::steady_clock::now();
  s.stop();
  const auto dt = std::chrono::steady_clock::now() - t0;
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(dt).count(), 1000);
}

TEST(TcpStream, NumericHostSkipsDnsAndStopStaysPrompt) {
  // 回归防护,专门盯着 run_client() 里 getaddrinfo() 的 AI_NUMERICHOST 快路径:
  // cfg.host 在这里被显式设成 IPv4 字面量(不依赖其它用例"默认值恰好也是
  // 数字地址"这件事),端口连不上、退避设得很长。如果这条快路径被改掉、
  // 数字地址也退回普通(可按主机名走 DNS 的)getaddrinfo() 调用,这条用例本身
  // 不会变慢——本机解析 "127.0.0.1" 不会真的发 DNS 请求——但它把"数字地址
  // 必须免于解析阻塞"这一点钉在测试里,而不是隐含在别处。
  // 说明一个已知的覆盖缺口:真正验证"主机名+resolver 卡住时 stop() 也不挂死"
  // 需要一个可控的慢/假 resolver(mock 掉 getaddrinfo 或用 LD_PRELOAD 之类
  // 的手段),依赖这台机器自己的 DNS 行为写不出诚实的测试,因此没有写—— 那条
  // 残留阻塞路径目前只能靠代码走查确认。
  Sink sink;
  std::mutex m;
  std::condition_variable cv;
  bool entered_backoff = false;
  TcpStreamConfig cfg;
  cfg.host = "127.0.0.1";             // 显式数字地址,不依赖默认值
  cfg.port = 1;                       // 特权端口,必然连不上
  cfg.initial_backoff_s = 10.0;       // 故意设很长,验证 stop 不是靠等退避结束
  TcpStream s(cfg, [&](const uint8_t* d, size_t n) { sink.push(d, n); },
              [&](bool connected, const std::string&, bool) {
                if (!connected) {
                  std::lock_guard<std::mutex> lk(m);
                  entered_backoff = true;
                  cv.notify_all();
                }
              });
  s.start();
  {
    std::unique_lock<std::mutex> lk(m);
    ASSERT_TRUE(cv.wait_for(lk, std::chrono::seconds(3), [&] { return entered_backoff; }))
        << "worker 从未报告过连接失败,说明它还没进入(或根本没有实现)退避等待";
  }
  const auto t0 = std::chrono::steady_clock::now();
  s.stop();
  const auto dt = std::chrono::steady_clock::now() - t0;
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(dt).count(), 1000)
      << "数字地址不应该走到任何可能阻塞的解析路径";
}

TEST(TcpStream, DestructorReleasesTheListeningSocket) {
  // 析构必须真的把线程 join 掉、把 socket 关掉,而不只是"没崩"。
  // 可观测的后果:析构后那个端口不再接受连接。
  int port = -1;
  {
    Sink sink;
    TcpStreamConfig cfg;
    cfg.listen = true;
    cfg.port = 0;
    TcpStream s(cfg, [&](const uint8_t* d, size_t n) { sink.push(d, n); });
    s.start();
    for (int i = 0; i < 100 && port <= 0; ++i) {
      port = s.bound_port();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(port, 0);
  }   // 无显式 stop(),只靠析构

  int c = ::socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
  a.sin_port = ::htons(static_cast<uint16_t>(port));
  const int rc = ::connect(c, reinterpret_cast<sockaddr*>(&a), sizeof(a));
  ::close(c);
  EXPECT_NE(rc, 0) << "析构后端口仍在监听,说明 socket 没被关掉";
}

TEST(TcpStream, ListenModeAcceptsPeerAndDeliversBytes) {
  Sink sink;
  TcpStreamConfig cfg;
  cfg.listen = true;
  cfg.port = 0;                       // 内核选端口
  TcpStream s(cfg, [&](const uint8_t* d, size_t n) { sink.push(d, n); });
  s.start();

  // 等绑定完成
  int port = -1;
  for (int i = 0; i < 100 && port <= 0; ++i) {
    port = s.bound_port();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GT(port, 0) << "监听模式必须报出实际绑定端口";

  int c = ::socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
  a.sin_port = ::htons(static_cast<uint16_t>(port));
  ASSERT_EQ(::connect(c, reinterpret_cast<sockaddr*>(&a), sizeof(a)), 0);
  const std::string payload = "PUSHED-RTCM";
  ::send(c, payload.data(), payload.size(), 0);

  EXPECT_TRUE(sink.wait_for_bytes(payload.size(), std::chrono::seconds(3)));
  EXPECT_EQ(sink.str(), payload);
  ::close(c);
  s.stop();
}

TEST(TcpStream, ListenModeAcceptsASecondPeerAfterFirstDisconnects) {
  Sink sink;
  TcpStreamConfig cfg;
  cfg.listen = true;
  cfg.port = 0;
  TcpStream s(cfg, [&](const uint8_t* d, size_t n) { sink.push(d, n); });
  s.start();
  int port = -1;
  for (int i = 0; i < 100 && port <= 0; ++i) {
    port = s.bound_port();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GT(port, 0);

  auto connect_send_close = [&](const std::string& payload) {
    int c = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
    a.sin_port = ::htons(static_cast<uint16_t>(port));
    ASSERT_EQ(::connect(c, reinterpret_cast<sockaddr*>(&a), sizeof(a)), 0);
    ::send(c, payload.data(), payload.size(), 0);
    ::close(c);
  };
  connect_send_close("AAA");
  EXPECT_TRUE(sink.wait_for_bytes(3, std::chrono::seconds(3)));
  connect_send_close("BBB");
  EXPECT_TRUE(sink.wait_for_bytes(6, std::chrono::seconds(3)))
      << "第一个对端断开后必须继续接受新连接,而不是退出";
  EXPECT_EQ(sink.str(), "AAABBB");
  s.stop();
}

TEST(TcpStream, ClientBackoffGrowsButIsCappedByMax) {
  // 连一个"接上就立刻关"的服务端(见 ImmediateCloseServer 上的注释)+
  // 很小的 max_backoff:每次重试都会产生一对 connected->disconnected 跳变,
  // 固定时间窗内跳变次数应受上限约束——既不会退化成忙等(次数爆炸),
  // 也不会一次就放弃(次数为 0)。
  ImmediateCloseServer srv;
  const int port = srv.start();

  std::atomic<int> disconnects{0};
  TcpStreamConfig cfg;
  cfg.port = port;
  cfg.initial_backoff_s = 0.05;
  cfg.max_backoff_s = 0.1;
  TcpStream s(cfg, [](const uint8_t*, size_t) {},
              [&](bool connected, const std::string&, bool) { if (!connected) ++disconnects; });
  s.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  s.stop();
  EXPECT_GE(disconnects.load(), 1) << "必须持续重试";
  EXPECT_LE(disconnects.load(), 20) << "必须退避,不能忙等";
}

TEST(TcpStream, ListenModeReportsBindFailureAndLeavesBoundPortNegative) {
  // 绑定失败必须经 on_state_ 报出来,而不是像 Task 1 遗留的最小实现那样
  // 悄悄地把 bound_port() 停在 -1、不给任何理由。用 TEST-NET-1
  // (RFC 5737,192.0.2.0/24)里的一个地址触发 bind() 的 EADDRNOTAVAIL——
  // 这个网段保留给文档示例,本机不会真的配置到网卡上。
  // 这同时也验证了 cfg_.host 确实传到了 bind() 里而不是被忽略:如果实现
  // 退回硬编码 INADDR_LOOPBACK,这次 bind 反而会成功,下面两条断言都会
  // 失败,而不是被这条测试放过。
  std::mutex m;
  std::condition_variable cv;
  bool got_failure = false;
  bool terminal_flag = false;
  std::string detail;
  TcpStreamConfig cfg;
  cfg.listen = true;
  cfg.host = "192.0.2.1";
  cfg.port = 0;
  TcpStream s(cfg, [](const uint8_t*, size_t) {},
              [&](bool connected, const std::string& d, bool terminal) {
                if (!connected) {
                  std::lock_guard<std::mutex> lk(m);
                  got_failure = true;
                  terminal_flag = terminal;
                  detail = d;
                  cv.notify_all();
                }
              });
  s.start();
  {
    std::unique_lock<std::mutex> lk(m);
    ASSERT_TRUE(cv.wait_for(lk, std::chrono::seconds(3), [&] { return got_failure; }))
        << "绑定失败必须经 on_state_ 报出来";
  }
  EXPECT_NE(detail.find("bind"), std::string::npos) << "detail=" << detail;
  // final-fix-wave 第 2 项:bind() 失败之后 worker 线程直接返回,不会有任何
  // 重试——这是终态,调用方必须能区分出来并记成 ERROR,而不是和普通的
  // 断线重连一样淹没在 INFO 里。
  EXPECT_TRUE(terminal_flag) << "bind() 失败是 worker 的终态,必须标记 terminal=true";
  s.stop();
  EXPECT_EQ(s.bound_port(), -1) << "绑定失败后 bound_port() 不应该变正";
}

TEST(TcpStream, ListenSocketErrorIsTerminalAndAllowsRestart) {
  // final-fix-wave 第 2 项回归测试:accept 循环里 poll() 报出监听 socket
  // 本身坏掉(POLLERR/POLLNVAL)之前,run_server() 只 report() 却忘了
  // running_.store(false)——对象因此会一直汇报"自己在跑",而工作线程早已
  // 退出;唯一的补救手段(重新调用 start())也会被 start() 顶部的
  // running_.exchange(true) 直接短路掉(误判"已经在跑,忽略"),永远拿不到
  // 一个新线程/新端口。这是本项修复的核心断言:running_ 必须被正确置回
  // false,重新 start() 之后必须真的又绑上了一个新端口。
  //
  // 用文件顶部的探针注入故障:武装之后,run_server() 建监听 socket 的那次
  // socket() 调用(唯一带 SOCK_NONBLOCK 标志的地方)会同步记下这个 fd 并
  // 武装 poll() 拦截器——这个动作和 run_server() 自己接下来第一次调用
  // poll() 发生在同一个线程上,不是跨线程竞态。
  tcp_stream_test_probe::g_arm_poll_fault_on_next_nonblock_socket.store(true);

  TcpStreamConfig cfg;
  cfg.listen = true;
  cfg.port = 0;

  std::mutex m;
  std::condition_variable cv;
  bool got_terminal = false;
  TcpStream s(cfg, [](const uint8_t*, size_t) {},
              [&](bool connected, const std::string&, bool terminal) {
                if (!connected && terminal) {
                  std::lock_guard<std::mutex> lk(m);
                  got_terminal = true;
                  cv.notify_all();
                }
              });
  s.start();

  {
    std::unique_lock<std::mutex> lk(m);
    ASSERT_TRUE(cv.wait_for(lk, std::chrono::seconds(3), [&] { return got_terminal; }))
        << "监听 socket 坏掉之后必须以 terminal=true 上报";
  }

  // 给 worker 线程一点时间真正跑到函数返回(report() 之后还有
  // bound_port_.store(-1) 这一步)。
  for (int i = 0; i < 100 && s.bound_port() != -1; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(s.bound_port(), -1);

  // 核心回归断言:running_ 必须已经被正确置回 false——不然这次 start()
  // 会被 running_.exchange(true) 直接短路,永远拿不到新线程、新端口也会
  // 一直停在 -1。
  s.start();
  int new_port = -1;
  for (int i = 0; i < 100 && new_port <= 0; ++i) {
    new_port = s.bound_port();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_GT(new_port, 0) << "poll()/监听 socket 出错后 running_ 没有被正确置回 false,"
                            "导致重新 start() 失败——对象卡在一个已经没有工作线程、"
                            "却仍然自称在跑的状态";
  s.stop();
}
