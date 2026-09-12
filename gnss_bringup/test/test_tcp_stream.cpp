#include <gtest/gtest.h>
#include <arpa/inet.h>
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
  TcpStreamConfig cfg;
  cfg.port = port;
  cfg.initial_backoff_s = 0.05;
  TcpStream s(cfg, [](const uint8_t*, size_t) {},
              [&](bool connected, const std::string&) {
                std::lock_guard<std::mutex> lk(m);
                states.push_back(connected);
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
              [&](bool connected, const std::string&) {
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
              [&](bool connected, const std::string&) {
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
  // 连不上的端口 + 很小的 max_backoff:在固定时间窗内断开回调的次数应受上限约束,
  // 既不会退化成忙等(次数爆炸),也不会一次就放弃(次数为 0)。
  std::atomic<int> disconnects{0};
  TcpStreamConfig cfg;
  cfg.port = 1;
  cfg.initial_backoff_s = 0.05;
  cfg.max_backoff_s = 0.1;
  TcpStream s(cfg, [](const uint8_t*, size_t) {},
              [&](bool connected, const std::string&) { if (!connected) ++disconnects; });
  s.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(600));
  s.stop();
  EXPECT_GE(disconnects.load(), 1) << "必须持续重试";
  EXPECT_LE(disconnects.load(), 20) << "必须退避,不能忙等";
}
