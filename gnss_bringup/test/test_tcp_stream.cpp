#include <gtest/gtest.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
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
  // 连一个没人监听的端口:start 后线程在退避循环里,stop 必须能及时打断
  Sink sink;
  TcpStreamConfig cfg;
  cfg.port = 1;                      // 特权端口,必然连不上
  cfg.initial_backoff_s = 10.0;      // 故意设很长,验证 stop 不是靠等退避结束
  TcpStream s(cfg, [&](const uint8_t* d, size_t n) { sink.push(d, n); });
  s.start();
  const auto t0 = std::chrono::steady_clock::now();
  s.stop();
  const auto dt = std::chrono::steady_clock::now() - t0;
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(dt).count(), 1000);
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
