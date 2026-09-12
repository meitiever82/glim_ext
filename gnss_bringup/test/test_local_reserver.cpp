#include <gtest/gtest.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include "gnss_bringup/local_reserver.hpp"
using namespace gnss_bringup;

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
