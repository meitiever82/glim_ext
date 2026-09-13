#include <gtest/gtest.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>
#include "gnss_bringup/local_reserver.hpp"
#include "gnss_bringup/port_probe.hpp"
using namespace gnss_bringup;

TEST(PortProbe, DetectsAListeningPort) {
  // 自己起一个监听,确认探测得到
  LocalReserver r;
  ASSERT_TRUE(r.start(0));
  const auto outcome = probe_local_port(r.bound_port());
  EXPECT_EQ(outcome.result, PortProbeResult::kListening);
  r.stop();
}

TEST(PortProbe, ReportsFreePortAsNotListening) {
  LocalReserver r;
  ASSERT_TRUE(r.start(0));
  const int p = r.bound_port();
  r.stop();                       // 释放后同一端口应当探测为空闲
  const auto outcome = probe_local_port(p);
  EXPECT_EQ(outcome.result, PortProbeResult::kFree);
}

TEST(PortProbe, FullAcceptQueueIsStillReportedAsListening) {
  // fix round 1 的 Important 3 附带的 Minor(独立评审实测复现):一个正在
  // 监听、但 accept 队列恰好占满的孤儿——它是被遗弃的,没有人在 accept()
  // ——不能被误判成"空闲"。这正是选择 bind() 探测而不是 connect() 探测的
  // 原因:bind() 只关心这个地址:端口有没有被别的 socket 占着,不受对方
  // accept 队列状态的影响。这里手搭一个 backlog=1 的监听 socket,故意不
  // accept(),连续发起若干个连接把队列灌爆,复现"有人占着,但看起来像是
  // 没人处理"这种状态。
  const int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
  ASSERT_GE(listen_fd, 0);
  int one = 1;
  ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = 0;
  ASSERT_EQ(::inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr), 1);
  ASSERT_EQ(::bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)), 0);

  socklen_t len = sizeof(addr);
  ASSERT_EQ(::getsockname(listen_fd, reinterpret_cast<sockaddr*>(&addr), &len), 0);
  const int port = ::ntohs(addr.sin_port);
  ASSERT_EQ(::listen(listen_fd, 1), 0);  // backlog 故意开得很小,方便灌爆

  std::vector<int> clients;
  for (int i = 0; i < 8; ++i) {
    // SOCK_NONBLOCK:backlog 只有 1,队列灌满之后内核可能不会立刻完成后面
    // 这些连接的三次握手——一个阻塞 connect() 在这种状态下可能要等上一段
    // TCP 重传超时才返回(实测会让这条用例挂起到 gtest 自身都没有设置的
    // 上限)。非阻塞 connect() 立刻返回 EINPROGRESS,不关心它最终有没有
    // 真正建立起来,只是要把 accept 队列先占住。
    const int c = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
    ASSERT_GE(c, 0);
    (void)::connect(c, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    clients.push_back(c);
  }

  const auto outcome = probe_local_port(port);
  EXPECT_EQ(outcome.result, PortProbeResult::kListening)
      << "accept 队列占满不应该被误判成端口空闲";

  for (const int c : clients) ::close(c);
  ::close(listen_fd);
}
