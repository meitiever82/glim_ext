#include <gtest/gtest.h>
#include "gnss_bringup/sol_stream_log_gate.hpp"
using gnss_bringup::SolStreamLogGate;

TEST(SolStreamLogGate, FirstConnectIsPrinted) {
  SolStreamLogGate g;
  EXPECT_TRUE(g.on_status(true, "connected to 127.0.0.1:15020"));
}

TEST(SolStreamLogGate, OnlyTheFirstIdleCycleIsPrinted) {
  // 隧道里 rtkrcv 长时间没有解算输出:TcpStream 每 sol_idle_timeout_s 报一次
  // idle timeout 并立刻重连。只打第一次,后面整夜的重复都降级。
  SolStreamLogGate g;
  g.on_status(true, "connected to 127.0.0.1:15020");
  EXPECT_TRUE(g.on_status(false, "idle timeout"));
  for (int i = 0; i < 5; ++i) {
    EXPECT_FALSE(g.on_status(true, "connected to 127.0.0.1:15020"));
    EXPECT_FALSE(g.on_status(false, "idle timeout"));
  }
  EXPECT_TRUE(g.quiet());
}

TEST(SolStreamLogGate, ASolutionLineEndsTheQuietPeriod) {
  SolStreamLogGate g;
  g.on_status(false, "idle timeout");
  g.on_solution_line();
  EXPECT_FALSE(g.quiet());
  EXPECT_TRUE(g.on_status(false, "idle timeout")) << "解算恢复之后再次空闲,要重新提示一次";
}

TEST(SolStreamLogGate, PeerCloseIsAlwaysPrintedAndEndsTheQuietPeriod) {
  // 对端关闭说明 rtkrcv 本身退出/重启了,不能被空闲安静期吞掉
  SolStreamLogGate g;
  g.on_status(false, "idle timeout");
  EXPECT_TRUE(g.on_status(false, "peer closed connection"));
  EXPECT_FALSE(g.quiet());
  EXPECT_TRUE(g.on_status(true, "connected to 127.0.0.1:15020"));
}

TEST(SolStreamLogGate, ConnectFailuresAreAlwaysPrinted) {
  SolStreamLogGate g;
  g.on_status(false, "idle timeout");
  EXPECT_TRUE(g.on_status(false, "connect() failed: Connection refused"));
}
