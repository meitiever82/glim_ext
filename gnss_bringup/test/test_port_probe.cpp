#include <gtest/gtest.h>
#include "gnss_bringup/local_reserver.hpp"
#include "gnss_bringup/port_probe.hpp"
using namespace gnss_bringup;

TEST(PortProbe, DetectsAListeningPort) {
  // 自己起一个监听,确认探测得到
  LocalReserver r;
  ASSERT_TRUE(r.start(0));
  EXPECT_TRUE(is_local_port_listening(r.bound_port()));
  r.stop();
}

TEST(PortProbe, ReportsFreePortAsNotListening) {
  LocalReserver r;
  ASSERT_TRUE(r.start(0));
  const int p = r.bound_port();
  r.stop();                       // 释放后同一端口应当探测为空闲
  EXPECT_FALSE(is_local_port_listening(p));
}
