#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>
#include "gnss_bringup/rtcm_bridge_params.hpp"

using gnss_bringup::find_duplicate_stream_name;
using gnss_bringup::is_positive_finite_backoff_seconds;
using gnss_bringup::is_valid_port;
using gnss_bringup::is_valid_port_for_direction;

// is_valid_port: 合法范围 [0, 65535]。0 在监听模式下表示"由内核选择"(测试用)。
TEST(IsValidPort, RejectsNegative) {
  EXPECT_FALSE(is_valid_port(-1));
}
TEST(IsValidPort, AcceptsZero) {
  EXPECT_TRUE(is_valid_port(0));
}
TEST(IsValidPort, AcceptsOne) {
  EXPECT_TRUE(is_valid_port(1));
}
TEST(IsValidPort, AcceptsMaxValidPort) {
  EXPECT_TRUE(is_valid_port(65535));
}
TEST(IsValidPort, RejectsJustAboveMax) {
  EXPECT_FALSE(is_valid_port(65536));
}
TEST(IsValidPort, RejectsFarAboveMax) {
  // 复现 reviewer 报告的 99999:htons(static_cast<uint16_t>(99999)) 会静默截断为 34463。
  EXPECT_FALSE(is_valid_port(99999));
}

// is_valid_port_for_direction: final-fix-wave 第 1 项——0 只在 listen 方向
// 合法,connect 方向(listen=false)必须拒绝 0,否则会一路传到
// connect(...:0),既不报错也永远连不上,worker 只会无限退避重试
// ("dials nothing, silently, forever")。覆盖 rtcm_bridge_node.cpp 里
// listen=false 流的端口校验,以及 rtkrcv_node.cpp 里 sol_port(客户端,
// 连 rtkrcv 的 outstr1)的校验——两者都复用这同一个谓词。
TEST(IsValidPortForDirection, AcceptsZeroWhenListening) {
  EXPECT_TRUE(is_valid_port_for_direction(0, /*listen=*/true));
}
TEST(IsValidPortForDirection, RejectsZeroWhenConnecting) {
  // 复现 reviewer 报告的两个场景:rtcm_bridge 的 `-p b.port:=0
  // -p b.listen:=false`,以及 rtkrcv_node 的 `-p sol_port:=0`——两者都是
  // "connect 方向的端口是 0",必须在这里被拒绝。
  EXPECT_FALSE(is_valid_port_for_direction(0, /*listen=*/false));
}
TEST(IsValidPortForDirection, AcceptsOrdinaryPortRegardlessOfDirection) {
  EXPECT_TRUE(is_valid_port_for_direction(15031, /*listen=*/true));
  EXPECT_TRUE(is_valid_port_for_direction(15031, /*listen=*/false));
}
TEST(IsValidPortForDirection, StillRejectsOutOfRangeRegardlessOfDirection) {
  EXPECT_FALSE(is_valid_port_for_direction(-1, /*listen=*/true));
  EXPECT_FALSE(is_valid_port_for_direction(-1, /*listen=*/false));
  EXPECT_FALSE(is_valid_port_for_direction(99999, /*listen=*/true));
  EXPECT_FALSE(is_valid_port_for_direction(99999, /*listen=*/false));
}
TEST(IsValidPortForDirection, AcceptsMaxValidPortRegardlessOfDirection) {
  EXPECT_TRUE(is_valid_port_for_direction(65535, /*listen=*/true));
  EXPECT_TRUE(is_valid_port_for_direction(65535, /*listen=*/false));
}

// find_duplicate_stream_name: streams 列表去重检测,用于在 declare_parameter 抛
// ParameterAlreadyDeclaredException 之前给出清晰诊断。
TEST(FindDuplicateStreamName, EmptyListHasNoDuplicate) {
  EXPECT_EQ(find_duplicate_stream_name({}), std::nullopt);
}
TEST(FindDuplicateStreamName, NoDuplicatesReturnsNullopt) {
  EXPECT_EQ(find_duplicate_stream_name({"rtcm_corrections", "raw_obs"}), std::nullopt);
}
TEST(FindDuplicateStreamName, OneDuplicateIsReported) {
  const auto dup = find_duplicate_stream_name({"a", "b", "a"});
  ASSERT_TRUE(dup.has_value());
  EXPECT_EQ(*dup, "a");
}

// is_positive_finite_backoff_seconds: 用于 initial_backoff_s/max_backoff_s/idle_timeout_s。
// 复现 review round 2 的 Important 1:idle_timeout_s<=0 会被 TcpStream::pump()
// 当成"彻底关闭空闲检测"的哨兵值(poll(..., -1) 永久阻塞),必须在这里挡住。
TEST(IsPositiveFiniteSeconds, RejectsZero) {
  EXPECT_FALSE(is_positive_finite_backoff_seconds(0.0));
}
TEST(IsPositiveFiniteSeconds, RejectsNegative) {
  EXPECT_FALSE(is_positive_finite_backoff_seconds(-1.0));
}
TEST(IsPositiveFiniteSeconds, RejectsNaN) {
  EXPECT_FALSE(is_positive_finite_backoff_seconds(std::nan("")));
}
TEST(IsPositiveFiniteSeconds, RejectsInfinity) {
  EXPECT_FALSE(is_positive_finite_backoff_seconds(std::numeric_limits<double>::infinity()));
}
TEST(IsPositiveFiniteSeconds, AcceptsOrdinaryPositiveValue) {
  EXPECT_TRUE(is_positive_finite_backoff_seconds(1.0));
}
TEST(IsPositiveFiniteSeconds, AcceptsSmallPositiveValue) {
  EXPECT_TRUE(is_positive_finite_backoff_seconds(0.001));
}
