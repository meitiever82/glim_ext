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
