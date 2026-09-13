#include <gtest/gtest.h>
#include <limits>
#include <string>
#include "gnss_bringup/pos_writer_params.hpp"

using gnss_bringup::is_sane_leap_seconds;
using gnss_bringup::is_sane_utc_stamp;
using gnss_bringup::parse_pos_time_system;

// ---------- time_system ----------

TEST(PosWriterParams, AcceptsExactLiteralsOnly) {
  EXPECT_EQ(parse_pos_time_system("GPST"), gnss_core::PosTimeSystem::GPST);
  EXPECT_EQ(parse_pos_time_system("UTC"), gnss_core::PosTimeSystem::UTC);
}

TEST(PosWriterParams, RejectsAnythingElse) {
  EXPECT_EQ(parse_pos_time_system(""), std::nullopt);
  EXPECT_EQ(parse_pos_time_system("gpst"), std::nullopt) << "大小写不匹配也要拒绝,不能悄悄归一化";
  EXPECT_EQ(parse_pos_time_system("utc"), std::nullopt);
  EXPECT_EQ(parse_pos_time_system("GPS"), std::nullopt);
  EXPECT_EQ(parse_pos_time_system("UTC "), std::nullopt) << "带多余空白也拒绝";
}

// ---------- leap_seconds ----------

TEST(PosWriterParams, TypicalLeapSecondsAreSane) {
  EXPECT_TRUE(is_sane_leap_seconds(18));
  EXPECT_TRUE(is_sane_leap_seconds(0));
  EXPECT_TRUE(is_sane_leap_seconds(60));
}

TEST(PosWriterParams, NegativeOrAbsurdLeapSecondsAreRejected) {
  EXPECT_FALSE(is_sane_leap_seconds(-1));
  EXPECT_FALSE(is_sane_leap_seconds(61));
  EXPECT_FALSE(is_sane_leap_seconds(3600)) << "典型的\"错填成秒数\"输入";
}

// ---------- record 时间戳闸门 ----------

TEST(PosWriterParams, TypicalStampsAreSane) {
  EXPECT_TRUE(is_sane_utc_stamp(1789208625.0));  // 2026-09-12
  EXPECT_TRUE(is_sane_utc_stamp(gnss_bringup::kMinSaneUtcStamp));
  EXPECT_TRUE(is_sane_utc_stamp(gnss_bringup::kMaxSaneUtcStamp));
}

TEST(PosWriterParams, ZeroStampIsRejected) {
  // Task 3/4 都把 0 当"合法但离谱"放行(会落进 19700101 目录)——本任务在
  // 节点这一层加一道闸门挡住它,不让这种记录静默地建出一个 1970 目录。
  EXPECT_FALSE(is_sane_utc_stamp(0.0));
}

TEST(PosWriterParams, NegativeAndOutOfRangeStampsAreRejected) {
  EXPECT_FALSE(is_sane_utc_stamp(-3600.0));
  EXPECT_FALSE(is_sane_utc_stamp(1.0));
  EXPECT_FALSE(is_sane_utc_stamp(gnss_bringup::kMaxSaneUtcStamp + 1.0));
  EXPECT_FALSE(is_sane_utc_stamp(1e300));
  EXPECT_FALSE(is_sane_utc_stamp(-1e300));
}

TEST(PosWriterParams, NonFiniteStampsAreRejected) {
  // floor(NaN/period) 转 long long(PosDecimator::accept 内部做的事)是未
  // 定义行为,这道闸门必须在调用 accept() 之前挡住 NaN/±inf。
  EXPECT_FALSE(is_sane_utc_stamp(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(is_sane_utc_stamp(std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(is_sane_utc_stamp(-std::numeric_limits<double>::infinity()));
}
