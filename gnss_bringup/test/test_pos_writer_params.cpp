#include <gtest/gtest.h>
#include <limits>
#include <string>
#include "gnss_bringup/pos_writer_params.hpp"

using gnss_bringup::classify_utc_stamp;
using gnss_bringup::describe_stamp_sanity;
using gnss_bringup::is_sane_leap_seconds;
using gnss_bringup::is_sane_period_seconds;
using gnss_bringup::is_sane_utc_stamp;
using gnss_bringup::parse_pos_time_system;
using gnss_bringup::StampSanity;

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

// ---------- 时间戳分类(round 2 review 的 Minor:丢弃原因不能一律说成
// "字段缺失",落在合理区间之外但既非 0 也非非有限数的时间戳要分开描述) ----------

TEST(PosWriterParams, ClassifiesInRangeStampAsOk) {
  EXPECT_EQ(classify_utc_stamp(1789208625.0), StampSanity::kOk);
}

TEST(PosWriterParams, ClassifiesZeroAndNearZeroAsNearEpoch) {
  EXPECT_EQ(classify_utc_stamp(0.0), StampSanity::kNearEpoch);
  EXPECT_EQ(classify_utc_stamp(0.5), StampSanity::kNearEpoch);
  EXPECT_EQ(classify_utc_stamp(-0.5), StampSanity::kNearEpoch);
}

TEST(PosWriterParams, ClassifiesFiniteButImplausibleStampAsOutOfRangeNotNearEpoch) {
  // 1990-01-01T00:00:00Z——字段本身有值(不是 0,也不接近 0),只是不像真实
  // 的 GNSS 解算/接收时刻。必须分类成 kOutOfRange,不能和"字段缺失"
  // (kNearEpoch)混为一谈,否则日志会误导操作人员去查错的地方。
  EXPECT_EQ(classify_utc_stamp(631152000.0), StampSanity::kOutOfRange);
  EXPECT_EQ(classify_utc_stamp(-3600.0), StampSanity::kOutOfRange);
  EXPECT_EQ(classify_utc_stamp(gnss_bringup::kMaxSaneUtcStamp + 1.0), StampSanity::kOutOfRange);
}

TEST(PosWriterParams, ClassifiesNonFiniteSeparatelyFromOutOfRange) {
  EXPECT_EQ(classify_utc_stamp(std::numeric_limits<double>::quiet_NaN()), StampSanity::kNonFinite);
  EXPECT_EQ(classify_utc_stamp(std::numeric_limits<double>::infinity()), StampSanity::kNonFinite);
}

TEST(PosWriterParams, DescribeStampSanityCoversEveryEnumerator) {
  // 每个分类都要有一条非空、彼此不同的说明文案——否则新增分类时很容易漏掉
  // 对应的 case,switch 会静默落到 default。
  const StampSanity all[] = {StampSanity::kOk, StampSanity::kNonFinite, StampSanity::kNearEpoch,
                              StampSanity::kOutOfRange};
  for (auto s : all) {
    EXPECT_STRNE(describe_stamp_sanity(s), "");
  }
  EXPECT_STRNE(describe_stamp_sanity(StampSanity::kNearEpoch),
               describe_stamp_sanity(StampSanity::kOutOfRange))
      << "这两条必须是不同的文案,否则日志区分不出'缺失'和'字段有值但离谱'";
}

// ---------- period_s(1 Hz 抽稀周期)下限 ----------

TEST(PosWriterParams, TypicalPeriodsAreSane) {
  EXPECT_TRUE(is_sane_period_seconds(1.0));
  EXPECT_TRUE(is_sane_period_seconds(0.5));
  EXPECT_TRUE(is_sane_period_seconds(gnss_bringup::kMinSanePeriodSeconds));
}

TEST(PosWriterParams, ExtremelySmallPeriodIsRejected) {
  // period_s=1e-12 能通过"正数且有限"这道更宽松的校验,但会让
  // PosDecimator::accept() 内部 floor(stamp / period_s) 产生一个远超
  // long long 表示范围的浮点数——static_cast<long long> 对此是未定义行为。
  EXPECT_FALSE(is_sane_period_seconds(1e-12));
  EXPECT_FALSE(is_sane_period_seconds(0.0));
  EXPECT_FALSE(is_sane_period_seconds(-1.0));
}

TEST(PosWriterParams, NonFinitePeriodIsRejected) {
  EXPECT_FALSE(is_sane_period_seconds(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(is_sane_period_seconds(std::numeric_limits<double>::infinity()));
}
