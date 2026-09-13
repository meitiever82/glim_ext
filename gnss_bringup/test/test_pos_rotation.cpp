#include <gtest/gtest.h>
#include <limits>
#include <string>
#include "gnss_bringup/pos_rotation.hpp"

using gnss_bringup::pos_path_for;
using gnss_bringup::should_rotate;

// ---------- brief 里的六条基础用例 ----------

TEST(PosRotation, PathIsRootSlashYyyymmddSlashSource) {
  // 2026-09-12 10:23:45 UTC
  EXPECT_EQ(pos_path_for("/data/gnss", "can", 1789208625.0), "/data/gnss/20260912/can.pos");
}

TEST(PosRotation, UsesUtcNotLocalTime) {
  // 2026-09-12 23:30:00 UTC —— 在 UTC+8 的本地时区已是 13 日,但目录必须按 UTC 走,
  // 否则同一份数据在不同时区的机器上会落进不同目录
  EXPECT_EQ(pos_path_for("/data/gnss", "can", 1789255800.0), "/data/gnss/20260912/can.pos");
}

TEST(PosRotation, RollsOverAtUtcMidnight) {
  const double before = 1789257599.0;   // 2026-09-12 23:59:59 UTC
  const double after  = 1789257600.0;   // 2026-09-13 00:00:00 UTC
  EXPECT_EQ(pos_path_for("/d", "rtkrcv", before), "/d/20260912/rtkrcv.pos");
  EXPECT_EQ(pos_path_for("/d", "rtkrcv", after),  "/d/20260913/rtkrcv.pos");
}

TEST(PosRotation, ShouldRotateOnlyWhenThePathChanges) {
  EXPECT_FALSE(should_rotate("/d/20260912/can.pos", "/d/20260912/can.pos"));
  EXPECT_TRUE(should_rotate("/d/20260912/can.pos", "/d/20260913/can.pos"));
  EXPECT_TRUE(should_rotate("", "/d/20260912/can.pos")) << "首次打开也算换文件";
}

TEST(PosRotation, DifferentSourcesGetDifferentFiles) {
  EXPECT_NE(pos_path_for("/d", "can", 1789208625.0), pos_path_for("/d", "gpchc", 1789208625.0));
}

TEST(PosRotation, TrailingSlashInRootIsHandled) {
  EXPECT_EQ(pos_path_for("/data/gnss/", "can", 1789208625.0), "/data/gnss/20260912/can.pos");
}

// ---------- 边界情况:utc_stamp ----------
// 决定见 pos_rotation.hpp 顶部注释:0/负数但有限的时间戳是"合法但离谱"的
// 输入(上一个任务在 gnss_time 和 header.stamp 都是 0 时确实会产出 0),按
// UTC 纪元正常换算,落进肉眼可见的反常目录(19700101 / 之前的日期),而不是
// 拒绝或者悄悄映射到别处;只有非有限数、或者超出 time_t 范围、无法安全转换
// 的值才拒绝(返回空串)。

TEST(PosRotation, ZeroUtcStampFallsIntoUnixEpochDirectory) {
  EXPECT_EQ(pos_path_for("/d", "can", 0.0), "/d/19700101/can.pos")
      << "0 是上一任务在 gnss_time/header.stamp 均缺失时真实可能产出的输入,"
         "不应被拒绝,而是落进肉眼可见的反常目录,便于排查";
}

TEST(PosRotation, NegativeButFiniteUtcStampProducesPreEpochDate) {
  // -3600s = 1969-12-31 23:00:00 UTC
  EXPECT_EQ(pos_path_for("/d", "can", -3600.0), "/d/19691231/can.pos");
}

TEST(PosRotation, NonFiniteUtcStampIsRejected) {
  EXPECT_EQ(pos_path_for("/d", "can", std::numeric_limits<double>::quiet_NaN()), "")
      << "NaN 转 time_t 是未定义行为,必须在转换前挡住";
  EXPECT_EQ(pos_path_for("/d", "can", std::numeric_limits<double>::infinity()), "");
  EXPECT_EQ(pos_path_for("/d", "can", -std::numeric_limits<double>::infinity()), "");
}

TEST(PosRotation, AbsurdlyOutOfRangeUtcStampIsRejected) {
  // 远超 time_t(即便是 64 位)能表示的范围,直接 cast 是未定义行为。
  EXPECT_EQ(pos_path_for("/d", "can", 1e300), "");
  EXPECT_EQ(pos_path_for("/d", "can", -1e300), "");
}

// ---------- 边界情况:root / source ----------
// 决定:空 root 会拼出 "/YYYYMMDD/source.pos" 这种指向文件系统根目录的绝对
// 路径,比报错更危险;source 为空或带路径分隔符同样有落错位置/路径穿越的
// 风险。三者都返回空字符串,可见地失败,而不是悄悄拼出一个能用但没意义
// (或者危险)的路径。函数本身不抛异常——调用方在下一个任务里是 ROS 订阅
// 回调,一条格式错误的输入不该打断整条订阅链路。

TEST(PosRotation, EmptyRootIsRejected) {
  EXPECT_EQ(pos_path_for("", "can", 1789208625.0), "")
      << "空 root 会拼成指向文件系统根目录的绝对路径,必须拒绝而不是照写";
}

TEST(PosRotation, EmptySourceIsRejected) {
  EXPECT_EQ(pos_path_for("/d", "", 1789208625.0), "");
}

TEST(PosRotation, SourceWithPathSeparatorIsRejected) {
  EXPECT_EQ(pos_path_for("/d", "../../etc/cron.d/x", 1789208625.0), "")
      << "source 带路径分隔符会逃出 <root>/YYYYMMDD 目录,必须拒绝";
  EXPECT_EQ(pos_path_for("/d", "a/b", 1789208625.0), "");
}

TEST(PosRotation, RootThatIsOnlySlashesNormalizesToFilesystemRoot) {
  // root="/" 去掉末尾多余的 '/' 之后不能变成空串——那样就绕过了空 root 的
  // 拒绝逻辑,还会拼出一个少了前导 '/' 的相对路径。
  EXPECT_EQ(pos_path_for("/", "can", 1789208625.0), "/20260912/can.pos");
}
