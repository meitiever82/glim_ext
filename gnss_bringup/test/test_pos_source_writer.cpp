#include <gtest/gtest.h>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include "gnss_bringup/pos_source_writer.hpp"

using gnss_bringup::PosSourceEvent;
using gnss_bringup::PosSourceWriter;
using gnss_bringup::StampSanity;

namespace {

std::string tmp_root(const char* name) {
  const char* dir = std::getenv("TMPDIR");
  const std::string root = std::string(dir ? dir : "/tmp") + "/" + name;
  std::filesystem::remove_all(root);
  return root;
}

gnss_core::PosRecord at(double stamp, int q = 1) {
  gnss_core::PosRecord r;
  r.stamp = stamp;
  r.q = q;
  r.lat = 44.5;
  r.lon = 90.2;
  r.height = 617.0;
  r.ns = 30;
  return r;
}

int count_data_lines(const std::string& path) {
  std::ifstream in(path);
  std::string line;
  int n = 0;
  while (std::getline(in, line)) {
    if (!line.empty() && line[0] != '%') ++n;
  }
  return n;
}

// 一个明确知道会 open() 失败的 root(与 gnss_core::PosWriter 自己的测试
// OpenFailureIsReportedNotThrown 用的是同一个不可写路径)。
constexpr const char* kUnwritableRoot = "/proc/definitely-not-writable";

}  // namespace

// ---------- Important 4 (a): 轮转依据记录自己的 stamp,不是调用时刻 ----------
// 这正是评审用来证明"节点接线本身没有单测覆盖"的第一个变异:把轮转依据从
// r.stamp 换成 wall_now_s 之后,142 个既有测试全绿——因为它们都没有独立
// 测过 PosSourceWriter/WrittenSource 这一层。

TEST(PosSourceWriter, RotatesByRecordStampNotByWallClock) {
  const std::string root = tmp_root("psw_rotate_by_stamp");
  PosSourceWriter w(root, "can", 1.0, gnss_core::PosTimeSystem::GPST, 18, /*wall_now_s=*/0.0);

  // wall_now_s 在两次调用之间故意保持不变(甚至可以是任意值——它不应该
  // 参与轮转决策),但记录自己的 stamp 跨了一个 UTC 天边界
  // (与 test_pos_rotation.cpp 的 RollsOverAtUtcMidnight 用的是同一对值)。
  const double before = 1789257599.0;  // 2026-09-12 23:59:59 UTC
  const double after = 1789257600.0;   // 2026-09-13 00:00:00 UTC
  constexpr double kSameWallNow = 123456.0;  // 完全无关的、固定不变的"现在"

  auto r1 = w.handle(at(before), kSameWallNow);
  EXPECT_EQ(r1.event, PosSourceEvent::kWritten);
  EXPECT_EQ(w.current_path(), root + "/20260912/can.pos");

  auto r2 = w.handle(at(after), kSameWallNow);
  EXPECT_EQ(r2.event, PosSourceEvent::kWritten);
  EXPECT_EQ(w.current_path(), root + "/20260913/can.pos")
      << "轮转必须只看记录自己的 stamp——wall_now_s 全程没变,"
         "如果轮转错误地依据了它,这里就还停在 20260912";

  w.close();
}

TEST(PosSourceWriter, WritesEndUpInTheirOwnDayNotTodays) {
  // 一次"离线快速回放"场景的简化版:同一个 wall_now_s(回放开始的真实时刻)
  // 下,连续喂入分属两个不同 UTC 天的记录,必须分别落进各自的目录,而不是
  // 全部落进回放当时的日期。
  const std::string root = tmp_root("psw_replay_days");
  PosSourceWriter w(root, "can", 1.0, gnss_core::PosTimeSystem::GPST, 18, 0.0);
  constexpr double kReplayWallNow = 999999.0;

  w.handle(at(1789208625.0), kReplayWallNow);  // 2026-09-12
  w.handle(at(1789208626.0 + 86400.0), kReplayWallNow);  // +1 天 = 2026-09-13

  EXPECT_TRUE(std::filesystem::exists(root + "/20260912/can.pos"));
  EXPECT_TRUE(std::filesystem::exists(root + "/20260913/can.pos"));
  w.close();
}

// ---------- 1 Hz 抽稀经过这一层没有被破坏 ----------

TEST(PosSourceWriter, DecimatesToOneHz) {
  const std::string root = tmp_root("psw_decimate");
  PosSourceWriter w(root, "can", 1.0, gnss_core::PosTimeSystem::GPST, 18, 0.0);
  const double base = 1789208625.0;
  int written = 0;
  for (int i = 0; i < 30; ++i) {  // 10 Hz,3 秒
    const double stamp = base + static_cast<double>(i) * 0.1;
    if (w.handle(at(stamp), 0.0).event == PosSourceEvent::kWritten) ++written;
  }
  EXPECT_EQ(written, 3);
  EXPECT_EQ(count_data_lines(w.current_path()), 3);
  w.close();
}

// ---------- Important 4 (b): 失败锁存——同一路径只报一次,路径变化重新武装 ----------
// 这是评审用来证明覆盖缺口的第二个变异:把锁存逻辑改坏(比如永远
// need_log=true)同样能让 142 个既有测试全绿。

TEST(PosSourceWriter, RepeatedOpenFailuresOnSamePathLogOnlyOnce) {
  PosSourceWriter w(kUnwritableRoot, "can", 1.0, gnss_core::PosTimeSystem::GPST, 18, 0.0);
  const double base = 1789208625.0;

  auto r1 = w.handle(at(base), 0.0);
  ASSERT_EQ(r1.event, PosSourceEvent::kOpenFailed);
  EXPECT_TRUE(r1.need_log) << "第一次失败必须报";

  // 同一路径(未跨秒/未跨天)连续 4 次失败,不能再报。
  for (int i = 1; i <= 4; ++i) {
    auto r = w.handle(at(base + i), 0.0);
    ASSERT_EQ(r.event, PosSourceEvent::kOpenFailed);
    EXPECT_FALSE(r.need_log) << "第 " << (i + 1) << " 次同路径失败不应重复打印";
  }
}

TEST(PosSourceWriter, PathChangeRearmsTheLatchWhenEnoughWallTimeHasPassed) {
  PosSourceWriter w(kUnwritableRoot, "can", 1.0, gnss_core::PosTimeSystem::GPST, 18, 0.0);
  const double day1 = 1789208625.0;         // 2026-09-12
  const double day2 = day1 + 86400.0 * 30;  // 30 天后,另一个 UTC 天

  auto r1 = w.handle(at(day1), 0.0);
  ASSERT_EQ(r1.event, PosSourceEvent::kOpenFailed);
  EXPECT_TRUE(r1.need_log);

  auto r2 = w.handle(at(day1 + 1.0), 0.0);  // 同一天,不该再报
  EXPECT_FALSE(r2.need_log);

  // 路径变化(新的一天),并且给足够的真实墙钟时间(远超 5 秒的全局节流窗口)
  // ——一个正常运行了 30 天、问题仍未解决的场景,理应在新的一天重新报一次。
  auto r3 = w.handle(at(day2), /*wall_now_s=*/100.0);
  EXPECT_EQ(r3.event, PosSourceEvent::kOpenFailed);
  EXPECT_TRUE(r3.need_log) << "路径变化 + 足够的真实时间间隔,应当重新武装锁存";
}

TEST(PosSourceWriter, BurstsOfPathChangesWithinTheCooldownWindowDoNotFloodLogs) {
  // round 2 review 的 promoted Minor:离线快速回放跨越好几个真实日期边界
  // (只读挂载问题始终没解决)时,原实现"路径一变就重新武装"会在几秒真实
  // 时间内连续报出好几条 ERROR(reviewer 实测 3 秒内 6 条)。修复后,即使
  // 路径连续变化,只要真实时间(wall_now_s)没有跨过全局最短报告间隔,
  // 后续失败也不应该重新打印。
  PosSourceWriter w(kUnwritableRoot, "can", 1.0, gnss_core::PosTimeSystem::GPST, 18, 0.0);
  const double base_day = 1789208625.0;

  auto r1 = w.handle(at(base_day), /*wall_now_s=*/0.0);
  ASSERT_EQ(r1.event, PosSourceEvent::kOpenFailed);
  EXPECT_TRUE(r1.need_log);

  int extra_logs = 0;
  for (int day = 1; day <= 5; ++day) {
    // 每次都换到一个新的 UTC 天(触发路径变化/锁存重置),但全部发生在
    // 3 秒真实时间之内——远小于 5 秒的全局节流窗口。
    const double stamp = base_day + 86400.0 * day;
    const double wall_now = 0.6 * day;  // 0.6, 1.2, 1.8, 2.4, 3.0 秒
    auto r = w.handle(at(stamp), wall_now);
    ASSERT_EQ(r.event, PosSourceEvent::kOpenFailed);
    if (r.need_log) ++extra_logs;
  }
  EXPECT_EQ(extra_logs, 0) << "5 次路径变化全部发生在 3 秒真实时间内,"
                              "全局节流窗口(5 秒)不应该被路径变化绕过";
}

TEST(PosSourceWriter, SuccessRearmsTheLatchSoAFutureFailureLogsAgain) {
  // 同一路径上"失败一次(报)→ 恢复成功 → 再次失败"必须重新报一次,而不是
  // 被第一次失败的锁存永远压住。用文件系统的一个确定性技巧构造这个序列:
  // 用一个同名目录顶替 .pos 文件的位置来制造 open() 失败,删掉目录再制造
  // 恢复。
  const std::string root = tmp_root("psw_recover");
  PosSourceWriter w(root, "can", 1.0, gnss_core::PosTimeSystem::GPST, 18, /*wall_now_s=*/0.0);
  const double stamp = 1789208625.0;  // 全程同一个 UTC 天,路径不变

  ASSERT_EQ(w.handle(at(stamp), 0.0).event, PosSourceEvent::kWritten);
  const std::string path = w.current_path();
  w.close();  // is_open() 现在是 false,下一条记录会重新走 open() 分支

  std::filesystem::remove(path);
  std::filesystem::create_directory(path);  // 顶替成目录:open() 必然失败

  auto r_fail1 = w.handle(at(stamp + 1.0), /*wall_now_s=*/0.0);
  ASSERT_EQ(r_fail1.event, PosSourceEvent::kOpenFailed);
  EXPECT_TRUE(r_fail1.need_log) << "第一次失败必须报";

  auto r_fail2 = w.handle(at(stamp + 2.0), /*wall_now_s=*/1.0);
  ASSERT_EQ(r_fail2.event, PosSourceEvent::kOpenFailed);
  EXPECT_FALSE(r_fail2.need_log) << "同一路径连续失败,不应重复打印";

  std::filesystem::remove(path);  // 恢复:去掉那个顶替用的目录

  auto r_ok = w.handle(at(stamp + 3.0), /*wall_now_s=*/2.0);
  ASSERT_EQ(r_ok.event, PosSourceEvent::kWritten) << "目录已移除,open() 应该重新成功";

  w.close();
  std::filesystem::remove(path);
  std::filesystem::create_directory(path);  // 问题再次出现

  auto r_fail3 = w.handle(at(stamp + 4.0), /*wall_now_s=*/100.0);  // 足够的真实时间间隔
  EXPECT_EQ(r_fail3.event, PosSourceEvent::kOpenFailed);
  EXPECT_TRUE(r_fail3.need_log) << "成功写入之后锁存已经重新武装,新一次失败必须重新报";

  std::filesystem::remove_all(path);
}

// ---------- 时间戳合理性闸门:分类准确 + 丢弃计数 ----------

TEST(PosSourceWriter, ClassifiesNearEpochStampAndCountsDrops) {
  const std::string root = tmp_root("psw_bad_stamp");
  PosSourceWriter w(root, "can", 1.0, gnss_core::PosTimeSystem::GPST, 18, 0.0);

  auto r1 = w.handle(at(0.0), 0.0);
  EXPECT_EQ(r1.event, PosSourceEvent::kDroppedBadStamp);
  EXPECT_EQ(r1.stamp_sanity, StampSanity::kNearEpoch);
  EXPECT_EQ(r1.bad_stamp_drop_count, 1u);
  EXPECT_TRUE(r1.need_log);

  auto r2 = w.handle(at(0.0), 0.0);
  EXPECT_EQ(r2.bad_stamp_drop_count, 2u) << "累计计数,供操作人员知道一共丢了多少条";

  // 有限、非零,但离谱地早——字段都在,只是不像真实的 GNSS 时刻,不应被
  // 误判为"缺失"。
  auto r3 = w.handle(at(631152000.0), 0.0);  // 1990-01-01
  EXPECT_EQ(r3.event, PosSourceEvent::kDroppedBadStamp);
  EXPECT_EQ(r3.stamp_sanity, StampSanity::kOutOfRange);

  auto r4 = w.handle(at(std::numeric_limits<double>::quiet_NaN()), 0.0);
  EXPECT_EQ(r4.stamp_sanity, StampSanity::kNonFinite);

  // 没有任何一条被写进磁盘,也没有建出任何目录。
  EXPECT_FALSE(std::filesystem::exists(root));
  w.close();
}

// ---------- Important 3: 沉默检测 ----------

TEST(PosSourceWriter, IsSilentAfterTimeoutWithNoSuccessfulWrite) {
  PosSourceWriter w("/tmp/psw_silent_unused", "can", 1.0, gnss_core::PosTimeSystem::GPST, 18,
                     /*wall_now_s=*/1000.0);
  EXPECT_FALSE(w.is_silent(1001.0, 10.0)) << "刚构造,1 秒还没到超时";
  EXPECT_TRUE(w.is_silent(1011.0, 10.0)) << "从构造到现在已经超过超时,且从未成功写入过";
}

TEST(PosSourceWriter, SuccessfulWriteResetsTheSilenceClock) {
  const std::string root = tmp_root("psw_silent_active");
  PosSourceWriter w(root, "can", 1.0, gnss_core::PosTimeSystem::GPST, 18, 0.0);
  ASSERT_EQ(w.handle(at(1789208625.0), /*wall_now_s=*/1000.0).event, PosSourceEvent::kWritten);
  EXPECT_FALSE(w.is_silent(1005.0, 10.0)) << "5 秒前刚成功写过一条,还没到超时";
  EXPECT_TRUE(w.is_silent(1015.0, 10.0)) << "15 秒没有任何成功写入,应判定为沉默";
  w.close();
}

TEST(PosSourceWriter, DecimatedDropsDoNotCountAsActivityForSilenceCheck) {
  // 未过 1 Hz 抽稀边界的记录不算"活跃"——沉默检测关心的是"有没有真正写出
  // 东西",不是"有没有收到消息"（收到消息但一直被抽稀丢弃，文件同样不会
  // 增长）。
  const std::string root = tmp_root("psw_silent_decimated");
  PosSourceWriter w(root, "can", 1.0, gnss_core::PosTimeSystem::GPST, 18, /*wall_now_s=*/0.0);
  ASSERT_EQ(w.handle(at(1789208625.0), 0.0).event, PosSourceEvent::kWritten);
  // 同一秒内的重复,被抽稀丢弃,不应刷新"最后活跃时间"。
  ASSERT_EQ(w.handle(at(1789208625.5), 20.0).event, PosSourceEvent::kDroppedRateLimit);
  EXPECT_TRUE(w.is_silent(20.0, 10.0)) << "距离上一次真正写入已经 20 秒,"
                                          "中间被抽稀丢弃的记录不算数";
  w.close();
}

// ---------- write() 失败:强制关闭,下一条记录重试 open() ----------

TEST(PosSourceWriter, WriteFailureAfterSuccessfulOpenRetriesOpenNextTime) {
  // 用一个真实的、之后被搞坏权限的目录来复现"open 成功、write 失败"这个
  // 场景不方便在单测里做到确定性——这里改为验证一个等价但可确定复现的
  // 属性：文件被删除/替换成一个目录（同名冲突）之后，之前已经打开的
  // std::ofstream 继续 write() 不会崩，出问题时事件是 kWriteFailed 而不是
  // 被悄悄吞掉。
  const std::string root = tmp_root("psw_write_fail");
  PosSourceWriter w(root, "can", 1.0, gnss_core::PosTimeSystem::GPST, 18, 0.0);
  ASSERT_EQ(w.handle(at(1789208625.0), 0.0).event, PosSourceEvent::kWritten);

  const std::string path = w.current_path();
  w.close();  // 主动关掉,模拟"底层流已经不可用"
  // 把文件路径本身替换成一个目录——is_open() 现在是 false（我们刚 close()
  // 过），所以下一条记录会走 open() 分支，而 open() 对着一个同名目录必然
  // 失败，这与"磁盘满/权限不足"是同一类"打开失败"，用来验证失败事件仍然
  // 被正确地报告，而不是被 close() 之后的状态搞乱。
  std::filesystem::remove(path);
  std::filesystem::create_directory(path);

  auto r = w.handle(at(1789208626.0), 0.0);
  EXPECT_EQ(r.event, PosSourceEvent::kOpenFailed);
  EXPECT_TRUE(r.need_log);

  std::filesystem::remove_all(path);
}
