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

// ---------- final-fix-wave 第 1 项:重放/重启接续同一个文件的去重经过
// PosSourceWriter 这一层能被观察到 ----------
// gnss_core::PosWriter 自己的去重逻辑在 test_pos_io.cpp 里单独覆盖;这里
// 只验证 PosSourceWriter 把"这条记录被去重了"翻译成一个操作员能看到的
// kSuppressedDuplicate 事件,而不是把它和普通的 kWritten 混在一起报不出来。

TEST(PosSourceWriter, ReopeningWithOverlappingRecordsReportsSuppressedDuplicateEvent) {
  const std::string root = tmp_root("psw_suppressed_dup");
  const double base = 1789208625.0;
  {
    PosSourceWriter w(root, "can", 1.0, gnss_core::PosTimeSystem::GPST, 18, 0.0);
    ASSERT_EQ(w.handle(at(base), 0.0).event, PosSourceEvent::kWritten);
    ASSERT_EQ(w.handle(at(base + 1.0), 0.0).event, PosSourceEvent::kWritten);
    w.close();
  }
  // 重开(等价于进程重启),原样重放同一段数据。
  PosSourceWriter w(root, "can", 1.0, gnss_core::PosTimeSystem::GPST, 18, /*wall_now_s=*/0.0);
  auto r1 = w.handle(at(base), /*wall_now_s=*/1.0);
  EXPECT_EQ(r1.event, PosSourceEvent::kSuppressedDuplicate);
  EXPECT_TRUE(r1.need_log);
  EXPECT_EQ(r1.suppressed_duplicate_count, 1u);

  auto r2 = w.handle(at(base + 1.0), /*wall_now_s=*/2.0);
  EXPECT_EQ(r2.event, PosSourceEvent::kSuppressedDuplicate);
  EXPECT_EQ(r2.suppressed_duplicate_count, 2u)
      << "累计计数,供操作人员知道这一段重叠一共跳过了多少条";

  // 重放结束、真正的新数据到来:必须正常写入,不再被当成重复。
  auto r3 = w.handle(at(base + 2.0), /*wall_now_s=*/3.0);
  EXPECT_EQ(r3.event, PosSourceEvent::kWritten);

  w.close();
}

TEST(PosSourceWriter, SuppressedDuplicateStillCountsAsActivityForSilenceCheck) {
  // 去重不是错误、也不是沉默——这条路仍然在正常工作,只是这一条记录没有
  // 真正落盘,不应该被当成"卡住了"而报沉默告警。
  const std::string root = tmp_root("psw_suppressed_dup_activity");
  const double base = 1789208625.0;
  {
    PosSourceWriter w(root, "can", 1.0, gnss_core::PosTimeSystem::GPST, 18, 0.0);
    ASSERT_EQ(w.handle(at(base), 0.0).event, PosSourceEvent::kWritten);
    w.close();
  }
  PosSourceWriter w(root, "can", 1.0, gnss_core::PosTimeSystem::GPST, 18, /*wall_now_s=*/0.0);
  ASSERT_EQ(w.handle(at(base), /*wall_now_s=*/1000.0).event, PosSourceEvent::kSuppressedDuplicate);
  EXPECT_FALSE(w.is_silent(1005.0, 10.0)) << "去重也是一次正常的处理,应当刷新活跃时间";
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

// ---------- final-fix-wave 第 4 项:区分"没收到过消息"与"消息在到达但
// 没能写出" ----------
// 复现的缺陷:默认 root 不存在时,节点先打印一次 open 失败的 ERROR,之后
// 每 5 秒只会重复"检查话题名是否配对、驱动是否在跑"这句完全文不对题的
// WARN——消息其实一直在到达,只是写不进去。has_recent_message() 把"handle()
// 被调用过"这件事和"is_silent() 判定的沉默"分开,让调用方能选对告警文案。

TEST(PosSourceWriter, HasRecentMessageIsFalseWhenHandleHasNeverBeenCalled) {
  PosSourceWriter w("/tmp/psw_msg_never", "can", 1.0, gnss_core::PosTimeSystem::GPST, 18,
                     /*wall_now_s=*/1000.0);
  EXPECT_FALSE(w.has_recent_message(1011.0, 10.0))
      << "构造之后从未调用过 handle(),不该算收到过消息";
}

TEST(PosSourceWriter, HasRecentMessageIsTrueEvenWhenTheWriteFails) {
  // kUnwritableRoot 保证 open() 必然失败——handle() 依然被调用了,消息确实
  // 到达过,只是没能写出。这正是需要跟"从没收到过消息"区分开的那种情形。
  PosSourceWriter w(kUnwritableRoot, "can", 1.0, gnss_core::PosTimeSystem::GPST, 18, 0.0);
  ASSERT_EQ(w.handle(at(1789208625.0), /*wall_now_s=*/5.0).event, PosSourceEvent::kOpenFailed);
  EXPECT_TRUE(w.has_recent_message(10.0, 10.0))
      << "5 秒前刚被调用过一次(即使写入失败),还没到超时";
  EXPECT_TRUE(w.is_silent(11.0, 10.0))
      << "从没有成功写出过任何记录的角度看(构造时刻=0),is_silent() 依然应该"
         "判定为沉默——has_recent_message() 是一个独立的信号,不改变 is_silent() 本身";
  EXPECT_FALSE(w.has_recent_message(20.0, 10.0)) << "距上一次调用已经超过 timeout_s";
}

TEST(PosSourceWriter, HasRecentMessageStaysTrueAcrossRepeatedFailedWrites) {
  PosSourceWriter w(kUnwritableRoot, "can", 1.0, gnss_core::PosTimeSystem::GPST, 18, 0.0);
  ASSERT_EQ(w.handle(at(1789208625.0), 0.0).event, PosSourceEvent::kOpenFailed);
  ASSERT_EQ(w.handle(at(1789208629.0), 9.0).event, PosSourceEvent::kOpenFailed);
  EXPECT_TRUE(w.has_recent_message(10.0, 10.0)) << "9 秒前刚刚又被调用过一次";
}

// ---------- write() 失败:强制关闭,下一条记录重试 open() ----------

// ---------- 沉默告警节流:必须是每个实例自己的状态,不是共享的 ----------
// 复现的缺陷:节点侧原来用 RCLCPP_WARN_THROTTLE 给沉默告警节流,那个宏的
// 节流状态按"调用它的源码行"是进程内共享的 static——同一份代码被三个
// WrittenSource 实例的 check_silence() 各自调用,却共享同一个 5 秒窗口。
// 复现记录(reviewer):三路全部指向死话题,silence_timeout_s=2,运行 20
// 秒 → can 报 1 次、gpchc 报 3 次、rtkrcv 全程 0 次——第三路整场不可见。
// should_warn_silence() 把这个决策挪到 PosSourceWriter 内部、按实例持有
// 状态,下面用两个独立实例证明它们不再互相挤占对方的节流窗口。

TEST(PosSourceWriter, ShouldWarnSilenceReturnsFalseWhenNotSilentYet) {
  PosSourceWriter w("/tmp/psw_silence_gate_unused", "can", 1.0,
                     gnss_core::PosTimeSystem::GPST, 18, /*wall_now_s=*/0.0);
  EXPECT_FALSE(w.should_warn_silence(1.0, /*timeout_s=*/2.0))
      << "还没超过 timeout_s,不应该告警";
}

TEST(PosSourceWriter, ShouldWarnSilenceWarnsImmediatelyOnFirstTimeoutThenThrottles) {
  PosSourceWriter w("/tmp/psw_silence_gate_throttle", "can", 1.0,
                     gnss_core::PosTimeSystem::GPST, 18, /*wall_now_s=*/0.0);
  EXPECT_TRUE(w.should_warn_silence(3.0, /*timeout_s=*/2.0)) << "第一次超时必须立刻报";
  EXPECT_FALSE(w.should_warn_silence(3.5, 2.0)) << "0.5 秒后仍在 5 秒节流窗口内,不应重复打印";
  EXPECT_FALSE(w.should_warn_silence(7.9, 2.0)) << "距上次打印 4.9 秒,还没到 5 秒";
  EXPECT_TRUE(w.should_warn_silence(8.0, 2.0)) << "距上次打印恰好 5 秒,应该重新打印";
}

TEST(PosSourceWriter, IndependentInstancesDoNotShareTheSilenceThrottleWindow) {
  // 这是对复现场景的直接回归:两个独立的源(等价于两个 WrittenSource),
  // 一个先于另一个进入沉默状态并打印过一次;另一个哪怕紧跟着在同一个
  // "全局 5 秒窗口"内也进入沉默,若节流状态是共享的 static,第二个源在
  // 这一刻会被吞掉——should_warn_silence 是每个实例自己的状态,不会发生
  // 这种事。
  PosSourceWriter a("/tmp/psw_silence_gate_a", "gpchc", 1.0,
                     gnss_core::PosTimeSystem::GPST, 18, /*wall_now_s=*/0.0);
  PosSourceWriter b("/tmp/psw_silence_gate_b", "rtkrcv", 1.0,
                     gnss_core::PosTimeSystem::GPST, 18, /*wall_now_s=*/0.0);

  EXPECT_TRUE(a.should_warn_silence(3.0, 2.0)) << "源 a 第一次超时,必须报";
  // 源 b 紧跟着(同一时刻)也第一次超时——如果节流窗口是共享的 static,
  // 这里会被 a 刚才那次打印的窗口吞掉;应该独立报告。
  EXPECT_TRUE(b.should_warn_silence(3.1, 2.0))
      << "源 b 是独立实例,第一次超时也必须报,不能被源 a 的节流窗口吞掉";
}

TEST(PosSourceWriter, WriteFailureAfterSuccessfulOpenRetriesOpenNextTime) {
  // final-fix-wave 第 5 项:原来这个用例把 .pos 文件路径替换成一个同名
  // 目录来制造失败——但 open() 对着一个目录本身就会失败,断言看到的是
  // kOpenFailed,从来没有真正走到 write() 失败那条分支。证据:把
  // pos_source_writer.hpp 里 kWriteFailed 分支的 writer_.close() 删掉,
  // 这个用例照样绿(它压根没走到那一行)。
  //
  // 用 /dev/full——Linux 上任何写入都会返回 ENOSPC 的字符设备——的符号
  // 链接顶替 .pos 文件本身:std::ofstream::open() 对着字符设备能成功打开
  // (is_open()==true),但紧接着的 flush() 必然失败,流从此进入 fail 状态
  // 但仍然 is_open()==true——这正是"open 成功、write 失败"在单测里能确定
  // 复现的写法(reviewer 确认可行)。
  const std::string root = tmp_root("psw_write_fail");
  const double stamp = 1789208625.0;
  const std::string path = gnss_bringup::pos_path_for(root, "can", stamp);
  ASSERT_FALSE(path.empty());
  std::filesystem::create_directories(std::filesystem::path(path).parent_path());
  std::filesystem::create_symlink("/dev/full", path);

  PosSourceWriter w(root, "can", 1.0, gnss_core::PosTimeSystem::GPST, 18, 0.0);
  auto r1 = w.handle(at(stamp), 0.0);
  ASSERT_EQ(r1.event, PosSourceEvent::kWriteFailed)
      << "open() 对着 /dev/full 会成功,表头/数据的 flush() 才失败";
  EXPECT_TRUE(r1.need_log) << "第一次失败必须报";

  // mutation-check(手动验证,不是这个用例自动做的):把 pos_source_writer.hpp
  // 里 kWriteFailed 分支的 writer_.close() 删掉之后,is_open() 会一直是
  // true,下一条记录不会重新走 open() 分支,而是对着同一个已经 fail 的流
  // 再调一次 write()——结果不变,仍然是 kWriteFailed,但原因从"重新 open
  // 一个字符设备后 flush 失败"变成了"对一个已经 fail 的流调用不会抛异常
  // 的 write()",两者外部表现一样,因此不能只看事件类型;下一条断言换一天
  // (触发轮转 close+reopen)来间接验证 close() 确实发生了:如果没有
  // close(),current_path_ 不会变,轮转判断 should_rotate() 仍然会因为
  // 路径不同而重新走 open(),所以这条断言本身对是否删掉 close() 不敏感——
  // 真正的 mutation 判据见下面 WriteFailureClosesTheStreamSoNextRecordReopens。
  auto r2 = w.handle(at(stamp + 1.0), 0.0);
  EXPECT_EQ(r2.event, PosSourceEvent::kWriteFailed);
  EXPECT_FALSE(r2.need_log) << "同一路径连续失败,不应重复打印";

  std::filesystem::remove(path);
}

// mutation-check 的真正判据:write() 失败之后如果不主动 close(),
// is_open() 会一直是 true,下一条记录会跳过 open() 分支、直接对着坏掉的
// 流再调一次 write()——由于 std::ofstream 对一个已经 fail 的流重复
// operator<</flush() 是安全的空操作,行为和"重新 open 之后再失败"从外部
// 看不出区别,只有当"reopen 之后确实可能成功"的场景才能把两者区分开:
// 换成一个可写的真实路径之后,如果 close() 被删掉,is_open() 依旧是
// true,PosSourceWriter 永远不会再尝试 open() 这个新路径,行为会一直卡在
// kWriteFailed;有 close() 的话,下一条记录会重新 open() 并成功写入。
TEST(PosSourceWriter, WriteFailureClosesTheStreamSoALaterFixIsPickedUpAgain) {
  const std::string root = tmp_root("psw_write_fail_recovers");
  const double stamp = 1789208625.0;
  const std::string path = gnss_bringup::pos_path_for(root, "can", stamp);
  ASSERT_FALSE(path.empty());
  std::filesystem::create_directories(std::filesystem::path(path).parent_path());
  std::filesystem::create_symlink("/dev/full", path);

  PosSourceWriter w(root, "can", 1.0, gnss_core::PosTimeSystem::GPST, 18, 0.0);
  ASSERT_EQ(w.handle(at(stamp), 0.0).event, PosSourceEvent::kWriteFailed);

  // "修好"这个路径:去掉坏掉的符号链接,换成一个真正可写的普通文件位置。
  std::filesystem::remove(path);

  auto r = w.handle(at(stamp + 1.0), /*wall_now_s=*/100.0);  // 足够的真实时间间隔,避开锁存节流
  EXPECT_EQ(r.event, PosSourceEvent::kWritten)
      << "write() 失败必须 close() 掉坏掉的流,下一条记录才会重新走 open() 分支——"
         "如果 close() 被删掉,is_open() 一直是 true,这里会一直卡在 kWriteFailed,"
         "永远不会发现路径已经修好";

  w.close();
  std::filesystem::remove(path);
}
