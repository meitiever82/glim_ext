#include <gtest/gtest.h>
#include <limits>
#include "gnss_bringup/rtk_fix_mapping.hpp"
using namespace gnss_bringup;
using gnss_core::PosRecord;

namespace {
PosRecord sample() {
  PosRecord r;
  r.stamp = 1789045801.0;
  r.lat = 44.50123456;
  r.lon = 90.28765432;
  r.height = 617.123;
  r.q = 1;
  r.ns = 38;
  r.sdne = Eigen::Vector3d(0.011, 0.022, 0.033);   // sdn, sde, sdu
  r.age = 0.8;
  r.ratio = 20.5;
  return r;
}
}  // namespace

TEST(RtkFixMapping, SigmaIsReorderedFromNeuToEnu) {
  // 这是最容易出的错:RTKLIB 给 N/E/U,RtkFix 要 E/N/U
  const auto m = to_rtk_fix(sample());
  EXPECT_DOUBLE_EQ(m.sigma_enu[0], 0.022) << "E 应取 sde";
  EXPECT_DOUBLE_EQ(m.sigma_enu[1], 0.011) << "N 应取 sdn";
  EXPECT_DOUBLE_EQ(m.sigma_enu[2], 0.033) << "U 应取 sdu";
}

TEST(RtkFixMapping, RtklibQMapsToNormalizedQuality) {
  PosRecord r = sample();
  r.q = 1; EXPECT_EQ(to_rtk_fix(r).quality, gnss_msgs::msg::RtkFix::QUALITY_FIXED);
  r.q = 2; EXPECT_EQ(to_rtk_fix(r).quality, gnss_msgs::msg::RtkFix::QUALITY_FLOAT);
  r.q = 4; EXPECT_EQ(to_rtk_fix(r).quality, gnss_msgs::msg::RtkFix::QUALITY_DGPS);
  r.q = 5; EXPECT_EQ(to_rtk_fix(r).quality, gnss_msgs::msg::RtkFix::QUALITY_SINGLE);
  r.q = 0; EXPECT_EQ(to_rtk_fix(r).quality, gnss_msgs::msg::RtkFix::QUALITY_NONE);
}

TEST(RtkFixMapping, KeepsRawRtklibQForTraceability) {
  PosRecord r = sample();
  r.q = 2;
  EXPECT_EQ(to_rtk_fix(r).raw_status, 2);
}

TEST(RtkFixMapping, GnssTimeComesFromTheSolutionEpoch) {
  const auto m = to_rtk_fix(sample());
  EXPECT_DOUBLE_EQ(m.gnss_time, 1789045801.0);
}

TEST(RtkFixMapping, PositionAndAgeAndSatsAreCopied) {
  const auto m = to_rtk_fix(sample());
  EXPECT_NEAR(m.latitude, 44.50123456, 1e-9);
  EXPECT_NEAR(m.longitude, 90.28765432, 1e-9);
  EXPECT_NEAR(m.altitude, 617.123, 1e-9);
  EXPECT_NEAR(m.diff_age, 0.8f, 1e-6);
  EXPECT_EQ(m.sats_used, 38);
}

TEST(RtkFixMapping, HeadingIsAlwaysInvalidForSingleAntennaSolution) {
  // review round 1:单独断言 heading_valid==false 对"整个函数体被误删/退化成
  // 占位实现,直接 return RtkFix{}"这种坏实现是重言式——默认构造出来的
  // heading_valid 本来就是 false,这条测试永远是绿的,起不到回归防护作用。
  // 加一条必须由真实映射逻辑产生的断言(quality 不是默认的 QUALITY_NONE)
  // 陪跑,才能让这条测试在实现被误删时真的失败。
  const auto m = to_rtk_fix(sample());
  ASSERT_NE(m.quality, gnss_msgs::msg::RtkFix::QUALITY_NONE);
  EXPECT_FLOAT_EQ(m.heading, 0.0f);
  EXPECT_FLOAT_EQ(m.heading_sigma, 0.0f);
  EXPECT_FALSE(m.heading_valid);
}

// ---------- LineSplitter ----------
// TcpStream 交付任意切分的字节块,不保证按行到达;这里覆盖“行被切在两个块
// 中间”这个最容易导致静默丢数据/脏数据的场景(brief 明确点名的坑)。

namespace {
std::vector<uint8_t> to_bytes(const std::string& s) {
  return std::vector<uint8_t>(s.begin(), s.end());
}
std::vector<std::string> feed_str(LineSplitter& sp, const std::string& s) {
  const auto b = to_bytes(s);
  return sp.feed(b.data(), b.size());
}
}  // namespace

TEST(LineSplitter, SingleChunkWithTrailingNewlineYieldsAllLines) {
  LineSplitter sp;
  const auto lines = feed_str(sp, "abc\ndef\n");
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_EQ(lines[0], "abc");
  EXPECT_EQ(lines[1], "def");
  EXPECT_EQ(sp.buffered(), 0u);
}

TEST(LineSplitter, PartialLineIsHeldUntilNewlineArrives) {
  LineSplitter sp;
  EXPECT_TRUE(feed_str(sp, "abc").empty());
  EXPECT_EQ(sp.buffered(), 3u);
  const auto lines = feed_str(sp, "def\n");
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "abcdef");
  EXPECT_EQ(sp.buffered(), 0u);
}

TEST(LineSplitter, LineSplitAcrossManyChunksIsNotLost) {
  // 一行被切成三块字节到达,每一块都不含 '\n':必须原样拼回,而不是被
  // 当成三条破损的行。
  LineSplitter sp;
  EXPECT_TRUE(feed_str(sp, "part").empty());
  EXPECT_TRUE(feed_str(sp, "ial-l").empty());
  const auto lines = feed_str(sp, "ine\n");
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "partial-line");
}

TEST(LineSplitter, CarriageReturnBeforeNewlineIsStripped) {
  LineSplitter sp;
  const auto lines = feed_str(sp, "crlf\r\n");
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "crlf");
}

TEST(LineSplitter, OneChunkCanYieldMultipleCompleteLinesPlusAPartial) {
  LineSplitter sp;
  const auto lines = feed_str(sp, "one\ntwo\nthree-partial");
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_EQ(lines[0], "one");
  EXPECT_EQ(lines[1], "two");
  EXPECT_EQ(sp.buffered(), std::string("three-partial").size());
  const auto rest = feed_str(sp, "\n");
  ASSERT_EQ(rest.size(), 1u);
  EXPECT_EQ(rest[0], "three-partial");
}

TEST(LineSplitter, EmptyLinesAreYieldedAsEmptyStrings) {
  LineSplitter sp;
  const auto lines = feed_str(sp, "\n\nafter\n");
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_EQ(lines[0], "");
  EXPECT_EQ(lines[1], "");
  EXPECT_EQ(lines[2], "after");
}

// review round 1, Important 2:没有上限的话,一个不按行协议发送的对端
// (outstr1-format 配错、或者 sol_port 接到了不相关的字节流上)会让内部
// 缓冲无界增长——实测 1MB 无换行数据能把 RSS 从 57MB 顶到 88MB 且还在涨。
TEST(LineSplitter, OverflowingLineIsDroppedAndBufferIsCleared) {
  LineSplitter sp(8);  // 很小的上限,测试用
  const auto lines = feed_str(sp, "0123456789ABCDEFGHIJ");  // 20 字节,没有 \n
  EXPECT_TRUE(lines.empty());
  EXPECT_EQ(sp.overflow_count(), 1u);
  EXPECT_EQ(sp.buffered(), 0u) << "超限之后必须清空缓冲,不能继续无界增长";
}

TEST(LineSplitter, StaysBoundedUnderContinuousNewlineFreeInput) {
  LineSplitter sp(1024);
  const std::string chunk(4096, 'x');  // 单次 feed 已经数倍于上限,且没有 \n
  const auto lines = feed_str(sp, chunk);
  EXPECT_TRUE(lines.empty());
  EXPECT_LE(sp.buffered(), 1024u);
  EXPECT_GE(sp.overflow_count(), 1u);
}

TEST(LineSplitter, ResyncsAtNextNewlineAfterOverflowWithoutGluingGarbage) {
  // 超限丢弃之后,丢弃点到下一个 \n 之间的残留字节必须被当成"还在丢弃中"
  // 吃掉,不能跟 \n 之后真正的新行拼成一条脏行发出去。
  LineSplitter sp(8);
  const auto dropped = feed_str(sp, "0123456789ABCDEFGHIJ");  // 触发溢出
  ASSERT_TRUE(dropped.empty());
  ASSERT_EQ(sp.overflow_count(), 1u);

  const auto lines = feed_str(sp, "leftover-before-sync\nreal-line\n");
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "real-line");
  // 溢出计数不应该因为正常重新同步而再涨一次。
  EXPECT_EQ(sp.overflow_count(), 1u);
}

// review round 1, Important 4:rtkrcv 被杀掉/连接断开时,LineSplitter 里
// 残留的半行必须被清空,否则重连后的第一条完整行会跟这段陈旧残留拼在一起,
// 拼出来的东西如果凑巧还有 >=10 个字段,parse_llh_solution 会"解析成功"、
// 发布一条新旧字段混杂的假解——不是崩溃,也不是缺数据,而是看起来正常的
// 错误数据,是这一类 bug 里最难被下游发现的形状。这里直接测 reset() 这个
// 节点在断连回调里会调用的机制本身:确认它真的清空了残留,并且清空之后
// 旧的半行不会污染下一条真正的新行。
TEST(LineSplitter, ResetDiscardsBufferedPartialLine) {
  LineSplitter sp;
  ASSERT_TRUE(feed_str(sp, "stale-partial-from-before-disconnect").empty());
  ASSERT_GT(sp.buffered(), 0u);

  sp.reset();
  EXPECT_EQ(sp.buffered(), 0u);

  const auto lines = feed_str(sp, "fresh-line-after-reconnect\n");
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "fresh-line-after-reconnect")
      << "reset() 之后的新行不能跟断连前的残留半行拼在一起";
}

// ---------- is_positive_finite_seconds ----------
// review round 1, Minor(promoted):stat_poll_interval_s / 各种退避秒数
// 参数如果是 0、负数或者非有限值,会让对应的 wait_for/退避逻辑退化成热
// 循环或者钉死行为,必须在读参数的地方挡住。

TEST(IsPositiveFiniteSeconds, RejectsZeroNegativeAndNonFinite) {
  EXPECT_FALSE(is_positive_finite_seconds(0.0));
  EXPECT_FALSE(is_positive_finite_seconds(-1.0));
  EXPECT_FALSE(is_positive_finite_seconds(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(is_positive_finite_seconds(std::numeric_limits<double>::infinity()));
}

TEST(IsPositiveFiniteSeconds, AcceptsOrdinaryPositiveValues) {
  EXPECT_TRUE(is_positive_finite_seconds(0.001));
  EXPECT_TRUE(is_positive_finite_seconds(0.2));
  EXPECT_TRUE(is_positive_finite_seconds(60.0));
}

// ---------- plan_stat_tail ----------
// review round 1, Important 3:rtkrcv 的 .stat tail 线程原来的三个坑——
// (a) 启动时把上一轮遗留的旧文件当成"新增"整段重放;(b) 运行期间新出现
// 的文件反而被当成"已存在"跳过内容;(c) 文件被截断/同名重开之后永远读不
// 到新内容——全部收敛成这一个纯函数,不需要真的碰文件系统就能覆盖。

TEST(PlanStatTail, NoCandidatesMeansNoTarget) {
  const auto d = plan_stat_tail({}, "", 0, /*is_first_poll=*/true);
  EXPECT_FALSE(d.has_target);
}

TEST(PlanStatTail, FirstPollOnPreExistingFileSkipsToEndNotReplayingHistory) {
  // 对应 review 的复现:run_dir 下预置了一个上一轮遗留的 rtkrcv_*.stat。
  const std::vector<StatFileInfo> candidates = {{"/run/rtkrcv_20200101000000.stat", 5000, 100}};
  const auto d = plan_stat_tail(candidates, "", 0, /*is_first_poll=*/true);
  ASSERT_TRUE(d.has_target);
  EXPECT_EQ(d.file, "/run/rtkrcv_20200101000000.stat");
  EXPECT_EQ(d.read_from, 5000u) << "首轮遇到的文件视为旧文件,应跳到末尾,不重放历史内容";
  EXPECT_EQ(d.read_to, 5000u) << "跳到末尾之后,这一轮不应该有新内容可读";
}

TEST(PlanStatTail, FileAppearingAfterFirstPollIsTailedFromZero) {
  // 线程启动时 run_dir 是空的(is_first_poll 那一轮 candidates 为空,调用方
  // 因此不会把 first_poll 标记延续到下一轮);rtkrcv 随后才创建文件。
  const std::vector<StatFileInfo> candidates = {{"/run/rtkrcv_20260912000000.stat", 200, 50}};
  const auto d = plan_stat_tail(candidates, "", 0, /*is_first_poll=*/false);
  ASSERT_TRUE(d.has_target);
  EXPECT_EQ(d.read_from, 0u) << "运行期间才出现的新文件必须从头开始 tail";
  EXPECT_EQ(d.read_to, 200u);
}

TEST(PlanStatTail, SameFileContinuesFromSavedOffset) {
  const std::vector<StatFileInfo> candidates = {{"/run/a.stat", 1200, 10}};
  const auto d = plan_stat_tail(candidates, "/run/a.stat", 1000, /*is_first_poll=*/false);
  ASSERT_TRUE(d.has_target);
  EXPECT_EQ(d.file, "/run/a.stat");
  EXPECT_EQ(d.read_from, 1000u);
  EXPECT_EQ(d.read_to, 1200u);
}

TEST(PlanStatTail, RotationToANewerFileStartsFromZero) {
  // rtkrcv 重启后开了一个新文件,mtime 比正在 tail 的旧文件新。
  const std::vector<StatFileInfo> candidates = {
      {"/run/old.stat", 9000, 10},
      {"/run/new.stat", 300, 20},
  };
  const auto d = plan_stat_tail(candidates, "/run/old.stat", 8000, /*is_first_poll=*/false);
  ASSERT_TRUE(d.has_target);
  EXPECT_EQ(d.file, "/run/new.stat");
  EXPECT_EQ(d.read_from, 0u);
  EXPECT_EQ(d.read_to, 300u);
}

TEST(PlanStatTail, TruncatedOrReopenedFileResetsOffsetToZero) {
  // 同名文件被截断(或者被重新创建),当前 size 比记录的 offset 还小。
  const std::vector<StatFileInfo> candidates = {{"/run/a.stat", 50, 10}};
  const auto d = plan_stat_tail(candidates, "/run/a.stat", 1000, /*is_first_poll=*/false);
  ASSERT_TRUE(d.has_target);
  EXPECT_EQ(d.read_from, 0u) << "size < 记录的 offset 必须归零重新开始,否则永远读不到新内容";
  EXPECT_EQ(d.read_to, 50u);
}

TEST(PlanStatTail, PicksNewestByMtimeNotByFilenameOrder) {
  // 现场 RTC 会跟着 GNSS 时间跳变,文件名里的时间戳字典序不再等价于真实
  // 时间序——必须按 mtime 选,不能按文件名字符串比较。
  const std::vector<StatFileInfo> candidates = {
      {"/run/rtkrcv_ZZZZZZZZZZZZZZ.stat", 10, 5},   // 文件名字典序最大,但 mtime 最旧
      {"/run/rtkrcv_AAAAAAAAAAAAAA.stat", 999, 999},  // mtime 最新
  };
  const auto d = plan_stat_tail(candidates, "", 0, /*is_first_poll=*/false);
  ASSERT_TRUE(d.has_target);
  EXPECT_EQ(d.file, "/run/rtkrcv_AAAAAAAAAAAAAA.stat");
}
