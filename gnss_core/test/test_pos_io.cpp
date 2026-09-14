#include <gtest/gtest.h>
#include <sys/stat.h>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include "gnss_core/pos_io.hpp"
#include "gnss_core/pos_io_test_hooks.hpp"
using namespace gnss_core;

namespace {
std::string tmp_path(const char* name) {
  const char* dir = std::getenv("TMPDIR");
  return std::string(dir ? dir : "/tmp") + "/" + name;
}

// 与既有 PosIo 用例(WriteReadRoundTrip)取值一致的一条样本记录。
PosRecord sample_record() {
  PosRecord r{};
  r.stamp = 1788431025.0 + 0.25;   // UTC unix,含小数秒
  r.lat = 44.50123456;
  r.lon = 90.28765432;
  r.height = 617.123;
  r.q = 1;
  r.ns = 30;
  r.sdne = Eigen::Vector3d(0.012, 0.011, 0.030);
  r.age = 0.8;
  r.ratio = 20.5;
  return r;
}
}  // namespace

TEST(PosIo, ParsesRtklibPos) {
  const std::string path = tmp_path("test_gnss_core.pos");
  std::ofstream f(path);
  f << "% program : RTKLIB\n";
  f << "%  GPST latitude longitude height Q ns sdn sde sdu ...\n";
  f << "2026/09/03 10:23:45.000 44.50123456 90.28765432 617.123 1 38 0.012 0.011 0.030 0.0 0.0 0.0 0.8 20.5\n";
  f.close();
  auto recs = read_pos(path);
  ASSERT_EQ(recs.size(), 1u);
  EXPECT_EQ(recs[0].q, 1);
  EXPECT_NEAR(recs[0].lat, 44.50123456, 1e-8);
  EXPECT_NEAR(recs[0].lon, 90.28765432, 1e-8);
  EXPECT_NEAR(recs[0].height, 617.123, 1e-6);
  EXPECT_EQ(recs[0].ns, 38);
  EXPECT_NEAR(recs[0].sdne(0), 0.012, 1e-9);
  EXPECT_NEAR(recs[0].sdne(1), 0.011, 1e-9);
  EXPECT_NEAR(recs[0].sdne(2), 0.030, 1e-9);
  EXPECT_NEAR(recs[0].age, 0.8, 1e-9);
  EXPECT_NEAR(recs[0].ratio, 20.5, 1e-6);
  // 2026/09/03 10:23:45 UTC = 1788431025 unix;默认 GPST → 再减 18 s
  EXPECT_NEAR(recs[0].stamp, 1788431025.0 - 18.0, 1e-6);
}

TEST(PosIo, QToQuality) {
  EXPECT_EQ(q_to_quality(1), Quality::FIXED);
  EXPECT_EQ(q_to_quality(2), Quality::FLOAT);
  EXPECT_EQ(q_to_quality(4), Quality::DGPS);
  EXPECT_EQ(q_to_quality(5), Quality::SINGLE);
  EXPECT_EQ(q_to_quality(0), Quality::NONE);
  EXPECT_EQ(q_to_quality(3), Quality::NONE);   // SBAS 不映射
  EXPECT_EQ(q_to_quality(6), Quality::NONE);   // PPP 不映射
}

TEST(PosIo, GpstHeaderSubtractsLeapSeconds) {
  const std::string path = tmp_path("test_gnss_core_gpst.pos");
  std::ofstream f(path);
  f << "% (lat/lon/height=WGS84/ellipsoidal,Q=1:fix,2:float,4:dgps,5:single, time=GPST)\n";
  f << "2026/09/03 10:23:45.000 44.5 90.28 617.0 1 38 0.01 0.01 0.03 0 0 0 0.8 20.5\n";
  f.close();
  auto gpst = read_pos(path);
  std::ofstream g(path);
  g << "% (time=UTC)\n";
  g << "2026/09/03 10:23:45.000 44.5 90.28 617.0 1 38 0.01 0.01 0.03 0 0 0 0.8 20.5\n";
  g.close();
  auto utc = read_pos(path);
  ASSERT_EQ(gpst.size(), 1u);
  ASSERT_EQ(utc.size(), 1u);
  EXPECT_NEAR(utc[0].stamp - gpst[0].stamp, 18.0, 1e-6);   // 同一行,GPST 解释晚 18 s → unix 秒小 18
  // 无头部标注 + default_time_system=UTC 时,与显式 UTC 头一致
  std::ofstream h(path);
  h << "2026/09/03 10:23:45.000 44.5 90.28 617.0 1 38 0.01 0.01 0.03 0 0 0 0.8 20.5\n";
  h.close();
  PosReadOptions opt;
  opt.default_time_system = PosTimeSystem::UTC;
  auto no_hdr = read_pos(path, opt);
  ASSERT_EQ(no_hdr.size(), 1u);
  EXPECT_NEAR(no_hdr[0].stamp, utc[0].stamp, 1e-6);
  // 闰秒可配置
  opt.default_time_system = PosTimeSystem::GPST;
  opt.leap_seconds = 19;
  auto leap19 = read_pos(path, opt);
  EXPECT_NEAR(utc[0].stamp - leap19[0].stamp, 19.0, 1e-6);
}

TEST(PosIo, WriteReadRoundTrip) {
  std::vector<PosRecord> recs;
  for (int i = 0; i < 3; ++i) {
    PosRecord r{};
    r.stamp = 1788431025.0 + i + 0.25;          // UTC unix,含小数秒
    r.lat = 44.50123456 + 1e-6 * i;
    r.lon = 90.28765432 - 2e-6 * i;
    r.height = 617.123 + 0.5 * i;
    r.q = (i == 1) ? 2 : 1;
    r.ns = 30 + i;
    r.sdne = Eigen::Vector3d(0.012, 0.011, 0.030) * (i + 1);
    r.age = 0.8 + i;
    r.ratio = 20.5 - i;
    recs.push_back(r);
  }
  for (PosTimeSystem ts : {PosTimeSystem::GPST, PosTimeSystem::UTC}) {
    const std::string path = tmp_path("test_gnss_core_roundtrip.pos");
    write_pos(path, recs, ts);
    // 头部应标明时间系统,读取端据此换算回同一 UTC 秒
    {
      std::ifstream in(path);
      std::string line, header;
      while (std::getline(in, line) && !line.empty() && line[0] == '%') header += line + "\n";
      EXPECT_NE(header.find(ts == PosTimeSystem::GPST ? "time=GPST" : "time=UTC"), std::string::npos);
      EXPECT_NE(header.find("Q=1:fix,2:float,4:dgps,5:single"), std::string::npos);
    }
    auto back = read_pos(path);
    ASSERT_EQ(back.size(), recs.size());
    for (size_t i = 0; i < recs.size(); ++i) {
      EXPECT_NEAR(back[i].stamp, recs[i].stamp, 1e-3) << "ts=" << static_cast<int>(ts) << " i=" << i;
      EXPECT_NEAR(back[i].lat, recs[i].lat, 1e-9);
      EXPECT_NEAR(back[i].lon, recs[i].lon, 1e-9);
      EXPECT_NEAR(back[i].height, recs[i].height, 1e-4);
      EXPECT_EQ(back[i].q, recs[i].q);
      EXPECT_EQ(back[i].ns, recs[i].ns);
      EXPECT_LT((back[i].sdne - recs[i].sdne).norm(), 1e-4);
      EXPECT_NEAR(back[i].age, recs[i].age, 1e-2);
      EXPECT_NEAR(back[i].ratio, recs[i].ratio, 1e-1);
    }
  }
}

TEST(PosIo, MissingFileThrows) {
  EXPECT_THROW(read_pos(tmp_path("definitely_missing_gnss_core.pos")), std::runtime_error);
}

TEST(PosIo, ColumnHeaderLineIdentifiesTimeSystem) {
  // RTKLIB 真实文件的列名行形如 "%  UTC  latitude(deg) ...",没有 time=UTC 键;
  // 第一个 token 为 UTC/GPST 也要识别。
  const std::string p_utc = tmp_path("test_gnss_core_colhdr_utc.pos");
  const std::string p_gpst = tmp_path("test_gnss_core_colhdr_gpst.pos");
  const char* row = "2026/09/03 10:23:45.000   40.000000000  116.000000000   50.0000   1  20   0.01   0.01   0.02   0.0   0.0   0.0   1.0   5.0\n";
  { std::ofstream f(p_utc);
    f << "% program   : rnx2rtkp\n";
    f << "%  UTC                   latitude(deg) longitude(deg)  height(m)   Q  ns   sdn(m)   sde(m)   sdu(m)\n";
    f << row; }
  { std::ofstream f(p_gpst);
    f << "% program   : rnx2rtkp\n";
    f << "%  GPST                  latitude(deg) longitude(deg)  height(m)   Q  ns   sdn(m)   sde(m)   sdu(m)\n";
    f << row; }
  PosReadOptions opt; opt.default_time_system = PosTimeSystem::GPST;
  auto utc = read_pos(p_utc, opt);
  ASSERT_EQ(utc.size(), 1u);
  EXPECT_NEAR(utc[0].stamp, 1788431025.0, 1e-6);   // UTC 头:不减闰秒
  opt.default_time_system = PosTimeSystem::UTC;
  auto gpst = read_pos(p_gpst, opt);
  ASSERT_EQ(gpst.size(), 1u);
  EXPECT_NEAR(gpst[0].stamp, 1788431025.0 - 18.0, 1e-6);   // GPST 头:减闰秒
  std::remove(p_utc.c_str()); std::remove(p_gpst.c_str());
}

// ---------- PosDecimator(1 Hz 抽稀, spec §5.3) ----------

namespace {
PosRecord at(double stamp, int q = 1) {
  PosRecord r;
  r.stamp = stamp;
  r.q = q;
  return r;
}
}  // namespace

TEST(PosDecimator, EmitsFirstRecord) {
  PosDecimator d;
  EXPECT_TRUE(d.accept(at(1000.0)));
}

TEST(PosDecimator, DropsFurtherRecordsWithinTheSameSecond) {
  PosDecimator d;
  ASSERT_TRUE(d.accept(at(1000.00)));
  EXPECT_FALSE(d.accept(at(1000.10)));
  EXPECT_FALSE(d.accept(at(1000.50)));
  EXPECT_FALSE(d.accept(at(1000.99)));
}

TEST(PosDecimator, EmitsFirstRecordOfEachNewSecond) {
  PosDecimator d;
  ASSERT_TRUE(d.accept(at(1000.00)));
  ASSERT_FALSE(d.accept(at(1000.90)));
  EXPECT_TRUE(d.accept(at(1001.00)));
  EXPECT_FALSE(d.accept(at(1001.40)));
  EXPECT_TRUE(d.accept(at(1002.20)));
}

TEST(PosDecimator, BucketsAlignToWholeSecondsNotToFirstSample) {
  // 起点在半秒:1000.6 与 1001.0 相隔仅 0.4 s,但分属不同整秒桶,两条都应写出。
  // 若按"上次 + 1 s"实现,1001.0 会被丢掉,输出时间戳也会偏离整秒。
  PosDecimator d;
  ASSERT_TRUE(d.accept(at(1000.60)));
  EXPECT_TRUE(d.accept(at(1001.00)));
}

TEST(PosDecimator, ClockJumpBackwardsDoesNotStallOutput) {
  PosDecimator d;
  ASSERT_TRUE(d.accept(at(2000.00)));
  EXPECT_TRUE(d.accept(at(1000.00))) << "时钟回跳后必须继续写出,而不是静默丢弃到追上为止";
}

// round 2 review 的 Important 2:一个按接收时刻打时间戳的源,相邻记录完全
// 可能在整秒边界两侧来回摆动(比如 625.995 → 626.003 → 625.998 → 626.001 →
// …)。修复前的规则("桶号只要变化就放行")会把这种摆动的每一次跳变都当成
// 新的一秒放行——10 Hz 输入实测被写出 30 行而不是 3 行(reviewer 复测数字)。
// 用显式的逐条断言而不是计数器,是为了不让"第一条记录无条件放行"这条规则
// (has_bin_==false 时没有'上一次'可比较)悄悄污染计数逻辑本身。
TEST(PosDecimator, JitterAcrossASecondBoundaryDoesNotMultiplyOutput) {
  PosDecimator d;
  EXPECT_TRUE(d.accept(at(1000.000)));   // 第一条:无条件放行,真实的第 1 秒
  EXPECT_FALSE(d.accept(at(1000.003)));  // 同一秒内的重复
  EXPECT_FALSE(d.accept(at(999.998)));   // 摆回上一个桶——抖动,丢弃
  EXPECT_FALSE(d.accept(at(1000.004)));  // 摆回来——还是同一秒,丢弃
  EXPECT_FALSE(d.accept(at(999.997)));   // 再摆一次——抖动,丢弃
  EXPECT_TRUE(d.accept(at(1001.000)));   // 真正推进到下一秒
  EXPECT_FALSE(d.accept(at(1001.004)));
  EXPECT_FALSE(d.accept(at(1000.998)));  // 摆回上一秒边界——抖动,丢弃
  EXPECT_FALSE(d.accept(at(1001.002)));
  EXPECT_TRUE(d.accept(at(1002.000)));   // 再推进一秒——3 个真实秒,3 次放行
  EXPECT_FALSE(d.accept(at(1002.003)));
  EXPECT_FALSE(d.accept(at(1001.995)));  // 摆回上一秒边界——抖动,丢弃
  EXPECT_FALSE(d.accept(at(1002.004)));
}

// 摆动本身(后退 1 个桶)必须被当成抖动丢弃,但更大的后退(真实回跳)
// 仍然必须继续放行——两条行为不能互相抵消。
TEST(PosDecimator, JitterToleranceDoesNotSwallowARealBackwardJump) {
  PosDecimator d;
  ASSERT_TRUE(d.accept(at(1001.00)));                 // bin=1001
  EXPECT_FALSE(d.accept(at(1000.995))) << "只后退 1 个桶——抖动,丢弃";
  EXPECT_TRUE(d.accept(at(998.00))) << "后退 3 个桶(相对上一次真正放行的 1001)——"
                                       "真实回跳,必须继续放行,不能被抖动容忍度吞掉";
}

TEST(PosDecimator, RespectsConfiguredPeriod) {
  PosDecimator d(0.5);
  ASSERT_TRUE(d.accept(at(1000.00)));
  EXPECT_FALSE(d.accept(at(1000.20)));
  EXPECT_TRUE(d.accept(at(1000.50)));
}

TEST(PosDecimator, ResetAllowsNextRecordThrough) {
  PosDecimator d;
  ASSERT_TRUE(d.accept(at(1000.00)));
  ASSERT_FALSE(d.accept(at(1000.30)));
  d.reset();
  EXPECT_TRUE(d.accept(at(1000.30)));
}

// ---------- PosWriter(可追加、崩溃安全, spec §5.3) ----------

TEST(PosWriter, WritesHeaderOnceForANewFile) {
  const std::string p = tmp_path("pw_new.pos");
  ::remove(p.c_str());
  PosRecord second = sample_record();
  second.stamp += 1.0;   // 与去重不变量(同一个毫秒键至多写一次)无关,这里只想要两条不同的行
  PosWriter w;
  ASSERT_TRUE(w.open(p));
  ASSERT_TRUE(w.write(sample_record()));
  ASSERT_TRUE(w.write(second));
  w.close();
  // 表头行(以 % 开头)只应出现在文件开头,且数量与 write_pos 一致
  std::ifstream in(p);
  std::string line; int header_lines = 0, data_lines = 0;
  while (std::getline(in, line)) { if (!line.empty() && line[0] == '%') ++header_lines; else if (!line.empty()) ++data_lines; }
  EXPECT_EQ(data_lines, 2);
  EXPECT_GT(header_lines, 0);
  EXPECT_EQ(header_lines, 3) << "表头应与 write_pos 相同,且只写一次";
}

TEST(PosWriter, ReopeningAnExistingFileAppendsWithoutRewritingTheHeader) {
  const std::string p = tmp_path("pw_append.pos");
  ::remove(p.c_str());
  { PosWriter w; ASSERT_TRUE(w.open(p)); ASSERT_TRUE(w.write(sample_record())); }
  // final-fix-wave 第 1 项之后:重开一个已有非空文件会以文件里最后一条
  // 记录的 stamp 作为去重分界线,<=分界线的记录会被静默跳过——这里第二次
  // 写入的 stamp 特意晚于第一次,是为了单独验证"重开不重写表头"这一件事,
  // 不与去重逻辑混在一起(去重本身在下面几个 PosWriter.Reopening* 用例里
  // 单独覆盖)。
  PosRecord second = sample_record();
  second.stamp += 5.0;
  { PosWriter w; ASSERT_TRUE(w.open(p)); ASSERT_TRUE(w.write(second)); }
  std::ifstream in(p);
  std::string line; int header_lines = 0, data_lines = 0;
  while (std::getline(in, line)) { if (!line.empty() && line[0] == '%') ++header_lines; else if (!line.empty()) ++data_lines; }
  EXPECT_EQ(data_lines, 2);
  EXPECT_EQ(header_lines, 3) << "重开不得再写一遍表头";
}

// ---------- final-fix-wave 第 1 项:重启/重放接续同一个文件时的去重 ----------
// 复现的缺陷:pos_writer 对着 can 数据跑 3 秒、SIGTERM、重启、原样重放同一段
// 数据 → can.pos 变成 …03,04,05,03,04,05。calibrate_sigma_scale 把每一行都
// 当成独立量测两两配对,被重放的这段因此被静默双倍加权。

TEST(PosWriter, FreshFileNeverSuppressesAnything) {
  const std::string p = tmp_path("pw_resume_fresh.pos");
  ::remove(p.c_str());
  PosWriter w;
  ASSERT_TRUE(w.open(p));
  EXPECT_EQ(w.suppressed_duplicate_count(), 0u);
  ASSERT_TRUE(w.write(sample_record()));
  EXPECT_FALSE(w.last_write_was_suppressed());
  EXPECT_EQ(w.suppressed_duplicate_count(), 0u)
      << "全新文件不存在任何“已有内容”,不应该有任何去重发生";
}

TEST(PosWriter, ReopeningWithLaterRecordsAppendsThemNormally) {
  const std::string p = tmp_path("pw_resume_later.pos");
  ::remove(p.c_str());
  const PosRecord a = sample_record();
  { PosWriter w; ASSERT_TRUE(w.open(p)); ASSERT_TRUE(w.write(a)); }

  PosRecord b = a;
  b.stamp += 10.0;   // 晚于文件里已有的最后一条——真正的新数据
  {
    PosWriter w;
    ASSERT_TRUE(w.open(p));
    EXPECT_TRUE(w.write(b));
    EXPECT_FALSE(w.last_write_was_suppressed());
    EXPECT_EQ(w.suppressed_duplicate_count(), 0u);
  }
  const auto back = read_pos(p);
  ASSERT_EQ(back.size(), 2u);
  EXPECT_NEAR(back[1].stamp, b.stamp, 1e-3);
}

TEST(PosWriter, ReopeningWithOverlappingRecordsSuppressesThemAndReportsTheCount) {
  const std::string p = tmp_path("pw_resume_overlap.pos");
  ::remove(p.c_str());
  const PosRecord a = sample_record();
  PosRecord b = a; b.stamp += 1.0;
  PosRecord c = a; c.stamp += 2.0;
  { PosWriter w; ASSERT_TRUE(w.open(p));
    ASSERT_TRUE(w.write(a)); ASSERT_TRUE(w.write(b)); ASSERT_TRUE(w.write(c)); }

  {
    // 原样重放 a/b/c:三条全部 <= 文件里最后一条(c)的 stamp。
    PosWriter w;
    ASSERT_TRUE(w.open(p));
    EXPECT_TRUE(w.write(a));
    EXPECT_TRUE(w.last_write_was_suppressed());
    EXPECT_TRUE(w.write(b));
    EXPECT_TRUE(w.write(c));
    EXPECT_TRUE(w.last_write_was_suppressed())
        << "等于分界线本身(“至多”而不是“严格小于”)也应该算重叠";
    EXPECT_EQ(w.suppressed_duplicate_count(), 3u)
        << "三条全部重叠,用一个汇总计数报告,而不是各自单独报";
  }
  const auto back = read_pos(p);
  EXPECT_EQ(back.size(), 3u) << "重放的三条一条也不应该落盘,文件内容不变";
}

TEST(PosWriter, ReopeningWithAMixOfOverlappingAndNewRecordsOnlyAppendsTheNewOnes) {
  const std::string p = tmp_path("pw_resume_mix.pos");
  ::remove(p.c_str());
  const PosRecord a = sample_record();
  PosRecord b = a; b.stamp += 1.0;
  PosRecord c = a; c.stamp += 2.0;   // 文件里最后一条,续写分界线
  { PosWriter w; ASSERT_TRUE(w.open(p));
    ASSERT_TRUE(w.write(a)); ASSERT_TRUE(w.write(b)); ASSERT_TRUE(w.write(c)); }

  PosRecord d = a; d.stamp += 3.0;   // 晚于分界线,真正的新数据
  {
    PosWriter w;
    ASSERT_TRUE(w.open(p));
    EXPECT_TRUE(w.write(b));  // 重叠,去重
    EXPECT_TRUE(w.write(c));  // 等于分界线本身,去重
    EXPECT_TRUE(w.write(d));  // 新数据,应当落盘
    EXPECT_FALSE(w.last_write_was_suppressed());
    EXPECT_EQ(w.suppressed_duplicate_count(), 2u);
  }
  const auto back = read_pos(p);
  ASSERT_EQ(back.size(), 4u);
  EXPECT_NEAR(back[3].stamp, d.stamp, 1e-3);
}

// bug A(final-fix-report.md 之后的复盘):断电导致最后一行只写了一半、
// 没有换行结尾。旧实现里 write() 会直接接着这半行继续追加,下一条记录被
// 物理粘连、连同这半行一起在 read_pos 里读成一整行(损坏的)数据——这个
// 用例原来从不读回 next,因此从没真正验证过"next 完整无损"这件事,该 bug
// 一直没被抓到。现在的修复是 open() 发现最后一个字节不是 '\n' 就把这半行
// 原地截断(discarded_incomplete_line()),因此这里必须实际 read_pos() 回来
// 核对 a 保留、半行消失、next 完整。
TEST(PosWriter, PartiallyWrittenLastRowFromAPowerCutIsDiscardedAndNextRecordStaysIntact) {
  // 模拟断电:文件最后一行只写了一部分列(不足 10 列,没有换行结尾)。
  const std::string p = tmp_path("pw_resume_partial.pos");
  ::remove(p.c_str());
  const PosRecord a = sample_record();
  {
    std::ofstream f(p);
    f << pos_header(PosTimeSystem::GPST);
    f << format_pos_record(a, PosTimeSystem::GPST, 18);
    f << "2026/09/03 10:23:46.000 44.5 90.2";  // 断电:后面的列全部缺失,没有换行
  }

  PosRecord overlap = a;                  // 与文件里的 a 是同一个毫秒键,应当被去重
  PosRecord next = a; next.stamp += 1.0;  // 新数据,应当落盘

  PosWriter w;
  ASSERT_TRUE(w.open(p));
  EXPECT_TRUE(w.discarded_incomplete_line())
      << "最后一行没有换行结尾,必须被判定为断电半行并原地丢弃";
  EXPECT_TRUE(w.write(overlap));
  EXPECT_TRUE(w.last_write_was_suppressed())
      << "a 是文件里唯一保留下来的完整记录,重发同一个毫秒键必须去重";
  EXPECT_TRUE(w.write(next));
  EXPECT_FALSE(w.last_write_was_suppressed());
  EXPECT_EQ(w.suppressed_duplicate_count(), 1u);

  // 这就是原来这个用例缺的一步:实际读回 next,证明它没有被半行拖累、
  // 也没有跟半行粘连成一整行损坏数据。
  const auto back = read_pos(p);
  ASSERT_EQ(back.size(), 2u) << "a 保留,半行丢弃,next 追加——一共 2 条,不多不少";
  EXPECT_NEAR(back[0].stamp, a.stamp, 1e-3);
  EXPECT_NEAR(back[0].lat, a.lat, 1e-8);
  EXPECT_NEAR(back[1].stamp, next.stamp, 1e-3);
  EXPECT_NEAR(back[1].lat, next.lat, 1e-8);
  EXPECT_NEAR(back[1].ratio, next.ratio, 1e-6) << "next 必须完整无损,不能被半行污染";

  std::ifstream in(p);
  const std::string all((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  EXPECT_EQ(all.find("44.5 90.2"), std::string::npos)
      << "残缺行本身不应该原样留在最终文件里";
}

// bug A 的另一种表现:cut 不是落在"列数不够"处,而是恰好落在最后一列
// (ratio)中间——旧实现里,断电前的这一行本身列数是够的,分界线会正常
// 定到它前一条完整记录上;但 write() 仍然会直接接着这个半行追加,粘连出
// 的一整行第一部分正好还是能被 parse_llh_solution 解析出来的合法记录,只是
// ratio 列被下一行日期字符串的数字污染成一个离谱的值(reviewer 复现里是
// ratio=2026)。新设计从根源上避免这种粘连:open() 直接把半行截掉,后面的
// 写入永远从一个干净的新行开始。
TEST(PosWriter, TruncatedLastLineWithCutInsideRatioColumnIsDiscardedOnOpen) {
  const std::string p = tmp_path("pw_truncate_ratio.pos");
  ::remove(p.c_str());
  const PosRecord a = sample_record();
  PosRecord b = a; b.stamp += 1.0; b.ratio = 12.3;   // 这一行断电时只写了一半
  const std::string b_full = format_pos_record(b, PosTimeSystem::GPST, 18);
  ASSERT_EQ(b_full.back(), '\n');
  const std::string b_cut = b_full.substr(0, b_full.size() - 3);  // 砍掉结尾,cut 落在 ratio 列中间
  ASSERT_NE(b_cut.back(), '\n');

  {
    std::ofstream f(p);
    f << pos_header(PosTimeSystem::GPST);
    f << format_pos_record(a, PosTimeSystem::GPST, 18);  // 断电前已经完整落盘的一行
    f << b_cut;                                          // 断电:这一行只写了一半,没有换行结尾
  }

  PosRecord next = a; next.stamp += 2.0;
  PosWriter w;
  ASSERT_TRUE(w.open(p));
  EXPECT_TRUE(w.discarded_incomplete_line())
      << "b 那一行没有换行结尾,必须被判定为断电半行并原地丢弃";
  ASSERT_TRUE(w.write(next));
  EXPECT_FALSE(w.last_write_was_suppressed());

  const auto back = read_pos(p);
  ASSERT_EQ(back.size(), 2u) << "a 保留,b 的半行丢弃,next 追加——一共 2 条";
  EXPECT_NEAR(back[0].stamp, a.stamp, 1e-3);
  EXPECT_NEAR(back[0].ratio, a.ratio, 1e-6);
  EXPECT_NEAR(back[1].stamp, next.stamp, 1e-3);
  EXPECT_NEAR(back[1].ratio, next.ratio, 1e-6);
  for (const auto& r : back) {
    EXPECT_NE(r.ratio, 2026.0)
        << "不应该出现旧实现那种被下一行日期数字污染出来的离谱 ratio";
  }
}

TEST(PosWriter, DiscardedIncompleteLineIsFalseWhenTheFileEndsCleanly) {
  const std::string p = tmp_path("pw_clean_reopen.pos");
  ::remove(p.c_str());
  { PosWriter w; ASSERT_TRUE(w.open(p)); ASSERT_TRUE(w.write(sample_record())); }
  PosWriter w;
  ASSERT_TRUE(w.open(p));
  EXPECT_FALSE(w.discarded_incomplete_line())
      << "正常换行结尾的文件不应该被误判成断电半行";
}

// ---------- bug B:sub-ms stamp 与 .pos 的毫秒分辨率 ----------
// .pos 只存到毫秒,旧的"stamp <= 文件最后一条(读回来已经是 ms 取整)的
// 裸浮点 stamp"比较,对亚毫秒级的差异只是运气——reviewer 实测约一半概率
// 放过本该去重的边界记录。新设计按 format_pos_record 实际会渲染出的整数
// 毫秒键做精确成员判定,不再有随机性。

TEST(PosWriter, SubMillisecondBoundaryStampsAreDedupedAtTheFilesOwnMillisecondResolution) {
  const std::string p = tmp_path("pw_resume_subms.pos");
  ::remove(p.c_str());
  const double whole = 1788431025.0;   // 整数秒,避免额外的进位干扰计算
  PosRecord last = sample_record();
  last.stamp = whole + 0.625;   // 四舍五入到 ms=625,文件里就是 "...:...625"
  { PosWriter w; ASSERT_TRUE(w.open(p)); ASSERT_TRUE(w.write(last)); }

  PosRecord resend_same_ms = last;
  resend_same_ms.stamp = whole + 0.6250004;   // lround(625.0004)=625——与文件里那一行同一个毫秒键
  PosRecord resend_next_ms = last;
  resend_next_ms.stamp = whole + 0.6259996;   // lround(625.9996)=626——进到下一个毫秒键,是新数据

  PosWriter w;
  ASSERT_TRUE(w.open(p));
  EXPECT_TRUE(w.write(resend_same_ms));
  EXPECT_TRUE(w.last_write_was_suppressed())
      << "渲染出来仍是 ms=625,与文件里已有的那一行是同一个毫秒键,必须稳定去重";
  EXPECT_TRUE(w.write(resend_next_ms));
  EXPECT_FALSE(w.last_write_was_suppressed())
      << "渲染出来是 ms=626,进到了下一个整毫秒——真正的新数据,必须写盘";
  EXPECT_EQ(w.suppressed_duplicate_count(), 1u);

  const auto back = read_pos(p);
  ASSERT_EQ(back.size(), 2u);
  EXPECT_NEAR(back[0].stamp, whole + 0.625, 5e-4);
  EXPECT_NEAR(back[1].stamp, whole + 0.626, 5e-4);
}

// ---------- bug C:补录更早时段的空洞不应该被当成"重叠"整体丢弃 ----------

TEST(PosWriter, BackfillingAnEarlierGapIsWrittenNotSuppressed) {
  const std::string p = tmp_path("pw_backfill.pos");
  ::remove(p.c_str());
  const PosRecord later = sample_record();   // 文件里已有的、更晚的记录
  { PosWriter w; ASSERT_TRUE(w.open(p)); ASSERT_TRUE(w.write(later)); }

  PosRecord earlier = later;
  earlier.stamp -= 7200.0;   // 2 小时前的历元——重放一段填补更早时段空洞的 bag

  PosWriter w;
  ASSERT_TRUE(w.open(p));
  EXPECT_TRUE(w.write(earlier));
  EXPECT_FALSE(w.last_write_was_suppressed())
      << "旧的“<=文件末尾 stamp 就判定重叠”规则会把这条合法的补录数据连同"
         "“重叠”一起吞掉——现在按精确成员判定,不在集合里就必须写盘";
  EXPECT_EQ(w.suppressed_duplicate_count(), 0u);

  const auto back = read_pos(p);
  ASSERT_EQ(back.size(), 2u);
  EXPECT_NEAR(back[0].stamp, later.stamp, 1e-3);
  EXPECT_NEAR(back[1].stamp, earlier.stamp, 1e-3)
      << "补录的记录追加在文件末尾(乱序),但确实落盘了——.pos 逐行解析,"
         "不要求整体按时间有序";
}

// ---------- 新去重规则的不变量:同一个毫秒键至多出现一次,包括同一次运行内 ----------

TEST(PosWriter, RestartResendOfIdenticalStampsIsSuppressedWithCorrectCount) {
  const std::string p = tmp_path("pw_restart_resend.pos");
  ::remove(p.c_str());
  const PosRecord a = sample_record();
  PosRecord b = a; b.stamp += 1.0;
  PosRecord c = a; c.stamp += 2.0;
  { PosWriter w; ASSERT_TRUE(w.open(p));
    ASSERT_TRUE(w.write(a)); ASSERT_TRUE(w.write(b)); ASSERT_TRUE(w.write(c)); }

  // 进程重启后原样重放 a/b/c——三条的毫秒键都已经在文件里。
  PosWriter w;
  ASSERT_TRUE(w.open(p));
  EXPECT_TRUE(w.write(a));
  EXPECT_TRUE(w.write(b));
  EXPECT_TRUE(w.write(c));
  EXPECT_TRUE(w.last_write_was_suppressed());
  EXPECT_EQ(w.suppressed_duplicate_count(), 3u)
      << "三条全部重放,汇总计数必须精确等于 3,而不是漏计或多计";
  const auto back = read_pos(p);
  EXPECT_EQ(back.size(), 3u) << "重放的三条一条也不应该落盘,文件内容不变";
}

TEST(PosWriter, SameSessionDuplicateStampIsSuppressedWithoutReopening) {
  const std::string p = tmp_path("pw_samesession_dup.pos");
  ::remove(p.c_str());
  PosWriter w;
  ASSERT_TRUE(w.open(p));
  ASSERT_TRUE(w.write(sample_record()));
  EXPECT_FALSE(w.last_write_was_suppressed());
  EXPECT_TRUE(w.write(sample_record()))   // 同一次运行内,完全相同的 stamp 再写一次
      << "write() 判定为重复也返回 true,不是 I/O 失败";
  EXPECT_TRUE(w.last_write_was_suppressed())
      << "旧设计里分界线只在 open() 时确定一次,同一次运行内的重复完全不受"
         "保护;新设计里 write() 成功后立刻把键加入集合,同一次运行内的"
         "重复也会被挡住(不变量:每个毫秒键在整份文件生命周期内至多一次)";
  EXPECT_EQ(w.suppressed_duplicate_count(), 1u);
  const auto back = read_pos(p);
  EXPECT_EQ(back.size(), 1u);
}

TEST(PosWriter, OutputIsReadableByReadPos) {
  // 写出的东西必须能被既有的 read_pos 原样读回 —— 这是格式没写歪的真正证明
  const std::string p = tmp_path("pw_roundtrip.pos");
  ::remove(p.c_str());
  PosRecord a = sample_record();
  PosRecord b = sample_record(); b.stamp += 1.0; b.q = 2; b.ns = 20;
  { PosWriter w; ASSERT_TRUE(w.open(p)); ASSERT_TRUE(w.write(a)); ASSERT_TRUE(w.write(b)); }
  const auto back = read_pos(p);
  ASSERT_EQ(back.size(), 2u);
  EXPECT_NEAR(back[0].lat, a.lat, 1e-8);
  EXPECT_NEAR(back[0].stamp, a.stamp, 1e-3);
  EXPECT_EQ(back[1].q, 2);
  EXPECT_EQ(back[1].ns, 20);
}

TEST(PosWriter, MatchesWritePosByteForByte) {
  // PosWriter 与 write_pos 必须产出完全相同的内容,否则格式就有两份实现了
  const std::string p1 = tmp_path("pw_a.pos"), p2 = tmp_path("pw_b.pos");
  ::remove(p1.c_str()); ::remove(p2.c_str());
  std::vector<PosRecord> recs{sample_record(), sample_record()};
  recs[1].stamp += 1.0;
  write_pos(p1, recs);
  { PosWriter w; ASSERT_TRUE(w.open(p2)); for (const auto& r : recs) ASSERT_TRUE(w.write(r)); }
  std::ifstream f1(p1), f2(p2);
  const std::string s1((std::istreambuf_iterator<char>(f1)), std::istreambuf_iterator<char>());
  const std::string s2((std::istreambuf_iterator<char>(f2)), std::istreambuf_iterator<char>());
  EXPECT_EQ(s1, s2);
}

TEST(PosWriter, CreatesMissingParentDirectories) {
  const std::string dir = tmp_path("pw_deep/20260912");
  const std::string p = dir + "/can.pos";
  std::filesystem::remove_all(tmp_path("pw_deep"));
  PosWriter w;
  EXPECT_TRUE(w.open(p)) << "按天轮转会写到当天的新目录里,必须自动建目录";
  EXPECT_TRUE(w.write(sample_record()));
}

TEST(PosWriter, OpenFailureIsReportedNotThrown) {
  PosWriter w;
  EXPECT_FALSE(w.open("/proc/definitely-not-writable/x.pos"));
  EXPECT_FALSE(w.is_open());
  EXPECT_FALSE(w.write(sample_record())) << "未打开时写入应返回 false 而不是崩";
}

TEST(PosWriter, EachRecordIsFlushedSoACrashKeepsWhatWasWritten) {
  const std::string p = tmp_path("pw_flush.pos");
  ::remove(p.c_str());
  PosWriter w;
  ASSERT_TRUE(w.open(p));
  ASSERT_TRUE(w.write(sample_record()));
  // 不 close,直接从另一个句柄读 —— 没 flush 的话读不到数据行
  const auto back = read_pos(p);
  EXPECT_EQ(back.size(), 1u) << "每条写完必须 flush,否则崩溃会丢掉整个缓冲";
}

// ---------- round 2 review BLOCKING 1/2:检查/截断不完整末行这一步本身
// 失败时,open() 必须直接失败,绝不能猜"大概不需要截断"就继续 append ----------
// reviewer 用 LD_PRELOAD 在真实 pos_writer 二进制上注入了一次瞬时 EIO,
// 复现出"1000 行的文件被截到 1 字节,表头和全部数据行都没了"——旧代码里
// in.get() 读最后一个字节失败时,失败被当成"读到了 0",0 != '\n',于是被
// 误判成"最后一行不完整"再往下走。LD_PRELOAD shim 在 gtest 里很难做到
// 确定性,这里用 pos_io_test_hooks.hpp 提供的注入点复现同一条代码路径。

namespace {
std::string read_raw(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}
}  // namespace

TEST(PosWriter, InjectedFailureReadingTheLastByteLeavesFileUntouchedAndOpenFails) {
  const std::string p = tmp_path("pw_inject_lastbyte.pos");
  ::remove(p.c_str());
  const PosRecord a = sample_record();
  {
    std::ofstream f(p);
    f << pos_header(PosTimeSystem::GPST);
    f << format_pos_record(a, PosTimeSystem::GPST, 18);   // 完整的一行,末尾有换行符
  }
  const std::string original = read_raw(p);

  gnss_core::testing::set_trailing_line_read_failure_injector(
      [](const char* step) { return std::string(step) == "last_byte"; });
  PosWriter w;
  const bool opened = w.open(p);
  gnss_core::testing::set_trailing_line_read_failure_injector(nullptr);   // 立刻恢复,不污染后面的用例

  EXPECT_FALSE(opened)
      << "读最后一个字节失败时必须让 open() 直接失败,而不是把失败的读"
         "当成“最后一行不完整”去做一次基于错误数据的截断";
  EXPECT_EQ(read_raw(p), original) << "读失败之后文件必须一字节都没被动过";
}

TEST(PosWriter, InjectedFailureReadingTheFullContentLeavesFileUntouchedAndOpenFails) {
  const std::string p = tmp_path("pw_inject_fullcontent.pos");
  ::remove(p.c_str());
  const PosRecord a = sample_record();
  {
    std::ofstream f(p);
    f << pos_header(PosTimeSystem::GPST);
    f << format_pos_record(a, PosTimeSystem::GPST, 18);
    f << "2026/09/03 10:23:46.000 44.5 90.2";   // 没有换行结尾,真的需要走到"读整份内容"这一步
  }
  const std::string original = read_raw(p);

  gnss_core::testing::set_trailing_line_read_failure_injector(
      [](const char* step) { return std::string(step) == "full_content"; });
  PosWriter w;
  const bool opened = w.open(p);
  gnss_core::testing::set_trailing_line_read_failure_injector(nullptr);

  EXPECT_FALSE(opened)
      << "把整份文件读进内存这一步失败时同样必须让 open() 失败——reviewer 的"
         "复现里,这一步读出的 content 只有极少字节,find_last_of('\\n') 会"
         "算出一个几乎清空整个文件的错误截断点";
  EXPECT_EQ(read_raw(p), original)
      << "读失败之后文件必须一字节都没被动过(不完整的最后一行也不能被截掉)";
}

TEST(PosWriter, TruncationFailureLeavesFileUntouchedAndOpenFails) {
  // BLOCKING 2 的原始复现:文件只给 owner 写权限(chmod 200),读不了、
  // 也就没法先判断最后一行是不是完整——旧代码把"判断失败"和"不需要截断"
  // 混在一起,于是当成后者继续往下 append,把新记录粘连到还没写完的半行
  // 上,bug A 又回来了。
  const std::string p = tmp_path("pw_truncate_denied.pos");
  ::remove(p.c_str());
  const PosRecord a = sample_record();
  {
    std::ofstream f(p);
    f << pos_header(PosTimeSystem::GPST);
    f << format_pos_record(a, PosTimeSystem::GPST, 18);
    f << "2026/09/03 10:23:46.000 44.5 90.2";   // 没有换行结尾
  }
  const std::string original = read_raw(p);

  ASSERT_EQ(::chmod(p.c_str(), 0200), 0)
      << "测试前置条件:只给 owner 写权限——读不了,也就没法判断最后一行是否完整";

  PosWriter w;
  const bool opened = w.open(p);
  ::chmod(p.c_str(), 0644);   // 恢复权限,方便下面校验内容、也方便临时文件之后被正常清理

  EXPECT_FALSE(opened)
      << "判断/截断这一步失败时 open() 必须直接失败,不能假装“不需要截断”"
         "就继续往下 append";
  EXPECT_EQ(read_raw(p), original) << "打不开去读/截断时,文件必须一字节都没被动过";
}

TEST(PosWriter, ResizeFileFailureLeavesFileUntouchedAndOpenFails) {
  // 与上一个用例不同:这里要单独覆盖"读完全能成功、只是真正 resize_file
  // 这一步本身失败"的分支——chmod 200 会在更早的 ifstream 打开这一步就
  // 失败,不会走到 resize_file。chmod 444(只读、没有写权限)则相反:读
  // 完全没问题,但 POSIX 的 truncate() 要求对文件有写权限,会在这里失败。
  const std::string p = tmp_path("pw_truncate_readonly.pos");
  ::remove(p.c_str());
  const PosRecord a = sample_record();
  {
    std::ofstream f(p);
    f << pos_header(PosTimeSystem::GPST);
    f << format_pos_record(a, PosTimeSystem::GPST, 18);
    f << "2026/09/03 10:23:46.000 44.5 90.2";   // 没有换行结尾,确实需要走到 resize_file
  }
  const std::string original = read_raw(p);

  ASSERT_EQ(::chmod(p.c_str(), 0444), 0) << "测试前置条件:只读,没有写权限——能读完,但 truncate() 会被拒绝";

  PosWriter w;
  const bool opened = w.open(p);
  ::chmod(p.c_str(), 0644);   // 恢复权限,方便下面校验内容、也方便临时文件之后被正常清理

  EXPECT_FALSE(opened)
      << "resize_file() 本身失败(没有写权限)时 open() 必须直接失败";
  EXPECT_EQ(read_raw(p), original) << "resize_file 失败时文件必须一字节都没被动过";
}

// round 3 review 纠正了上一轮报告里的一个错误结论:上面这个 chmod 444
// 用例曾被当成"resize_file 失败分支是 defense-in-depth、可有可无"的证据,
// 理由是它跟 out_.open(path, ios::app) 共用同一个"要有写权限"的判定。
// reviewer 用 chattr +a(只读文件系统的 append-only 属性:允许 O_APPEND
// 追加,拒绝 truncate())构造出一个"能读、能 append,但 truncate() 会被
// EPERM 拒绝"的真实场景,证明这两个权限判定并不总是绑在一起——去掉
// resize_file 失败检查之后,open() 会误判成功,write() 真的把新记录
// append 了上去,直接粘连在还没被截掉的半行后面(reviewer 复现:988 字节
// 变成 1129 字节)。chattr +a 需要 root/CAP_LINUX_IMMUTABLE,这台机器上的
// 普通用户做不到(试过,Operation not permitted),所以这里用跟上面两个
// "读失败"用例同一套思路的确定性注入:让 resize 这一步本身失败,而真正
// 的 resize_file() 系统调用被跳过,不依赖任何特殊权限或 root。
TEST(PosWriter, InjectedResizeFailureLeavesFileUntouchedAndOpenFails) {
  const std::string p = tmp_path("pw_inject_resize.pos");
  ::remove(p.c_str());
  const PosRecord a = sample_record();
  {
    std::ofstream f(p);
    f << pos_header(PosTimeSystem::GPST);
    f << format_pos_record(a, PosTimeSystem::GPST, 18);
    f << "2026/09/03 10:23:46.000 44.5 90.2";   // 没有换行结尾,确实需要走到 resize_file 这一步
  }
  const std::string original = read_raw(p);

  gnss_core::testing::set_trailing_line_read_failure_injector(
      [](const char* step) { return std::string(step) == "resize"; });
  PosWriter w;
  const bool opened = w.open(p);
  gnss_core::testing::set_trailing_line_read_failure_injector(nullptr);

  EXPECT_FALSE(opened)
      << "resize_file() 本身失败这条分支是真正的第一道防线(reviewer 用 "
         "chattr +a 证明它跟 out_.open(app) 的权限判定并不总是绑在一起),"
         "去掉它 open() 就会误判成功";
  EXPECT_EQ(read_raw(p), original)
      << "注入失败时真正的 resize_file() 调用必须被跳过,文件一字节都不能变";
}

// ---------- round 2 review:三个当前没有测试守住的 mutant ----------

// (a) 如果 open() 收集去重键时把时间系统写死成 GPST,UTC 的 PosWriter
// 重启/补录之后就再也认不出文件里已经有哪些记录——此前没有任何一个
// PosWriter 用例用 UTC 构造过。

TEST(PosWriter, RestartResendIsSuppressedWhenTimeSystemIsUtc) {
  const std::string p = tmp_path("pw_utc_restart.pos");
  ::remove(p.c_str());
  const PosRecord a = sample_record();
  { PosWriter w(PosTimeSystem::UTC, 18); ASSERT_TRUE(w.open(p)); ASSERT_TRUE(w.write(a)); }

  PosWriter w(PosTimeSystem::UTC, 18);
  ASSERT_TRUE(w.open(p));
  EXPECT_TRUE(w.write(a));
  EXPECT_TRUE(w.last_write_was_suppressed())
      << "mutant:如果去重键的时间系统被写死成 GPST,UTC 的 PosWriter 重启后"
         "就认不出文件里已经有这条记录了";
  EXPECT_EQ(w.suppressed_duplicate_count(), 1u);
}

TEST(PosWriter, BackfillIsWrittenWhenTimeSystemIsUtc) {
  const std::string p = tmp_path("pw_utc_backfill.pos");
  ::remove(p.c_str());
  const PosRecord later = sample_record();
  { PosWriter w(PosTimeSystem::UTC, 18); ASSERT_TRUE(w.open(p)); ASSERT_TRUE(w.write(later)); }

  PosRecord earlier = later;
  earlier.stamp -= 7200.0;
  PosWriter w(PosTimeSystem::UTC, 18);
  ASSERT_TRUE(w.open(p));
  EXPECT_TRUE(w.write(earlier));
  EXPECT_FALSE(w.last_write_was_suppressed());
  const auto back = read_pos(p);
  ASSERT_EQ(back.size(), 2u);
}

// (b) 如果 open() 收集去重键时(或者 read_pos 本身)把闰秒写死成 18,
// 非 18 闰秒的会话重启后同样认不出已有记录——用一个明显不是 18 的值
// (37,当前真实的 GPS-UTC 闰秒数)覆盖。

TEST(PosWriter, RestartResendIsSuppressedWithNonDefaultLeapSeconds) {
  const std::string p = tmp_path("pw_leap37_restart.pos");
  ::remove(p.c_str());
  const PosRecord a = sample_record();
  { PosWriter w(PosTimeSystem::GPST, 37); ASSERT_TRUE(w.open(p)); ASSERT_TRUE(w.write(a)); }

  PosWriter w(PosTimeSystem::GPST, 37);
  ASSERT_TRUE(w.open(p));
  EXPECT_TRUE(w.write(a));
  EXPECT_TRUE(w.last_write_was_suppressed())
      << "mutant:如果去重键计算(或者 open() 传给 read_pos 的 leap_seconds)"
         "被写死成 18,非 18 闰秒的会话重启后就认不出已有记录";
  EXPECT_EQ(w.suppressed_duplicate_count(), 1u);
}

// (c) is_new 必须在"截断到 0 字节"之后被重新判定——否则唯一一行本来就
// 残缺、截完文件变空时,open() 会漏写表头,产出一个没有表头、下游工具
// 认不出时间系统的文件。用"连表头第一行都没写完整"这个更极端的场景来钉住:
// 这一行本身就是要被截掉的对象,截完之后整个文件必须变回"全新文件"那样
// 重新写出完整表头。

TEST(PosWriter, TruncationInsideTheFirstHeaderLineStillProducesACleanHeaderOnReopen) {
  const std::string p = tmp_path("pw_truncate_header.pos");
  ::remove(p.c_str());
  const std::string full_header = pos_header(PosTimeSystem::GPST);
  const std::string first_line = full_header.substr(0, full_header.find('\n'));
  ASSERT_GT(first_line.size(), 3u);
  const std::string cut = first_line.substr(0, first_line.size() - 3);   // 连第一行本身都没写完整
  { std::ofstream f(p); f << cut; }   // 没有换行结尾,文件里唯一的内容也是残缺的

  const PosRecord a = sample_record();
  PosWriter w;
  ASSERT_TRUE(w.open(p));
  EXPECT_TRUE(w.discarded_incomplete_line());
  ASSERT_TRUE(w.write(a));
  w.close();

  std::ifstream in(p);
  std::string line;
  int header_lines = 0, data_lines = 0;
  while (std::getline(in, line)) {
    if (!line.empty() && line[0] == '%') ++header_lines;
    else if (!line.empty()) ++data_lines;
  }
  EXPECT_EQ(header_lines, 3)
      << "截断到 0 字节之后必须重新判定成“新文件”,完整重写表头——不能因为"
         "“截断前文件非空”就跳过,否则会产出一个没有表头的文件";
  EXPECT_EQ(data_lines, 1);

  const auto back = read_pos(p);
  ASSERT_EQ(back.size(), 1u);
  EXPECT_NEAR(back[0].stamp, a.stamp, 1e-3);
}

// ---------- round 3 review:"读失败就必须 fail closed"这条规则少了一层 ----------
// open() 打开一个已有文件时,截断检查通过之后还会再调一次 read_pos() 去
// 收集去重键——这次调用原来包在一个"吞掉任何异常、照样返回 true(只是
// 没有去重保护)"的 try/catch 里,理由是"文件已经确认存在,这里失败极
// 不寻常"。但这次调用跟上面的截断检查读的是同一个文件、可能撞上同一种
// 瞬时 I/O 错误——继续吞掉、照样成功,existing_keys_ 就是空的,新写入的
// 记录不会跟文件里已有的内容去重,等于让这一整轮修复的三个数据正确性 bug
// (A/B/C)原样复发,只是触发条件从"截断"换成了"去重键收集"。这里跟上面
// 几个注入测试用的是同一个注入点(throw_if_injected),只是换了一个 step
// 名字("dedup_scan"),从 PosWriter::open() 里紧挨着 read_pos() 调用的
// 那一行之前抛出。

TEST(PosWriter, InjectedDedupScanFailureLeavesFileUntouchedAndOpenFails) {
  const std::string p = tmp_path("pw_inject_dedup_scan.pos");
  ::remove(p.c_str());
  const PosRecord a = sample_record();
  {
    PosWriter w;
    ASSERT_TRUE(w.open(p));
    ASSERT_TRUE(w.write(a));   // 完整、干净的文件——不需要触发截断分支,专门测去重键收集这一步
  }
  const std::string original = read_raw(p);

  gnss_core::testing::set_trailing_line_read_failure_injector(
      [](const char* step) { return std::string(step) == "dedup_scan"; });
  PosWriter w;
  const bool opened = w.open(p);
  gnss_core::testing::set_trailing_line_read_failure_injector(nullptr);

  EXPECT_FALSE(opened)
      << "收集去重键这一步(read_pos())失败时,open() 必须直接失败——不能"
         "像旧代码那样吞掉异常、照样返回 true 但没有去重保护,那等于让"
         "重放/重启重复写行的 bug 原样复发";
  EXPECT_EQ(read_raw(p), original) << "去重键收集失败时文件必须一字节都没被动过";
}
