#include <gtest/gtest.h>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include "gnss_core/pos_io.hpp"
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
  PosWriter w;
  ASSERT_TRUE(w.open(p));
  ASSERT_TRUE(w.write(sample_record()));
  ASSERT_TRUE(w.write(sample_record()));
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
  { PosWriter w; ASSERT_TRUE(w.open(p)); ASSERT_TRUE(w.write(sample_record())); }
  std::ifstream in(p);
  std::string line; int header_lines = 0, data_lines = 0;
  while (std::getline(in, line)) { if (!line.empty() && line[0] == '%') ++header_lines; else if (!line.empty()) ++data_lines; }
  EXPECT_EQ(data_lines, 2);
  EXPECT_EQ(header_lines, 3) << "重开不得再写一遍表头";
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
