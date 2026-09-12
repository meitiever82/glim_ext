#include <gtest/gtest.h>
#include <cstdio>
#include <fstream>
#include <string>
#include "gnss_core/pos_io.hpp"
using namespace gnss_core;

namespace {
std::string tmp_path(const char* name) {
  const char* dir = std::getenv("TMPDIR");
  return std::string(dir ? dir : "/tmp") + "/" + name;
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
