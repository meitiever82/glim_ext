#include <gtest/gtest.h>
#include <string>
#include <cmath>
#include <limits>
#include <sstream>
#include "gnss_bringup/rtkrcv_conf.hpp"
using namespace gnss_bringup;

namespace {
bool has_line(const std::string& conf, const std::string& line) {
  // Anchor match to start of string or immediately after a newline
  if (conf.find(line + "\n") == 0) return true;
  std::string search = "\n" + line + "\n";
  return conf.find(search) != std::string::npos;
}
}  // namespace

TEST(RtkrcvConf, WiresBothInputsAsLocalTcpClients) {
  // rtkrcv 必须主动连本机的两个端口,而不是自己去连平台——
  // 对外 TCP 只在 rtcm_bridge 里存在(spec §5.1 单一路径)
  RtkrcvConfParams p;
  p.obs_port = 15032;
  p.corr_port = 15031;
  const auto c = render_rtkrcv_conf(p);
  EXPECT_TRUE(has_line(c, "inpstr1-type =tcpcli"));
  EXPECT_TRUE(has_line(c, "inpstr1-path =127.0.0.1:15032"));
  EXPECT_TRUE(has_line(c, "inpstr2-type =tcpcli"));
  EXPECT_TRUE(has_line(c, "inpstr2-path =127.0.0.1:15031"));
}

TEST(RtkrcvConf, PublishesSolutionAsLocalTcpServer) {
  RtkrcvConfParams p;
  p.sol_port = 15020;
  const auto c = render_rtkrcv_conf(p);
  EXPECT_TRUE(has_line(c, "outstr1-type =tcpsvr"));
  EXPECT_TRUE(has_line(c, "outstr1-path =:15020"));
  EXPECT_TRUE(has_line(c, "outstr1-format =llh"));
}

TEST(RtkrcvConf, SolutionFormatIsHeadlessLlhInGpst) {
  // parse_llh_solution 按无表头、GPST 的 llh 列序解析;这三行是它的前提
  const auto c = render_rtkrcv_conf(RtkrcvConfParams{});
  EXPECT_TRUE(has_line(c, "out-solformat =llh"));
  EXPECT_TRUE(has_line(c, "out-outhead =off"));
  EXPECT_TRUE(has_line(c, "out-timesys =gpst"));
}

TEST(RtkrcvConf, StreamFormatsAreConfigurable) {
  // P0 未定:板卡原始格式若不是 rtcm3(如 oem4 / ublox),只改参数不改代码
  RtkrcvConfParams p;
  p.obs_format = "oem4";
  p.corr_format = "rtcm3";
  const auto c = render_rtkrcv_conf(p);
  EXPECT_TRUE(has_line(c, "inpstr1-format =oem4"));
  EXPECT_TRUE(has_line(c, "inpstr2-format =rtcm3"));
}

TEST(RtkrcvConf, DefaultFormatsMatchRtkMonitorAssumption) {
  const auto c = render_rtkrcv_conf(RtkrcvConfParams{});
  EXPECT_TRUE(has_line(c, "inpstr1-format =rtcm3"));
  EXPECT_TRUE(has_line(c, "inpstr2-format =rtcm3"));
}

TEST(RtkrcvConf, PositioningOptionsAreConfigurable) {
  RtkrcvConfParams p;
  p.pos_mode = "static";
  p.elmask = 15.0;
  p.ar_mode = "fix-and-hold";
  p.navsys = 5;
  const auto c = render_rtkrcv_conf(p);
  EXPECT_TRUE(has_line(c, "pos1-posmode =static"));
  EXPECT_TRUE(has_line(c, "pos1-elmask =15"));
  EXPECT_TRUE(has_line(c, "pos2-armode =fix-and-hold"));
  EXPECT_TRUE(has_line(c, "pos1-navsys =5"));
}

TEST(RtkrcvConf, RejectsObsFormatWithNewline) {
  RtkrcvConfParams p;
  p.obs_format = "rtcm3\ninpstr1-path =attacker:1234";
  EXPECT_THROW(render_rtkrcv_conf(p), std::invalid_argument);
}

TEST(RtkrcvConf, RejectsCorrFormatWithNewline) {
  RtkrcvConfParams p;
  p.corr_format = "rtcm3\ninpstr2-path =attacker:1234";
  EXPECT_THROW(render_rtkrcv_conf(p), std::invalid_argument);
}

TEST(RtkrcvConf, RejectsObsFormatWithEqualsSign) {
  RtkrcvConfParams p;
  p.obs_format = "rtcm3=evil";
  EXPECT_THROW(render_rtkrcv_conf(p), std::invalid_argument);
}

TEST(RtkrcvConf, RejectsCorrFormatWithEqualsSign) {
  RtkrcvConfParams p;
  p.corr_format = "rtcm3=evil";
  EXPECT_THROW(render_rtkrcv_conf(p), std::invalid_argument);
}

TEST(RtkrcvConf, RejectsPosModeWithCarriageReturn) {
  RtkrcvConfParams p;
  p.pos_mode = "kinematic\rkinematic";
  EXPECT_THROW(render_rtkrcv_conf(p), std::invalid_argument);
}

TEST(RtkrcvConf, RejectsArModeWithNewline) {
  RtkrcvConfParams p;
  p.ar_mode = "continuous\ninpstr1-type =evil";
  EXPECT_THROW(render_rtkrcv_conf(p), std::invalid_argument);
}

TEST(RtkrcvConf, ElmaskMinBoundaryIsValid) {
  RtkrcvConfParams p;
  p.elmask = 0.0;
  const auto c = render_rtkrcv_conf(p);
  EXPECT_TRUE(has_line(c, "pos1-elmask =0"));
}

TEST(RtkrcvConf, ElmaskMaxBoundaryIsValid) {
  RtkrcvConfParams p;
  p.elmask = 90.0;
  const auto c = render_rtkrcv_conf(p);
  EXPECT_TRUE(has_line(c, "pos1-elmask =90"));
}

TEST(RtkrcvConf, ElmaskBelowMinRejected) {
  RtkrcvConfParams p;
  p.elmask = -1.0;
  EXPECT_THROW(render_rtkrcv_conf(p), std::invalid_argument);
}

TEST(RtkrcvConf, ElmaskAboveMaxRejected) {
  RtkrcvConfParams p;
  p.elmask = 91.0;
  EXPECT_THROW(render_rtkrcv_conf(p), std::invalid_argument);
}

TEST(RtkrcvConf, ElmaskNaNRejected) {
  RtkrcvConfParams p;
  p.elmask = std::nan("");
  EXPECT_THROW(render_rtkrcv_conf(p), std::invalid_argument);
}

TEST(RtkrcvConf, ElmaskInfinityRejected) {
  RtkrcvConfParams p;
  p.elmask = std::numeric_limits<double>::infinity();
  EXPECT_THROW(render_rtkrcv_conf(p), std::invalid_argument);
}

TEST(RtkrcvConf, FractionalElmaskFormatsCorrectly) {
  RtkrcvConfParams p;
  p.elmask = 13.5;
  const auto c = render_rtkrcv_conf(p);
  EXPECT_TRUE(has_line(c, "pos1-elmask =13.5"));
}

TEST(RtkrcvConf, WholeNumberElmaskFormatsWithoutDecimal) {
  RtkrcvConfParams p;
  p.elmask = 15.0;
  const auto c = render_rtkrcv_conf(p);
  EXPECT_TRUE(has_line(c, "pos1-elmask =15"));
}

TEST(RtkrcvConf, BasePositionComesFromRtcmByDefault) {
  // 回归(2026-09-14 实测):不写 ant2-postype 时 rtkrcv 默认 llh 0,0,0,
  // RTK 模式一条解都不输出。
  const auto c = render_rtkrcv_conf(RtkrcvConfParams{});
  EXPECT_TRUE(has_line(c, "ant2-postype =rtcm"));
}

TEST(RtkrcvConf, SinglePointBasePositionIsSelectable) {
  RtkrcvConfParams p;
  p.base_pos_type = "single";
  EXPECT_TRUE(has_line(render_rtkrcv_conf(p), "ant2-postype =single"));
}

TEST(RtkrcvConf, BasePositionTypesNeedingCoordinatesAreRejected) {
  RtkrcvConfParams p;
  p.base_pos_type = "llh";
  EXPECT_THROW(render_rtkrcv_conf(p), std::invalid_argument);
}

TEST(RtkrcvConf, AmbiguityResolutionOptionsArePinnedExplicitly) {
  // 显式写出 2.5.1 自身的默认值:换 RTKLIB 版本时默认值悄悄变化不会带进来
  const auto c = render_rtkrcv_conf(RtkrcvConfParams{});
  EXPECT_TRUE(has_line(c, "pos2-bdsarmode =off"));
  EXPECT_TRUE(has_line(c, "pos2-gloarmode =fix-and-hold"));

  RtkrcvConfParams p;
  p.bds_ar_mode = "on";
  p.glo_ar_mode = "autocal";
  const auto c2 = render_rtkrcv_conf(p);
  EXPECT_TRUE(has_line(c2, "pos2-bdsarmode =on"));
  EXPECT_TRUE(has_line(c2, "pos2-gloarmode =autocal"));
}

TEST(RtkrcvConf, UnknownEnumValuesAreRejectedInsteadOfSilentlyFallingBack) {
  // rtkrcv 遇到非法取值只打一行警告、回落到默认值继续跑(实测
  // pos2-armode =continuouss → fix-and-hold),所以必须在生成 conf 时拒绝
  const auto expect_rejected = [](RtkrcvConfParams p, const std::string& field) {
    try {
      render_rtkrcv_conf(p);
      ADD_FAILURE() << field << " 的非法取值没有被拒绝";
    } catch (const std::invalid_argument& e) {
      EXPECT_EQ(std::string(e.what()).rfind(field + "=", 0), 0u) << e.what();
    }
  };
  RtkrcvConfParams p;
  p.pos_mode = "kinematicc";   expect_rejected(p, "pos_mode");     p = {};
  p.ar_mode = "continuouss";   expect_rejected(p, "ar_mode");      p = {};
  p.obs_format = "novatel";    expect_rejected(p, "obs_format");   p = {};
  p.corr_format = "rtcm";      expect_rejected(p, "corr_format");  p = {};
  p.bds_ar_mode = "yes";       expect_rejected(p, "bds_ar_mode");  p = {};
  p.glo_ar_mode = "hold";      expect_rejected(p, "glo_ar_mode");  p = {};
  p.base_pos_type = "xyz";     expect_rejected(p, "base_pos_type");
}

TEST(RtkrcvConf, EveryRtklibEx251PositioningModeIsAccepted) {
  for (const char* m : {"single", "dgps", "kinematic", "static", "static-start", "movingbase",
                        "fixed", "ppp-kine", "ppp-static", "ppp-fixed"}) {
    RtkrcvConfParams p;
    p.pos_mode = m;
    EXPECT_NO_THROW(render_rtkrcv_conf(p)) << m;
  }
}

TEST(RtkrcvConf, NavsysOutsideTheSystemBitmaskIsRejected) {
  RtkrcvConfParams p;
  p.navsys = 0;
  EXPECT_THROW(render_rtkrcv_conf(p), std::invalid_argument);
  p.navsys = 128;
  EXPECT_THROW(render_rtkrcv_conf(p), std::invalid_argument);
  p.navsys = 127;
  EXPECT_NO_THROW(render_rtkrcv_conf(p));
}

// ---------- Task 6 F1:模糊度固定的高度角门限(pos2-arelmask)----------

TEST(RtkrcvConf, ArElevationMaskDefaultsTo15Degrees) {
  // 现场回归(红沙泉 2026-09-15 seg_164931_165748,rnx2rtkp 离线复现与实时一致):
  // elmask 10、不写 arelmask 时 0/498 固定;elmask 10 + arelmask 15 时 490/498,首次固定 +8 s。
  // 10–15° 的卫星伪距多路径大,参与浮点解无妨,参与模糊度固定会把 ratio 压在 1.1–1.6。
  const auto c = render_rtkrcv_conf(RtkrcvConfParams{});
  EXPECT_TRUE(has_line(c, "pos1-elmask =10")) << c;
  EXPECT_TRUE(has_line(c, "pos2-arelmask =15")) << c;
}

TEST(RtkrcvConf, ArElevationMaskIsConfigurable) {
  RtkrcvConfParams p;
  p.ar_elmask = 12.5;
  EXPECT_TRUE(has_line(render_rtkrcv_conf(p), "pos2-arelmask =12.5"));
  p.ar_elmask = 0.0;
  EXPECT_TRUE(has_line(render_rtkrcv_conf(p), "pos2-arelmask =0"));
  p.ar_elmask = 90.0;
  EXPECT_TRUE(has_line(render_rtkrcv_conf(p), "pos2-arelmask =90"));
}

TEST(RtkrcvConf, ArElevationMaskBelowElmaskIsAcceptedAsHarmless) {
  // RTKLIB-EX 2.5.1 rtkpos.c 里 elmaskar 只在挑选参与固定的卫星时比较;低于 elmask 的卫星
  // 本来就不在解里,所以 ar_elmask < elmask 等于不起作用,不是错误配置
  RtkrcvConfParams p;
  p.elmask = 15.0;
  p.ar_elmask = 5.0;
  EXPECT_TRUE(has_line(render_rtkrcv_conf(p), "pos2-arelmask =5"));
}

TEST(RtkrcvConf, ArElevationMaskOutOfRangeOrNonFiniteIsRejected) {
  for (double bad : {-0.5, 90.5, std::nan(""), std::numeric_limits<double>::infinity(),
                     -std::numeric_limits<double>::infinity()}) {
    RtkrcvConfParams p;
    p.ar_elmask = bad;
    EXPECT_THROW(render_rtkrcv_conf(p), std::invalid_argument) << bad;
  }
}

// ---------- 键名防拼错:rtkrcv 对不认识的键静默忽略 ----------
namespace {
// 抄自 RTKLIB-EX 2.5.1 源码(/home/steve/Documents/GitHub/gnss-alg/RTKLIB-2.5.1,
// src/options.c sysopts[] sha256 92751e9f…,app/consapp/rtkrcv/rtkrcv.c rcvopts[] sha256 4e6d36f7…),
// 保持表内原顺序。提取命令:grep -oP '^\s*\{"\K[^"]+(?=",\s*\d)' <file>
const char* const kSysopts[] = {
    "pos1-posmode", "pos1-frequency", "pos1-soltype", "pos1-elmask", "pos1-snrmask_r",
    "pos1-snrmask_b", "pos1-snrmask_L1", "pos1-snrmask_L2", "pos1-snrmask_L5", "pos1-snrmask_L6",
    "pos1-dynamics", "pos1-tidecorr", "pos1-ionoopt", "pos1-tropopt", "pos1-sateph",
    "pos1-posopt1", "pos1-posopt2", "pos1-posopt3", "pos1-posopt4", "pos1-posopt5",
    "pos1-posopt6", "pos1-exclsats", "pos1-navsys", "pos2-armode", "pos2-gloarmode",
    "pos2-bdsarmode", "pos2-arfilter", "pos2-arthres", "pos2-arthresmin", "pos2-arthresmax",
    "pos2-arthres1", "pos2-arthres2", "pos2-arthres3", "pos2-arthres4", "pos2-varholdamb",
    "pos2-gainholdamb", "pos2-arlockcnt", "pos2-minfixsats", "pos2-minholdsats", "pos2-mindropsats",
    "pos2-arelmask", "pos2-arminfix", "pos2-armaxiter", "pos2-elmaskhold", "pos2-aroutcnt",
    "pos2-maxage", "pos2-syncsol", "pos2-slipthres", "pos2-dopthres", "pos2-rejionno",
    "pos2-rejphase", "pos2-rejcode", "pos2-niter", "pos2-baselen", "pos2-basesig",
    "out-solformat", "out-outhead", "out-outopt", "out-outvel", "out-timesys",
    "out-timeform", "out-timendec", "out-degform", "out-fieldsep", "out-outsingle",
    "out-maxsolstd", "out-height", "out-geoid", "out-solstatic", "out-nmeaintv1",
    "out-nmeaintv2", "out-outstat", "stats-eratio1", "stats-eratio2", "stats-eratio5",
    "stats-eratio6", "stats-errphase", "stats-errphaseel", "stats-errphasebl", "stats-errdoppler",
    "stats-snrmax", "stats-errsnr", "stats-errrcv", "stats-stdbias", "stats-stdiono",
    "stats-stdtrop", "stats-prnaccelh", "stats-prnaccelv", "stats-prnbias", "stats-prniono",
    "stats-prntrop", "stats-prnpos", "stats-clkstab", "ant1-postype", "ant1-pos1",
    "ant1-pos2", "ant1-pos3", "ant1-anttype", "ant1-antdele", "ant1-antdeln",
    "ant1-antdelu", "ant2-postype", "ant2-pos1", "ant2-pos2", "ant2-pos3",
    "ant2-anttype", "ant2-antdele", "ant2-antdeln", "ant2-antdelu", "ant2-maxaveep",
    "ant2-initrst", "misc-timeinterp", "misc-sbasatsel", "misc-rnxopt1", "misc-rnxopt2",
    "misc-pppopt", "file-satantfile", "file-rcvantfile", "file-staposfile", "file-geoidfile",
    "file-ionofile", "file-dcbfile", "file-eopfile", "file-blqfile", "file-tempdir",
    "file-geexefile", "file-solstatfile", "file-tracefile"};
const char* const kRcvopts[] = {
    "console-passwd", "console-timetype", "console-soltype", "console-solflag", "inpstr1-type",
    "inpstr2-type", "inpstr3-type", "inpstr1-path", "inpstr2-path", "inpstr3-path",
    "inpstr1-format", "inpstr2-format", "inpstr3-format", "inpstr1-rcvopt", "inpstr2-rcvopt",
    "inpstr3-rcvopt", "inpstr2-nmeareq", "inpstr2-nmealat", "inpstr2-nmealon", "inpstr2-nmeahgt",
    "outstr1-type", "outstr2-type", "outstr1-path", "outstr2-path", "outstr1-format",
    "outstr2-format", "logstr1-type", "logstr2-type", "logstr3-type", "logstr1-path",
    "logstr2-path", "logstr3-path", "misc-svrcycle", "misc-timeout", "misc-reconnect",
    "misc-nmeacycle", "misc-buffsize", "misc-navmsgsel", "misc-proxyaddr", "misc-fswapmargin",
    "misc-startcmd", "misc-stopcmd", "file-cmdfile1", "file-cmdfile2", "file-cmdfile3"};

// 照搬 options.c searchopt():按表序返回第一个"表内键名包含给定键"的条目(strstr,不是全等)
template <std::size_t N>
const char* rtklib_searchopt(const std::string& key, const char* const (&table)[N]) {
  for (std::size_t i = 0; i < N; ++i) {
    if (std::string(table[i]).find(key) != std::string::npos) return table[i];
  }
  return nullptr;
}
}  // namespace

TEST(RtkrcvConf, EveryRenderedKeyResolvesToItselfInRtklibEx251OptionTables) {
  // rtkrcv 先后用 rcvopts、sysopts 两张表 loadopts 同一份 conf,查不到的键直接 continue(无任何输出)。
  // 所以键名拼错不会报错,只是设置不生效;而且 searchopt 是子串匹配,键名还不能是别的键的子串。
  const auto c = render_rtkrcv_conf(RtkrcvConfParams{});
  std::istringstream in(c);
  std::string line;
  int n = 0;
  while (std::getline(in, line)) {
    const auto eq = line.find('=');
    ASSERT_NE(eq, std::string::npos) << line;
    std::string key = line.substr(0, eq);
    while (!key.empty() && key.back() == ' ') key.pop_back();
    const char* sys = rtklib_searchopt(key, kSysopts);
    const char* rcv = rtklib_searchopt(key, kRcvopts);
    EXPECT_TRUE(sys != nullptr || rcv != nullptr) << "RTKLIB-EX 2.5.1 不认识的键: " << key;
    if (sys) EXPECT_EQ(std::string(sys), key) << "sysopts 会把 " << key << " 当成 " << sys;
    if (rcv) EXPECT_EQ(std::string(rcv), key) << "rcvopts 会把 " << key << " 当成 " << rcv;
    ++n;
  }
  EXPECT_GT(n, 0);  // 防止空串让本用例空转
}
