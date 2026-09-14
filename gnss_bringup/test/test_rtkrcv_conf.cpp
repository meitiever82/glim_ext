#include <gtest/gtest.h>
#include <string>
#include <cmath>
#include <limits>
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
