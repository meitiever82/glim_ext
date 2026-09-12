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
  // P0 未定:板卡原始格式若不是 rtcm3(如 novatel / ublox),只改参数不改代码
  RtkrcvConfParams p;
  p.obs_format = "novatel";
  p.corr_format = "rtcm3";
  const auto c = render_rtkrcv_conf(p);
  EXPECT_TRUE(has_line(c, "inpstr1-format =novatel"));
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
