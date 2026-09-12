#include <gtest/gtest.h>
#include <string>
#include "gnss_bringup/rtkrcv_conf.hpp"
using namespace gnss_bringup;

namespace {
bool has_line(const std::string& conf, const std::string& line) {
  return conf.find(line + "\n") != std::string::npos;
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
