#include <gtest/gtest.h>
#include <cstdint>
#include <vector>
#include "gnss_core/rtcm.hpp"
using namespace gnss_core;

namespace {

// 测试向量由 rtk-monitor 的参考实现(src/rtk_monitor/parsers/rtcm.py,已交付验证)生成,
// 并在生成时用该实现自检过分帧与坐标解析的往返一致性。跨实现交叉验证:
// 若本处 C++ 的 CRC 或位域取法与之不符,下列用例即失败。
const std::vector<uint8_t> kFrame1005 = {0xd3, 0x00, 0x13, 0x3e, 0xd4, 0xd2, 0x52, 0x3f, 0xf2, 0x9d,
                                         0x29, 0x8e, 0x0a, 0x96, 0xe4, 0x90, 0x87, 0x0a, 0x5d, 0x24,
                                         0x8a, 0xe5, 0x04, 0x70, 0x35};
const std::vector<uint8_t> kFrame1006 = {0xd3, 0x00, 0x15, 0x3e, 0xe4, 0xd2, 0x52, 0x3f, 0xf2, 0x9d,
                                         0x29, 0x8e, 0x0a, 0x96, 0xe4, 0x90, 0x87, 0x0a, 0x5d, 0x24,
                                         0x8a, 0xe5, 0x30, 0x39, 0x5b, 0x9b, 0x84};
constexpr double kX = -22458.1234, kY = 4548123.4567, kZ = 4451234.8901;
constexpr int kStationId = 1234;

std::vector<uint8_t> concat(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) {
  std::vector<uint8_t> out = a;
  out.insert(out.end(), b.begin(), b.end());
  return out;
}

}  // namespace

TEST(Crc24q, MatchesFrameTrailerOfReferenceVector) {
  // 帧尾 3 字节即帧前部的 CRC-24Q
  const size_t n = kFrame1005.size();
  const uint32_t expected = (static_cast<uint32_t>(kFrame1005[n - 3]) << 16) |
                            (static_cast<uint32_t>(kFrame1005[n - 2]) << 8) |
                            static_cast<uint32_t>(kFrame1005[n - 1]);
  EXPECT_EQ(crc24q(kFrame1005.data(), n - 3), expected);
}

TEST(RtcmFramer, ExtractsSingleMessage) {
  RtcmFramer framer;
  auto msgs = framer.feed(kFrame1005);
  ASSERT_EQ(msgs.size(), 1u);
  EXPECT_EQ(msgs[0].msg_type, 1005);
  EXPECT_EQ(msgs[0].payload.size(), kFrame1005.size() - 6);
  EXPECT_EQ(framer.crc_errors(), 0u);
  EXPECT_EQ(framer.buffered(), 0u);
}

TEST(RtcmFramer, ExtractsTwoBackToBackMessages) {
  RtcmFramer framer;
  auto msgs = framer.feed(concat(kFrame1005, kFrame1006));
  ASSERT_EQ(msgs.size(), 2u);
  EXPECT_EQ(msgs[0].msg_type, 1005);
  EXPECT_EQ(msgs[1].msg_type, 1006);
}

TEST(RtcmFramer, ReassemblesAcrossArbitraryChunkBoundaries) {
  // TCP 不保证按帧到达:逐字节喂,只有最后一个字节到达时才应吐出消息
  RtcmFramer framer;
  size_t total = 0;
  for (size_t i = 0; i + 1 < kFrame1005.size(); ++i) {
    total += framer.feed(&kFrame1005[i], 1).size();
  }
  EXPECT_EQ(total, 0u) << "帧未完整前不得吐出消息";
  auto msgs = framer.feed(&kFrame1005[kFrame1005.size() - 1], 1);
  ASSERT_EQ(msgs.size(), 1u);
  EXPECT_EQ(msgs[0].msg_type, 1005);
}

TEST(RtcmFramer, SkipsLeadingGarbageBeforeSyncByte) {
  RtcmFramer framer;
  std::vector<uint8_t> stream = {0x00, 0xff, 'J', 'U', 'N', 'K'};
  stream.insert(stream.end(), kFrame1005.begin(), kFrame1005.end());
  auto msgs = framer.feed(stream);
  ASSERT_EQ(msgs.size(), 1u);
  EXPECT_EQ(msgs[0].msg_type, 1005);
  EXPECT_EQ(framer.crc_errors(), 0u) << "前导垃圾不是 CRC 错误";
}

TEST(RtcmFramer, CountsCrcErrorAndStillRecoversNextFrame) {
  std::vector<uint8_t> corrupt = kFrame1005;
  corrupt[10] ^= 0xff;                       // 篡改 payload 一个字节
  RtcmFramer framer;
  auto msgs = framer.feed(concat(corrupt, kFrame1006));
  ASSERT_EQ(msgs.size(), 1u) << "坏帧必须被丢弃,后面的好帧必须仍被找到";
  EXPECT_EQ(msgs[0].msg_type, 1006);
  EXPECT_GE(framer.crc_errors(), 1u);
}

TEST(RtcmFramer, ResetClearsBufferAndCounters) {
  RtcmFramer framer;
  framer.feed(kFrame1005.data(), 5);         // 半个帧留在缓冲里
  EXPECT_GT(framer.buffered(), 0u);
  framer.reset();
  EXPECT_EQ(framer.buffered(), 0u);
  EXPECT_EQ(framer.crc_errors(), 0u);
}

TEST(ParseBaseStation, Parses1005EcefAndStationId) {
  RtcmFramer framer;
  auto msgs = framer.feed(kFrame1005);
  ASSERT_EQ(msgs.size(), 1u);
  BaseStationCoords c;
  ASSERT_TRUE(parse_base_station(msgs[0], c));
  EXPECT_EQ(c.station_id, kStationId);
  EXPECT_NEAR(c.x, kX, 1e-4);
  EXPECT_NEAR(c.y, kY, 1e-4);
  EXPECT_NEAR(c.z, kZ, 1e-4);
  EXPECT_FALSE(c.has_antenna_height) << "1005 不含天线高";
}

TEST(ParseBaseStation, Parses1006AntennaHeight) {
  RtcmFramer framer;
  auto msgs = framer.feed(kFrame1006);
  ASSERT_EQ(msgs.size(), 1u);
  BaseStationCoords c;
  ASSERT_TRUE(parse_base_station(msgs[0], c));
  EXPECT_NEAR(c.x, kX, 1e-4);
  EXPECT_TRUE(c.has_antenna_height);
  EXPECT_NEAR(c.antenna_height, 1.2345, 1e-4);
}

TEST(ParseBaseStation, RejectsOtherMessageTypes) {
  RtcmMessage m;
  m.msg_type = 1074;
  m.payload.assign(30, 0);
  BaseStationCoords c;
  EXPECT_FALSE(parse_base_station(m, c));
}

TEST(ParseBaseStation, RejectsTruncatedPayload) {
  RtcmMessage m;
  m.msg_type = 1005;
  m.payload.assign(10, 0);                   // 1005 需要 19 字节
  BaseStationCoords c;
  EXPECT_FALSE(parse_base_station(m, c));
}

TEST(ParseBaseStation, DecodesNegativeEcefComponent) {
  // kX 为负,38 bit 有符号必须正确扩展(位域取错会得到一个巨大的正数)
  RtcmFramer framer;
  auto msgs = framer.feed(kFrame1005);
  BaseStationCoords c;
  ASSERT_TRUE(parse_base_station(msgs[0], c));
  EXPECT_LT(c.x, 0.0);
}
