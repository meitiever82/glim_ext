#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

namespace gnss_core {

// RTCM3 帧尾的 CRC-24Q(生成多项式 0x1864CFB,初值 0),对帧头到 payload 末尾计算。
uint32_t crc24q(const uint8_t* data, size_t len);

struct RtcmMessage {
  int msg_type = 0;                // DF002,payload 前 12 bit
  std::vector<uint8_t> payload;    // 不含 3 字节帧头与 3 字节 CRC
};

// 增量分帧器:喂任意切分的字节块(TCP 不保证按帧到达),吐出通过 CRC 校验的完整消息。
// 遇到 CRC 不过的帧:计数 +1,并只丢弃 1 字节后重新找同步字(避免把后面真正的帧一起丢掉)。
class RtcmFramer {
public:
  std::vector<RtcmMessage> feed(const uint8_t* data, size_t len);
  std::vector<RtcmMessage> feed(const std::vector<uint8_t>& data) { return feed(data.data(), data.size()); }

  size_t crc_errors() const { return crc_errors_; }
  size_t buffered() const { return buf_.size(); }   // 尚未凑成完整帧的残留字节
  void reset() { buf_.clear(); crc_errors_ = 0; }

private:
  std::vector<uint8_t> buf_;
  size_t crc_errors_ = 0;
};

// 1005 / 1006 基站天线参考点坐标(spec §3 C2 base_shift 的输入)。
struct BaseStationCoords {
  int station_id = 0;
  double x = 0.0, y = 0.0, z = 0.0;   // ECEF, m
  double antenna_height = 0.0;        // m,仅 1006 提供
  bool has_antenna_height = false;
};

// 解析 1005/1006;msg_type 非 1005/1006 或 payload 长度不足时返回 false。
bool parse_base_station(const RtcmMessage& msg, BaseStationCoords& out);

}  // namespace gnss_core
