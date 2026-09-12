#include "gnss_core/rtcm.hpp"

namespace gnss_core {

namespace {

constexpr uint32_t kCrc24qPoly = 0x1864CFB;
constexpr uint8_t kSyncByte = 0xD3;
constexpr size_t kHeaderBytes = 3;
constexpr size_t kCrcBytes = 3;

// 从 payload 的第 start 个 bit 起取 length 个 bit(MSB 在前),无符号。
uint64_t get_bits(const std::vector<uint8_t>& d, size_t start, size_t length) {
  uint64_t v = 0;
  for (size_t i = start; i < start + length; ++i) {
    v = (v << 1) | ((d[i / 8] >> (7 - i % 8)) & 1);
  }
  return v;
}

// 同上,但按二进制补码做符号扩展(1005 的 ECEF 分量是 38 bit 有符号)。
int64_t get_sbits(const std::vector<uint8_t>& d, size_t start, size_t length) {
  const uint64_t v = get_bits(d, start, length);
  const uint64_t sign_bit = uint64_t(1) << (length - 1);
  return (v & sign_bit) ? static_cast<int64_t>(v) - static_cast<int64_t>(uint64_t(1) << length)
                        : static_cast<int64_t>(v);
}

}  // namespace

uint32_t crc24q(const uint8_t* data, size_t len) {
  uint32_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint32_t>(data[i]) << 16;
    for (int b = 0; b < 8; ++b) {
      crc <<= 1;
      if (crc & 0x1000000) crc ^= kCrc24qPoly;
    }
  }
  return crc & 0xFFFFFF;
}

std::vector<RtcmMessage> RtcmFramer::feed(const uint8_t* data, size_t len) {
  buf_.insert(buf_.end(), data, data + len);

  std::vector<RtcmMessage> out;
  while (true) {
    // 找同步字;之前的都是垃圾(不计 CRC 错误)
    size_t start = 0;
    while (start < buf_.size() && buf_[start] != kSyncByte) ++start;
    if (start > 0) buf_.erase(buf_.begin(), buf_.begin() + start);
    if (buf_.size() < kHeaderBytes + kCrcBytes) break;

    // 头 3 字节:D3 + 6 bit 保留 + 10 bit payload 长度
    const size_t payload_len = ((static_cast<size_t>(buf_[1]) & 0x03) << 8) | buf_[2];
    const size_t total = kHeaderBytes + payload_len + kCrcBytes;
    if (buf_.size() < total) break;   // 帧未到齐,等下一块

    const uint32_t want = (static_cast<uint32_t>(buf_[total - 3]) << 16) |
                          (static_cast<uint32_t>(buf_[total - 2]) << 8) |
                          static_cast<uint32_t>(buf_[total - 1]);
    if (crc24q(buf_.data(), total - kCrcBytes) == want) {
      RtcmMessage msg;
      msg.payload.assign(buf_.begin() + kHeaderBytes, buf_.begin() + kHeaderBytes + payload_len);
      // DF002 消息号 = payload 前 12 bit
      msg.msg_type = payload_len >= 2 ? static_cast<int>((msg.payload[0] << 4) | (msg.payload[1] >> 4)) : 0;
      out.push_back(std::move(msg));
      buf_.erase(buf_.begin(), buf_.begin() + total);
    } else {
      // 只丢 1 字节再找同步字:这个 0xD3 可能是数据里的巧合,真正的帧头还在后面
      ++crc_errors_;
      buf_.erase(buf_.begin());
    }
  }
  return out;
}

bool parse_base_station(const RtcmMessage& msg, BaseStationCoords& out) {
  const bool is_1006 = msg.msg_type == 1006;
  if (msg.msg_type != 1005 && !is_1006) return false;
  // 1005 = 152 bit = 19 字节;1006 多 16 bit 天线高 = 21 字节
  const size_t need = is_1006 ? 21u : 19u;
  if (msg.payload.size() < need) return false;

  out = BaseStationCoords{};
  out.station_id = static_cast<int>(get_bits(msg.payload, 12, 12));   // DF003
  out.x = static_cast<double>(get_sbits(msg.payload, 34, 38)) * 1e-4;   // DF025
  out.y = static_cast<double>(get_sbits(msg.payload, 74, 38)) * 1e-4;   // DF026
  out.z = static_cast<double>(get_sbits(msg.payload, 114, 38)) * 1e-4;  // DF027
  if (is_1006) {
    out.antenna_height = static_cast<double>(get_bits(msg.payload, 152, 16)) * 1e-4;  // DF028
    out.has_antenna_height = true;
  }
  return true;
}

}  // namespace gnss_core
