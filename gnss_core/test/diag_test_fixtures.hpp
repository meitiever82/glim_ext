#pragma once
// 诊断相关测试共用的输入构造:rtkrcv $SAT 行、RTCM 1005 帧。
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "gnss_core/rtcm.hpp"

namespace gnss_core::test_fixtures {

// RTKLIB 列序:$SAT,week,tow,sat,frq,az,el,resp,resc,vsat,snr,fix,slip,lock,outc,slipc,rejc
inline std::string sat_line(const char* sat, double tow, int frq, double el, double resp, double snr,
                            int vsat, int slipc, int rejc) {
  return "$SAT," + std::to_string(2380) + "," + std::to_string(tow) + "," + sat + "," +
         std::to_string(frq) + ",123.4," + std::to_string(el) + "," + std::to_string(resp) +
         ",0.001," + std::to_string(vsat) + "," + std::to_string(snr) + ",1,0,100,0," +
         std::to_string(slipc) + "," + std::to_string(rejc);
}

inline void set_bits(std::vector<uint8_t>& buf, int pos, int len, int64_t value) {
  const uint64_t v = static_cast<uint64_t>(value);   // 负数取二进制补码的低 len 位
  for (int i = 0; i < len; ++i) {
    if ((v >> (len - 1 - i)) & 1u) {
      const int idx = pos + i;
      buf[static_cast<size_t>(idx / 8)] |= static_cast<uint8_t>(0x80 >> (idx % 8));
    }
  }
}

// RTCM3 1005:DF002(12) DF003(12) DF021(6) DF022-024+DF141(4) DF025 X(38) DF142+DF001(2)
//            DF026 Y(38) DF364(2) DF027 Z(38),共 152 bit = 19 字节;坐标单位 0.1 mm
inline std::vector<uint8_t> make_1005_frame(int station_id, double x, double y, double z) {
  std::vector<uint8_t> payload(19, 0);
  set_bits(payload, 0, 12, 1005);
  set_bits(payload, 12, 12, station_id);
  set_bits(payload, 34, 38, std::llround(x * 10000.0));
  set_bits(payload, 74, 38, std::llround(y * 10000.0));
  set_bits(payload, 114, 38, std::llround(z * 10000.0));
  std::vector<uint8_t> frame = {0xD3, 0x00, static_cast<uint8_t>(payload.size())};
  frame.insert(frame.end(), payload.begin(), payload.end());
  const uint32_t crc = crc24q(frame.data(), frame.size());
  frame.push_back(static_cast<uint8_t>((crc >> 16) & 0xFF));
  frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  frame.push_back(static_cast<uint8_t>(crc & 0xFF));
  return frame;
}

}  // namespace gnss_core::test_fixtures
