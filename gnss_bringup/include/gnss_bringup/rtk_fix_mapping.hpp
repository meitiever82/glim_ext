#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <gnss_msgs/msg/rtk_fix.hpp>
#include "gnss_core/pos_io.hpp"

namespace gnss_bringup {

// PosRecord(rtkrcv llh 解)→ RtkFix。
// 注意 σ 顺序:PosRecord.sdne 是 RTKLIB 的 (sdn, sde, sdu) = N/E/U,
// 而 RtkFix.sigma_enu 是 E/N/U —— 前两项必须交换(spec §4.1 v2)。
inline gnss_msgs::msg::RtkFix to_rtk_fix(const gnss_core::PosRecord& r) {
  gnss_msgs::msg::RtkFix m;
  m.gnss_time = r.stamp;                 // parse_llh_solution 已换算为 UTC unix 秒
  m.quality = static_cast<uint8_t>(gnss_core::q_to_quality(r.q));
  m.raw_status = static_cast<uint8_t>(r.q);   // 保留 RTKLIB 原始 Q 供追溯
  m.latitude = r.lat;
  m.longitude = r.lon;
  m.altitude = r.height;
  m.sigma_enu[0] = r.sdne(1);            // sde → E
  m.sigma_enu[1] = r.sdne(0);            // sdn → N
  m.sigma_enu[2] = r.sdne(2);            // sdu → U
  m.diff_age = static_cast<float>(r.age);
  m.sats_used = static_cast<uint8_t>(r.ns);
  m.sats_main = 0;
  m.sats_aux = 0;
  m.heading = 0.0f;
  m.heading_sigma = 0.0f;
  m.heading_valid = false;               // rtkrcv 单天线解无双天线航向
  return m;
}

// 增量按行切分器:TcpStream 交付的是任意切分的字节块,不是行。
// feed() 攒内部缓冲,吐出所有已经凑齐的完整行(不含末尾 \n,also strips
// a trailing \r 以兼容 CRLF);半行留在缓冲里等下一次 feed()。
// 与 gnss_core::RtcmFramer 是同一个道理(见其头文件注释)。
class LineSplitter {
public:
  std::vector<std::string> feed(const uint8_t* data, size_t len) {
    buf_.append(reinterpret_cast<const char*>(data), len);
    std::vector<std::string> lines;
    size_t start = 0;
    for (;;) {
      const size_t nl = buf_.find('\n', start);
      if (nl == std::string::npos) break;
      size_t end = nl;
      if (end > start && buf_[end - 1] == '\r') --end;  // 兼容 CRLF
      lines.emplace_back(buf_, start, end - start);
      start = nl + 1;
    }
    buf_.erase(0, start);
    return lines;
  }

  // 尚未凑成一整行的残留字节数(供诊断/测试用)
  size_t buffered() const { return buf_.size(); }
  void reset() { buf_.clear(); }

private:
  std::string buf_;
};

}  // namespace gnss_bringup
