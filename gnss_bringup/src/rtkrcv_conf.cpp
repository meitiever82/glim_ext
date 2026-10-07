#include "gnss_bringup/rtkrcv_conf.hpp"

#include <cstdio>
#include <cmath>
#include <initializer_list>
#include <sstream>
#include <stdexcept>

namespace gnss_bringup {

namespace {

// Validate that a string field doesn't contain problematic characters
void validate_string_field(const std::string& value, const char* field_name) {
  if (value.find('\n') != std::string::npos) {
    throw std::invalid_argument(
        std::string(field_name) + " contains newline: " + value);
  }
  if (value.find('\r') != std::string::npos) {
    throw std::invalid_argument(
        std::string(field_name) + " contains carriage return: " + value);
  }
  if (value.find('=') != std::string::npos) {
    throw std::invalid_argument(
        std::string(field_name) + " contains equals sign: " + value);
  }
}

// Format a double without trailing zeros, avoiding scientific notation
// Uses locale-independent formatting (C locale decimal point)
std::string format_double_without_trailing_zeros(double value) {
  // Use %f with enough precision, then strip trailing zeros
  char buf[32];
  snprintf(buf, sizeof(buf), "%.6f", value);
  std::string str(buf);

  // Remove trailing zeros after decimal point
  size_t decimal_pos = str.find('.');
  if (decimal_pos != std::string::npos) {
    // Strip trailing zeros after decimal point
    size_t last = str.length() - 1;
    while (last > decimal_pos && str[last] == '0') {
      last--;
    }
    // Remove trailing decimal point if all fractional digits were zero
    if (last == decimal_pos) {
      str = str.substr(0, decimal_pos);
    } else {
      str = str.substr(0, last + 1);
    }
  }

  return str;
}

// rtkrcv 对非法取值只打一行 "invalid option value" 就回落到默认值继续运行,
// 进程照样活着——所以取值必须在生成 conf 时校验,不能指望 rtkrcv 报错。
void validate_one_of(const std::string& value, const char* field,
                     std::initializer_list<const char*> allowed) {
  for (const char* a : allowed) {
    if (value == a) return;
  }
  std::string list;
  for (const char* a : allowed) {
    if (!list.empty()) list += ", ";
    list += a;
  }
  throw std::invalid_argument(std::string(field) + "=\"" + value +
                              "\" 不是 RTKLIB-EX 2.5.1 认识的取值(可选: " + list + ")");
}

}  // namespace

std::string render_rtkrcv_conf(const RtkrcvConfParams& p) {
  // Validate all string fields
  validate_string_field(p.obs_format, "obs_format");
  validate_string_field(p.corr_format, "corr_format");
  validate_string_field(p.pos_mode, "pos_mode");
  validate_string_field(p.ar_mode, "ar_mode");

  const std::initializer_list<const char*> kStreamFormats = {
      "rtcm2", "rtcm3", "oem4", "ubx", "swift", "hemis", "skytraq",
      "javad", "nvs", "binex", "rt17", "sbf", "unicore"};
  validate_one_of(p.obs_format, "obs_format", kStreamFormats);
  validate_one_of(p.corr_format, "corr_format", kStreamFormats);
  validate_one_of(p.pos_mode, "pos_mode",
                  {"single", "dgps", "kinematic", "static", "static-start", "movingbase",
                   "fixed", "ppp-kine", "ppp-static", "ppp-fixed"});
  validate_one_of(p.ar_mode, "ar_mode", {"off", "continuous", "instantaneous", "fix-and-hold"});
  validate_one_of(p.glo_ar_mode, "glo_ar_mode", {"off", "on", "autocal", "fix-and-hold"});
  validate_one_of(p.bds_ar_mode, "bds_ar_mode", {"off", "on"});
  // llh / xyz 需要另给基准站坐标参数,本轮不支持
  validate_one_of(p.base_pos_type, "base_pos_type", {"rtcm", "single"});
  if (p.navsys < 1 || p.navsys > 127) {
    throw std::invalid_argument("navsys=" + std::to_string(p.navsys) +
                                " 超出系统位掩码范围 [1,127](1:GPS 2:SBAS 4:GLO 8:GAL 16:QZS 32:BDS 64:NavIC)");
  }

  // Validate elmask: must be in range [0, 90] and finite
  if (!std::isfinite(p.elmask)) {
    throw std::invalid_argument("elmask is not finite: " +
                                std::to_string(p.elmask));
  }
  if (p.elmask < 0.0 || p.elmask > 90.0) {
    throw std::invalid_argument("elmask out of range [0, 90]: " +
                                std::to_string(p.elmask));
  }

  // 模糊度固定高度角门限:同 elmask 的范围;允许小于 elmask(不起作用,但不是错误——
  // RTKLIB-EX 2.5.1 rtkpos.c 只在挑选参与固定的卫星时拿它比较)
  if (!std::isfinite(p.ar_elmask) || p.ar_elmask < 0.0 || p.ar_elmask > 90.0) {
    throw std::invalid_argument("ar_elmask 必须是 [0, 90] 内的有限值(度),收到 " +
                                std::to_string(p.ar_elmask));
  }

  std::ostringstream oss;

  // Input stream 1 (observations)
  oss << "inpstr1-type =" << "tcpcli" << "\n";
  oss << "inpstr1-path =127.0.0.1:" << p.obs_port << "\n";
  oss << "inpstr1-format =" << p.obs_format << "\n";

  // Input stream 2 (corrections)
  oss << "inpstr2-type =" << "tcpcli" << "\n";
  oss << "inpstr2-path =127.0.0.1:" << p.corr_port << "\n";
  oss << "inpstr2-format =" << p.corr_format << "\n";

  // 输入流空闲断开与重连(键在 RTKLIB-EX 2.5.1 app/consapp/rtkrcv/rtkrcv.c rcvopts[]:
  // misc-timeout / misc-reconnect,默认各 10000 ms,经 strsetopt() 设给 stream.c 的 toinact/ticonnect)。
  // 不做成参数:两个输入都是本机回环连到 rtkrcv_node 的 LocalReserver,不存在需要靠空闲检测发现的
  // 半开连接——对端真的关闭时 recv 返回 0/出错,readtcpcli 会立刻断开并按重连间隔重连。
  // - misc-timeout =0:stream.c waittcpcli() 只在 toinact>0 时做空闲检查,strsetopt() 对 0 原样保留
  //   (0<值<1000 才抬到 1000)。默认 10 s 会让隧道里 ≥10 s 的差分中断变成"断开 10 s 再重连",
  //   中间 LocalReserver 没有客户端,链路恢复后的前 ≤10 s 数据被丢掉(Task 3 dryB trace 实测)。
  // - misc-reconnect =1000:对端关闭后 1 s 重连;strsetopt() 的下限就是 1000。
  oss << "misc-timeout =" << 0 << "\n";
  oss << "misc-reconnect =" << 1000 << "\n";

  // Output stream 1 (solution)
  oss << "outstr1-type =" << "tcpsvr" << "\n";
  oss << "outstr1-path =:" << p.sol_port << "\n";
  oss << "outstr1-format =" << "llh" << "\n";

  // Solution format options
  oss << "out-solformat =" << "llh" << "\n";
  oss << "out-outhead =" << "off" << "\n";
  oss << "out-timesys =" << "gpst" << "\n";

  // Positioning options
  oss << "pos1-posmode =" << p.pos_mode << "\n";
  oss << "pos1-elmask =" << format_double_without_trailing_zeros(p.elmask)
      << "\n";
  oss << "pos2-armode =" << p.ar_mode << "\n";
  // 键名已对照 RTKLIB-EX 2.5.1 src/options.c sysopts[](见 test_rtkrcv_conf 的键名表用例)
  oss << "pos2-arelmask =" << format_double_without_trailing_zeros(p.ar_elmask) << "\n";
  oss << "pos1-navsys =" << p.navsys << "\n";
  oss << "pos2-gloarmode =" << p.glo_ar_mode << "\n";
  oss << "pos2-bdsarmode =" << p.bds_ar_mode << "\n";

  // 基准站坐标来源——不写时 rtkrcv 默认 llh 0,0,0,RTK 一条解都不输出
  oss << "ant2-postype =" << p.base_pos_type << "\n";

  return oss.str();
}

}  // namespace gnss_bringup
