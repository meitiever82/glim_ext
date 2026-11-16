/**
 * @file judge_replay.cpp
 * @brief 定位门控离线回放 CLI（期 1 Task 7 骨架）。
 *
 * 输入为品牌原始 ASCII 流（北云 NovAtel / 华测 GPCHC），按帧提取判决所需字段，
 * 调用 gnss_core::judge 纯函数，输出 CSV 判决序列。
 *
 * 用法:
 *   judge_replay beiyun <raw.log> --origin lat,lon,alt > verdicts.csv
 *   judge_replay huace  <raw.log> --origin lat,lon,alt > verdicts.csv
 *
 * 输出 CSV 列：gps_sec,is_valid,rtk_status,fault_code,invalid_reason
 *
 * 说明：
 * - 北云帧以 '#' 或 '$' 开头，本工具只解析 INSPVAXA / HEADINGA / GPFPD / GPGGA。
 * - 华测帧以 '$' 开头，本工具只解析 GPCHC / GPGGA。
 * - ENU 由命令行指定的原点做局部平面转换（GeographicLib LocalCartesian）。
 * - 这是“骨架”实现：完整协议解析将在期 2 由适配器节点复用 localization_parser
 *   的 parser 头文件完成；本 CLI 只用于离线快速验证 judge 行为。
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "gnss_core/geodetic.hpp"
#include "gnss_core/judge.hpp"

namespace j = gnss_core::judge;

static void PrintUsage(const char* argv0) {
  std::fprintf(stderr,
               "usage: %s <beiyun|huace> <raw.log> [--origin lat,lon,alt]\n"
               "  --origin  局部 ENU 原点（deg,deg,m）；默认 0,0,0\n"
               "  output: gps_sec,is_valid,rtk_status,fault_code\n",
               argv0);
}

static std::vector<std::string> SplitComma(std::string_view s) {
  std::vector<std::string> out;
  size_t i = 0;
  while (i < s.size()) {
    size_t j = s.find(',', i);
    if (j == std::string_view::npos) j = s.size();
    out.emplace_back(s.substr(i, j - i));
    i = j + 1;
  }
  return out;
}

static std::string_view StripFrame(std::string_view line) {
  // 去掉行首 #/$ 和行尾 *checksum 及回车换行
  if (!line.empty() && (line.back() == '\n' || line.back() == '\r')) line.remove_suffix(1);
  if (!line.empty() && (line.back() == '\r')) line.remove_suffix(1);
  if (!line.empty() && (line.front() == '#' || line.front() == '$')) line.remove_prefix(1);
  auto star = line.find('*');
  if (star != std::string_view::npos) line.remove_suffix(line.size() - star);
  return line;
}

static double ParseDouble(const std::string& s) {
  try {
    return std::stod(s);
  } catch (...) {
    return 0.0;
  }
}

static int ParseInt(const std::string& s) {
  try {
    return std::stoi(s);
  } catch (...) {
    return 0;
  }
}

static uint8_t ParseHexUint8(const std::string& s) {
  try {
    return static_cast<uint8_t>(std::stoul(s, nullptr, 16));
  } catch (...) {
    return 0;
  }
}

static uint16_t ParseHexUint16(const std::string& s) {
  try {
    return static_cast<uint16_t>(std::stoul(s, nullptr, 16));
  } catch (...) {
    return 0;
  }
}

static double GpsSec(uint32_t week, double seconds) {
  return static_cast<double>(week) * 7.0 * 86400.0 + seconds;
}

// fault_code → invalid_reason 粗分类映射（对齐 2026-10-20 plan Task 5）
static uint8_t MapInvalidReason(j::FaultCode fc) {
  switch (fc) {
    case j::FaultCode::None: return 0;
    case j::FaultCode::HardwareWarning:
    case j::FaultCode::BaselineAbnormal: return 1;  // 设备异常
    case j::FaultCode::InsNotConverged: return 2;   // 初始化
    case j::FaultCode::GraceTimeout: return 3;      // 差分龄期大
    case j::FaultCode::SatelliteInsufficient: return 4;  // 卫星信号差
    case j::FaultCode::EnuOutOfBounds: return 5;    // 解算结果异常（位置越界）
    case j::FaultCode::ParseFail:
    case j::FaultCode::DataTimeout:
    case j::FaultCode::NoData: return 99;           // 其他
  }
  return 99;
}

struct Origin {
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
};

static std::optional<Origin> ParseOrigin(const std::string& s) {
  auto p = SplitComma(s);
  if (p.size() != 3) return std::nullopt;
  Origin o;
  try {
    o.lat = std::stod(p[0]);
    o.lon = std::stod(p[1]);
    o.alt = std::stod(p[2]);
  } catch (...) {
    return std::nullopt;
  }
  return o;
}

// === 北云解析状态 ============================================================
struct BeiyunState {
  double gga_age = 0.0;
  uint16_t gpfpd_nsv1 = 0;
  bool has_heading = false;
  bool heading_narrow_int = false;
  double heading_length = 0.0;
  uint16_t heading_obs = 0;
};

static bool ProcessBeiyunLine(std::string_view line, BeiyunState& bs, j::Evidence& ev,
                              double& out_gps_sec) {
  auto frame = StripFrame(line);
  if (frame.empty()) return false;

  auto f = SplitComma(std::string(frame));
  if (f.empty()) return false;
  const std::string& head = f[0];

  if (head == "GPGGA" && f.size() > 7) {
    // $GPGGA,time,lat,ns,lon,ew,quality,sats,hdop,alt,alt_unit,undulation,unit,age,station*cs
    bs.gga_age = f.size() > 14 ? ParseDouble(f[14]) : 0.0;
    return false;
  }
  if (head == "GPFPD" && f.size() > 14) {
    // #GPFPD,GPSWeek,GPSTime,hdg,pitch,roll,lat,lon,alt,e0,e1,e2,base,nsv1,nsv2,sig*cs
    bs.gpfpd_nsv1 = static_cast<uint16_t>(ParseInt(f[13]));
    return false;
  }
  if (head == "HEADINGA" && f.size() > 12) {
    // header;data...; heading_type=df[1], length=df[2], obs=df[11]
    // 去掉可能存在的 ; 分割：先按 ; 拆
    auto semi = frame.find(';');
    std::string_view data_part = (semi == std::string_view::npos) ? frame : frame.substr(semi + 1);
    auto df = SplitComma(std::string(data_part));
    if (df.size() >= 12) {
      bs.has_heading = true;
      bs.heading_narrow_int = (df[1] == "NARROW_INT");
      bs.heading_length = ParseDouble(df[2]);
      bs.heading_obs = static_cast<uint16_t>(ParseInt(df[11]));
    }
    return false;
  }
  if (head == "INSPVAXA" && f.size() > 12) {
    auto semi = frame.find(';');
    std::string_view data_part = (semi == std::string_view::npos) ? frame : frame.substr(semi + 1);
    auto df = SplitComma(std::string(data_part));
    if (df.size() < 12) return false;

    // header: #INSPVAXA,COM1,0,49.5,FINESTEERING,2204,325284.000,00000000,0000,1272;...
    // hf[5]=GPSWeek, hf[6]=GPSSeconds
    uint32_t week = 0;
    double seconds = 0.0;
    if (f.size() > 6) {
      week = static_cast<uint32_t>(ParseInt(f[5]));
      seconds = ParseDouble(f[6]);
    }
    out_gps_sec = GpsSec(week, seconds);

    ev.pos_type = j::PosType::None;
    const std::string& pt = df[1];
    if (pt == "INS_PSRSP") ev.pos_type = j::PosType::Single;
    else if (pt == "INS_PSRDIFF") ev.pos_type = j::PosType::Psrdiff;
    else if (pt == "INS_RTKFLOAT") ev.pos_type = j::PosType::RtkFloat;
    else if (pt == "INS_RTKFIXED") ev.pos_type = j::PosType::RtkFixed;

    ev.ins_good = (df[0] == "INS_SOLUTION_GOOD");
    ev.diff_age = bs.gga_age;
    ev.sats_main = bs.gpfpd_nsv1;
    ev.sats_aux = bs.heading_obs;
    ev.heading_narrow_int = bs.heading_narrow_int;
    ev.heading_length = bs.heading_length;
    ev.heading_valid_frame = bs.has_heading;

    // lat=df[2], lon=df[3], alt=df[4]
    return true;
  }
  return false;
}

// === 华测解析状态 ============================================================
static bool ProcessHuaceLine(std::string_view line, j::Evidence& ev, double& out_gps_sec) {
  auto frame = StripFrame(line);
  if (frame.empty()) return false;

  auto f = SplitComma(std::string(frame));
  if (f.empty()) return false;
  if (f[0] != "GPCHC" && f[0] != "GPCHCX") return false;
  if (f.size() < 24) return false;

  uint32_t week = static_cast<uint32_t>(ParseInt(f[1]));
  double seconds = ParseDouble(f[2]);
  out_gps_sec = GpsSec(week, seconds);

  uint8_t sta = ParseHexUint8(f[21]);
  uint16_t warning = ParseHexUint16(f[23]);

  ev.huace_gnss_pos_type = (sta & 0xF0) >> 4;
  ev.ins_good = (sta & 0x0F) == 2;
  ev.hw_warning = (warning & 0x1FD) != 0;
  ev.huace_gnss_pos_valid = (warning & 0x01) == 0;
  ev.sats_main = static_cast<uint16_t>(ParseInt(f[19]));
  ev.sats_aux = static_cast<uint16_t>(ParseInt(f[20]));

  // lat=f[12], lon=f[13], alt=f[14]
  return true;
}

// === 主函数 ==================================================================
int main(int argc, char** argv) {
  if (argc < 3) {
    PrintUsage(argv[0]);
    return 2;
  }

  const std::string brand = argv[1];
  const std::string path = argv[2];
  Origin origin;
  for (int i = 3; i < argc; ++i) {
    if (std::strcmp(argv[i], "--origin") == 0 && i + 1 < argc) {
      auto o = ParseOrigin(argv[++i]);
      if (!o) {
        std::fprintf(stderr, "error: --origin must be lat,lon,alt\n");
        return 2;
      }
      origin = *o;
    }
  }

  const bool is_beiyun = (brand == "beiyun");
  const bool is_huace = (brand == "huace");
  if (!is_beiyun && !is_huace) {
    std::fprintf(stderr, "error: brand must be 'beiyun' or 'huace'\n");
    return 2;
  }

  std::ifstream in(path);
  if (!in) {
    std::fprintf(stderr, "error: cannot open %s\n", path.c_str());
    return 2;
  }

  gnss_core::LlaToEnu lla_to_enu(origin.lat, origin.lon, origin.alt);
  j::Config cfg;
  j::GateState state;
  BeiyunState bs;

  std::cout << "gps_sec,is_valid,rtk_status,fault_code,invalid_reason\n";

  std::string line;
  while (std::getline(in, line)) {
    j::Evidence ev;
    double gps_sec = 0.0;
    bool have_main = false;

    if (is_beiyun) {
      have_main = ProcessBeiyunLine(line, bs, ev, gps_sec);
    } else {
      have_main = ProcessHuaceLine(line, ev, gps_sec);
    }

    if (!have_main) continue;

    // 提取经纬高并转换到 ENU（简单 sscanf 骨架）
    auto f = SplitComma(StripFrame(line));
    double lat = 0.0, lon = 0.0, alt = 0.0;
    if (is_beiyun && f.size() > 4) {
      // INSPVAXA header;data...; data 中 df[2]=lat, df[3]=lon, df[4]=alt
      auto semi = std::string_view(line).find(';');
      std::string_view data_part = (semi == std::string_view::npos)
                                       ? std::string_view(line)
                                       : std::string_view(line).substr(semi + 1);
      auto df = SplitComma(std::string(data_part));
      if (df.size() > 4) {
        lat = ParseDouble(df[2]);
        lon = ParseDouble(df[3]);
        alt = ParseDouble(df[4]);
      }
    } else if (is_huace && f.size() > 14) {
      lat = ParseDouble(f[12]);
      lon = ParseDouble(f[13]);
      alt = ParseDouble(f[14]);
    }

    Eigen::Vector3d enu = lla_to_enu.forward(lat, lon, alt);
    ev.enu_x = enu.x();
    ev.enu_y = enu.y();
    ev.enu_z = enu.z();
    ev.now = gps_sec;

    j::Verdict v = is_beiyun ? j::judge_beiyun(ev, state, cfg)
                              : j::judge_huace(ev, state, cfg);

    std::printf("%.3f,%d,%d,%d,%d\n", gps_sec,
                static_cast<int>(v.is_valid),
                static_cast<int>(static_cast<uint8_t>(v.rtk_status)),
                static_cast<int>(static_cast<uint8_t>(v.fault_code)),
                static_cast<int>(MapInvalidReason(v.fault_code)));
  }

  return 0;
}
