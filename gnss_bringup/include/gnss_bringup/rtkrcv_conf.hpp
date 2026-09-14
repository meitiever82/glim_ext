#pragma once
#include <string>

namespace gnss_bringup {

// rtkrcv.conf 的可变项。键名与取值针对 RTKLIB-EX 2.5.1(rtklibexplorer,原 demo5)。
// obs_format / corr_format 是 spec §13 的 P0 未定项:板卡原始观测格式与平台差分格式
// 现场确认前沿用 rtk-monitor 的假定值 "rtcm3"。二者是参数,改默认值不需要改代码。
struct RtkrcvConfParams {
  int obs_port = 15032;        // 本机喂板卡原始观测的端口(rtkrcv 作为 tcpcli 连入)
  int corr_port = 15031;       // 本机喂差分改正的端口
  int sol_port = 15020;        // rtkrcv 吐解的端口(rtkrcv 作为 tcpsvr)
  std::string obs_format = "rtcm3";
  std::string corr_format = "rtcm3";
  std::string pos_mode = "kinematic";
  int navsys = 63;
  double elmask = 10.0;
  std::string ar_mode = "continuous";
  // 基准站坐标来源(ant2-postype)。"rtcm":取差分流里的 RTCM 1005/1006;"single":
  // 基准站观测的单点解。不写这个键时 rtkrcv 默认 llh 0,0,0,RTK 一条解都不输出。
  std::string base_pos_type = "rtcm";
  // 北斗 / GLONASS 模糊度固定。默认值与 RTKLIB-EX 2.5.1 自身默认一致,显式写进 conf,
  // 避免换版本后默认值悄悄变化。
  std::string bds_ar_mode = "off";
  std::string glo_ar_mode = "fix-and-hold";
};

// 任何字段取值不在 RTKLIB-EX 2.5.1 允许范围内时抛 std::invalid_argument。
std::string render_rtkrcv_conf(const RtkrcvConfParams& p);

}  // namespace gnss_bringup
