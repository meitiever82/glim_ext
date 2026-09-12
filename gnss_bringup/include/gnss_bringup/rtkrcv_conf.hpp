#pragma once
#include <string>

namespace gnss_bringup {

// rtkrcv.conf 的可变项。键名针对 RTKLIB demo5。
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
};

std::string render_rtkrcv_conf(const RtkrcvConfParams& p);

}  // namespace gnss_bringup
