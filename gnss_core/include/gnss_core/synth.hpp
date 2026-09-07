#pragma once
// 轨迹 → 合成 RTK 观测(spec §12.3,Task 13)。纯函数,无 ROS 依赖。
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "gnss_core/pos_io.hpp"
#include "gnss_core/types.hpp"

namespace gnss_core {

// glim_rosbag dump 的 IMU 轨迹一帧(TUM 格式:t x y z qx qy qz qw)
struct TrajPose {
  double stamp = 0.0;
  Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();
};

// 读 TUM 格式轨迹;'#' 开头与空行跳过;列数不足的行跳过;打不开抛 std::runtime_error。
// 输出按 stamp 升序(文件本身乱序时排序)。
std::vector<TrajPose> read_glim_traj(const std::string& path);

// 在 traj 上插值 t 时刻位姿:位置线性、姿态 slerp。t 超出 [front, back] 返回 false。
bool interpolate_pose(const std::vector<TrajPose>& traj, double t, TrajPose& out);

struct SynthConfig {
  Eigen::Vector3d lever_imu = Eigen::Vector3d::Zero();               // 天线在 IMU 系
  Eigen::Isometry3d T_enu_world = Eigen::Isometry3d::Identity();     // 把 world 放到 ENU 里的任意位姿
  double lat0 = 44.5, lon0 = 90.28, alt0 = 617.0;                    // ENU 原点
  double rate_hz = 10.0;
  Eigen::Vector3d sigma_fixed = {0.01, 0.01, 0.03};                  // 真噪声(FIXED)
  Eigen::Vector3d sigma_float = {0.3, 0.3, 0.6};
  unsigned seed = 42;
};

// 注入脚本:按时间段/比例篡改
struct Injection {
  double wrong_fix_ratio = 0.0;            // 该比例历元:位置偏 1 m(随机水平方向)、σ 仍报 FIXED 级、quality=FIXED
  double stale_from = -1, stale_to = -1;   // 时间段内 diff_age 从 0 线性增长到 60 s
  double float_from = -1, float_to = -1;   // 时间段内 quality=FLOAT、σ 放大到 sigma_float
};

struct SynthResult {
  std::vector<RtkFixSample> samples;
  std::vector<Eigen::Vector3d> truth_enu;   // 与 samples 一一对应的无噪声天线 ENU 真值
};

// 对轨迹时间范围按 rate_hz 等间隔采样,p_enu = T_enu_world · T_world_imu(t) · lever_imu,
// 加高斯噪声(σ 按当前质量档),reverse 回 lat/lon/alt。sigma_enu 填报告值(错误固定仍报 sigma_fixed)。
// sats_used=20、diff_age=1;stamp = gnss_time = t,header_stamp = t + 0.05(模拟 50 ms 接收延迟)。
// 错误固定用 seed 确定性选取 round(N·ratio) 个历元。
SynthResult synthesize(const std::vector<TrajPose>& traj, const SynthConfig& cfg, const Injection& inj);

// RtkFixSample ↔ PosRecord(sdne = [σN, σE, σU];Q 由 Quality 映射:FIXED→1 FLOAT→2 DGPS→4 SINGLE→5 NONE→0)
int quality_to_q(Quality q);
PosRecord sample_to_pos_record(const RtkFixSample& s);
// stamp/gnss_time = r.stamp,header_stamp = r.stamp(.pos 无到达时间)
RtkFixSample pos_record_to_sample(const PosRecord& r);

}  // namespace gnss_core
