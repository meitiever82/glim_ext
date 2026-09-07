#pragma once
// 杆臂 + T_world_enu + 时间偏移联合估计(spec §9.3,Task 14)。
//
// 模型:对每个 RTK 样本 i(时间 stamp_i + Δt 处插值轨迹得 R_i, t_i):
//     R_i · lever + t_i = R_yaw · enu_i + p          (T_world_enu = (R_yaw, p),仅 yaw + 3D 平移)
// 外层对 Δt 网格搜索(dt_min..dt_max,步长 dt_step,末了用抛物线细化),内层给定 Δt 交替求解:
//     2D Umeyama 取 (yaw, p) 初值 → 给定 yaw 对 (lever, p) 线性 LS → 给定 lever 重解 yaw …(数轮)
// 取残差最小的 Δt。
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "gnss_core/geodetic.hpp"
#include "gnss_core/synth.hpp"
#include "gnss_core/types.hpp"

namespace gnss_core {

struct LeverArmEstimate {
  Eigen::Vector3d lever_imu = Eigen::Vector3d::Zero();              // 天线在 IMU 系
  Eigen::Isometry3d T_world_enu = Eigen::Isometry3d::Identity();    // yaw + 平移(z 平移也解)
  double time_offset = 0.0;      // s;加到 RTK 样本时间上使其与轨迹对齐
  double rms_residual = 0.0;     // m
  int n_pairs = 0;
  bool lever_observable = false;     // 轨迹 yaw 极差 < min_yaw_range_deg 时 false(水平杆臂不可观)
  bool lever_z_observable = false;   // 轨迹俯仰/横滚极差 < min_tilt_range_deg 时 false:lever.z 钉 0,由 p.z 吸收
  bool time_offset_observable = false;   // 残差随 Δt 无明显变化(匀速直线:时移等价于平移)时 false;此时 Δt 取最靠近 0 的极小
  double yaw_range_deg = 0.0;        // 诊断:轨迹 yaw 极差
  double tilt_range_deg = 0.0;       // 诊断:轨迹倾斜(重力方向与 z 轴夹角)极差
};

struct LeverArmOptions {
  double dt_min = -0.5, dt_max = 0.5, dt_step = 0.01;   // Δt 网格(s)
  double pair_tol = 0.05;               // 样本时刻到最近轨迹点的最大间隔(s),超过则不配对
  Quality min_quality = Quality::FIXED;
  double min_yaw_range_deg = 30.0;
  double min_tilt_range_deg = 5.0;
  int inner_iters = 5;
  int min_pairs = 10;
};

// traj 需按 stamp 升序(read_glim_traj 已排序);fixes 的时间与 traj 同一时间轴。
// 配对数 < min_pairs 时返回默认值(n_pairs 为实际配对数,lever_observable=false)。
LeverArmEstimate estimate_lever_arm(const std::vector<TrajPose>& traj, const std::vector<RtkFixSample>& fixes,
                                    const LlaToEnu& conv, const LeverArmOptions& opt = {});

}  // namespace gnss_core
