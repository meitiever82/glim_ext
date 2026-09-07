#pragma once
// 测试共用的合成轨迹(Task 13 / Task 14)。
#include <cmath>
#include <vector>
#include <Eigen/Geometry>
#include "gnss_core/synth.hpp"

namespace gnss_core {
namespace test_fixtures {

// 0–20 s 沿 +x 直行 100 m,20–30 s 原地转 90°,30–50 s 沿 +y 直行 100 m(10 Hz)
inline std::vector<TrajPose> straight_then_turn() {
  std::vector<TrajPose> tr;
  for (int k = 0; k <= 500; ++k) {
    const double t = k * 0.1;
    TrajPose p;
    p.stamp = t;
    p.T_world_imu = Eigen::Isometry3d::Identity();
    const double yaw = (t < 20) ? 0 : (t < 30 ? (t - 20) / 10 * M_PI_2 : M_PI_2);
    p.T_world_imu.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    p.T_world_imu.translation() =
        (t < 20) ? Eigen::Vector3d(5 * t, 0, 0)
                 : (t < 30 ? Eigen::Vector3d(100, 0, 0) : Eigen::Vector3d(100, 5 * (t - 30), 0));
    tr.push_back(p);
  }
  return tr;
}

// 同 straight_then_turn,但两段直线各带坡道:0–20 s 以 pitch=+8° 爬升,30–50 s 以 −8° 下降
// (R = Rz(yaw)·Ry(pitch),Ry 正角把 x 轴压向 −z,故爬升取负号)。用于让 z 杆臂可观。
inline std::vector<TrajPose> straight_then_turn_with_ramp() {
  std::vector<TrajPose> tr = straight_then_turn();
  const double slope = 8.0 * M_PI / 180.0;
  const double z_top = 100.0 * std::tan(slope);
  for (auto& p : tr) {
    const double t = p.stamp;
    double pitch = 0.0, z = z_top;
    if (t < 20) { pitch = -slope; z = 5 * t * std::tan(slope); }
    else if (t >= 30) { pitch = +slope; z = z_top - 5 * (t - 30) * std::tan(slope); }
    p.T_world_imu.linear() = p.T_world_imu.linear() * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
    p.T_world_imu.translation().z() = z;
  }
  return tr;
}

}  // namespace test_fixtures
}  // namespace gnss_core
