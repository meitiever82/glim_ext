#pragma once

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim {

// Extract yaw (Z) from a rotation, returning the roll/pitch-only rotation (yaw removed).
inline Eigen::Matrix3d roll_pitch_of(const Eigen::Matrix3d& R) {
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  const Eigen::Matrix3d Rz_inv = Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return Rz_inv * R;
}

// Build 6DoF T_map_imu from a 2D guess (x,y,yaw) + map ground z + roll/pitch taken from X(t).
inline Eigen::Isometry3d compose_initial_pose(double x, double y, double yaw, double z,
                                              const Eigen::Isometry3d& X_t) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  const Eigen::Matrix3d R_rp = roll_pitch_of(X_t.rotation());
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * R_rp;
  T.translation() = Eigen::Vector3d(x, y, z);
  return T;
}

// T_map_odom = T_map_imu * X(t)^-1  (X(t) = T_odom_imu at apply time; NOT assumed identity).
inline Eigen::Isometry3d compute_T_map_odom(const Eigen::Isometry3d& T_map_imu,
                                            const Eigen::Isometry3d& X_t) {
  return T_map_imu * X_t.inverse();
}

}  // namespace glim
