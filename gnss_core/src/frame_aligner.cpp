#include "gnss_core/frame_aligner.hpp"

namespace gnss_core {

FrameAligner::FrameAligner(double min_baseline) : min_baseline_(min_baseline) {}

void FrameAligner::add(const Eigen::Vector3d& submap_xyz, const Eigen::Vector3d& enu) {
  est_.push_back(submap_xyz);
  enu_.push_back(enu);
  if (initialized_ || est_.size() < 2) return;
  if ((est_.back() - est_.front()).norm() < min_baseline_) return;

  // 2D Umeyama(仅 yaw + 平移):在 XY 平面上对齐 enu → est
  Eigen::Vector3d mean_est = Eigen::Vector3d::Zero(), mean_enu = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < est_.size(); ++i) { mean_est += est_[i]; mean_enu += enu_[i]; }
  mean_est /= static_cast<double>(est_.size());
  mean_enu /= static_cast<double>(enu_.size());

  // cov = sum( (enu_centered) * (est_centered)^T ),使得 R = U V^T 得到 enu->est 的旋转
  Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
  for (size_t i = 0; i < est_.size(); ++i)
    cov += (enu_[i].head<2>() - mean_enu.head<2>()) * (est_[i].head<2>() - mean_est.head<2>()).transpose();

  Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d R2 = svd.matrixV() * svd.matrixU().transpose();
  if (R2.determinant() < 0) { Eigen::Matrix2d V = svd.matrixV(); V.col(1) *= -1; R2 = V * svd.matrixU().transpose(); }

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  R.block<2,2>(0,0) = R2;
  T_world_enu_ = Eigen::Isometry3d::Identity();
  T_world_enu_.linear() = R;
  T_world_enu_.translation() = mean_est - R * mean_enu;
  initialized_ = true;
}

bool FrameAligner::initialized() const { return initialized_; }
Eigen::Isometry3d FrameAligner::T_world_enu() const { return T_world_enu_; }

}  // namespace gnss_core
