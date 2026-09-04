#pragma once
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gnss_core {

class FrameAligner {
public:
  explicit FrameAligner(double min_baseline);
  void add(const Eigen::Vector3d& submap_xyz, const Eigen::Vector3d& enu);
  bool initialized() const;
  Eigen::Isometry3d T_world_enu() const;   // 未初始化返回 identity
private:
  double min_baseline_;
  bool initialized_ = false;
  Eigen::Isometry3d T_world_enu_ = Eigen::Isometry3d::Identity();
  std::vector<Eigen::Vector3d> est_, enu_;
};

}  // namespace gnss_core
