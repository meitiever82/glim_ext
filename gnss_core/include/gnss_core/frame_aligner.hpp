#pragma once
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gnss_core {

// 用成对的 (submap 位置 in world, ENU 位置) 求 T_world_enu(把 ENU 坐标映射到 world)。
//
// 冻结语义(spec §6.1 [v2]):
//   累积点对,当 submap 位置首尾间距 ≥ min_baseline 时用 2D Umeyama(仅 yaw + 平移,
//   z 不参与旋转)一次性 bootstrap 出 T_world_enu,**之后冻结**——initialized() 为 true 后
//   add() 只累积、不重解,T_world_enu() 恒定。不随优化推进重估的原因:RTK 因子进图后
//   node 位置已被 T_world_enu × ENU 拉过去,用它们反推 T 是用结果验证前提;且旧因子按旧 gauge、
//   新因子按新 gauge 会让图里同时存在两套基准。联合估计走 spec §7.乙(不在本轮)。
class FrameAligner {
public:
  explicit FrameAligner(double min_baseline);
  void add(const Eigen::Vector3d& submap_xyz, const Eigen::Vector3d& enu);   // 已初始化后只累积
  bool initialized() const;
  Eigen::Isometry3d T_world_enu() const;   // 未初始化返回 identity;初始化后冻结
private:
  double min_baseline_;
  bool initialized_ = false;
  Eigen::Isometry3d T_world_enu_ = Eigen::Isometry3d::Identity();
  std::vector<Eigen::Vector3d> est_, enu_;
};

}  // namespace gnss_core
