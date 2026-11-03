#pragma once
// 进程级共享 gauge（2026-10-19 shared-gauge 设计 F1）:
//   一次会话全局只有一个 T_world_enu。rtk_odometry / rtk_global 各自持有 AnchorPipeline
//   （各自的 FrameAligner），历史上各自 bootstrap、互不协调，实车 seg B 上两 gauge 矛盾 9.5m。
//   本单例提供"先 bootstrap 者发布、后来者采用"的协调点；冻结语义不变（FrameAligner 注释）。
//   作用域：单进程（glim_rosbag / glim_rosnode 内以 .so 共存的扩展模块）。跨进程不支持。
#include <mutex>
#include <string>
#include <Eigen/Geometry>

namespace gnss_core {

class GaugeRegistry {
public:
  static GaugeRegistry& instance();   // 定义在 .cpp，保证跨 .so 唯一

  bool initialized() const;
  Eigen::Isometry3d T_world_enu() const;   // 未初始化返回 identity
  std::string owner() const;               // 发布者模块名（日志追溯），未初始化返回 ""

  // 候选者提交本地解出的 gauge。首个提交被接受（返回 true），之后一律拒绝（返回 false，
  // 调用方应 adopt T_world_enu()）。T 非有限直接拒绝。
  bool try_publish(const Eigen::Isometry3d& T, const std::string& owner);

  void reset();   // 单元测试专用：清空发布状态

private:
  GaugeRegistry() = default;
  mutable std::mutex mutex_;
  bool initialized_ = false;
  Eigen::Isometry3d T_ = Eigen::Isometry3d::Identity();
  std::string owner_;
};

}  // namespace gnss_core
