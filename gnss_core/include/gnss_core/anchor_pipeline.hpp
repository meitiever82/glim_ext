#pragma once
// GLIM 壳(rtk_global 挂 submap、rtk_odometry 挂 odometry 帧)共用的 RTK 约束后台逻辑:
//   RTK 样本缓冲 → 按 anchor 时间插值 → 噪声门限 → ENU 原点 / T_world_enu bootstrap → 输出约束。
// 纯数据、无线程、无 ROS、无日志:壳负责取样(effective_stamp、finite/skew 过滤)、线程与队列、
// 把 Constraint 变成因子(key 与 body_point 由壳决定,调 AntennaPriorFactor::create)。
#include <array>
#include <deque>
#include <memory>
#include <optional>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtsam/linear/NoiseModel.h>

#include "gnss_core/frame_aligner.hpp"
#include "gnss_core/geodetic.hpp"
#include "gnss_core/rtk_fix_buffer.hpp"
#include "gnss_core/rtk_noise_policy.hpp"
#include "gnss_core/types.hpp"

namespace gnss_core {

// 被约束 node:id(壳映射到 gtsam key)、时间、当前估计位置(world 系,喂 FrameAligner)
struct Anchor {
  long id = 0;
  double stamp = 0.0;
  Eigen::Vector3d t_world = Eigen::Vector3d::Zero();
};

// 一条待加入图的位置约束:天线在 world 系的位置 + 噪声模型。不含因子对象。
struct Constraint {
  long id = 0;
  double stamp = 0.0;
  Eigen::Vector3d p_world = Eigen::Vector3d::Zero();
  gtsam::SharedNoiseModel model;
};

enum class AnchorOutcome {
  Constrained,      // 已输出约束
  Deferred,         // 通过门限但 T_world_enu 未 bootstrap,暂存,bootstrap 时补发
  WaitingForFixes,  // 缓冲不足 2 个样本或 anchor 晚于最新样本:等待,后续 anchor 不处理
  TooOld,           // anchor 早于缓冲最老样本(已 prune 或从未到达):丢弃
  Gap,              // anchor 两侧样本间隔 > fix_max_gap:丢弃
  Gated,            // 插值样本未过 RtkNoisePolicy:丢弃
  TimeSkew,         // 预留:壳侧 skew 过滤;本类不产生
};

struct PipelineConfig {
  NoisePolicyConfig noise;
  double min_baseline = 10.0;         // FrameAligner bootstrap 基线(m)
  double fix_buffer_horizon = 600.0;  // 缓冲保留时长(数据时间,s)
  double fix_max_gap = 2.5;           // 插值允许的最大左右样本间隔(s)
  std::vector<double> enu_origin;     // 空 = 以首个过门限样本为原点;或 [lat, lon, alt]
};

struct PipelineStats {
  size_t fixes = 0;              // push_fix 次数
  size_t anchors = 0;            // push_anchor 次数
  size_t constrained = 0;        // 输出的 Constraint 数(含补发)
  size_t deferred_released = 0;  // bootstrap 时补发的 deferred 数
  size_t too_old = 0;
  size_t gap = 0;
  size_t gated = 0;
  // 按 RejectReason 下标累计的 Gated 计数(下标 0 = None 恒为 0);sum == gated
  std::array<size_t, kRejectReasonCount> gated_by_reason{};
};

class AnchorPipeline {
public:
  // 非法配置抛 std::invalid_argument:NoisePolicyConfig 校验同 RtkNoisePolicy;
  // enu_origin 非空且 size != 3;min_baseline/fix_buffer_horizon/fix_max_gap 非正或非有限。
  explicit AnchorPipeline(const PipelineConfig& cfg);

  // 假定调用方已做 finite / stamp skew 过滤;push 后按 (fix_buffer_horizon, latest_stamp) prune。
  void push_fix(const RtkFixSample& s);
  void push_anchor(const Anchor& a);   // FIFO

  // 按 FIFO 处理所有能处理的 anchor,遇到需等待(WaitingForFixes)的停下。
  // 返回本次新产生的约束(首次 bootstrap 时先补发全部 deferred,再输出当前;p_world = T_world_enu · enu)。
  std::vector<Constraint> process();

  bool initialized() const { return aligner_.initialized(); }
  Eigen::Isometry3d T_world_enu() const { return aligner_.T_world_enu(); }
  std::optional<std::array<double, 3>> enu_origin() const { return enu_origin_; }

  const PipelineStats& stats() const { return stats_; }
  size_t pending_anchors() const { return pending_.size(); }
  size_t deferred() const { return deferred_.size(); }
  size_t buffered_fixes() const { return buffer_.size(); }
  AnchorOutcome last_outcome() const { return last_outcome_; }   // 最近一个 anchor 的结局

private:
  struct Deferred {
    long id;
    double stamp;
    Eigen::Vector3d enu;
    gtsam::SharedNoiseModel model;
  };

  // 处理一个 anchor;返回 false 表示需等待(anchor 留在队列)。产生的约束追加到 out。
  bool process_one(const Anchor& a, std::vector<Constraint>& out);
  Constraint make_constraint(long id, double stamp, const Eigen::Vector3d& enu,
                             const gtsam::SharedNoiseModel& model);

  PipelineConfig cfg_;
  RtkNoisePolicy policy_;
  FrameAligner aligner_;
  RtkFixBuffer buffer_;
  std::unique_ptr<LlaToEnu> lla_to_enu_;
  std::optional<std::array<double, 3>> enu_origin_;

  std::deque<Anchor> pending_;
  std::vector<Deferred> deferred_;
  PipelineStats stats_;
  AnchorOutcome last_outcome_ = AnchorOutcome::WaitingForFixes;
};

}  // namespace gnss_core
