#pragma once
// Stage-2 RTK relative-translation pipeline (spec 2.1 §5.4).
//
// Feeds the fixed-lag odometry smoother with pairwise horizontal translation constraints
// between adjacent accepted RTK epochs:
//
//   r = (X(t2).t - X(t1).t)_xy - R_ENU_world * dp_ENU_xy
//
// The yaw of R_ENU_world is bootstrapped independently from displacement pairs and then
// frozen, using a shorter baseline than the absolute gauge (yaw_baseline ~= 10 m vs
// min_baseline ~= 50 m). This gives the smoother a usable shape constraint before the
// absolute anchor is available and while LiDAR geometry is degenerate.
//
// Pure data, no ROS, no logging: the shell owns threads, queues and factor construction.

#include <array>
#include <deque>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <gtsam/linear/NoiseModel.h>

#include "gnss_core/anchor_pipeline.hpp"
#include "gnss_core/frame_aligner.hpp"
#include "gnss_core/geodetic.hpp"
#include "gnss_core/rtk_fix_buffer.hpp"
#include "gnss_core/rtk_noise_policy.hpp"
#include "gnss_core/types.hpp"

namespace gnss_core {

struct RelativeConfig {
  bool enabled = false;        // relative_enabled
  double sigma_floor = 0.05;   // m; differencing amplifies noise by sqrt(2), keep a floor
  double yaw_baseline = 10.0;  // m; independent yaw bootstrap baseline
  double max_pair_gap = 2.5;   // s; do not pair anchors farther apart than this
};

struct RelativeStats {
  size_t fixes = 0;
  size_t anchors = 0;
  size_t pairs = 0;          // emitted relative constraints
  size_t too_old = 0;        // anchor older than the buffered fixes
  size_t gap = 0;            // interpolation gap
  size_t gated = 0;          // sample failed RtkNoisePolicy
  size_t pair_gap = 0;       // pair dropped because dt > max_pair_gap
  double yaw_bootstrapped_stamp = 0.0;
};

struct RelativeConstraint {
  long id1 = 0;
  long id2 = 0;
  double stamp1 = 0.0;
  double stamp2 = 0.0;
  Eigen::Vector2d dp_world_xy = Eigen::Vector2d::Zero();
  gtsam::SharedNoiseModel model;
};

class RelativePipeline {
public:
  // 非法配置(noise policy、fix_max_gap、fix_buffer_horizon、rel.sigma_floor、rel.yaw_baseline、
  // rel.max_pair_gap)抛 std::invalid_argument。
  RelativePipeline(const PipelineConfig& pipeline_cfg, const RelativeConfig& rel_cfg);

  void push_fix(const RtkFixSample& s);
  void push_anchor(const Anchor& a);

  // FIFO 处理能处理的 anchor,遇到需要等待的停止。返回本次新产生的相对约束。
  std::vector<RelativeConstraint> process();

  bool yaw_initialized() const { return aligner_.initialized(); }
  Eigen::Isometry3d T_world_enu() const { return aligner_.T_world_enu(); }
  const RelativeStats& stats() const { return stats_; }
  size_t pending_anchors() const { return pending_.size(); }
  size_t buffered_fixes() const { return buffer_.size(); }

private:
  struct Prev {
    long id;
    double stamp;
    Eigen::Vector3d t_world;
    Eigen::Vector3d enu;
    Eigen::Vector3d sigma;
  };

  bool process_one(const Anchor& a, std::vector<RelativeConstraint>& out);

  PipelineConfig cfg_;
  RelativeConfig rel_;
  RtkNoisePolicy policy_;
  FrameAligner aligner_;
  RtkFixBuffer buffer_;
  std::unique_ptr<LlaToEnu> lla_to_enu_;
  std::optional<std::array<double, 3>> enu_origin_;

  std::deque<Anchor> pending_;
  std::optional<Prev> prev_;
  RelativeStats stats_;
};

}  // namespace gnss_core
