#include "gnss_core/anchor_pipeline.hpp"

#include <cmath>
#include <stdexcept>

namespace gnss_core {

namespace {

void require_positive_finite(double v, const char* name) {
  if (!(std::isfinite(v) && v > 0.0)) {
    throw std::invalid_argument(std::string("PipelineConfig: ") + name + " must be finite and > 0");
  }
}

}  // namespace

AnchorPipeline::AnchorPipeline(const PipelineConfig& cfg)
    : cfg_(cfg), policy_(cfg.noise), aligner_(cfg.min_baseline) {   // policy_ 构造校验 noise
  require_positive_finite(cfg.min_baseline, "min_baseline");
  require_positive_finite(cfg.fix_buffer_horizon, "fix_buffer_horizon");
  require_positive_finite(cfg.fix_max_gap, "fix_max_gap");
  if (!cfg.enu_origin.empty()) {
    if (cfg.enu_origin.size() != 3) {
      throw std::invalid_argument("PipelineConfig: enu_origin must be empty or [lat, lon, alt]");
    }
    for (double v : cfg.enu_origin) {
      if (!std::isfinite(v)) throw std::invalid_argument("PipelineConfig: enu_origin must be finite");
    }
    enu_origin_ = std::array<double, 3>{cfg.enu_origin[0], cfg.enu_origin[1], cfg.enu_origin[2]};
    lla_to_enu_ = std::make_unique<LlaToEnu>(cfg.enu_origin[0], cfg.enu_origin[1], cfg.enu_origin[2]);
  }
}

void AnchorPipeline::push_fix(const RtkFixSample& s) {
  buffer_.push(s);
  buffer_.prune(cfg_.fix_buffer_horizon, buffer_.latest_stamp());
  ++stats_.fixes;
}

void AnchorPipeline::push_anchor(const Anchor& a) {
  pending_.push_back(a);
  ++stats_.anchors;
}

std::vector<Constraint> AnchorPipeline::process() {
  std::vector<Constraint> out;
  while (!pending_.empty() && process_one(pending_.front(), out)) {
    pending_.pop_front();
  }
  return out;
}

Constraint AnchorPipeline::make_constraint(long id, double stamp, const Eigen::Vector3d& enu,
                                           const gtsam::SharedNoiseModel& model) {
  Constraint c;
  c.id = id;
  c.stamp = stamp;
  c.p_world = aligner_.T_world_enu() * enu;
  c.model = model;
  ++stats_.constrained;
  return c;
}

bool AnchorPipeline::process_one(const Anchor& a, std::vector<Constraint>& out) {
  if (buffer_.size() < 2) {
    last_outcome_ = AnchorOutcome::WaitingForFixes;
    return false;
  }
  const double oldest = buffer_.oldest_stamp();
  const double latest = buffer_.latest_stamp();
  if (a.stamp < oldest) {
    // 周围样本已被 prune(后端落后 RTK 流超过 horizon)或从未到达
    last_outcome_ = AnchorOutcome::TooOld;
    ++stats_.too_old;
    return true;
  }
  if (a.stamp > latest) {
    last_outcome_ = AnchorOutcome::WaitingForFixes;
    return false;   // 等更新的样本;后面的 anchor 更晚,一并等待
  }

  const auto sample = buffer_.interpolate(a.stamp, cfg_.fix_max_gap);
  if (!sample) {
    last_outcome_ = AnchorOutcome::Gap;
    ++stats_.gap;
    return true;
  }

  const Verdict verdict = policy_.evaluate_verdict(*sample);
  if (!verdict) {
    last_outcome_ = AnchorOutcome::Gated;
    ++stats_.gated;
    ++stats_.gated_by_reason[static_cast<size_t>(verdict.reason)];
    return true;
  }
  const gtsam::SharedNoiseModel& model = verdict.model;

  // 只有过门限的样本才定 ENU 原点、进 aligner
  if (!lla_to_enu_) {
    enu_origin_ = std::array<double, 3>{sample->lat, sample->lon, sample->alt};
    lla_to_enu_ = std::make_unique<LlaToEnu>(sample->lat, sample->lon, sample->alt);
  }
  const Eigen::Vector3d enu = lla_to_enu_->forward(sample->lat, sample->lon, sample->alt);

  aligner_.add(a.t_world, enu);
  if (!aligner_.initialized()) {
    deferred_.push_back(Deferred{a.id, a.stamp, enu, model});
    last_outcome_ = AnchorOutcome::Deferred;
    return true;
  }

  // 首次 initialized:补发 bootstrap 前暂存的约束(之后 deferred_ 恒空)
  for (const auto& d : deferred_) {
    out.push_back(make_constraint(d.id, d.stamp, d.enu, d.model));
    ++stats_.deferred_released;
  }
  deferred_.clear();

  out.push_back(make_constraint(a.id, a.stamp, enu, model));
  last_outcome_ = AnchorOutcome::Constrained;
  return true;
}

}  // namespace gnss_core
