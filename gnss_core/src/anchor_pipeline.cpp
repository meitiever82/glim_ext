#include "gnss_core/anchor_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "gnss_core/gauge_registry.hpp"

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
  if (!(std::isfinite(cfg.heading_sigma_floor_deg) && cfg.heading_sigma_floor_deg > 0.0)) {
    throw std::invalid_argument("PipelineConfig: heading_sigma_floor_deg must be finite and > 0");
  }
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

Constraint AnchorPipeline::make_constraint(const Deferred& d) {
  Constraint c;
  c.id = d.id;
  c.stamp = d.stamp;
  c.p_world = aligner_.T_world_enu() * d.enu;
  c.model = d.model;

  if (cfg_.heading_enabled && d.heading_valid) {
    // 板卡航向:北零顺时针(deg)。ENU 中车辆朝向的 yaw = pi/2 - heading;
    // world yaw = yaw(T_world_enu) + ENU yaw。
    const Eigen::Isometry3d T = aligner_.T_world_enu();
    const double yaw_offset = std::atan2(T.linear()(1, 0), T.linear()(0, 0));
    const double heading_rad = d.heading_deg * M_PI / 180.0;
    const double sigma_deg = std::max(cfg_.heading_sigma_floor_deg, d.heading_sigma_deg);
    c.has_heading = true;
    c.heading_world = yaw_offset + 0.5 * M_PI - heading_rad;
    c.heading_model = make_diagonal_robust_model(Eigen::VectorXd::Constant(1, sigma_deg * M_PI / 180.0), cfg_.noise);
  }
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
  last_quality_ = sample->quality;
  last_sigma_ = verdict.sigma;

  // 只有过门限的样本才定 ENU 原点、进 aligner
  if (!lla_to_enu_) {
    enu_origin_ = std::array<double, 3>{sample->lat, sample->lon, sample->alt};
    lla_to_enu_ = std::make_unique<LlaToEnu>(sample->lat, sample->lon, sample->alt);
  }
  const Eigen::Vector3d enu = lla_to_enu_->forward(sample->lat, sample->lon, sample->alt);

  // F2(FIXED-only bootstrap):仅高质量样本参与 gauge 求解;
  // 低质量样本的因子照常走 deferred/输出(降权),只是不参与定这个全程冻结的基准。
  if (static_cast<int>(sample->quality) >= cfg_.aligner_min_quality) {
    aligner_.add(a.t_world, enu);
  }

  if (!aligner_.initialized()) {
    // F1(shared gauge):本地未 bootstrap,尝试采用进程内共享 gauge
    if (cfg_.shared_gauge && GaugeRegistry::instance().initialized()) {
      aligner_.adopt(GaugeRegistry::instance().T_world_enu());
      gauge_source_ = "shared";
    }
  } else if (gauge_source_.empty()) {
    // 本地刚 bootstrap:发布为共享 gauge(先发布者胜;被抢先则采用胜者值)
    gauge_source_ = "local";
    if (cfg_.shared_gauge && !GaugeRegistry::instance().try_publish(aligner_.T_world_enu(), cfg_.name)) {
      aligner_.adopt(GaugeRegistry::instance().T_world_enu());
      gauge_source_ = "shared";
    }
  }

  if (!aligner_.initialized()) {
    Deferred d;
    d.id = a.id;
    d.stamp = a.stamp;
    d.enu = enu;
    d.model = model;
    d.heading_valid = sample->heading_valid;
    d.heading_deg = sample->heading;
    d.heading_sigma_deg = sample->heading_sigma;
    deferred_.push_back(d);
    last_outcome_ = AnchorOutcome::Deferred;
    return true;
  }

  // 首次 initialized:补发 bootstrap 前暂存的约束(之后 deferred_ 恒空)
  for (const auto& d : deferred_) {
    out.push_back(make_constraint(d));
    ++stats_.deferred_released;
  }
  deferred_.clear();

  Deferred current;
  current.id = a.id;
  current.stamp = a.stamp;
  current.enu = enu;
  current.model = model;
  current.heading_valid = sample->heading_valid;
  current.heading_deg = sample->heading;
  current.heading_sigma_deg = sample->heading_sigma;
  out.push_back(make_constraint(current));
  last_outcome_ = AnchorOutcome::Constrained;
  return true;
}

}  // namespace gnss_core
