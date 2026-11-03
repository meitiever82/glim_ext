#include "gnss_core/relative_pipeline.hpp"

#include <cmath>
#include <stdexcept>

namespace gnss_core {

namespace {

void require_positive_finite(double v, const char* name) {
  if (!(std::isfinite(v) && v > 0.0)) {
    throw std::invalid_argument(std::string("RelativeConfig: ") + name + " must be finite and > 0");
  }
}

}  // namespace

RelativePipeline::RelativePipeline(const PipelineConfig& pipeline_cfg, const RelativeConfig& rel_cfg)
    : cfg_(pipeline_cfg), rel_(rel_cfg), policy_(pipeline_cfg.noise), aligner_(rel_cfg.yaw_baseline) {
  // 与 AnchorPipeline 相同的管线约束;noise policy 由 policy_ 构造校验。
  require_positive_finite(pipeline_cfg.fix_buffer_horizon, "fix_buffer_horizon");
  require_positive_finite(pipeline_cfg.fix_max_gap, "fix_max_gap");
  require_positive_finite(rel_cfg.sigma_floor, "sigma_floor");
  require_positive_finite(rel_cfg.yaw_baseline, "yaw_baseline");
  require_positive_finite(rel_cfg.max_pair_gap, "max_pair_gap");

  if (!pipeline_cfg.enu_origin.empty()) {
    if (pipeline_cfg.enu_origin.size() != 3) {
      throw std::invalid_argument("PipelineConfig: enu_origin must be empty or [lat, lon, alt]");
    }
    for (double v : pipeline_cfg.enu_origin) {
      if (!std::isfinite(v)) throw std::invalid_argument("PipelineConfig: enu_origin must be finite");
    }
    enu_origin_ = std::array<double, 3>{pipeline_cfg.enu_origin[0], pipeline_cfg.enu_origin[1], pipeline_cfg.enu_origin[2]};
    lla_to_enu_ = std::make_unique<LlaToEnu>((*enu_origin_)[0], (*enu_origin_)[1], (*enu_origin_)[2]);
  }
}

void RelativePipeline::push_fix(const RtkFixSample& s) {
  buffer_.push(s);
  buffer_.prune(cfg_.fix_buffer_horizon, buffer_.latest_stamp());
  ++stats_.fixes;
}

void RelativePipeline::push_anchor(const Anchor& a) {
  pending_.push_back(a);
  ++stats_.anchors;
}

std::vector<RelativeConstraint> RelativePipeline::process() {
  std::vector<RelativeConstraint> out;
  if (!rel_.enabled) {
    pending_.clear();
    return out;
  }
  while (!pending_.empty() && process_one(pending_.front(), out)) {
    pending_.pop_front();
  }
  return out;
}

bool RelativePipeline::process_one(const Anchor& a, std::vector<RelativeConstraint>& out) {
  if (buffer_.size() < 2) {
    return false;
  }
  const double oldest = buffer_.oldest_stamp();
  const double latest = buffer_.latest_stamp();
  if (a.stamp < oldest) {
    ++stats_.too_old;
    return true;
  }
  if (a.stamp > latest) {
    return false;   // 等更新的样本;后面的 anchor 更晚,一并等待
  }

  const auto sample = buffer_.interpolate(a.stamp, cfg_.fix_max_gap);
  if (!sample) {
    ++stats_.gap;
    return true;
  }

  const Verdict verdict = policy_.evaluate_verdict(*sample);
  if (!verdict) {
    ++stats_.gated;
    return true;
  }

  if (!lla_to_enu_) {
    enu_origin_ = std::array<double, 3>{sample->lat, sample->lon, sample->alt};
    lla_to_enu_ = std::make_unique<LlaToEnu>(sample->lat, sample->lon, sample->alt);
  }
  const Eigen::Vector3d enu = lla_to_enu_->forward(sample->lat, sample->lon, sample->alt);

  // 独立 yaw bootstrap:与绝对 gauge 相同的 2D Umeyama,但基线更短;初始化后冻结。
  if (static_cast<int>(sample->quality) >= cfg_.aligner_min_quality) {
    const bool was_initialized = aligner_.initialized();
    aligner_.add(a.t_world, enu);
    if (!was_initialized && aligner_.initialized()) {
      stats_.yaw_bootstrapped_stamp = a.stamp;
    }
  }

  if (aligner_.initialized() && prev_) {
    const double dt = a.stamp - prev_->stamp;
    if (dt <= 0.0 || dt > rel_.max_pair_gap) {
      ++stats_.pair_gap;
    } else {
      const Eigen::Vector2d dp_enu_xy = (enu - prev_->enu).head<2>();
      const Eigen::Matrix2d R = aligner_.T_world_enu().linear().block<2, 2>(0, 0);
      const Eigen::Vector2d measured = R * dp_enu_xy;

      // 差分放大 sqrt(2):两端 σ 平方和,再套相对约束下界。
      Eigen::Vector2d sigma = (prev_->sigma.head<2>().array().square() + verdict.sigma.head<2>().array().square()).sqrt();
      sigma = sigma.cwiseMax(rel_.sigma_floor);

      RelativeConstraint c;
      c.id1 = prev_->id;
      c.id2 = a.id;
      c.stamp1 = prev_->stamp;
      c.stamp2 = a.stamp;
      c.dp_world_xy = measured;
      c.model = make_diagonal_robust_model(sigma, cfg_.noise);
      out.push_back(std::move(c));
      ++stats_.pairs;
    }
  }

  prev_ = Prev{a.id, a.stamp, a.t_world, enu, verdict.sigma};
  return true;
}

}  // namespace gnss_core
