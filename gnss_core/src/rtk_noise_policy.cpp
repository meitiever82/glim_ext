#include "gnss_core/rtk_noise_policy.hpp"
#include <algorithm>
#include <stdexcept>

namespace gnss_core {

RtkNoisePolicy::RtkNoisePolicy(const NoisePolicyConfig& cfg) : cfg_(cfg) {
  if (cfg.min_quality < 0 || cfg.min_quality > 4)
    throw std::invalid_argument("NoisePolicyConfig: min_quality must be in [0,4]");
  if ((cfg.sigma_floor.array() <= 0.0).any())
    throw std::invalid_argument("NoisePolicyConfig: sigma_floor must be > 0");
  if (cfg.robust_kernel != "none" && cfg.robust_kernel != "huber" && cfg.robust_kernel != "cauchy")
    throw std::invalid_argument("NoisePolicyConfig: robust_kernel must be none|huber|cauchy");
}

gtsam::SharedNoiseModel RtkNoisePolicy::evaluate(const RtkFixSample& s) const {
  return evaluate_verdict(s).model;
}

Verdict RtkNoisePolicy::evaluate_verdict(const RtkFixSample& s) const {
  const auto reject = [](RejectReason r) { return Verdict{nullptr, r}; };
  const int q = static_cast<int>(s.quality);
  if (q < 0 || q > 4) return reject(RejectReason::QualityOutOfRange);  // 非法值:拒绝而非越界抛异常
  if (q < cfg_.min_quality) return reject(RejectReason::BelowMinQuality);
  if (s.diff_age > cfg_.max_diff_age) return reject(RejectReason::DiffAge);
  if (s.sats_used < cfg_.min_sats) return reject(RejectReason::Sats);
  if (!s.sigma_enu.allFinite()) return reject(RejectReason::NonFiniteSigma);   // NaN/Inf σ 不得进入噪声模型

  const double scale = cfg_.quality_sigma_scale.at(q);
  if (scale <= 0.0) return reject(RejectReason::NonPositiveScale);   // 防零 σ / 无穷权重

  Eigen::Vector3d sigma = s.sigma_enu * scale;
  sigma = sigma.cwiseMax(cfg_.sigma_floor);
  sigma(2) *= cfg_.vertical_scale;

  gtsam::SharedNoiseModel base = gtsam::noiseModel::Diagonal::Sigmas(sigma);
  if (cfg_.robust_kernel == "none") return Verdict{base, RejectReason::None};
  gtsam::noiseModel::mEstimator::Base::shared_ptr m;
  if (cfg_.robust_kernel == "cauchy")
    m = gtsam::noiseModel::mEstimator::Cauchy::Create(cfg_.robust_delta);
  else
    m = gtsam::noiseModel::mEstimator::Huber::Create(cfg_.robust_delta);
  return Verdict{gtsam::noiseModel::Robust::Create(m, base), RejectReason::None};
}

}  // namespace gnss_core
