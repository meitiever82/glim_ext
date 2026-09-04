#include "gnss_core/rtk_noise_policy.hpp"
#include <algorithm>

namespace gnss_core {

RtkNoisePolicy::RtkNoisePolicy(const NoisePolicyConfig& cfg) : cfg_(cfg) {}

gtsam::SharedNoiseModel RtkNoisePolicy::evaluate(const RtkFixSample& s) const {
  const int q = static_cast<int>(s.quality);
  if (q < cfg_.min_quality) return nullptr;
  if (s.diff_age > cfg_.max_diff_age) return nullptr;
  if (s.sats_used < cfg_.min_sats) return nullptr;

  const double scale = cfg_.quality_sigma_scale.at(q);
  if (scale <= 0.0) return nullptr;         // 防零 σ / 无穷权重

  Eigen::Vector3d sigma = s.sigma_enu * scale;
  sigma = sigma.cwiseMax(cfg_.sigma_floor);
  sigma(2) *= cfg_.vertical_scale;

  gtsam::SharedNoiseModel base = gtsam::noiseModel::Diagonal::Sigmas(sigma);
  if (cfg_.robust_kernel == "none") return base;
  gtsam::noiseModel::mEstimator::Base::shared_ptr m;
  if (cfg_.robust_kernel == "cauchy")
    m = gtsam::noiseModel::mEstimator::Cauchy::Create(cfg_.robust_delta);
  else
    m = gtsam::noiseModel::mEstimator::Huber::Create(cfg_.robust_delta);
  return gtsam::noiseModel::Robust::Create(m, base);
}

}  // namespace gnss_core
