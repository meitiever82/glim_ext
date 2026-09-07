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
  const int q = static_cast<int>(s.quality);
  if (q < 0 || q > 4) return nullptr;       // 非法 quality 值:拒绝而非越界抛异常
  if (q < cfg_.min_quality) return nullptr;
  if (s.diff_age > cfg_.max_diff_age) return nullptr;
  if (s.sats_used < cfg_.min_sats) return nullptr;
  if (!s.sigma_enu.allFinite()) return nullptr;   // NaN/Inf σ 不得进入噪声模型

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
