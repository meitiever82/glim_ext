#pragma once
#include <array>
#include <string>
#include <Eigen/Core>
#include <gtsam/linear/NoiseModel.h>
#include "gnss_core/types.hpp"

namespace gnss_core {

struct NoisePolicyConfig {
  int min_quality = 3;                    // FLOAT
  double max_diff_age = 15.0;
  int min_sats = 6;
  std::array<double, 5> quality_sigma_scale = {0.0, 50.0, 20.0, 5.0, 1.0};
  // σ 下限(m,E/N/U)。杆臂未标定期间默认 1.0 m(spec §7.5 v2,防止未建模的杆臂误差被当高精度约束);
  // 杆臂标定后改为 {0.02, 0.02, 0.05}。
  Eigen::Vector3d sigma_floor = {1.0, 1.0, 1.0};
  double vertical_scale = 3.0;
  std::string robust_kernel = "huber";    // none|huber|cauchy
  double robust_delta = 1.345;
};

class RtkNoisePolicy {
public:
  // cfg 非法(min_quality∉[0,4]、sigma_floor 任一 ≤0、robust_kernel 非 none|huber|cauchy)
  // 抛 std::invalid_argument
  explicit RtkNoisePolicy(const NoisePolicyConfig& cfg);
  gtsam::SharedNoiseModel evaluate(const RtkFixSample& s) const;  // 过门限→model,否则 nullptr
private:
  NoisePolicyConfig cfg_;
};

}  // namespace gnss_core
