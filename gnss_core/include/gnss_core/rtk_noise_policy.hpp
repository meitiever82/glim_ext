#pragma once
#include <array>
#include <cstddef>
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

// 拒绝原因(按 evaluate 的检查顺序;None 表示通过)。枚举值连续,可作 gated_by_reason 数组下标。
enum class RejectReason {
  None = 0,
  QualityOutOfRange,   // quality 值 ∉ [0,4]
  BelowMinQuality,     // quality < cfg.min_quality
  DiffAge,             // diff_age > cfg.max_diff_age
  Sats,                // sats_used < cfg.min_sats
  NonFiniteSigma,      // sigma_enu 含 NaN/Inf
  NonPositiveScale,    // cfg.quality_sigma_scale[quality] <= 0
};
constexpr size_t kRejectReasonCount = 7;

struct Verdict {
  gtsam::SharedNoiseModel model;   // 通过时非空
  RejectReason reason = RejectReason::None;
  explicit operator bool() const { return model != nullptr; }
};

class RtkNoisePolicy {
public:
  // cfg 非法(min_quality∉[0,4]、sigma_floor 任一 ≤0、robust_kernel 非 none|huber|cauchy)
  // 抛 std::invalid_argument
  explicit RtkNoisePolicy(const NoisePolicyConfig& cfg);
  gtsam::SharedNoiseModel evaluate(const RtkFixSample& s) const;  // 过门限→model,否则 nullptr
  Verdict evaluate_verdict(const RtkFixSample& s) const;          // 同上,附带拒绝原因
private:
  NoisePolicyConfig cfg_;
};

}  // namespace gnss_core
