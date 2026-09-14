#include "gnss_core/divergence_monitor.hpp"

#include <algorithm>
#include <cmath>

namespace gnss_core {

DivergenceMonitor::DivergenceMonitor(const DiagnosisConfig& cfg)
    : sigma_mult_(cfg.divergence_sigma),
      window_s_(cfg.divergence_window_s),
      floor_m_(cfg.divergence_sigma_floor_m),
      min_samples_(static_cast<size_t>(std::max(cfg.divergence_min_samples, 0))) {}

DivergenceState DivergenceMonitor::update(double t, std::optional<double> divergence_m,
                                          double fallback_sigma_m) {
  // 用 <=(而非严格 <):恰好 window_s_ 秒前的样本已经不算"最近 window_s_ 秒内"。
  while (!window_.empty() && window_.front().first <= t - window_s_) window_.pop_front();

  DivergenceState s;
  double sigma = 0.0;
  if (window_.size() >= min_samples_ && !window_.empty()) {
    double sum_sq = 0.0;
    for (const auto& [ts, d] : window_) sum_sq += d * d;
    sigma = std::max(floor_m_, std::sqrt(sum_sq / static_cast<double>(window_.size())));
    s.empirical = true;
  } else {
    sigma = std::max(1e-3, fallback_sigma_m);
  }
  s.threshold_m = sigma_mult_ * sigma;

  if (!divergence_m) {
    since_.reset();
    return s;
  }
  s.divergence_m = divergence_m;
  // 排除超限样本只在已有经验基线(empirical)时才有意义:窗口样本不足、还在
  // 用回退 σ 的阶段,若也按同一(此时可能很小、不具代表性的)回退阈值排除样本,
  // 窗口会在 610 自报 σ 偏小时永远填不满,经验基线永远建立不起来。
  if (s.empirical && *divergence_m > s.threshold_m) {
    if (!since_) since_ = t;   // 超限样本不入窗口:不让偏差段拉高自己的阈值
  } else {
    since_.reset();
    window_.emplace_back(t, *divergence_m);
  }
  s.since = since_;
  return s;
}

}  // namespace gnss_core
