#pragma once
// device_divergence 规则的阈值与计时(spec §3 C8)。
// σ 用最近 divergence_window_s 内"未超限"时的偏差 RMS——设计文档要求经验 σ 且排除当前偏差段,
// 这样 610 "自信地错"(自报 σ 很小)也能被抓到;样本不足时回退到 rtkrcv 自报 σ。
#include <cstddef>
#include <deque>
#include <optional>
#include <utility>

#include "gnss_core/diagnosis.hpp"

namespace gnss_core {

struct DivergenceState {
  std::optional<double> divergence_m;   // 本 tick 的偏差;两路没配上时为空
  std::optional<double> since;          // 持续超限的起始时刻
  double threshold_m = 0.0;
  bool empirical = false;               // true:阈值来自窗口 RMS;false:来自回退 σ
};

class DivergenceMonitor {
public:
  explicit DivergenceMonitor(const DiagnosisConfig& cfg);
  // fallback_sigma_m:窗口样本不足时使用的 σ(调用方传 hypot(sdn, sde),无独立解时传 0)
  DivergenceState update(double t, std::optional<double> divergence_m, double fallback_sigma_m);
  size_t window_size() const { return window_.size(); }

private:
  double sigma_mult_, window_s_, floor_m_;
  size_t min_samples_;
  std::deque<std::pair<double, double>> window_;   // (t, d)
  std::optional<double> since_;
};

}  // namespace gnss_core
