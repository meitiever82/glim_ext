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

  // 回退/经验两种模式的阈值含义不同(前者是 rtkrcv 自报 σ,后者是窗口 RMS),
  // 模式切换的瞬间不能借用上一种模式攒下的"持续超限"计时——否则经验模式下的
  // 一次真正超限会被误判成"早就超限了"(计时起点仍是旧模式下攒的时刻),
  // 而不是拥有自己的起始时刻。
  if (s.empirical != prev_empirical_) since_.reset();
  prev_empirical_ = s.empirical;

  if (!divergence_m) {
    since_.reset();
    return s;
  }
  s.divergence_m = divergence_m;

  if (*divergence_m > s.threshold_m) {
    if (!since_) since_ = t;   // 超限:起算或保持
    // design decision 2:样本不足、用回退 σ 判定时仍然判定,不是不判定——但
    // 为了能攒起经验基线,回退阶段的样本无论是否超限都要入窗口,否则 610
    // 自报 σ 偏小("自信地错")时,真实偏差会一直被判"超限"而永远进不了
    // 窗口,经验基线永远建立不起来。代价:预热期里若真的发生过一次偏差,
    // 会被计入第一份经验基线。经验阶段沿用原规则:超限样本排除在窗口外,
    // 不让偏差段拉高自己的阈值。
    if (!s.empirical) window_.emplace_back(t, *divergence_m);
  } else {
    since_.reset();
    window_.emplace_back(t, *divergence_m);
  }
  s.since = since_;
  return s;
}

}  // namespace gnss_core
