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

  // 规则 5:非有限值(NaN/inf)一律当作没配上处理——不入窗口、不影响 since、
  // 也不更新 last_paired_t_(下面统一走 !divergence_m 分支)。
  if (divergence_m && !std::isfinite(*divergence_m)) divergence_m.reset();

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

  // 规则 4:只有真正的数据缺口(两次配对样本时刻间隔 >= divergence_window_s)
  // 才重新预热;持续故障期间每秒都有配对样本,不会触发。nullopt 的 tick 不算
  // "配对上"、不更新 last_paired_t_。
  if (divergence_m && last_paired_t_ && (t - *last_paired_t_) >= window_s_) {
    warming_up_ = true;
    since_.reset();
  }

  if (!divergence_m) {
    since_.reset();
    return s;
  }
  last_paired_t_ = t;
  s.divergence_m = divergence_m;

  // 规则 2:预热期里窗口第一次攒够 min_samples,转入经验模式,并在判定这一拍
  // 之前清零 since——经验模式的第一次真正超限该有自己的起始时刻,不能借用
  // 预热期攒下的旧计时。这是唯一会清零 since 的模式切换。
  if (warming_up_ && window_.size() >= min_samples_) {
    warming_up_ = false;
    since_.reset();
  }

  if (warming_up_) {
    // 规则 1:预热期——样本无论是否超限都要入窗口,否则 610 自报 σ 偏小
    // ("自信地错")时,真实偏差会一直被判"超限"而永远进不了窗口,经验基线
    // 永远建立不起来。代价:预热期里若真的发生过一次偏差,会被计入第一份
    // 经验基线。
    if (*divergence_m > s.threshold_m) {
      if (!since_) since_ = t;
    } else {
      since_.reset();
    }
    window_.emplace_back(t, *divergence_m);
  } else {
    // 规则 3:预热期结束后——超限样本排除在窗口外、不重启计时;不超限则清零
    // since 并入窗口。不论此刻阈值是经验的还是(窗口被剪枝耗尽后回落到的)
    // 回退阈值都一样:一次持续的故障不能靠耗尽窗口把自己"学"成新基线。
    if (*divergence_m > s.threshold_m) {
      if (!since_) since_ = t;
    } else {
      since_.reset();
      window_.emplace_back(t, *divergence_m);
    }
  }
  s.since = since_;
  return s;
}

}  // namespace gnss_core
