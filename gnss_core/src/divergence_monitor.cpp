#include "gnss_core/divergence_monitor.hpp"

#include <algorithm>
#include <cmath>

namespace gnss_core {

DivergenceMonitor::DivergenceMonitor(const DiagnosisConfig& cfg)
    : sigma_mult_(cfg.divergence_sigma),
      window_s_(cfg.divergence_window_s),
      floor_m_(cfg.divergence_sigma_floor_m),
      cap_m_(cfg.divergence_sigma_max_m),
      min_samples_(static_cast<size_t>(std::max(cfg.divergence_min_samples, 0))) {}

DivergenceState DivergenceMonitor::update(double t, std::optional<double> divergence_m,
                                          double current_sigma_m, bool learnable) {
  // 用 <=(而非严格 <):恰好 window_s_ 秒前的样本已经不算"最近 window_s_ 秒内"。
  while (!window_.empty() && window_.front().first <= t - window_s_) window_.pop_front();

  // fix2 规则 C:重新预热的判定挪到每一拍最前面(配对/未配对/非有限值都要跑),
  // 只要与上一次真正配对上的时刻隔了一整个窗口,就说明中间是真正的数据缺口
  // (而不是持续故障——故障或正常数据流期间每秒都有配对样本,gap 恒为一个
  // tick,不会触发)。重新预热要把 held 的经验基线也清掉,否则马上又会被下面
  // 的规则 B 捡回来,预热等于白做。幂等:即使连续多拍都满足这个条件也没问题。
  if (last_paired_t_ && (t - *last_paired_t_) >= window_s_) {
    warming_up_ = true;
    baseline_sigma_.reset();
    since_.reset();
  }

  // 规则 5:非有限值(NaN/inf)一律当作没配上处理——不入窗口、清零 since、
  // 也不更新 last_paired_t_(下面统一走 !divergence_m 分支)。
  if (divergence_m && !std::isfinite(*divergence_m)) divergence_m.reset();

  DivergenceState s;
  // 阈值 σ = max(5 cm 下限, 经验基线, 独立解当前自报 σ):
  //   - 下限任何时候都在(启动/重新预热时也不能掉到 rtkrcv 自报的几毫米);
  //   - 经验基线只来自窗口(规则 A)或 held 的上一次窗口值(规则 B),没有就不参与;
  //   - 当前自报 σ 任何时候都能抬高阈值(rtkrcv 掉到 FLOAT/SINGLE 时自己只有分米级精度,
  //     不能继续按 FIXED 时学到的 0.15 m 判定)。它只抬高本拍阈值,不进 baseline_sigma_。
  double base = 0.0;
  if (window_.size() >= min_samples_ && !window_.empty()) {
    // 规则 A:窗口够 min_samples 时,基线是已入窗样本的 RMS(带下限),同时把它
    // 记成 baseline_sigma_——留着给窗口以后被剪枝耗尽时用(规则 B)。
    double sum_sq = 0.0;
    for (const auto& [ts, d] : window_) sum_sq += d * d;
    base = std::min(cap_m_, std::max(floor_m_, std::sqrt(sum_sq / static_cast<double>(window_.size()))));
    baseline_sigma_ = base;
    s.empirical = true;
  } else if (!warming_up_ && baseline_sigma_) {
    // 规则 B:预热已经结束、窗口却不够 min_samples(被剪枝耗尽)时,沿用上一次学到的
    // 经验基线——否则一次长故障把窗口剪空后,阈值只剩下限/自报 σ,低于正常偏差
    // 的水平时样本进不了窗口,since 清不掉(fix2 修的 N1)。
    base = *baseline_sigma_;
    s.empirical = true;
  }
  const double sigma = std::max({floor_m_, base, current_sigma_m});
  s.threshold_m = sigma_mult_ * sigma;

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
    // 规则 1:预热期——样本无论是否超限都要入窗口,否则两路的正常偏差水平
    // 高于"下限/当前自报 σ"给出的阈值时,样本会一直被判"超限"而永远进不了
    // 窗口,经验基线永远建立不起来。判定照常做(since 照常起算/清零)。
    // 代价:预热期里若真的存在持续偏移,会被计入第一份经验基线、当作正常。
    if (*divergence_m > s.threshold_m) {
      if (!since_) since_ = t;
    } else {
      since_.reset();
    }
    if (learnable) window_.emplace_back(t, *divergence_m);
  } else {
    // 规则 3:预热期结束后——超限样本排除在窗口外、不重启计时;不超限则清零
    // since 并入窗口。"超限"按本拍的最终阈值判(含当前自报 σ 的抬高):独立解
    // 自己精度差时没有超限的样本同样入窗口。一次持续的故障、或窗口被剪枝耗尽,
    // 都不能靠"学"出新基线来平息计时——预热结束后一次真正发生的偏移变化会一直
    // 被判"超限",直到偏移本身消失,或者出现一次数据缺口强制重新预热(规则 4)。
    if (*divergence_m > s.threshold_m) {
      if (!since_) since_ = t;
    } else {
      since_.reset();
      if (learnable) window_.emplace_back(t, *divergence_m);
    }
  }
  s.since = since_;
  return s;
}

}  // namespace gnss_core
