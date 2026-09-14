#pragma once
// device_divergence 规则的阈值与计时(spec §3 C8)。
// σ 是已入窗样本(被判定为"未超限"而收纳进 window_ 的那些偏差)的 RMS——设计文档
// 要求经验 σ 且排除当前偏差段,这样 610 "自信地错"(自报 σ 很小)也能被抓到;
// 样本不足(divergence_min_samples)时回退到 rtkrcv 自报 σ 判定(design decision 2:
// 回退阶段仍然判定,不是不判定)。
//
// 用显式的预热状态 warming_up_,不再从 empirical 派生("回退/经验切换即重置计时"
// 那版规则会让一次持续 ~divergence_window_s 的偏差被剪枝耗尽窗口、误判成新基线,
// round3a fix1 已发现并推翻):
//   1. 预热期(warming_up_ == true):样本无论是否超限都要入窗口,并用当前阈值
//      (窗口样本不够 min_samples 时是回退 σ)判定——超限则起算/保持 since,
//      否则清零。
//   2. 预热期里窗口第一次攒够 divergence_min_samples,转入经验模式,并在判定
//      这一拍之前清零 since(经验模式的第一次真正超限该有自己的起始时刻,不能
//      借用预热期攒下的旧计时)。这是唯一会清零 since 的模式切换。
//   3. 预热期结束后:超限样本排除在窗口外、不重启计时;不超限则清零 since 并
//      入窗口。
//   4. 只有真正的数据缺口(两次配对样本时刻间隔 >= divergence_window_s)才重新
//      预热;持续故障或正常数据流每秒都有配对样本,不会触发。未配对
//      (nullopt)、非有限值的 tick 不更新配对时刻。
//   5. 非有限值(NaN/inf)一律当作没配上处理:不入窗口、不重启/保持 since、
//      不更新配对时刻。
//
// round3a fix2:预热结束后,若窗口被剪枝耗尽到不够 min_samples,不再掉回 rtkrcv
// 自报的回退 σ(那往往只有几毫米,连正常偏差都会被判"超限",样本永远进不了
// 窗口,since 永远清不掉——fix1 之后发现的 N1)。改为沿用 baseline_sigma_:上一次
// 窗口够 min_samples 时学到的经验 σ,一直保留(held)到窗口重新攒够样本、或者
// 出现一次 >= divergence_window_s 的数据缺口(此时随 warming_up_ 一起清零,交给
// 下一次预热重新学)。
// 代价:(1) 预热期里若真的发生过一次偏差,会被计入第一份经验基线;
//      (2) 预热结束后一次真正发生的 610/rtkrcv 偏移变化,会一直被判"超限",
//          直到偏移本身消失,或者出现一次数据缺口强制重新预热。
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
  bool warming_up_ = true;              // 是否还在预热(窗口尚未攒够经验基线)
  std::optional<double> last_paired_t_; // 最近一次"配对上"(divergence_m 有限值)的时刻
  std::optional<double> baseline_sigma_; // 上一次窗口够 min_samples 时学到的经验 σ(held)
};

}  // namespace gnss_core
