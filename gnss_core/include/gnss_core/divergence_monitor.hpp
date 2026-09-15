#pragma once
// device_divergence 规则的阈值与计时(spec §3 C8、§8)。
// 偏差 d 是 610 融合解与 rtkrcv 独立解的水平距离;每拍的判定阈值
//   threshold = divergence_sigma × max(divergence_sigma_floor_m, 经验基线, current_sigma_m)
//   - 5 cm 下限任何时候都生效(spec §8「设 5 cm 下限」),启动/重新预热时也不例外;
//   - 经验基线 = min(divergence_sigma_max_m, max(下限, 已入窗样本的 RMS))。只有 learnable 的
//     样本(调用方:两路都是 FIXED)才可能入窗;设计文档要求经验 σ 且排除当前偏差段,
//     这样 610 "自信地错"也能被抓到;窗口样本不足
//     divergence_min_samples 时:预热结束后沿用 held 的上一次窗口基线,预热期里没有基线;
//   - current_sigma_m 是独立解本拍自报的 hypot(sdn, sde)(无独立解时 0),任何时候都能
//     抬高阈值——rtkrcv 掉到 FLOAT/SINGLE 时两路差几分米是独立解自己的误差,不该报警。
//     它只影响本拍阈值,不写入 held 基线。
//
// 状态与计时:
//   1. 预热期(warming_up_,启动时为真):样本无论是否超限都要入窗口(且 learnable),
//      并用本拍阈值判定——超限则起算/保持 since,否则清零。
//   2. 预热期里窗口第一次攒够 divergence_min_samples(按入窗前的窗口计数),转入经验
//      模式,并在判定这一拍之前清零 since(经验模式的第一次真正超限该有自己的起始
//      时刻)。这是唯一会清零 since 的模式切换。
//   3. 预热期结束后:超限样本排除在窗口外、不重启计时;不超限且 learnable 则清零 since
//      并入窗口(不超限但不 learnable 时同样清零 since,但不入窗口)。
//      窗口被剪枝(<= t - divergence_window_s 的样本出窗)耗尽到不够 min_samples 时,
//      沿用 baseline_sigma_(上一次窗口够 min_samples 时学到的值,held)。
//   4. 只有真正的数据缺口(与上一次配对样本间隔 >= divergence_window_s)才重新预热,
//      同时清掉 held 基线与 since;持续故障或正常数据流每秒都有配对样本,不会触发。
//      未配对(nullopt)、非有限值的 tick 不更新配对时刻。
//   5. 非有限值(NaN/inf)一律当作没配上处理:不入窗口、清零 since、不更新配对时刻。
//
// 学习约束(维护者 2026-09-15 决定):只有 learnable 的样本入窗,rtkrcv 浮点解期间被当前 σ
// 抬高的阈值不会反过来拉宽经验基线;学到的 σ 设上限,启动时或长缺口后重新预热期间存在的
// 持续偏移最多被学成 divergence_sigma_max_m,超过 divergence_sigma × 上限的偏移仍会报出。
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
  bool empirical = false;               // true:本拍有经验基线(窗口 RMS 或 held 基线)参与取 max
};

class DivergenceMonitor {
public:
  explicit DivergenceMonitor(const DiagnosisConfig& cfg);
  // current_sigma_m:独立解本拍自报 σ(调用方传 hypot(sdn, sde),无独立解时传 0),
  // 与下限、经验基线一起取 max,可以抬高阈值但不会被学进基线
  // learnable:调用方判定的"这一拍的偏差代表正常水平"(两路都是 FIXED)。
  //   =false:照常判定与计时,但不入经验窗口
  DivergenceState update(double t, std::optional<double> divergence_m, double current_sigma_m,
                         bool learnable = true);
  size_t window_size() const { return window_.size(); }

private:
  double sigma_mult_, window_s_, floor_m_, cap_m_;
  size_t min_samples_;
  std::deque<std::pair<double, double>> window_;   // (t, d)
  std::optional<double> since_;
  bool warming_up_ = true;              // 是否还在预热(窗口尚未攒够经验基线)
  std::optional<double> last_paired_t_; // 最近一次"配对上"(divergence_m 有限值)的时刻
  std::optional<double> baseline_sigma_; // 上一次窗口够 min_samples 时学到的经验 σ(held)
};

}  // namespace gnss_core
