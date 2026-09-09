#pragma once
#include <map>
#include <vector>
#include "gnss_core/pos_io.hpp"
#include "gnss_core/types.hpp"

namespace gnss_core {

// 一个质量档的误差统计(spec §9.2 v2)。
struct QualityStats {
  int n = 0;                         // 配对成功的历元数(含 σ_h=0 的历元)
  double rmse_h = 0, rmse_v = 0;     // test 相对 ref 的水平 / 垂直 RMSE(m)
  double sigma_ratio_h_mean = 0;     // mean(err_h / σ_h) —— 仅参考,受外点主导
  // median(err_h / σ_h):抗外点,但**有偏**——若 E/N 误差各为 N(0,σ) 且 σ_h = hypot(σ_n, σ_e),
  // 则 err_h/σ_h 服从 Rayleigh(1/√2),其中位数 = √(ln 4)/√2 ≈ 0.83。
  // 即对标定良好的板卡此值期望约 0.83 而非 1;把它当"1 = 无偏"会把 σ 低估约 17%。
  double sigma_ratio_h_median = 0;
  // RMS(err_h) / RMS(σ_h):同一假设下期望恰为 1,才是无偏估计;受外点影响,建议与 median 对照看。
  double sigma_ratio_h_rms = 0;
};

// 以 ref 为基准,对每个 test 记录按最近时间戳(容差 tol_s)在 ref 中配对,按 test 的 quality 分档统计。
// ENU 原点取首个配对的 ref 记录;err_h = hypot(dE,dN),err_v = |dU|;σ_h = hypot(sdn, sde)。
// σ_h == 0 的历元计入 n 与 RMSE,但不参与三种比值。ref 或 test 为空、或无任何配对时返回空 map。
// ref_min_q(RTKLIB Q 语义:1 fix 2 float 4 dgps 5 single;0 = 无解):
//   ref_min_q > 0 时,ref 中 q > ref_min_q 或 q == 0 的记录不作基准(被过滤,不参与配对);
//   默认 1 = 只用 FIXED 的 ref;传 0 = 不过滤(沿用旧行为)。
std::map<Quality, QualityStats> compare_by_quality(
    const std::vector<PosRecord>& ref, const std::vector<PosRecord>& test, double tol_s = 0.1,
    int ref_min_q = 1);

}  // namespace gnss_core
