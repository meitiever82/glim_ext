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
  double sigma_ratio_h_median = 0;   // median(err_h / σ_h) —— 建议系数取此
  double sigma_ratio_h_rms = 0;      // RMS(err_h) / RMS(σ_h)
};

// 以 ref 为基准,对每个 test 记录按最近时间戳(容差 tol_s)在 ref 中配对,按 test 的 quality 分档统计。
// ENU 原点取首个配对的 ref 记录;err_h = hypot(dE,dN),err_v = |dU|;σ_h = hypot(sdn, sde)。
// σ_h == 0 的历元计入 n 与 RMSE,但不参与三种比值。ref 或 test 为空、或无任何配对时返回空 map。
std::map<Quality, QualityStats> compare_by_quality(
    const std::vector<PosRecord>& ref, const std::vector<PosRecord>& test, double tol_s = 0.1);

}  // namespace gnss_core
