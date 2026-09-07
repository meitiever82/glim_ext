// calibrate_sigma_scale <ref.pos> <test.pos> [tol_s]
//
// 以 ref.pos(后处理基准或质量最高的轨迹)为真值,把 test.pos 按 test 的解质量分档,
// 打印各档 n / RMSE / 三种 "实际水平误差 / 板卡报 σ_h" 比值,并给出建议的
// quality_sigma_scale(取中位数比值,归一到 FIXED=1)。见 spec §9.2 v2。
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "gnss_core/pos_io.hpp"
#include "gnss_core/trajectory_compare.hpp"

using namespace gnss_core;

namespace {

const char* quality_name(Quality q) {
  switch (q) {
    case Quality::NONE: return "NONE";
    case Quality::SINGLE: return "SINGLE";
    case Quality::DGPS: return "DGPS";
    case Quality::FLOAT: return "FLOAT";
    case Quality::FIXED: return "FIXED";
  }
  return "?";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: calibrate_sigma_scale <ref.pos> <test.pos> [tol_s=0.1]\n"
                 "  ref/test 均为 RTKLIB .pos(头部 time=GPST|UTC 自动识别,默认 GPST,闰秒 18 s)\n";
    return 1;
  }
  const double tol = (argc > 3) ? std::atof(argv[3]) : 0.1;

  std::vector<PosRecord> ref, test;
  try {
    ref = read_pos(argv[1]);
    test = read_pos(argv[2]);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 2;
  }
  const auto stats = compare_by_quality(ref, test, tol);

  std::printf("ref : %s (%zu records)\n", argv[1], ref.size());
  std::printf("test: %s (%zu records)\n", argv[2], test.size());
  std::printf("pair tolerance: %.3f s\n\n", tol);
  if (stats.empty()) {
    std::printf("no epochs paired within tolerance -- check time system (GPST vs UTC) and overlap.\n");
    return 3;
  }

  std::printf("%-7s %6s %10s %10s %13s %10s %11s\n",
              "quality", "n", "rmse_h(m)", "rmse_v(m)", "ratio_median", "ratio_rms", "ratio_mean");
  double fixed_ratio = 0.0;
  for (const auto& [q, s] : stats) {
    std::printf("%-7s %6d %10.3f %10.3f %13.3f %10.3f %11.3f\n", quality_name(q), s.n, s.rmse_h, s.rmse_v,
                s.sigma_ratio_h_median, s.sigma_ratio_h_rms, s.sigma_ratio_h_mean);
    if (q == Quality::FIXED) fixed_ratio = s.sigma_ratio_h_median;
  }

  std::printf("\nsuggested quality_sigma_scale (median ratio, normalized to FIXED=1");
  if (fixed_ratio <= 0.0) std::printf("; no FIXED tier -> raw median ratio");
  std::printf("):\n");
  for (const auto& [q, s] : stats) {
    const double v = (fixed_ratio > 0.0) ? s.sigma_ratio_h_median / fixed_ratio : s.sigma_ratio_h_median;
    std::printf("  %-7s = %.3f\n", quality_name(q), v);
  }
  if (fixed_ratio > 0.0) {
    std::printf("\nnote: FIXED tier raw median ratio = %.3f; if far from 1, also scale sigma_floor / "
                "the board sigma globally by this amount.\n", fixed_ratio);
  }
  return 0;
}
