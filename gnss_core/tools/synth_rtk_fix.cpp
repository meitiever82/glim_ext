// synth_rtk_fix —— GLIM 轨迹 → 合成 RTK 观测(含故障注入)→ 标准 .pos(spec §12.3,Task 13)
//
// usage: synth_rtk_fix <traj_imu.txt> <out.pos> [--lever x y z] [--wrong-fix 0.05] [--stale from to]
//                      [--float from to] [--seed N] [--truth out_truth.pos]
//                      [--origin lat lon alt] [--rate hz] [--sigma-fixed e n u] [--sigma-float e n u]
//                      [--yaw deg] [--offset x y z]
//
// - 时间段参数(--stale/--float)是**绝对时间**(与轨迹 stamp 同一时间轴,通常是 unix 秒);
//   也可写成 "+12.5" 表示相对轨迹起点的秒数。
// - --yaw/--offset 设 T_enu_world(把 world 放进 ENU 的位姿),用于测 FrameAligner;默认单位阵。
// - 输出 .pos 头写 time=UTC(stamp 原样为 unix 秒);--truth 另写无噪声、无注入的真值 .pos。
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "gnss_core/pos_io.hpp"
#include "gnss_core/synth.hpp"

using namespace gnss_core;

namespace {

void usage() {
  std::fprintf(stderr,
               "usage: synth_rtk_fix <traj_imu.txt> <out.pos> [--lever x y z] [--wrong-fix 0.05] [--stale from to]\n"
               "                     [--float from to] [--seed N] [--truth out_truth.pos]\n"
               "                     [--origin lat lon alt] [--rate hz] [--sigma-fixed e n u] [--sigma-float e n u]\n"
               "                     [--yaw deg] [--offset x y z]\n"
               "  时间段参数为绝对时间;前缀 '+' 表示相对轨迹起点的秒数。\n");
}

// "+12.5" → t0 + 12.5;否则原样
double parse_time(const std::string& s, double t0) {
  if (!s.empty() && s[0] == '+') return t0 + std::atof(s.c_str() + 1);
  return std::atof(s.c_str());
}

bool need(int argc, int i, int n) {
  if (i + n >= argc) { usage(); std::exit(2); }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) { usage(); return 2; }
  const std::string traj_path = argv[1], out_path = argv[2];

  std::vector<TrajPose> traj;
  try {
    traj = read_glim_traj(traj_path);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  if (traj.size() < 2) { std::fprintf(stderr, "error: trajectory has < 2 poses\n"); return 1; }
  const double t0 = traj.front().stamp;

  SynthConfig cfg;
  Injection inj;
  std::string truth_path;
  double yaw_deg = 0.0;
  Eigen::Vector3d offset = Eigen::Vector3d::Zero();

  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--lever" && need(argc, i, 3)) {
      cfg.lever_imu = Eigen::Vector3d(std::atof(argv[i + 1]), std::atof(argv[i + 2]), std::atof(argv[i + 3])); i += 3;
    } else if (a == "--wrong-fix" && need(argc, i, 1)) {
      inj.wrong_fix_ratio = std::atof(argv[++i]);
    } else if (a == "--stale" && need(argc, i, 2)) {
      inj.stale_from = parse_time(argv[i + 1], t0); inj.stale_to = parse_time(argv[i + 2], t0); i += 2;
    } else if (a == "--float" && need(argc, i, 2)) {
      inj.float_from = parse_time(argv[i + 1], t0); inj.float_to = parse_time(argv[i + 2], t0); i += 2;
    } else if (a == "--seed" && need(argc, i, 1)) {
      cfg.seed = static_cast<unsigned>(std::atol(argv[++i]));
    } else if (a == "--truth" && need(argc, i, 1)) {
      truth_path = argv[++i];
    } else if (a == "--origin" && need(argc, i, 3)) {
      cfg.lat0 = std::atof(argv[i + 1]); cfg.lon0 = std::atof(argv[i + 2]); cfg.alt0 = std::atof(argv[i + 3]); i += 3;
    } else if (a == "--rate" && need(argc, i, 1)) {
      cfg.rate_hz = std::atof(argv[++i]);
    } else if (a == "--sigma-fixed" && need(argc, i, 3)) {
      cfg.sigma_fixed = Eigen::Vector3d(std::atof(argv[i + 1]), std::atof(argv[i + 2]), std::atof(argv[i + 3])); i += 3;
    } else if (a == "--sigma-float" && need(argc, i, 3)) {
      cfg.sigma_float = Eigen::Vector3d(std::atof(argv[i + 1]), std::atof(argv[i + 2]), std::atof(argv[i + 3])); i += 3;
    } else if (a == "--yaw" && need(argc, i, 1)) {
      yaw_deg = std::atof(argv[++i]);
    } else if (a == "--offset" && need(argc, i, 3)) {
      offset = Eigen::Vector3d(std::atof(argv[i + 1]), std::atof(argv[i + 2]), std::atof(argv[i + 3])); i += 3;
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
      usage();
      return 2;
    }
  }
  cfg.T_enu_world = Eigen::Isometry3d::Identity();
  cfg.T_enu_world.linear() = Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  cfg.T_enu_world.translation() = offset;

  const SynthResult res = synthesize(traj, cfg, inj);
  std::vector<PosRecord> recs;
  recs.reserve(res.samples.size());
  for (const auto& s : res.samples) recs.push_back(sample_to_pos_record(s));
  write_pos(out_path, recs, PosTimeSystem::UTC);

  if (!truth_path.empty()) {
    SynthConfig clean = cfg;
    clean.sigma_fixed.setZero();
    clean.sigma_float.setZero();
    const SynthResult tr = synthesize(traj, clean, Injection{});
    std::vector<PosRecord> trecs;
    trecs.reserve(tr.samples.size());
    for (const auto& s : tr.samples) trecs.push_back(sample_to_pos_record(s));
    write_pos(truth_path, trecs, PosTimeSystem::UTC);
  }

  int n_float = 0, n_stale = 0;
  for (const auto& s : res.samples) {
    n_float += (s.quality == Quality::FLOAT);
    n_stale += (s.diff_age > 1.5);
  }
  std::printf("traj: %zu poses, %.1f s  ->  %zu samples @ %.1f Hz -> %s\n", traj.size(),
              traj.back().stamp - t0, res.samples.size(), cfg.rate_hz, out_path.c_str());
  std::printf("lever_imu=[%.3f %.3f %.3f] yaw=%.1f deg offset=[%.2f %.2f %.2f] seed=%u\n", cfg.lever_imu.x(),
              cfg.lever_imu.y(), cfg.lever_imu.z(), yaw_deg, offset.x(), offset.y(), offset.z(), cfg.seed);
  std::printf("injection: wrong_fix=%.3f (%d epochs) float=%d epochs stale=%d epochs\n", inj.wrong_fix_ratio,
              static_cast<int>(std::lround(res.samples.size() * inj.wrong_fix_ratio)), n_float, n_stale);
  if (!truth_path.empty()) std::printf("truth -> %s\n", truth_path.c_str());
  return 0;
}
