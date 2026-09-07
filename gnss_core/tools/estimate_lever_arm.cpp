// estimate_lever_arm —— GLIM 轨迹 + RTK .pos → lever_imu、T_world_enu、time_offset(spec §9.3,Task 14)
//
// usage: estimate_lever_arm <traj_imu.txt> <rtk.pos> [--origin lat lon alt] [--dt-range -0.5 0.5]
//                           [--dt-step 0.01] [--min-quality 4] [--pair-tol 0.05] [--pos-time GPST|UTC] [--leap 18]
//
// - traj_imu.txt:glim_rosbag dump 的 TUM 轨迹(t x y z qx qy qz qw),时间轴须与 .pos 一致(unix 秒)。
// - rtk.pos:同段 RtkFix 用 export_bag_to_pos.py 导出;或 synth_rtk_fix 的合成数据(自检)。
// - --origin 缺省取 .pos 首个达标历元的经纬高。
// - --min-quality 用归一化枚举:4=FIXED 3=FLOAT 2=DGPS 1=SINGLE。
// 输出末尾两行可直接粘进 config_rtk_global.json。
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "gnss_core/geodetic.hpp"
#include "gnss_core/lever_arm_estimator.hpp"
#include "gnss_core/pos_io.hpp"
#include "gnss_core/synth.hpp"

using namespace gnss_core;

namespace {

void usage() {
  std::fprintf(stderr,
               "usage: estimate_lever_arm <traj_imu.txt> <rtk.pos> [--origin lat lon alt] [--dt-range -0.5 0.5]\n"
               "                          [--dt-step 0.01] [--min-quality 4] [--pair-tol 0.05]\n"
               "                          [--pos-time GPST|UTC] [--leap 18]\n");
}

bool need(int argc, int i, int n) {
  if (i + n >= argc) { usage(); std::exit(2); }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) { usage(); return 2; }
  const std::string traj_path = argv[1], pos_path = argv[2];

  LeverArmOptions opt;
  PosReadOptions pos_opt;
  bool have_origin = false;
  double lat0 = 0, lon0 = 0, alt0 = 0;
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--origin" && need(argc, i, 3)) {
      lat0 = std::atof(argv[i + 1]); lon0 = std::atof(argv[i + 2]); alt0 = std::atof(argv[i + 3]); i += 3;
      have_origin = true;
    } else if (a == "--dt-range" && need(argc, i, 2)) {
      opt.dt_min = std::atof(argv[i + 1]); opt.dt_max = std::atof(argv[i + 2]); i += 2;
    } else if (a == "--dt-step" && need(argc, i, 1)) {
      opt.dt_step = std::atof(argv[++i]);
    } else if (a == "--min-quality" && need(argc, i, 1)) {
      opt.min_quality = static_cast<Quality>(std::atoi(argv[++i]));
    } else if (a == "--pair-tol" && need(argc, i, 1)) {
      opt.pair_tol = std::atof(argv[++i]);
    } else if (a == "--pos-time" && need(argc, i, 1)) {
      const std::string v = argv[++i];
      pos_opt.default_time_system = (v == "UTC") ? PosTimeSystem::UTC : PosTimeSystem::GPST;
    } else if (a == "--leap" && need(argc, i, 1)) {
      pos_opt.leap_seconds = std::atoi(argv[++i]);
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
      usage();
      return 2;
    }
  }

  std::vector<TrajPose> traj;
  std::vector<PosRecord> recs;
  try {
    traj = read_glim_traj(traj_path);
    recs = read_pos(pos_path, pos_opt);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  if (traj.size() < 2) { std::fprintf(stderr, "error: trajectory has < 2 poses\n"); return 1; }
  if (recs.empty()) { std::fprintf(stderr, "error: no epochs in %s\n", pos_path.c_str()); return 1; }

  std::vector<RtkFixSample> fixes;
  fixes.reserve(recs.size());
  int n_quality_ok = 0;
  for (const auto& r : recs) {
    fixes.push_back(pos_record_to_sample(r));
    if (static_cast<int>(fixes.back().quality) >= static_cast<int>(opt.min_quality)) {
      if (!have_origin) { lat0 = r.lat; lon0 = r.lon; alt0 = r.height; have_origin = true; }
      ++n_quality_ok;
    }
  }
  if (!have_origin) { std::fprintf(stderr, "error: no epoch reaches min quality; cannot pick origin\n"); return 1; }

  std::printf("traj : %zu poses  [%.3f, %.3f]  (%.1f s)\n", traj.size(), traj.front().stamp, traj.back().stamp,
              traj.back().stamp - traj.front().stamp);
  std::printf("rtk  : %zu epochs [%.3f, %.3f], %d with quality >= %d\n", fixes.size(), fixes.front().stamp,
              fixes.back().stamp, n_quality_ok, static_cast<int>(opt.min_quality));
  const double overlap = std::min(traj.back().stamp, fixes.back().stamp) - std::max(traj.front().stamp, fixes.front().stamp);
  if (overlap <= 0) {
    std::fprintf(stderr, "error: no time overlap between trajectory and .pos (check time systems: --pos-time)\n");
    return 1;
  }
  std::printf("origin: lat=%.8f lon=%.8f alt=%.3f\n", lat0, lon0, alt0);

  LlaToEnu conv(lat0, lon0, alt0);
  const LeverArmEstimate est = estimate_lever_arm(traj, fixes, conv, opt);

  std::printf("\n--- result ---\n");
  std::printf("n_pairs        : %d\n", est.n_pairs);
  if (est.n_pairs < opt.min_pairs) {
    std::fprintf(stderr, "error: too few pairs (%d < %d); check time overlap / pair_tol / quality\n", est.n_pairs,
                 opt.min_pairs);
    return 1;
  }
  const double yaw = std::atan2(est.T_world_enu.linear()(1, 0), est.T_world_enu.linear()(0, 0));
  std::printf("lever_imu      : [%.3f, %.3f, %.3f] m\n", est.lever_imu.x(), est.lever_imu.y(), est.lever_imu.z());
  std::printf("time_offset    : %+.3f s  (add to RTK stamps)\n", est.time_offset);
  std::printf("T_world_enu    : yaw=%.3f deg  t=[%.3f, %.3f, %.3f]\n", yaw * 180.0 / M_PI,
              est.T_world_enu.translation().x(), est.T_world_enu.translation().y(), est.T_world_enu.translation().z());
  std::printf("rms_residual   : %.4f m\n", est.rms_residual);
  std::printf("yaw range      : %.1f deg   tilt range: %.1f deg\n", est.yaw_range_deg, est.tilt_range_deg);

  bool warn = false;
  if (!est.lever_observable) {
    std::printf("WARNING: lever NOT observable (yaw range %.1f deg < %.0f deg) — horizontal lever meaningless; measure it\n",
                est.yaw_range_deg, opt.min_yaw_range_deg);
    warn = true;
  }
  if (!est.lever_z_observable) {
    std::printf("NOTE   : lever.z not observable (tilt range %.1f deg < %.0f deg) — pinned to 0, absorbed by T_world_enu.z\n",
                est.tilt_range_deg, opt.min_tilt_range_deg);
  }
  if (!est.time_offset_observable) {
    std::printf("WARNING: time_offset NOT observable (residual flat vs dt: straight constant-speed run?) — keep 0\n");
    warn = true;
  }
  if (est.rms_residual > 0.2) {
    std::printf("WARNING: rms_residual %.3f m is large — trajectory/.pos mismatch or drift; result unreliable\n",
                est.rms_residual);
    warn = true;
  }

  std::printf("\n--- paste into config_rtk_global.json ---\n");
  const Eigen::Vector3d L = est.lever_observable ? est.lever_imu : Eigen::Vector3d::Zero();
  std::printf("\"T_imu_gnss\": [%.3f, %.3f, %.3f],\n", L.x(), L.y(), L.z());
  std::printf("\"time_offset\": %.3f,\n", est.time_offset_observable ? est.time_offset : 0.0);
  return warn ? 3 : 0;
}
