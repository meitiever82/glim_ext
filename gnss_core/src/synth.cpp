#include "gnss_core/synth.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>

#include "gnss_core/geodetic.hpp"

namespace gnss_core {

std::vector<TrajPose> read_glim_traj(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("read_glim_traj: cannot open " + path);
  std::vector<TrajPose> out;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    double t, x, y, z, qx, qy, qz, qw;
    if (!(ss >> t >> x >> y >> z >> qx >> qy >> qz >> qw)) continue;   // 列数不足:跳过
    TrajPose p;
    p.stamp = t;
    p.T_world_imu = Eigen::Isometry3d::Identity();
    p.T_world_imu.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
    p.T_world_imu.translation() = Eigen::Vector3d(x, y, z);
    out.push_back(p);
  }
  std::stable_sort(out.begin(), out.end(), [](const TrajPose& a, const TrajPose& b) { return a.stamp < b.stamp; });
  return out;
}

bool interpolate_pose(const std::vector<TrajPose>& traj, double t, TrajPose& out) {
  if (traj.empty()) return false;
  if (t < traj.front().stamp || t > traj.back().stamp) return false;
  // 第一个 stamp > t 的元素;t == back().stamp 时取最后一段
  auto hi = std::upper_bound(traj.begin(), traj.end(), t,
                             [](double v, const TrajPose& p) { return v < p.stamp; });
  if (hi == traj.begin()) { out = traj.front(); out.stamp = t; return true; }
  if (hi == traj.end()) { out = traj.back(); out.stamp = t; return true; }
  const TrajPose& a = *(hi - 1);
  const TrajPose& b = *hi;
  const double span = b.stamp - a.stamp;
  const double s = (span > 0.0) ? (t - a.stamp) / span : 0.0;
  const Eigen::Quaterniond qa(a.T_world_imu.linear()), qb(b.T_world_imu.linear());
  out.stamp = t;
  out.T_world_imu = Eigen::Isometry3d::Identity();
  out.T_world_imu.linear() = qa.slerp(s, qb).toRotationMatrix();
  out.T_world_imu.translation() = (1.0 - s) * a.T_world_imu.translation() + s * b.T_world_imu.translation();
  return true;
}

namespace {

inline bool in_segment(double t, double from, double to) { return from >= 0 && to > from && t >= from && t <= to; }

}  // namespace

SynthResult synthesize(const std::vector<TrajPose>& traj, const SynthConfig& cfg, const Injection& inj) {
  SynthResult res;
  if (traj.empty() || cfg.rate_hz <= 0.0) return res;

  const double t0 = traj.front().stamp, t1 = traj.back().stamp;
  const double dt = 1.0 / cfg.rate_hz;
  const int n = static_cast<int>(std::floor((t1 - t0) / dt + 1e-9)) + 1;
  res.samples.reserve(n);
  res.truth_enu.reserve(n);

  LlaToEnu conv(cfg.lat0, cfg.lon0, cfg.alt0);
  std::mt19937 rng(cfg.seed);
  std::normal_distribution<double> gauss(0.0, 1.0);
  std::uniform_real_distribution<double> uni(0.0, 2.0 * M_PI);

  // 错误固定历元:用 seed 确定性选 round(N·ratio) 个(与噪声流独立的 RNG,避免比例改变时噪声跟着变)
  std::vector<char> wrong(n, 0);
  if (inj.wrong_fix_ratio > 0.0) {
    const int n_wrong = std::min(n, static_cast<int>(std::lround(n * inj.wrong_fix_ratio)));
    std::vector<int> idx(n);
    std::iota(idx.begin(), idx.end(), 0);
    std::mt19937 rng_sel(cfg.seed ^ 0x9e3779b9u);
    std::shuffle(idx.begin(), idx.end(), rng_sel);
    for (int i = 0; i < n_wrong; ++i) wrong[idx[i]] = 1;
  }

  for (int k = 0; k < n; ++k) {
    const double t = t0 + k * dt;
    TrajPose p;
    if (!interpolate_pose(traj, std::min(t, t1), p)) continue;

    const Eigen::Vector3d truth = cfg.T_enu_world * (p.T_world_imu * cfg.lever_imu);

    RtkFixSample s;
    s.stamp = t;
    s.gnss_time = t;
    s.header_stamp = t + 0.05;
    s.sats_used = 20;
    s.diff_age = 1.0;
    s.quality = Quality::FIXED;

    const bool is_float = in_segment(t, inj.float_from, inj.float_to);
    const Eigen::Vector3d sigma_true = is_float ? cfg.sigma_float : cfg.sigma_fixed;
    if (is_float) s.quality = Quality::FLOAT;
    s.sigma_enu = sigma_true;   // 报告值 = 当前档 σ;错误固定仍报 sigma_fixed

    Eigen::Vector3d enu = truth;
    for (int i = 0; i < 3; ++i) enu[i] += sigma_true[i] * gauss(rng);

    if (wrong[k]) {
      const double dir = uni(rng);
      enu += Eigen::Vector3d(std::cos(dir), std::sin(dir), 0.0);   // 位置偏 1 m,随机水平方向
    }

    if (in_segment(t, inj.stale_from, inj.stale_to)) {
      s.diff_age = 60.0 * (t - inj.stale_from) / (inj.stale_to - inj.stale_from);
    }

    const Eigen::Vector3d lla = conv.reverse(enu);
    s.lat = lla.x();
    s.lon = lla.y();
    s.alt = lla.z();

    res.samples.push_back(s);
    res.truth_enu.push_back(truth);
  }
  return res;
}

int quality_to_q(Quality q) {
  switch (q) {
    case Quality::FIXED: return 1;
    case Quality::FLOAT: return 2;
    case Quality::DGPS: return 4;
    case Quality::SINGLE: return 5;
    default: return 0;
  }
}

PosRecord sample_to_pos_record(const RtkFixSample& s) {
  PosRecord r;
  r.stamp = s.stamp;
  r.lat = s.lat;
  r.lon = s.lon;
  r.height = s.alt;
  r.q = quality_to_q(s.quality);
  r.ns = s.sats_used;
  r.sdne = Eigen::Vector3d(s.sigma_enu.y(), s.sigma_enu.x(), s.sigma_enu.z());
  r.age = s.diff_age;
  r.ratio = 0.0;
  return r;
}

RtkFixSample pos_record_to_sample(const PosRecord& r) {
  RtkFixSample s;
  s.stamp = r.stamp;
  s.gnss_time = r.stamp;
  s.header_stamp = r.stamp;
  s.quality = q_to_quality(r.q);
  s.lat = r.lat;
  s.lon = r.lon;
  s.alt = r.height;
  s.sigma_enu = Eigen::Vector3d(r.sdne(1), r.sdne(0), r.sdne(2));
  s.diff_age = r.age;
  s.sats_used = r.ns;
  return s;
}

}  // namespace gnss_core
