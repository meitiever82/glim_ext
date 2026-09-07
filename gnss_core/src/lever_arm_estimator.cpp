#include "gnss_core/lever_arm_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Dense>

namespace gnss_core {

namespace {

struct Pair {
  Eigen::Matrix3d R;      // R_world_imu(t_i + Δt)
  Eigen::Vector3d t;      // t_world_imu
  Eigen::Vector3d enu;
};

struct InnerSolution {
  Eigen::Vector3d lever = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  double sum_sq = std::numeric_limits<double>::infinity();
};

Eigen::Matrix3d Rz(double yaw) { return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix(); }

// 2D Umeyama(仅 yaw + 平移):R·src + p ≈ dst。返回 yaw;p 由调用方按需重算。
// 与 FrameAligner 同款,但不带冻结语义,独立实现以免耦合。
double umeyama_yaw(const std::vector<Eigen::Vector3d>& src, const std::vector<Eigen::Vector3d>& dst) {
  Eigen::Vector2d ms = Eigen::Vector2d::Zero(), md = Eigen::Vector2d::Zero();
  for (size_t i = 0; i < src.size(); ++i) { ms += src[i].head<2>(); md += dst[i].head<2>(); }
  ms /= static_cast<double>(src.size());
  md /= static_cast<double>(src.size());
  Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
  for (size_t i = 0; i < src.size(); ++i) cov += (src[i].head<2>() - ms) * (dst[i].head<2>() - md).transpose();
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d R2 = svd.matrixV() * svd.matrixU().transpose();
  if (R2.determinant() < 0) {
    Eigen::Matrix2d V = svd.matrixV();
    V.col(1) *= -1;
    R2 = V * svd.matrixU().transpose();
  }
  return std::atan2(R2(1, 0), R2(0, 0));
}

double sum_sq_residual(const std::vector<Pair>& pairs, const InnerSolution& s) {
  const Eigen::Matrix3d Ry = Rz(s.yaw);
  double acc = 0.0;
  for (const auto& q : pairs) acc += (q.R * s.lever + q.t - (Ry * q.enu + s.p)).squaredNorm();
  return acc;
}

// 给定 Δt 的内层求解。solve_z=false 时 lever.z 钉 0(只解 5 个未知量)。
InnerSolution solve_inner(const std::vector<Pair>& pairs, bool solve_z, int iters) {
  const int N = static_cast<int>(pairs.size());
  InnerSolution s;

  // 初值:lever = 0 时的 2D Umeyama
  std::vector<Eigen::Vector3d> src(N), dst(N);
  for (int i = 0; i < N; ++i) { src[i] = pairs[i].enu; dst[i] = pairs[i].t; }
  s.yaw = umeyama_yaw(src, dst);

  const int n_unk = solve_z ? 6 : 5;   // [lever_x lever_y (lever_z) p_x p_y p_z]
  Eigen::MatrixXd A(3 * N, n_unk);
  Eigen::VectorXd b(3 * N);
  for (int it = 0; it < iters; ++it) {
    // (1) 给定 yaw:R_i·lever − p = R_yaw·enu_i − t_i,线性 LS
    const Eigen::Matrix3d Ry = Rz(s.yaw);
    for (int i = 0; i < N; ++i) {
      A.block<3, 2>(3 * i, 0) = pairs[i].R.leftCols<2>();
      int c = 2;
      if (solve_z) { A.block<3, 1>(3 * i, c) = pairs[i].R.col(2); ++c; }
      A.block<3, 3>(3 * i, c) = -Eigen::Matrix3d::Identity();
      b.segment<3>(3 * i) = Ry * pairs[i].enu - pairs[i].t;
    }
    const Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
    s.lever = Eigen::Vector3d(x(0), x(1), solve_z ? x(2) : 0.0);
    s.p = x.tail<3>();

    // (2) 给定 lever:对校正后的点 (t_i + R_i·lever) 重做 Umeyama 取 yaw
    for (int i = 0; i < N; ++i) dst[i] = pairs[i].t + pairs[i].R * s.lever;
    const double yaw_new = umeyama_yaw(src, dst);
    const bool converged = std::abs(yaw_new - s.yaw) < 1e-9;
    s.yaw = yaw_new;
    if (converged) break;
  }
  // 最后一轮 yaw 更新后再解一次 (lever, p),保证输出自洽
  {
    const Eigen::Matrix3d Ry = Rz(s.yaw);
    for (int i = 0; i < N; ++i) b.segment<3>(3 * i) = Ry * pairs[i].enu - pairs[i].t;
    const Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
    s.lever = Eigen::Vector3d(x(0), x(1), solve_z ? x(2) : 0.0);
    s.p = x.tail<3>();
  }
  s.sum_sq = sum_sq_residual(pairs, s);
  return s;
}

// 在 Δt 处建立配对
std::vector<Pair> build_pairs(const std::vector<TrajPose>& traj, const std::vector<RtkFixSample>& fixes,
                              const std::vector<Eigen::Vector3d>& enu, double dt, double pair_tol) {
  std::vector<Pair> out;
  out.reserve(fixes.size());
  for (size_t i = 0; i < fixes.size(); ++i) {
    const double t = fixes[i].stamp + dt;
    TrajPose p;
    if (!interpolate_pose(traj, t, p)) continue;
    // 最近轨迹点须在 pair_tol 内(轨迹有断档时不跨段插值)
    auto hi = std::lower_bound(traj.begin(), traj.end(), t,
                               [](const TrajPose& a, double v) { return a.stamp < v; });
    double gap = std::numeric_limits<double>::infinity();
    if (hi != traj.end()) gap = std::min(gap, hi->stamp - t);
    if (hi != traj.begin()) gap = std::min(gap, t - (hi - 1)->stamp);
    if (gap > pair_tol + 1e-9) continue;
    out.push_back({p.T_world_imu.linear(), p.T_world_imu.translation(), enu[i]});
  }
  return out;
}

double unwrap_range_deg(std::vector<double> angles) {
  if (angles.empty()) return 0.0;
  double prev = angles[0], acc = angles[0];
  double lo = acc, hi = acc;
  for (size_t i = 1; i < angles.size(); ++i) {
    double d = angles[i] - prev;
    while (d > M_PI) d -= 2 * M_PI;
    while (d < -M_PI) d += 2 * M_PI;
    acc += d;
    prev = angles[i];
    lo = std::min(lo, acc);
    hi = std::max(hi, acc);
  }
  return (hi - lo) * 180.0 / M_PI;
}

}  // namespace

LeverArmEstimate estimate_lever_arm(const std::vector<TrajPose>& traj, const std::vector<RtkFixSample>& fixes,
                                    const LlaToEnu& conv, const LeverArmOptions& opt) {
  LeverArmEstimate est;
  if (traj.size() < 2 || fixes.empty()) return est;

  // 可观性诊断(基于整条轨迹)
  {
    std::vector<double> yaws;
    yaws.reserve(traj.size());
    double tilt_lo = std::numeric_limits<double>::infinity(), tilt_hi = -tilt_lo;
    for (const auto& p : traj) {
      const Eigen::Matrix3d& R = p.T_world_imu.linear();
      yaws.push_back(std::atan2(R(1, 0), R(0, 0)));
      // 倾斜角:body z 轴与 world z 轴夹角
      const double tilt = std::acos(std::clamp(R(2, 2), -1.0, 1.0));
      tilt_lo = std::min(tilt_lo, tilt);
      tilt_hi = std::max(tilt_hi, tilt);
    }
    est.yaw_range_deg = unwrap_range_deg(yaws);
    est.tilt_range_deg = (tilt_hi - tilt_lo) * 180.0 / M_PI;
  }
  const bool yaw_ok = est.yaw_range_deg >= opt.min_yaw_range_deg;
  const bool tilt_ok = est.tilt_range_deg >= opt.min_tilt_range_deg;

  // 质量门限 + ENU
  std::vector<RtkFixSample> good;
  std::vector<Eigen::Vector3d> enu;
  for (const auto& f : fixes) {
    if (static_cast<int>(f.quality) < static_cast<int>(opt.min_quality)) continue;
    good.push_back(f);
    enu.push_back(conv.forward(f.lat, f.lon, f.alt));
  }

  // 外层 Δt 搜索:粗网格(5×dt_step)→ 细网格(dt_step,粗最优 ±1 粗步)→ 抛物线细化
  struct Cand { double dt; InnerSolution sol; int n; bool ok; };
  auto eval = [&](double dt) -> Cand {
    auto pairs = build_pairs(traj, good, enu, dt, opt.pair_tol);
    if (static_cast<int>(pairs.size()) < opt.min_pairs) return {dt, {}, static_cast<int>(pairs.size()), false};
    return {dt, solve_inner(pairs, tilt_ok, opt.inner_iters), static_cast<int>(pairs.size()), true};
  };
  auto msr = [](const Cand& c) { return c.sol.sum_sq / c.n; };
  // 平局(相对差 < 1e-6)取 |Δt| 最小者:匀速直线上时移等价于平移,残差对 Δt 不敏感
  auto better = [&](const Cand& a, const Cand& b) {   // a 优于 b?
    if (!a.ok) return false;
    if (!b.ok) return true;
    const double ma = msr(a), mb = msr(b);
    if (ma < mb * (1 - 1e-6) - 1e-12) return true;
    if (ma > mb * (1 + 1e-6) + 1e-12) return false;
    return std::abs(a.dt) < std::abs(b.dt);
  };
  auto grid_search = [&](double lo, double hi, double step) {
    std::vector<Cand> cs;
    const int n = std::max(0, static_cast<int>(std::floor((hi - lo) / step + 1e-9)));
    for (int k = 0; k <= n; ++k) cs.push_back(eval(lo + k * step));
    return cs;
  };
  auto argbest = [&](const std::vector<Cand>& cs) {
    size_t b = 0;
    for (size_t i = 1; i < cs.size(); ++i) if (better(cs[i], cs[b])) b = i;
    return b;
  };

  const double step = opt.dt_step;
  const double coarse = 5.0 * step;
  std::vector<Cand> cands = grid_search(opt.dt_min, opt.dt_max, coarse);
  size_t best = argbest(cands);
  if (!cands[best].ok) {
    est.n_pairs = static_cast<int>(build_pairs(traj, good, enu, 0.0, opt.pair_tol).size());
    return est;
  }
  {
    const double c = cands[best].dt;
    cands = grid_search(std::max(opt.dt_min, c - coarse), std::min(opt.dt_max, c + coarse), step);
    best = argbest(cands);
  }
  const double msr_min = msr(cands[best]);
  double dt_best = cands[best].dt;

  // 抛物线细化:均方残差在极小值附近近似二次
  if (best > 0 && best + 1 < cands.size() && cands[best - 1].ok && cands[best + 1].ok) {
    const double y0 = msr(cands[best - 1]), y1 = msr_min, y2 = msr(cands[best + 1]);
    const double denom = y0 - 2 * y1 + y2;
    if (denom > 1e-15) {
      const double delta = 0.5 * (y0 - y2) / denom;   // 单位:步长
      if (std::abs(delta) <= 1.0) dt_best += delta * step;
    }
  }
  Cand final = cands[best];
  if (dt_best != cands[best].dt) {
    Cand refined = eval(dt_best);
    if (refined.ok && msr(refined) <= msr_min) final = refined;
    else dt_best = cands[best].dt;
  }
  InnerSolution sol = final.sol;
  const int n = final.n;

  // Δt 可观性:离最优点 ±0.1 s 处残差须明显上升(相对最优值翻倍以上)
  {
    double worst = msr_min;
    for (double d : {dt_best - 0.1, dt_best + 0.1}) {
      if (d < opt.dt_min || d > opt.dt_max) continue;
      Cand c = eval(d);
      if (c.ok) worst = std::max(worst, msr(c));
    }
    est.time_offset_observable = worst > 2.0 * msr_min + 1e-6;
  }

  est.lever_imu = sol.lever;
  est.T_world_enu = Eigen::Isometry3d::Identity();
  est.T_world_enu.linear() = Rz(sol.yaw);
  est.T_world_enu.translation() = sol.p;
  est.time_offset = dt_best;
  est.rms_residual = std::sqrt(sol.sum_sq / n);
  est.n_pairs = n;
  est.lever_observable = yaw_ok;
  est.lever_z_observable = tilt_ok;
  return est;
}

}  // namespace gnss_core
