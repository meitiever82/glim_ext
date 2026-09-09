#include "gnss_core/trajectory_compare.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include "gnss_core/geodetic.hpp"

namespace gnss_core {

namespace {

struct Sample {
  double err_h, err_v, sigma_h;
};

double median_of(std::vector<double> v) {
  if (v.empty()) return 0.0;
  const size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  const double hi = v[mid];
  if (v.size() % 2 == 1) return hi;
  const double lo = *std::max_element(v.begin(), v.begin() + mid);
  return 0.5 * (lo + hi);
}

}  // namespace

std::map<Quality, QualityStats> compare_by_quality(
    const std::vector<PosRecord>& ref, const std::vector<PosRecord>& test, double tol_s,
    int ref_min_q) {
  std::map<Quality, QualityStats> out;
  if (ref.empty() || test.empty()) return out;

  // 过滤后的 ref 索引(ref_min_q=0 不过滤),按 stamp 排序,便于二分
  std::vector<size_t> order;
  order.reserve(ref.size());
  for (size_t i = 0; i < ref.size(); ++i) {
    if (ref_min_q > 0 && (ref[i].q == 0 || ref[i].q > ref_min_q)) continue;
    order.push_back(i);
  }
  if (order.empty()) return out;
  std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return ref[a].stamp < ref[b].stamp; });
  std::vector<double> ref_stamps(order.size());
  for (size_t i = 0; i < order.size(); ++i) ref_stamps[i] = ref[order[i]].stamp;

  std::unique_ptr<LlaToEnu> conv;   // 原点:首个配对成功的 ref 记录
  std::map<Quality, std::vector<Sample>> samples;

  for (const auto& t : test) {
    auto it = std::lower_bound(ref_stamps.begin(), ref_stamps.end(), t.stamp);
    size_t best = ref_stamps.size();
    double best_dt = tol_s;
    for (int k = -1; k <= 0; ++k) {
      auto jt = it + k;
      if (jt < ref_stamps.begin() || jt >= ref_stamps.end()) continue;
      const double dt = std::fabs(*jt - t.stamp);
      if (dt <= best_dt) { best_dt = dt; best = static_cast<size_t>(jt - ref_stamps.begin()); }
    }
    if (best == ref_stamps.size()) continue;
    const PosRecord& r = ref[order[best]];

    if (!conv) conv = std::make_unique<LlaToEnu>(r.lat, r.lon, r.height);
    const Eigen::Vector3d er = conv->forward(r.lat, r.lon, r.height);
    const Eigen::Vector3d et = conv->forward(t.lat, t.lon, t.height);
    const Eigen::Vector3d d = et - er;
    Sample s;
    s.err_h = std::hypot(d.x(), d.y());
    s.err_v = std::fabs(d.z());
    s.sigma_h = std::hypot(t.sdne(0), t.sdne(1));
    samples[q_to_quality(t.q)].push_back(s);
  }

  for (const auto& [q, v] : samples) {
    QualityStats st;
    st.n = static_cast<int>(v.size());
    double sum_h2 = 0, sum_v2 = 0;
    for (const auto& s : v) { sum_h2 += s.err_h * s.err_h; sum_v2 += s.err_v * s.err_v; }
    st.rmse_h = std::sqrt(sum_h2 / v.size());
    st.rmse_v = std::sqrt(sum_v2 / v.size());

    std::vector<double> ratios;
    double sum_ratio = 0, sum_e2 = 0, sum_s2 = 0;
    for (const auto& s : v) {
      if (s.sigma_h <= 0.0) continue;   // 板卡未报 σ 的历元不参与比值
      const double r = s.err_h / s.sigma_h;
      ratios.push_back(r);
      sum_ratio += r;
      sum_e2 += s.err_h * s.err_h;
      sum_s2 += s.sigma_h * s.sigma_h;
    }
    if (!ratios.empty()) {
      st.sigma_ratio_h_mean = sum_ratio / ratios.size();
      st.sigma_ratio_h_median = median_of(ratios);
      st.sigma_ratio_h_rms = std::sqrt(sum_e2 / ratios.size()) / std::sqrt(sum_s2 / ratios.size());
    }
    out[q] = st;
  }
  return out;
}

}  // namespace gnss_core
