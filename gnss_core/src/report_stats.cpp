#include "gnss_core/report_stats.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <utility>

#include "gnss_core/geodetic.hpp"

namespace gnss_core {

namespace {
constexpr int kFixedQ = 1;   // RTKLIB Q:固定解(设计决定 5)

std::optional<double> ratio(int num, int den) {
  if (den <= 0) return std::nullopt;
  return static_cast<double>(num) / static_cast<double>(den);
}

SourceStats source_stats(const std::string& name, const std::vector<PosRecord>& recs, const ReportWindow& w) {
  SourceStats s;
  s.name = name;
  s.epochs = static_cast<int>(recs.size());
  for (const auto& r : recs) {
    switch (r.q) {
      case 1: ++s.counts.fixed; break;
      case 2: ++s.counts.floating; break;
      case 4: ++s.counts.dgps; break;
      case 5: ++s.counts.single; break;
      default: ++s.counts.other; break;
    }
  }
  s.fix_ratio = ratio(s.counts.fixed, s.epochs);

  const long long h0 = static_cast<long long>(std::floor(w.t0 / 3600.0));
  const long long h1 = static_cast<long long>(std::ceil(w.t1 / 3600.0));
  if (h1 <= h0) return s;
  std::vector<int> n(static_cast<size_t>(h1 - h0), 0), fixed(n.size(), 0);
  for (const auto& r : recs) {
    const long long idx = static_cast<long long>(std::floor(r.stamp / 3600.0)) - h0;
    if (idx < 0 || idx >= static_cast<long long>(n.size())) continue;
    ++n[static_cast<size_t>(idx)];
    if (r.q == kFixedQ) ++fixed[static_cast<size_t>(idx)];
  }
  for (size_t i = 0; i < n.size(); ++i) {
    s.hourly.push_back(HourBucket{static_cast<double>(h0 + static_cast<long long>(i)) * 3600.0, n[i], fixed[i],
                                  ratio(fixed[i], n[i])});
  }
  return s;
}

// 设计决定 6:对每个独立解历元取时间最近的设备历元,|Δt| <= tol 才配对
DivergenceStats divergence_stats(const std::string& device, const std::vector<PosRecord>& dev,
                                 const std::vector<PosRecord>& ref, double tol) {
  DivergenceStats d;
  d.device = device;
  double sum = 0.0;
  for (const auto& r : ref) {
    const auto it = std::lower_bound(dev.begin(), dev.end(), r.stamp,
                                     [](const PosRecord& a, double t) { return a.stamp < t; });
    const PosRecord* best = nullptr;
    double best_dt = std::numeric_limits<double>::infinity();
    if (it != dev.end()) {
      best = &*it;
      best_dt = std::abs(it->stamp - r.stamp);
    }
    if (it != dev.begin()) {
      const auto prev = std::prev(it);
      if (std::abs(prev->stamp - r.stamp) <= best_dt) {   // |Δt| 相同取更早的历元,与 rtk-monitor 的 min() 一致
        best = &*prev;
        best_dt = std::abs(prev->stamp - r.stamp);
      }
    }
    if (!best || best_dt > tol) continue;
    const double dist = geodesic_distance_m(r.lat, r.lon, best->lat, best->lon);
    ++d.n;
    sum += dist;
    d.max_m = d.max_m ? std::max(*d.max_m, dist) : dist;
  }
  if (d.n > 0) d.mean_m = sum / d.n;
  return d;
}

struct TrackSplit {
  std::vector<std::pair<size_t, size_t>> ranges;   // 每段 [首, 尾] 下标(含)
  size_t endpoints = 0;                            // 段端点总数(单点段算 1)
  double gap_s = 0.0;
};

TrackSplit split_with_gap(const std::vector<PosRecord>& recs, double gap_s) {
  TrackSplit out;
  out.gap_s = gap_s;
  size_t a = 0;
  while (a < recs.size()) {
    size_t b = a;
    while (b + 1 < recs.size() && recs[b + 1].stamp - recs[b].stamp <= gap_s) ++b;
    out.ranges.emplace_back(a, b);
    out.endpoints += a == b ? 1 : 2;
    a = b + 1;
  }
  return out;
}

// 设计决定 11 + final review:按 track_gap_s 断段;若段端点就占掉一半以上的点数上限(稀疏、断断续续的数据),
// 说明这个比例尺下小间断本来就画不出来——把断开间隔逐次翻倍再分段,直到端点不超过上限的一半。
// 间隔翻倍到超过全源时长时只剩一段,循环必然结束;长时间停车(远大于最终间隔)仍然断开。
TrackSplit split_track(const std::vector<PosRecord>& recs, double track_gap_s, size_t max_points) {
  TrackSplit split = split_with_gap(recs, track_gap_s);
  double gap = std::max(track_gap_s, 1.0);
  while (split.endpoints > max_points / 2 && split.ranges.size() > 1) {
    gap *= 2.0;
    split = split_with_gap(recs, gap);
  }
  return split;
}
}  // namespace

std::vector<std::string> ordered_source_names(const std::map<std::string, std::vector<PosRecord>>& sources) {
  static const char* const kPreferred[] = {"can", "gpchc", "rtkrcv", "ref"};
  std::vector<std::string> out;
  for (const char* n : kPreferred) {
    if (sources.count(n)) out.emplace_back(n);
  }
  for (const auto& [name, recs] : sources) {
    if (std::find(out.begin(), out.end(), name) == out.end()) out.push_back(name);
  }
  return out;
}

ReportStats compute_report(const ReportInputs& in, const ReportParams& p) {
  ReportStats s;
  s.params = p;
  s.warnings = in.warnings;
  s.events = in.events;
  const auto names = ordered_source_names(in.sources);

  for (const auto& name : names) s.sources.push_back(source_stats(name, in.sources.at(name), p.window));

  const auto ref = in.sources.find("rtkrcv");
  if (ref != in.sources.end()) {
    for (const char* device : {"can", "gpchc"}) {
      const auto dev = in.sources.find(device);
      if (dev != in.sources.end()) s.divergence.push_back(divergence_stats(device, dev->second, ref->second, p.pair_tol_s));
    }
  }

  // 设计决定 7:每个源的固定解,只比最近的控制点
  if (!p.control_points.empty()) {
    for (const auto& name : names) {
      for (const auto& r : in.sources.at(name)) {
        if (r.q != kFixedQ) continue;
        const ControlPoint* nearest = nullptr;
        double nearest_m = std::numeric_limits<double>::infinity();
        for (const auto& cp : p.control_points) {
          const double d = geodesic_distance_m(r.lat, r.lon, cp.lat, cp.lon);
          if (d < nearest_m) {
            nearest_m = d;
            nearest = &cp;
          }
        }
        if (nearest && nearest_m <= p.abs_ref_radius_m) {
          s.abs_ref.samples.push_back(AbsRefSample{r.stamp, name, nearest->name, nearest_m});
        }
      }
    }
    std::stable_sort(s.abs_ref.samples.begin(), s.abs_ref.samples.end(),
                     [](const AbsRefSample& a, const AbsRefSample& b) { return a.t < b.t; });
    for (const auto& x : s.abs_ref.samples) s.abs_ref.max_m = s.abs_ref.max_m ? std::max(*s.abs_ref.max_m, x.dev_m) : x.dev_m;
    s.abs_ref.exceeded = s.abs_ref.max_m && *s.abs_ref.max_m > p.abs_ref_max_m;
  }

  // 设计决定 8(final review 修订):基准优先取 t0 之前最后一条记录——gnss_diag_node 只在坐标变化时写行,
  // 以窗口内第一行为基准会把"窗口开头那次变动"本身当成基准而看不见;序列从 t0 处的 0 开始。
  // 没有更早记录时退回 rtk-monitor 的口径:窗口内第一条。
  if (in.base_before_window || !in.base_history.empty()) {
    const BaseSample ref = in.base_before_window ? *in.base_before_window : in.base_history.front();
    s.base.reference_t = ref.t;
    s.base.reference_before_window = in.base_before_window.has_value();
    const auto add = [&](double t, const Ecef& b) {
      const Ecef& p0 = ref.p;
      const double off = std::sqrt((b.x - p0.x) * (b.x - p0.x) + (b.y - p0.y) * (b.y - p0.y) + (b.z - p0.z) * (b.z - p0.z));
      s.base.series.push_back(BaseOffsetSample{t, off});
      s.base.max_m = s.base.max_m ? std::max(*s.base.max_m, off) : off;
    };
    if (in.base_before_window) add(std::max(ref.t, p.window.t0), ref.p);
    for (const auto& b : in.base_history) add(b.t, b.p);
    s.base.exceeded = *s.base.max_m > p.base_shift_m;
  }

  // 事件按规则码汇总
  std::map<std::string, EventCodeSummary> by_code;
  for (const auto& e : in.events) {
    auto& x = by_code[e.code];
    x.code = e.code;
    ++x.count;
    if (e.t_close) {
      x.closed_duration_s += *e.t_close - e.t_open;
    } else {
      ++x.unclosed;
    }
  }
  for (auto& [code, x] : by_code) s.event_summary.push_back(x);
  std::stable_sort(s.event_summary.begin(), s.event_summary.end(),
                   [](const EventCodeSummary& a, const EventCodeSummary& b) { return a.count > b.count; });

  // 设计决定 11:共用一个局部 ENU 原点
  std::unique_ptr<LlaToEnu> origin;
  double origin_height = 0.0;   // 事件只有经纬度,按原点高度投影,远离原点时才不会与轨迹错开
  for (const auto& name : names) {
    const auto& recs = in.sources.at(name);
    if (recs.empty()) continue;   // 原点取第一个非空源;空源(front() 未定义行为)跳过
    origin = std::make_unique<LlaToEnu>(recs.front().lat, recs.front().lon, recs.front().height);
    origin_height = recs.front().height;
    s.track_origin = name;
    break;
  }
  if (!origin) {
    for (const auto& e : in.events) {
      if (!e.pos) continue;
      origin = std::make_unique<LlaToEnu>(e.pos->lat, e.pos->lon, 0.0);
      s.track_origin = "事件";
      break;
    }
  }
  if (!origin) return s;

  const size_t max_points = std::max<size_t>(p.max_track_points, 2);
  for (const auto& name : names) {
    const auto& recs = in.sources.at(name);
    if (recs.empty()) continue;   // 空源不产生轨迹条目
    Track track;
    track.source = name;
    const auto split = split_track(recs, p.track_gap_s, max_points);
    track.gap_s = split.gap_s;
    // 段端点全保留;其余点按全源统一的步长抽稀,步长按"上限减去端点数"算,总点数不超过 max_points
    const size_t ends = split.endpoints;
    const size_t interior = recs.size() - ends;
    const size_t budget = max_points - ends;
    const size_t stride = budget == 0 ? interior + 1 : std::max<size_t>(1, (interior + budget - 1) / budget);
    size_t interior_seen = 0;
    for (const auto& [a, b] : split.ranges) {
      std::vector<TrackPoint> seg;
      for (size_t i = a; i <= b; ++i) {
        if (i != a && i != b && (budget == 0 || (interior_seen++ % stride) != 0)) continue;
        const Eigen::Vector3d enu = origin->forward(recs[i].lat, recs[i].lon, recs[i].height);
        seg.push_back(TrackPoint{recs[i].stamp, enu.x(), enu.y(), recs[i].q});
      }
      track.segments.push_back(std::move(seg));
    }
    s.tracks.push_back(std::move(track));
  }
  for (size_t i = 0; i < in.events.size(); ++i) {
    const auto& e = in.events[i];
    if (!e.pos) continue;
    const Eigen::Vector3d enu = origin->forward(e.pos->lat, e.pos->lon, origin_height);
    s.event_markers.push_back(EventMarker{static_cast<int>(i + 1), enu.x(), enu.y()});
  }
  return s;
}

}  // namespace gnss_core
