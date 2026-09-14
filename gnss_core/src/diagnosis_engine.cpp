#include "gnss_core/diagnosis_engine.hpp"

#include <cmath>
#include <map>
#include <utility>

#include "gnss_core/geodetic.hpp"

namespace gnss_core {

namespace {
const DiagnosisConfig& validated(const DiagnosisConfig& cfg) {
  validate_diagnosis_config(cfg);
  return cfg;
}
}  // namespace

DiagnosisEngine::DiagnosisEngine(DiagnosisConfig cfg, std::vector<ControlPoint> control_points,
                                 bool solver_enabled, std::optional<Ecef> persisted_baseline,
                                 std::optional<Ecef> last_history)
    : cfg_(validated(cfg)),
      control_points_(std::move(control_points)),
      solver_enabled_(solver_enabled),
      base_(cfg_.base_warmup_s, persisted_baseline, last_history),
      divergence_(cfg_),
      events_(cfg_.close_hysteresis_s) {}

bool DiagnosisEngine::fresh(const std::optional<double>& t, double now) const {
  return t && now - *t < cfg_.sol_stale_s;
}

std::vector<BaseUpdate> DiagnosisEngine::on_corrections(double t, const uint8_t* data, size_t len) {
  std::vector<BaseUpdate> out;
  if (len == 0) return out;
  corr_last_t_ = t;   // 任何差分字节都算链路存活,不只是 1005/1006
  for (const auto& msg : framer_.feed(data, len)) {
    BaseStationCoords coords;
    if (!parse_base_station(msg, coords)) continue;
    BaseUpdate u;
    u.t = t;
    u.coords = coords;
    u.feed = base_.feed(t, Ecef{coords.x, coords.y, coords.z});
    if (u.feed.offset_m) base_offset_m_ = u.feed.offset_m;   // 位移保持到下一条 1005/1006
    out.push_back(u);
  }
  return out;
}

void DiagnosisEngine::on_solution(double t, const SolutionSample& s) {
  sol_ = s;
  sol_t_ = t;
}

void DiagnosisEngine::on_device_solution(double t, const SolutionSample& s) {
  dev_ = s;
  dev_t_ = t;
}

void DiagnosisEngine::on_stat_line(double t, const std::string& line) {
  SatStat s;
  if (!parse_sat_line(line, s)) return;
  stat_epoch_.feed(line);
  slips_.feed(t, s.sat, s.slipc);   // 周跳窗口按到达时刻计,不用 $SAT 的 tow
  stat_t_ = t;
}

TickResult DiagnosisEngine::tick(double now) {
  const std::optional<SolutionSample> sol = fresh(sol_t_, now) ? sol_ : std::nullopt;
  const std::optional<SolutionSample> dev = fresh(dev_t_, now) ? dev_ : std::nullopt;

  DiagnosisInput in;
  in.now = now;
  in.corr_last_t = corr_last_t_;
  if (sol) {
    in.corr_age = sol->age;
  } else if (dev) {
    in.corr_age = dev->age;
  }
  in.base_offset_m = base_offset_m_;
  in.sol = sol;
  if (fresh(stat_t_, now)) in.sats = stat_epoch_.epoch();
  in.slip_count_30s = slips_.count(now);

  std::optional<double> d;
  if (sol && dev && std::abs(*sol_t_ - *dev_t_) < cfg_.divergence_pair_max_dt_s) {
    d = geodesic_distance_m(sol->lat, sol->lon, dev->lat, dev->lon);
  }
  TickResult out;
  out.divergence = divergence_.update(now, d, sol ? std::hypot(sol->sdn, sol->sde) : 0.0);
  in.divergence_m = out.divergence.divergence_m;
  in.divergence_since = out.divergence.since;
  in.divergence_threshold_m = out.divergence.threshold_m;
  in.solver_enabled = solver_enabled_;
  in.control_points = control_points_;

  out.result = evaluate_rules(in, cfg_);

  std::optional<LatLon> pos;
  if (sol) {
    pos = LatLon{sol->lat, sol->lon};
  } else if (dev) {
    pos = LatLon{dev->lat, dev->lon};
  }
  std::map<std::string, double> metrics;
  metrics["divergence_m"] = d.value_or(0.0);
  if (sol) metrics["sats_min"] = static_cast<double>(sol->ns);   // 无解时不报,免得峰值被拉成 0
  metrics["corr_gap_s"] = corr_last_t_ ? now - *corr_last_t_ : 0.0;
  out.transitions = events_.update(now, out.result.verdicts, pos, metrics);
  return out;
}

std::vector<EventTransition> DiagnosisEngine::shutdown(double now) { return events_.close_all(now); }

}  // namespace gnss_core
