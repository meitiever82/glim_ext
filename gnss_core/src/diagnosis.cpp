#include "gnss_core/diagnosis.hpp"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <utility>

#include "gnss_core/geodetic.hpp"

namespace gnss_core {

namespace {
std::string format(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  return buf;
}

void require(bool ok, const char* field, const char* why) {
  if (!ok) throw std::invalid_argument(std::string(field) + ": " + why);
}
}  // namespace

const char* level_name(Level level) {
  switch (level) {
    case Level::Ok: return "ok";
    case Level::Info: return "info";
    case Level::Warning: return "warning";
    case Level::Serious: return "serious";
    case Level::Critical: return "critical";
  }
  return "ok";
}

bool level_opens_event(Level level) {
  return level == Level::Warning || level == Level::Serious || level == Level::Critical;
}

const char* quality_name(Quality q) {
  switch (q) {
    case Quality::NONE: return "NONE";
    case Quality::SINGLE: return "SINGLE";
    case Quality::DGPS: return "DGPS";
    case Quality::FLOAT: return "FLOAT";
    case Quality::FIXED: return "FIXED";
  }
  return "NONE";
}

void validate_diagnosis_config(const DiagnosisConfig& c) {
  require(c.corr_gap_s > 0.0, "corr_gap_s", "必须 > 0");
  require(c.age_max_s > 0.0, "age_max_s", "必须 > 0");
  require(c.base_shift_m > 0.0, "base_shift_m", "必须 > 0");
  require(c.min_sats >= 0, "min_sats", "必须 >= 0");
  require(c.resid_max_m > 0.0, "resid_max_m", "必须 > 0");
  require(c.low_el_deg >= 0.0 && c.low_el_deg <= 90.0, "low_el_deg", "必须在 [0, 90]");
  require(c.low_snr_dbhz >= 0.0, "low_snr_dbhz", "必须 >= 0");
  require(c.min_ratio > 0.0, "min_ratio", "必须 > 0");
  require(c.slip_max_per_30s >= 0, "slip_max_per_30s", "必须 >= 0");
  require(c.divergence_sigma > 0.0, "divergence_sigma", "必须 > 0");
  require(c.divergence_hold_s >= 0.0, "divergence_hold_s", "必须 >= 0");
  require(c.close_hysteresis_s >= 0.0, "close_hysteresis_s", "必须 >= 0");
  require(c.sol_stale_s > 0.0, "sol_stale_s", "必须 > 0");
  require(c.abs_ref_max_m > 0.0, "abs_ref_max_m", "必须 > 0");
  require(c.abs_ref_radius_m > c.abs_ref_max_m, "abs_ref_radius_m", "必须大于 abs_ref_max_m,否则该规则永远不会触发");
  require(c.divergence_window_s > 0.0, "divergence_window_s", "必须 > 0");
  require(c.divergence_min_samples >= 2, "divergence_min_samples", "必须 >= 2");
  require(c.divergence_sigma_floor_m > 0.0, "divergence_sigma_floor_m", "必须 > 0");
  require(c.divergence_pair_max_dt_s > 0.0, "divergence_pair_max_dt_s", "必须 > 0");
  require(c.divergence_epoch_max_dt_s > 0.0, "divergence_epoch_max_dt_s", "必须 > 0");
  require(c.base_warmup_s >= 0.0, "base_warmup_s", "必须 >= 0");
}

DiagnosisResult evaluate_rules(const DiagnosisInput& in, const DiagnosisConfig& cfg) {
  DiagnosisResult r;
  const auto push = [&r](Level level, const char* code, std::string message) {
    r.verdicts.push_back(Verdict{level, code, std::move(message)});
  };

  // 0 no_data:什么都没有时其他规则没有意义,直接返回
  if (!in.sol && !in.corr_last_t) {
    push(Level::Warning, "no_data", "无数据——检查采集链路与设备连接");
    return r;
  }

  // 1 corr_outage
  std::optional<double> gap;
  if (in.corr_last_t) gap = in.now - *in.corr_last_t;
  const bool gap_hit = gap && *gap > cfg.corr_gap_s;
  const bool age_hit = in.corr_age && *in.corr_age > cfg.age_max_s;
  if (gap_hit || age_hit) {
    const int n = static_cast<int>(gap_hit ? *gap : *in.corr_age);
    push(Level::Serious, "corr_outage", format("差分中断 %ds——5G 链路或平台转发问题", n));
  }

  // 2 base_shift
  if (in.base_offset_m && *in.base_offset_m > cfg.base_shift_m) {
    push(Level::Critical, "base_shift",
         format("⚠ 基站坐标变动 %.2fm——所有定位结果将整体平移", *in.base_offset_m));
  }

  // 3 abs_ref_shift:只在固定解时判(0.2 m 门限只对 cm 级固定解有意义),只判最近的控制点
  if (in.sol && in.sol->quality == Quality::FIXED && !in.control_points.empty()) {
    const ControlPoint* nearest = nullptr;
    double best = std::numeric_limits<double>::infinity();
    for (const auto& cp : in.control_points) {
      const double d = geodesic_distance_m(in.sol->lat, in.sol->lon, cp.lat, cp.lon);
      if (d < best) {
        best = d;
        nearest = &cp;
      }
    }
    if (nearest && best <= cfg.abs_ref_radius_m && best > cfg.abs_ref_max_m) {
      push(Level::Critical, "abs_ref_shift",
           format("⚠ 绝对基准偏差 %.2fm@控制点 %s——疑似整体基准平移", best, nearest->name.c_str()));
    }
  }

  // 4 low_sats
  if (in.sol && in.sol->ns < cfg.min_sats) {
    push(Level::Serious, "low_sats", format("卫星数不足（%d 颗）——疑似遮挡", in.sol->ns));
  }

  // 5 multipath:残差取绝对值(RTKLIB resp 带符号)
  std::vector<const SatStat*> bad;
  for (const auto& s : in.sats) {
    if (std::abs(s.resp) > cfg.resid_max_m && (s.el < cfg.low_el_deg || s.snr < cfg.low_snr_dbhz)) {
      bad.push_back(&s);
    }
  }
  if (bad.size() >= 2) {
    std::string names;
    for (size_t i = 0; i < bad.size() && i < 4; ++i) {
      if (i > 0) names += "、";
      names += bad[i]->sat;
    }
    push(Level::Warning, "multipath", names + " 残差异常——疑似多路径");
  }

  // 6 ambiguity
  if (in.sol && in.sol->quality == Quality::FLOAT && in.sol->ratio && *in.sol->ratio < cfg.min_ratio) {
    push(Level::Warning, "ambiguity",
         format("模糊度无法固定（ratio=%.1f）——遮挡过渡区常见", *in.sol->ratio));
  }

  // 7 cycle_slip
  if (in.slip_count_30s > cfg.slip_max_per_30s) {
    push(Level::Warning, "cycle_slip", "载波频繁失锁——动态遮挡或天线/馈线问题");
  }

  // 8 device_divergence:阈值与起始时刻由 DivergenceMonitor 给出,这里不再重算
  if (in.divergence_m && in.divergence_since && in.sol &&
      *in.divergence_m > in.divergence_threshold_m &&
      in.now - *in.divergence_since >= cfg.divergence_hold_s) {
    push(Level::Serious, "device_divergence",
         format("610 输出与独立解算偏差 %.2fm——疑似 610 融合问题", *in.divergence_m));
  }

  // 9 状态
  if (!in.sol) {
    if (in.solver_enabled) {
      push(Level::Warning, "no_solution", "独立解算无输出——rtkrcv 未运行或未收敛");
    } else {
      push(Level::Info, "no_solution", "独立解算未启用");
    }
  } else if (in.sol->quality != Quality::FIXED) {
    push(Level::Info, "not_fixed", format("非固定解（%s）", quality_name(in.sol->quality)));
  } else {
    push(Level::Ok, "rtk_fixed", "RTK 固定");
  }
  return r;
}

}  // namespace gnss_core
