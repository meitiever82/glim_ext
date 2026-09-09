// Shared glue for the RTK constraint shells (rtk_global on submaps, rtk_odometry on
// odometry frames). Everything algorithmic lives in gnss_core::AnchorPipeline; this
// header only holds config loading and RtkFix ingestion (finite / stamp-skew filtering),
// which both shells would otherwise duplicate verbatim.

#pragma once

#include <atomic>
#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <spdlog/spdlog.h>

#include <glim/util/config.hpp>
#include <glim/util/convert_to_string.hpp>
#include <gnss_core/anchor_pipeline.hpp>
#include <gnss_core/types.hpp>
#include <gnss_msgs/msg/rtk_fix.hpp>

namespace glim {
namespace rtk {

struct ShellConfig {
  std::string rtk_fix_topic = "/gnss_cgi610/rtk_fix";
  gnss_core::StampSource stamp_source = gnss_core::StampSource::GnssTime;
  std::string stamp_source_name = "gnss_time";
  double time_offset = 0.0;
  double max_stamp_skew = 5.0;
  Eigen::Vector3d lever_imu = Eigen::Vector3d::Zero();
  gnss_core::PipelineConfig pipeline;
};

/// Read the common key set (spec §7.5) from `section` of `config`. Throws on invalid values.
inline ShellConfig load_shell_config(const Config& config, const std::string& section) {
  ShellConfig c;
  c.rtk_fix_topic = config.param<std::string>(section, "rtk_fix_topic", c.rtk_fix_topic);

  c.stamp_source_name = config.param<std::string>(section, "stamp_source", "gnss_time");
  if (c.stamp_source_name == "gnss_time") {
    c.stamp_source = gnss_core::StampSource::GnssTime;
  } else if (c.stamp_source_name == "header") {
    c.stamp_source = gnss_core::StampSource::Header;
  } else {
    throw std::invalid_argument(section + ": stamp_source must be 'header' or 'gnss_time', got '" + c.stamp_source_name + "'");
  }
  c.time_offset = config.param<double>(section, "time_offset", 0.0);
  c.max_stamp_skew = config.param<double>(section, "max_stamp_skew", 5.0);
  c.lever_imu = config.param<Eigen::Vector3d>(section, "T_imu_gnss", Eigen::Vector3d::Zero());

  auto& n = c.pipeline.noise;
  n.min_quality = config.param<int>(section, "min_quality", 3);
  n.max_diff_age = config.param<double>(section, "max_diff_age", 15.0);
  n.min_sats = config.param<int>(section, "min_sats", 6);
  const auto scale = config.param<std::vector<double>>(section, "quality_sigma_scale", std::vector<double>{0.0, 50.0, 20.0, 5.0, 1.0});
  if (scale.size() != n.quality_sigma_scale.size()) {
    throw std::invalid_argument(section + ": quality_sigma_scale must have 5 entries [NONE, SINGLE, DGPS, FLOAT, FIXED]");
  }
  for (size_t i = 0; i < scale.size(); ++i) {
    n.quality_sigma_scale[i] = scale[i];
  }
  n.sigma_floor = config.param<Eigen::Vector3d>(section, "sigma_floor", Eigen::Vector3d(1.0, 1.0, 1.0));
  n.vertical_scale = config.param<double>(section, "vertical_scale", 3.0);
  n.robust_kernel = config.param<std::string>(section, "robust_kernel", "huber");
  n.robust_delta = config.param<double>(section, "robust_delta", 1.345);

  auto& p = c.pipeline;
  p.min_baseline = config.param<double>(section, "min_baseline", 50.0);
  p.fix_buffer_horizon = config.param<double>(section, "fix_buffer_horizon", 600.0);
  p.fix_max_gap = config.param<double>(section, "fix_max_gap", 2.5);
  p.enu_origin = config.param<std::vector<double>>(section, "enu_origin", std::vector<double>{});
  return c;
}

inline void log_shell_config(spdlog::logger& logger, const ShellConfig& c) {
  logger.info(
    "topic={} stamp_source={} time_offset={:.3f}s max_stamp_skew={}s min_quality={} max_diff_age={}s min_sats={} sigma_floor={} lever_imu={} min_baseline={}m fix_max_gap={}s fix_buffer_horizon={}s",
    c.rtk_fix_topic,
    c.stamp_source_name,
    c.time_offset,
    c.max_stamp_skew,
    c.pipeline.noise.min_quality,
    c.pipeline.noise.max_diff_age,
    c.pipeline.noise.min_sats,
    convert_to_string(c.pipeline.noise.sigma_floor),
    convert_to_string(c.lever_imu),
    c.pipeline.min_baseline,
    c.pipeline.fix_max_gap,
    c.pipeline.fix_buffer_horizon);
  if (c.lever_imu.isZero() && c.pipeline.noise.sigma_floor.minCoeff() < 0.5) {
    logger.warn(
      "T_imu_gnss is zero but sigma_floor is {} m: with an uncalibrated lever arm the robust kernel will down-weight good fixes in turns (spec §7.3 v2)",
      c.pipeline.noise.sigma_floor.minCoeff());
  }
}

template <typename Stamp>
inline double to_sec(const Stamp& stamp) {
  return stamp.sec + stamp.nanosec / 1e9;
}

/// Converts RtkFix messages to RtkFixSample and applies the shell-side input filters
/// (non-finite fields, header-vs-board time skew). Thread-safe for one producer.
class RtkFixIngest {
public:
  RtkFixIngest(const ShellConfig& cfg, std::shared_ptr<spdlog::logger> logger) : cfg_(cfg), logger_(std::move(logger)) {}

  /// Returns the sample if accepted; std::nullopt if dropped (counted in dropped()).
  std::optional<gnss_core::RtkFixSample> convert(const gnss_msgs::msg::RtkFix& msg) {
    gnss_core::RtkFixSample s;
    s.header_stamp = to_sec(msg.header.stamp);
    s.gnss_time = msg.gnss_time;
    s.stamp = gnss_core::effective_stamp(s.header_stamp, s.gnss_time, cfg_.stamp_source, cfg_.time_offset);
    s.quality = static_cast<gnss_core::Quality>(msg.quality);
    s.lat = msg.latitude;
    s.lon = msg.longitude;
    s.alt = msg.altitude;
    s.sigma_enu = Eigen::Vector3d(msg.sigma_enu[0], msg.sigma_enu[1], msg.sigma_enu[2]);
    s.diff_age = msg.diff_age;
    s.sats_used = msg.sats_used;
    s.heading = msg.heading;
    s.heading_valid = msg.heading_valid;

    ++received_;
    const bool finite = std::isfinite(s.stamp) && std::isfinite(s.lat) && std::isfinite(s.lon) && std::isfinite(s.alt) && std::isfinite(s.diff_age) &&
                        s.sigma_enu.allFinite();
    if (!finite) {
      ++dropped_;
      return std::nullopt;
    }
    // A single bogus board time (week rollover, driver bug) would become the buffer's
    // latest_stamp and prune everything; header vs board time must stay within max_stamp_skew.
    if (s.gnss_time > 0.0 && cfg_.max_stamp_skew > 0.0 && std::abs(s.gnss_time - s.header_stamp) > cfg_.max_stamp_skew + std::abs(cfg_.time_offset)) {
      if (!skew_warned_.exchange(true)) {
        logger_->warn(
          "RtkFix header.stamp={:.3f} vs gnss_time={:.3f} differ by {:.1f}s (> max_stamp_skew {}s); dropping such samples. Check clock domains / timestamp_source.",
          s.header_stamp,
          s.gnss_time,
          s.gnss_time - s.header_stamp,
          cfg_.max_stamp_skew);
      }
      ++dropped_;
      return std::nullopt;
    }
    return s;
  }

  size_t received() const { return received_; }
  size_t dropped() const { return dropped_; }

private:
  ShellConfig cfg_;
  std::shared_ptr<spdlog::logger> logger_;
  std::atomic<size_t> received_{0};
  std::atomic<size_t> dropped_{0};
  std::atomic_bool skew_warned_{false};
};

inline const char* outcome_name(gnss_core::AnchorOutcome o) {
  switch (o) {
    case gnss_core::AnchorOutcome::Constrained: return "constrained";
    case gnss_core::AnchorOutcome::Deferred: return "deferred";
    case gnss_core::AnchorOutcome::WaitingForFixes: return "waiting";
    case gnss_core::AnchorOutcome::TooOld: return "too_old";
    case gnss_core::AnchorOutcome::Gap: return "gap";
    case gnss_core::AnchorOutcome::Gated: return "gated";
    case gnss_core::AnchorOutcome::TimeSkew: return "time_skew";
  }
  return "?";
}

}  // namespace rtk
}  // namespace glim
