#include <glim_ext/rtk_odometry_module.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <glim/util/config.hpp>
#include <glim/util/convert_to_string.hpp>
#include <glim_ext/fusion/fusion_status_bus.hpp>
#include <glim_ext/util/config_ext.hpp>

#include <gnss_core/antenna_prior_factor.hpp>
#include <gnss_core/heading_prior_factor.hpp>
#include <gnss_core/relative_translation_factor.hpp>

namespace glim {

using gtsam::symbol_shorthand::X;

RtkOdometry::RtkOdometry() : logger_(create_module_logger("rtk_odometry")) {
  logger_->info("initializing rtk_odometry (quality-aware RTK constraints on odometry frames)");

  const std::string config_path = GlobalConfigExt::get_config_path("config_rtk_odometry");
  logger_->info("rtk_odometry_config_path={}", config_path);
  const Config config(config_path);
  cfg_ = rtk::load_shell_config(config, "rtk_odometry");
  smoother_lag_margin_ = config.param<double>("rtk_odometry", "smoother_lag_margin", 1.0);
  rtk::log_shell_config(*logger_, cfg_);

  // The lag window is owned by the odometry module's config; read it from there.
  const Config odom_config(GlobalConfig::get_config_path("config_odometry"));
  smoother_lag_ = odom_config.param<double>("odometry_estimation", "smoother_lag", 5.0);
  logger_->info("smoother_lag={}s lag_margin={}s -> RTK factors only for frames newer than {}s behind the latest", smoother_lag_, smoother_lag_margin_, smoother_lag_ - smoother_lag_margin_);
  if (smoother_lag_ - smoother_lag_margin_ <= 0.0) {
    throw std::invalid_argument("rtk_odometry: smoother_lag_margin must be smaller than the odometry smoother_lag");
  }

  ingest_ = std::make_unique<rtk::RtkFixIngest>(cfg_, logger_);
  pipeline_ = std::make_unique<gnss_core::AnchorPipeline>(cfg_.pipeline);  // throws on invalid config
  if (cfg_.relative.enabled) {
    relative_pipeline_ = std::make_unique<gnss_core::RelativePipeline>(cfg_.pipeline, cfg_.relative);
    logger_->info(
      "relative RTK constraints enabled (sigma_floor={}m yaw_baseline={}m max_pair_gap={}s)",
      cfg_.relative.sigma_floor,
      cfg_.relative.yaw_baseline,
      cfg_.relative.max_pair_gap);
  }
  if (const auto o = pipeline_->enu_origin()) {
    logger_->info("ENU origin from config: lat={:.8f} lon={:.8f} alt={:.3f}", (*o)[0], (*o)[1], (*o)[2]);
  }

  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;
  OdometryEstimationCallbacks::on_insert_imu.add(std::bind(&RtkOdometry::on_insert_imu, this, _1, _2, _3));
  OdometryEstimationCallbacks::on_update_new_frame.add(std::bind(&RtkOdometry::on_update_new_frame, this, _1));
  OdometryEstimationCallbacks::on_smoother_update.add(std::bind(&RtkOdometry::on_smoother_update, this, _1, _2, _3, _4));

  kill_switch_ = false;
  thread_ = std::thread([this] { backend_task(); });
}

RtkOdometry::~RtkOdometry() {
  stop_backend();
}

void RtkOdometry::at_exit(const std::string& /*dump_path*/) {
  stop_backend();
}

void RtkOdometry::stop_backend() {
  if (!thread_.joinable()) {
    return;
  }
  kill_switch_ = true;
  thread_.join();
  const auto& s = pipeline_->stats();
  logger_->info(
    "rtk_odometry stats: fix_received={} fix_dropped={} fix_pushed={} anchors={} factors_added_total={} heading_factors_added={} abs_skipped_by_interval={} relative_factors_added={} dropped_by_lag={} deferred_released={} too_old={} gap={} gated={} pending_anchors={} deferred_pending={}",
    ingest_->received(),
    ingest_->dropped(),
    s.fixes,
    s.anchors,
    n_factors_added_.load(),
    n_heading_added_.load(),
    n_absolute_skipped_.load(),
    n_relative_added_.load(),
    n_dropped_lag_.load(),
    s.deferred_released,
    s.too_old,
    s.gap,
    s.gated,
    pipeline_->pending_anchors(),
    pipeline_->deferred());
  if (relative_pipeline_) {
    const auto& rs = relative_pipeline_->stats();
    logger_->info(
      "rtk_odometry relative: fixes={} anchors={} pairs={} yaw_bootstrapped_at={:.3f} too_old={} gap={} gated={} pair_gap={} pending_anchors={}",
      rs.fixes,
      rs.anchors,
      rs.pairs,
      rs.yaw_bootstrapped_stamp,
      rs.too_old,
      rs.gap,
      rs.gated,
      rs.pair_gap,
      relative_pipeline_->pending_anchors());
  }
}

std::vector<GenericTopicSubscription::Ptr> RtkOdometry::create_subscriptions() {
  const auto sub = std::make_shared<TopicSubscription<gnss_msgs::msg::RtkFix>>(cfg_.rtk_fix_topic, [this](const gnss_msgs::msg::RtkFix::ConstSharedPtr msg) { rtk_fix_callback(msg); });
  return {sub};
}

void RtkOdometry::rtk_fix_callback(const gnss_msgs::msg::RtkFix::ConstSharedPtr& msg) {
  if (const auto s = ingest_->convert(*msg)) {
    // Data-time stream health, used for FusionStatus.rtk_outage (raw stream, not gated).
    const double prev = latest_fix_stamp_.load();
    if (s->stamp > prev) {
      latest_fix_stamp_ = s->stamp;
    }
    input_fix_queue_.push_back(*s);
  }
}

void RtkOdometry::on_insert_imu(double stamp, const Eigen::Vector3d& /*linear_acc*/, const Eigen::Vector3d& /*angular_vel*/) {
  has_imu_ = true;
  const double prev = latest_imu_stamp_.load();
  if (stamp > prev) {
    latest_imu_stamp_ = stamp;
  }
}

glim::FusionStatus RtkOdometry::snapshot_status() const {
  std::lock_guard<std::mutex> lock(status_mutex_);
  return status_snapshot_;
}

void RtkOdometry::update_status_snapshot() {
  glim::FusionStatus s;
  const auto q = pipeline_->last_quality();
  s.rtk_quality = static_cast<glim::RtkQuality>(static_cast<int>(q));
  const Eigen::Vector3d& sigma = pipeline_->last_sigma();
  for (int i = 0; i < 3; ++i) {
    s.sigma_enu[i] = sigma(i);
  }

  const double now = latest_frame_stamp_.load();
  s.absolute_anchored = pipeline_->initialized();
  s.seconds_since_anchor = (now > 0.0 && last_absolute_stamp_ > -1e8) ? std::max(0.0, now - last_absolute_stamp_) : 0.0;

  const double last_fix = latest_fix_stamp_.load();
  s.rtk_outage = now > 0.0 && (last_fix <= 0.0 || now - last_fix > cfg_.rtk_outage_timeout);
  s.degraded_rtk = s.rtk_outage || static_cast<int>(q) < static_cast<int>(gnss_core::Quality::FLOAT);

  const double last_imu = latest_imu_stamp_.load();
  s.degraded_imu = has_imu_.load() && now > 0.0 && (now - last_imu > cfg_.imu_gap_timeout);

  // Relative constraints emit at the anchor rate; if none arrived for a while the shape
  // support is no longer contributing (RTK stream stalled or pairs over the gap limit).
  s.shape_confidence = (relative_pipeline_ && relative_pipeline_->yaw_initialized() && now > 0.0 && last_relative_stamp_ > -1e8 &&
                        now - last_relative_stamp_ <= 2.0)
                         ? 1.0f
                         : 0.0f;

  s.map_aligned = false;   // stage 2: S2M not implemented

  std::lock_guard<std::mutex> lock(status_mutex_);
  status_snapshot_ = s;
}

void RtkOdometry::on_update_new_frame(const EstimationFrame::ConstPtr& frame) {
  // Odometry thread. Corrected (post-optimization) pose of the newest frame.
  gnss_core::Anchor a;
  a.id = frame->id;
  a.stamp = frame->stamp;
  a.t_world = frame->T_world_imu.translation();
  latest_frame_stamp_ = frame->stamp;
  input_anchor_queue_.push_back(a);

  // FusionResult (2.1 §8): base_frame_id is blank in config_ros, so base == imu.
  glim::FusionResult r;
  r.stamp = frame->stamp;
  r.T_world_base = frame->T_world_imu;
  r.velocity = frame->v_world_imu;
  r.status = snapshot_status();
  if (frame->cov_computed) {
    r.covariance = frame->pose_cov;
    // Stage-2 LiDAR degradation proxy: horizontal position sigma of the latest pose.
    const Eigen::Matrix3d cov_t = frame->pose_cov.block<3, 3>(0, 0);
    const double hsigma = std::sqrt(std::max(0.0, std::max(cov_t(0, 0), cov_t(1, 1))));
    r.status.degraded_lo = hsigma > cfg_.degraded_lo_sigma;
  }
  glim::FusionStatusBus::instance().publish(r);
}

void RtkOdometry::on_smoother_update(
  gtsam_points::IncrementalFixedLagSmootherExtWithFallback& /*smoother*/,
  gtsam::NonlinearFactorGraph& new_factors,
  gtsam::Values& /*new_values*/,
  std::map<std::uint64_t, double>& new_stamps) {
  const auto pending = output_.get_all_and_clear();
  if (pending.empty()) {
    return;
  }

  // Newest timestamp the smoother will hold after this update: the frame being inserted now.
  double latest = latest_frame_stamp_;
  for (const auto& kv : new_stamps) {
    latest = std::max(latest, kv.second);
  }
  const double oldest_allowed = latest - (smoother_lag_ - smoother_lag_margin_);

  size_t added = 0;
  for (const auto& p : pending) {
    if (p.stamp < oldest_allowed) {
      ++n_dropped_lag_;
      logger_->debug("frame {} (t={:.3f}) left the lag window ({:.3f}); RTK factor dropped", p.frame_id, p.stamp, oldest_allowed);
      continue;
    }
    new_factors.add(p.factor);
    ++added;
  }
  n_factors_added_ += added;
  if (added) {
    logger_->info("insert {} RTK prior factors into the odometry smoother (graph now {} factors)", added, new_factors.size());
  } else if (!pending.empty()) {
    logger_->info("drain: {} pending but all dropped by lag guard (oldest_allowed={:.1f})", pending.size(), oldest_allowed);
  }
}

void RtkOdometry::backend_task() {
  logger_->info("starting rtk_odometry backend thread");
  while (!kill_switch_) {
    const auto fixes = input_fix_queue_.get_all_and_clear();
    for (const auto& f : fixes) {
      pipeline_->push_fix(f);
      if (relative_pipeline_) {
        relative_pipeline_->push_fix(f);
      }
    }
    const auto anchors = input_anchor_queue_.get_all_and_clear();
    for (const auto& a : anchors) {
      pipeline_->push_anchor(a);
      if (relative_pipeline_) {
        relative_pipeline_->push_anchor(a);
      }
    }

    // Pre-filter against the lag window here (backend thread) so that the bulk of
    // deferred constraints released at bootstrap never becomes factor objects; the
    // guard in on_smoother_update remains as the final check.
    const double oldest_allowed = latest_frame_stamp_.load() - (smoother_lag_ - smoother_lag_margin_);

    const auto constraints = pipeline_->process();
    size_t kept_abs = 0;
    size_t kept_heading = 0;
    for (const auto& c : constraints) {
      if (c.stamp < oldest_allowed) {
        ++n_dropped_lag_;
        continue;
      }
      // Absolute factors are decimated: relative factors carry the motion shape, the
      // anchor only fixes the global gauge every absolute_min_interval seconds.
      if (cfg_.absolute_min_interval > 0.0 && c.stamp - last_absolute_stamp_ < cfg_.absolute_min_interval) {
        ++n_absolute_skipped_;
        continue;
      }
      last_absolute_stamp_ = c.stamp;

      // body_point = lever_imu: X(frame) is T_world_imu, antenna sits at lever_imu in the IMU frame (spec §7.3).
      PendingFactor p;
      p.frame_id = c.id;
      p.stamp = c.stamp;
      p.factor = gnss_core::AntennaPriorFactor::create(X(c.id), c.p_world, cfg_.lever_imu, c.model);
      output_.push_back(p);
      ++kept_abs;

      if (c.has_heading) {
        PendingFactor hp;
        hp.frame_id = c.id;
        hp.stamp = c.stamp;
        hp.factor = gnss_core::HeadingPriorFactor::create(X(c.id), c.heading_world, c.heading_model);
        output_.push_back(hp);
        ++n_heading_added_;
        ++kept_heading;
      }
    }

    size_t kept_rel = 0;
    if (relative_pipeline_) {
      const auto rel = relative_pipeline_->process();
      for (const auto& rc : rel) {
        // Two-key factor: the lag guard must use the earlier key, otherwise one endpoint
        // may already be marginalized and the smoother would be corrupted.
        if (rc.stamp1 < oldest_allowed) {
          ++n_dropped_lag_;
          continue;
        }
        PendingFactor p;
        p.frame_id = rc.id2;         // newer key; drives the frame_id diagnostics
        p.stamp = rc.stamp1;         // earlier key; drives the final lag guard in on_smoother_update
        p.factor = gnss_core::RelativeTranslationFactor::create(X(rc.id1), X(rc.id2), rc.dp_world_xy, rc.model);
        output_.push_back(p);
        last_relative_stamp_ = rc.stamp2;
        ++n_relative_added_;
        ++kept_rel;
      }

      if (!relative_logged_ && relative_pipeline_->yaw_initialized()) {
        relative_logged_ = true;
        logger_->info("relative yaw bootstrapped (baseline {} m): T_world_enu={}", cfg_.relative.yaw_baseline, convert_to_string(relative_pipeline_->T_world_enu()));
      }
    }

    if (kept_abs || kept_heading || kept_rel) {
      logger_->debug("queued RTK factors: absolute={} heading={} relative={} (oldest_allowed={:.1f})", kept_abs, kept_heading, kept_rel, oldest_allowed);
    }

    if (!bootstrap_logged_ && pipeline_->initialized()) {
      bootstrap_logged_ = true;
      logger_->info("T_world_enu={} (source={})", convert_to_string(pipeline_->T_world_enu()), pipeline_->gauge_source());
      logger_->info(
        "bootstrap: released {} deferred RTK constraints, kept {} absolute inside the lag window, dropped {} (odometry is fixed-lag; the first ~min_baseline runs without RTK by design)",
        pipeline_->stats().deferred_released,
        kept_abs,
        constraints.size() - kept_abs);
    }
    if (!anchors.empty()) {
      const auto o = pipeline_->last_outcome();
      if (o == gnss_core::AnchorOutcome::TooOld && !horizon_warned_.exchange(true)) {
        logger_->warn("frame anchor older than the RTK buffer ({} s horizon): clock domains differ or RTK stream stalled (check stamp_source/time_offset)", cfg_.pipeline.fix_buffer_horizon);
      }
      logger_->debug("anchor outcome={} pending={} deferred={} buffered_fixes={}", rtk::outcome_name(o), pipeline_->pending_anchors(), pipeline_->deferred(), pipeline_->buffered_fixes());
    }

    // Publish the FusionStatus snapshot consumed by the odometry thread (2.2 / stage 4).
    update_status_snapshot();

    if (fixes.empty() && anchors.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));  // frames arrive at LiDAR rate; keep latency well under one frame
    }
  }
  logger_->info("rtk_odometry backend thread finished");
}

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::RtkOdometry();
}
