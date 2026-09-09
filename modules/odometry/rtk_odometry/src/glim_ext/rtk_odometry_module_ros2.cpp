#include <glim_ext/rtk_odometry_module.hpp>

#include <algorithm>
#include <chrono>
#include <functional>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <glim/util/config.hpp>
#include <glim/util/convert_to_string.hpp>
#include <glim_ext/util/config_ext.hpp>

#include <gnss_core/antenna_prior_factor.hpp>

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
  if (const auto o = pipeline_->enu_origin()) {
    logger_->info("ENU origin from config: lat={:.8f} lon={:.8f} alt={:.3f}", (*o)[0], (*o)[1], (*o)[2]);
  }

  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;
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
    "rtk_odometry stats: fix_received={} fix_dropped={} fix_pushed={} anchors={} factors_added={} dropped_by_lag={} deferred_released={} too_old={} gap={} gated={} pending_anchors={} deferred_pending={}",
    ingest_->received(),
    ingest_->dropped(),
    s.fixes,
    s.anchors,
    n_factors_added_.load(),
    n_dropped_lag_.load(),
    s.deferred_released,
    s.too_old,
    s.gap,
    s.gated,
    pipeline_->pending_anchors(),
    pipeline_->deferred());
}

std::vector<GenericTopicSubscription::Ptr> RtkOdometry::create_subscriptions() {
  const auto sub = std::make_shared<TopicSubscription<gnss_msgs::msg::RtkFix>>(cfg_.rtk_fix_topic, [this](const gnss_msgs::msg::RtkFix::ConstSharedPtr msg) { rtk_fix_callback(msg); });
  return {sub};
}

void RtkOdometry::rtk_fix_callback(const gnss_msgs::msg::RtkFix::ConstSharedPtr& msg) {
  if (const auto s = ingest_->convert(*msg)) {
    input_fix_queue_.push_back(*s);
  }
}

void RtkOdometry::on_update_new_frame(const EstimationFrame::ConstPtr& frame) {
  // Odometry thread. Corrected (post-optimization) pose of the newest frame.
  gnss_core::Anchor a;
  a.id = frame->id;
  a.stamp = frame->stamp;
  a.t_world = frame->T_world_imu.translation();
  latest_frame_stamp_ = frame->stamp;
  input_anchor_queue_.push_back(a);
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
    logger_->debug("insert {} RTK prior factors into the odometry smoother", added);
  }
}

void RtkOdometry::backend_task() {
  logger_->info("starting rtk_odometry backend thread");
  while (!kill_switch_) {
    const auto fixes = input_fix_queue_.get_all_and_clear();
    for (const auto& f : fixes) {
      pipeline_->push_fix(f);
    }
    const auto anchors = input_anchor_queue_.get_all_and_clear();
    for (const auto& a : anchors) {
      pipeline_->push_anchor(a);
    }

    const auto constraints = pipeline_->process();
    // Pre-filter against the lag window here (backend thread) so that the bulk of
    // deferred constraints released at bootstrap never becomes factor objects; the
    // guard in on_smoother_update remains as the final check.
    const double oldest_allowed = latest_frame_stamp_.load() - (smoother_lag_ - smoother_lag_margin_);
    size_t kept = 0;
    for (const auto& c : constraints) {
      if (c.stamp < oldest_allowed) {
        ++n_dropped_lag_;
        continue;
      }
      // body_point = lever_imu: X(frame) is T_world_imu, antenna sits at lever_imu in the IMU frame (spec §7.3).
      PendingFactor p;
      p.frame_id = c.id;
      p.stamp = c.stamp;
      p.factor = gnss_core::AntennaPriorFactor::create(X(c.id), c.p_world, cfg_.lever_imu, c.model);
      output_.push_back(p);
      ++kept;
    }

    if (!bootstrap_logged_ && pipeline_->initialized()) {
      bootstrap_logged_ = true;
      logger_->info("T_world_enu={}", convert_to_string(pipeline_->T_world_enu()));
      logger_->info(
        "bootstrap: released {} deferred RTK constraints, kept {} inside the lag window, dropped {} (odometry is fixed-lag; the first ~min_baseline runs without RTK by design)",
        pipeline_->stats().deferred_released,
        kept,
        constraints.size() - kept);
    }
    if (!anchors.empty()) {
      const auto o = pipeline_->last_outcome();
      if (o == gnss_core::AnchorOutcome::TooOld && !horizon_warned_.exchange(true)) {
        logger_->warn("frame anchor older than the RTK buffer ({} s horizon): clock domains differ or RTK stream stalled (check stamp_source/time_offset)", cfg_.pipeline.fix_buffer_horizon);
      }
      logger_->debug("anchor outcome={} pending={} deferred={} buffered_fixes={}", rtk::outcome_name(o), pipeline_->pending_anchors(), pipeline_->deferred(), pipeline_->buffered_fixes());
    }

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
