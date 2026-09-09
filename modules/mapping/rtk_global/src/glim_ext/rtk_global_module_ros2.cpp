#include <glim_ext/rtk_global_module.hpp>

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

RtkGlobal::RtkGlobal() : logger_(create_module_logger("rtk_global")) {
  logger_->info("initializing rtk_global (quality-aware RTK constraints on submaps)");

  const std::string config_path = GlobalConfigExt::get_config_path("config_rtk_global");
  logger_->info("rtk_global_config_path={}", config_path);
  cfg_ = rtk::load_shell_config(Config(config_path), "rtk_global");
  rtk::log_shell_config(*logger_, cfg_);

  ingest_ = std::make_unique<rtk::RtkFixIngest>(cfg_, logger_);
  pipeline_ = std::make_unique<gnss_core::AnchorPipeline>(cfg_.pipeline);  // throws on invalid config
  if (const auto o = pipeline_->enu_origin()) {
    logger_->info("ENU origin from config: lat={:.8f} lon={:.8f} alt={:.3f}", (*o)[0], (*o)[1], (*o)[2]);
  }

  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  GlobalMappingCallbacks::on_insert_submap.add(std::bind(&RtkGlobal::on_insert_submap, this, _1));
  GlobalMappingCallbacks::on_smoother_update.add(std::bind(&RtkGlobal::on_smoother_update, this, _1, _2, _3));

  kill_switch_ = false;
  thread_ = std::thread([this] { backend_task(); });
}

RtkGlobal::~RtkGlobal() {
  stop_backend();
}

void RtkGlobal::at_exit(const std::string& /*dump_path*/) {
  stop_backend();
}

void RtkGlobal::stop_backend() {
  if (!thread_.joinable()) {
    return;
  }
  kill_switch_ = true;
  thread_.join();
  const auto& s = pipeline_->stats();
  logger_->info(
    "rtk_global stats: fix_received={} fix_dropped={} fix_pushed={} anchors={} factors_added={} deferred_released={} too_old={} gap={} gated={} pending_anchors={} deferred_pending={}",
    ingest_->received(),
    ingest_->dropped(),
    s.fixes,
    s.anchors,
    n_factors_added_.load(),
    s.deferred_released,
    s.too_old,
    s.gap,
    s.gated,
    pipeline_->pending_anchors(),
    pipeline_->deferred());
}

std::vector<GenericTopicSubscription::Ptr> RtkGlobal::create_subscriptions() {
  const auto sub = std::make_shared<TopicSubscription<gnss_msgs::msg::RtkFix>>(cfg_.rtk_fix_topic, [this](const gnss_msgs::msg::RtkFix::ConstSharedPtr msg) { rtk_fix_callback(msg); });
  return {sub};
}

void RtkGlobal::rtk_fix_callback(const gnss_msgs::msg::RtkFix::ConstSharedPtr& msg) {
  if (const auto s = ingest_->convert(*msg)) {
    input_fix_queue_.push_back(*s);
  }
}

void RtkGlobal::on_insert_submap(const SubMap::ConstPtr& submap) {
  // Runs on the global mapping thread: the only place T_world_origin may be read.
  gnss_core::Anchor a;
  a.id = submap->id;
  a.stamp = submap->origin_frame()->stamp;
  a.t_world = submap->T_world_origin.translation();
  input_anchor_queue_.push_back(a);
}

void RtkGlobal::on_smoother_update(gtsam_points::ISAM2Ext& /*isam2*/, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& /*new_values*/) {
  const auto factors = output_factors_.get_all_and_clear();
  if (!factors.empty()) {
    logger_->debug("insert {} RTK prior factors", factors.size());
    new_factors.add(factors);
  }
}

void RtkGlobal::backend_task() {
  logger_->info("starting rtk_global backend thread");
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
    for (const auto& c : constraints) {
      // body_point = lever_imu: the sample is taken at the submap origin frame (T_origin_frame = I), spec §7.3 v2.
      output_factors_.push_back(gnss_core::AntennaPriorFactor::create(X(c.id), c.p_world, cfg_.lever_imu, c.model));
      ++n_factors_added_;
      logger_->debug("submap {}: RTK prior p_world={}", c.id, convert_to_string(c.p_world));
    }

    if (!bootstrap_logged_ && pipeline_->initialized()) {
      bootstrap_logged_ = true;
      logger_->info("T_world_enu={}", convert_to_string(pipeline_->T_world_enu()));
      logger_->info("released {} deferred RTK factors for submaps before bootstrap", pipeline_->stats().deferred_released);
    }
    if (!anchors.empty()) {
      const auto o = pipeline_->last_outcome();
      if (o == gnss_core::AnchorOutcome::TooOld && !horizon_warned_.exchange(true)) {
        logger_->warn(
          "submap anchor older than the RTK buffer ({} s horizon): global mapping lags the RTK stream, or clock domains differ (check stamp_source/time_offset)",
          cfg_.pipeline.fix_buffer_horizon);
      }
      logger_->debug("anchor outcome={} pending={} deferred={} buffered_fixes={}", rtk::outcome_name(o), pipeline_->pending_anchors(), pipeline_->deferred(), pipeline_->buffered_fixes());
    }

    if (fixes.empty() && anchors.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  logger_->info("rtk_global backend thread finished");
}

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::RtkGlobal();
}
