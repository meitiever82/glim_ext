// rtk_odometry — quality-aware RTK/GNSS position constraints on GLIM's odometry
// frames (fixed-lag smoother), spec §7.0 "rtk_odometry" (round 1.5).
//
// Thin shell around gnss_core::AnchorPipeline. Differences from rtk_global:
//   * anchors are odometry frames (on_update_new_frame: id, stamp, corrected T_world_imu)
//   * factors go into OdometryEstimationCallbacks::on_smoother_update, and only for
//     keys still inside the fixed-lag window — a factor on a marginalized key would
//     corrupt the smoother. Constraints older than (smoother_lag - smoother_lag_margin)
//     relative to the newest frame are dropped and counted.
//   * its own T_world_enu (odometry world != global mapping world)
//
// Threading:
//   * rtk_fix_callback       — ROS executor thread   → input_fix_queue_
//   * on_update_new_frame    — odometry thread       → input_anchor_queue_ (POD copy)
//   * backend_task           — own thread; sole owner of pipeline_
//   * on_smoother_update     — odometry thread; drains output_ with the lag guard
//
// Can run together with rtk_global (different graphs). Not with gnss_global-style
// odometry priors (none exist upstream).

#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#define GLIM_ROS2

#include <glim/odometry/callbacks.hpp>
#include <glim/util/concurrent_vector.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/util/logging.hpp>

#include <gtsam/nonlinear/NonlinearFactor.h>

#include <glim_ext/rtk/rtk_shell_common.hpp>
#include <gnss_core/anchor_pipeline.hpp>

namespace glim {

class RtkOdometry : public ExtensionModuleROS2 {
public:
  RtkOdometry();
  ~RtkOdometry() override;

  std::vector<GenericTopicSubscription::Ptr> create_subscriptions() override;
  void at_exit(const std::string& dump_path) override;

private:
  struct PendingFactor {
    long frame_id;
    double stamp;
    gtsam::NonlinearFactor::shared_ptr factor;
  };

  void rtk_fix_callback(const gnss_msgs::msg::RtkFix::ConstSharedPtr& msg);
  void on_update_new_frame(const EstimationFrame::ConstPtr& frame);
  void on_smoother_update(
    gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    gtsam::NonlinearFactorGraph& new_factors,
    gtsam::Values& new_values,
    std::map<std::uint64_t, double>& new_stamps);
  void backend_task();
  void stop_backend();

  rtk::ShellConfig cfg_;
  double smoother_lag_ = 5.0;          // from config_odometry
  double smoother_lag_margin_ = 1.0;   // s; keep this far from the marginalization edge
  std::unique_ptr<rtk::RtkFixIngest> ingest_;
  std::unique_ptr<gnss_core::AnchorPipeline> pipeline_;   // backend thread only
  bool bootstrap_logged_ = false;                          // backend thread only
  std::atomic_bool horizon_warned_{false};

  ConcurrentVector<gnss_core::RtkFixSample> input_fix_queue_;
  ConcurrentVector<gnss_core::Anchor> input_anchor_queue_;
  ConcurrentVector<PendingFactor> output_;

  std::atomic<double> latest_frame_stamp_{0.0};   // written by the odometry thread, read by backend + odometry threads
  std::atomic<size_t> n_factors_added_{0};
  std::atomic<size_t> n_dropped_lag_{0};   // constraints that arrived after their frame left the lag window
  std::atomic_bool kill_switch_{false};
  std::thread thread_;

  std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace glim
