// rtk_global — quality-aware RTK/GNSS position constraints for GLIM's global
// mapping graph (spec: docs/gnss/specs/2026-09-03-gnss-glim-modules-design.md v2).
//
// Thin shell around gnss_core::AnchorPipeline: subscribes gnss_msgs/RtkFix, hooks
// GlobalMappingCallbacks, turns pipeline Constraints into AntennaPriorFactor on X(submap).
//
// Threading (spec §7.1 v2):
//   * rtk_fix_callback   — ROS executor thread  → input_fix_queue_
//   * on_insert_submap   — global mapping thread → copies a POD Anchor into input_anchor_queue_
//                          (never holds SubMap::Ptr: T_world_origin is only safe on that thread)
//   * backend_task       — own thread; sole owner of pipeline_
//   * on_smoother_update — global mapping thread; drains output_factors_ only
//
// Lifetime: CallbackSlot registrations are never removed (no API, same as gnss_global);
// at_exit() stops the backend early so late callbacks only meet idle queues.
//
// Mutually exclusive with libgnss_global.so (both add position priors on X(submap)).

#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#define GLIM_ROS2

#include <glim/mapping/callbacks.hpp>
#include <glim/util/concurrent_vector.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/util/logging.hpp>

#include <gtsam/nonlinear/NonlinearFactor.h>

#include <glim_ext/rtk/rtk_shell_common.hpp>
#include <gnss_core/anchor_pipeline.hpp>

namespace glim {

class RtkGlobal : public ExtensionModuleROS2 {
public:
  RtkGlobal();
  ~RtkGlobal() override;

  std::vector<GenericTopicSubscription::Ptr> create_subscriptions() override;
  void at_exit(const std::string& dump_path) override;

private:
  void rtk_fix_callback(const gnss_msgs::msg::RtkFix::ConstSharedPtr& msg);
  void on_insert_submap(const SubMap::ConstPtr& submap);
  void on_smoother_update(gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values);
  void backend_task();
  void stop_backend();

  rtk::ShellConfig cfg_;
  std::unique_ptr<rtk::RtkFixIngest> ingest_;
  std::unique_ptr<gnss_core::AnchorPipeline> pipeline_;   // backend thread only
  bool bootstrap_logged_ = false;                          // backend thread only
  std::atomic_bool horizon_warned_{false};

  ConcurrentVector<gnss_core::RtkFixSample> input_fix_queue_;
  ConcurrentVector<gnss_core::Anchor> input_anchor_queue_;
  ConcurrentVector<gtsam::NonlinearFactor::shared_ptr> output_factors_;

  std::atomic<size_t> n_factors_added_{0};
  std::atomic_bool kill_switch_{false};
  std::thread thread_;

  std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace glim
