#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <glim/odometry/callbacks.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/util/concurrent_vector.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/logging.hpp>

#include <glim_ext/semantic/linefit_ground_segmentation.hpp>
#include <glim_ext/semantic/semantic_map_bus.hpp>
#include <glim_ext/semantic/semantic_voxel_map.hpp>
#include <glim_ext/semantic/submap_manager.hpp>
#include <glim_ext/util/config_ext.hpp>

namespace glim {

// Stage-2 (2.2) semantic map shell: ground segmentation + 100m semantic voxel submap.
// Hooks on_update_new_frame (corrected odometry pose), decimates to voxel_update_min_interval
// and does the heavy work (segmentation / voxel insertion) on a background thread so the
// odometry estimator is never blocked by map updates.
class SemanticVoxelMapModule : public ExtensionModule {
public:
  SemanticVoxelMapModule() : logger(create_module_logger("semantic_map")) {
    logger->info("Starting ...");

    Config config(GlobalConfigExt::get_config_path("config_semantic_map"));
    const std::string section = "semantic_map";

    const double submap_radius = config.param<double>(section, "submap_radius", 50.0);
    min_interval_ = config.param<double>(section, "voxel_update_min_interval", 0.1);
    const std::string vehicle_id = config.param<std::string>(section, "vehicle_id", "");
    const uint32_t extrinsic_version = config.param<int>(section, "extrinsic_version", 0);

    SemanticVoxelMap::Config map_cfg;
    map_cfg.near_resolution = config.param<double>(section, "near_resolution", 0.15);
    map_cfg.far_resolution = config.param<double>(section, "far_resolution", 0.4);
    map_cfg.far_switch_distance = config.param<double>(section, "far_switch_distance", 25.0);

    LineFitGroundSegmentation::Config seg_cfg;
    seg_cfg.n_sectors = config.param<int>(section, "ground_sectors", 60);
    seg_cfg.range_bin_width = config.param<double>(section, "ground_range_bin_width", 0.5);
    seg_cfg.min_range = config.param<double>(section, "ground_min_range", 1.0);
    seg_cfg.max_range = config.param<double>(section, "ground_max_range", 60.0);
    seg_cfg.max_line_dist = config.param<double>(section, "ground_max_line_dist", 0.15);
    seg_cfg.max_slope = config.param<double>(section, "ground_max_slope", 0.35);
    seg_cfg.min_ground_points = static_cast<size_t>(std::max(0, config.param<int>(section, "ground_min_points", 100)));
    segmentation_ = LineFitGroundSegmentation(seg_cfg);

    manager_ = std::make_shared<SubmapManager>(map_cfg, submap_radius, vehicle_id, extrinsic_version);
    SemanticMapBus::instance().set_manager(manager_);

    using std::placeholders::_1;
    OdometryEstimationCallbacks::on_update_new_frame.add(
      std::bind(&SemanticVoxelMapModule::on_update_new_frame, this, _1));

    worker_ = std::thread([this] { worker_loop(); });
  }

  ~SemanticVoxelMapModule() override {
    queue.submit_end_of_data();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void at_exit(const std::string& dump_path) override {
    const uint64_t frames = frames_processed_.load();
    logger->info(
      "semantic map done: frames={} skipped={} recenters={} voxels={} avg_segmentation={:.2f}ms",
      frames,
      frames_skipped_.load(),
      manager_ ? manager_->version() : 0,
      manager_ ? manager_->get_active_submap()->size() : 0,
      frames ? segmentation_total_us_.load() / (1000.0 * static_cast<double>(frames)) : 0.0);
  }

private:
  void on_update_new_frame(const EstimationFrame::ConstPtr& frame) {
    // Keep only the newest frame: map updates are decimated anyway.
    queue.push_back(frame);
  }

  void worker_loop() {
    while (true) {
      auto frame = queue.pop_wait();
      if (!frame) {
        break;
      }
      process(*frame);
    }
  }

  void process(const EstimationFrame::ConstPtr& frame) {
    if (!frame || !frame->raw_frame || frame->raw_frame->points.empty()) {
      return;
    }
    if (frame->stamp - last_stamp_ < min_interval_) {
      ++frames_skipped_;
      return;
    }
    last_stamp_ = frame->stamp;

    const auto t0 = std::chrono::high_resolution_clock::now();

    const PreprocessedFrame& pre = *frame->raw_frame;
    const Eigen::Isometry3d T_world_lidar = frame->T_world_lidar;

    GroundSegmentationResult seg;
    segmentation_.segment(pre.points, T_world_lidar, seg);
    manager_->update(pre, T_world_lidar, seg);

    const auto t1 = std::chrono::high_resolution_clock::now();
    segmentation_total_us_.fetch_add(
      static_cast<uint64_t>(std::chrono::duration<double, std::micro>(t1 - t0).count()));
    frames_processed_.fetch_add(1);
  }

private:
  std::shared_ptr<spdlog::logger> logger;
  LineFitGroundSegmentation segmentation_;
  std::shared_ptr<SubmapManager> manager_;

  ConcurrentVector<EstimationFrame::ConstPtr> queue{DataStorePolicy::UPTO(1)};
  std::thread worker_;

  double min_interval_ = 0.1;
  double last_stamp_ = -1.0;

  std::atomic<uint64_t> frames_processed_{0};
  std::atomic<uint64_t> frames_skipped_{0};
  std::atomic<uint64_t> segmentation_total_us_{0};
};

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::SemanticVoxelMapModule();
}
