#include <glim_ext/bbox_filter_module.hpp>

#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/preprocess/callbacks.hpp>
#include <glim_ext/util/config_ext.hpp>

namespace glim {

BBoxFilterModule::BBoxFilterModule() : logger(create_module_logger("bbox")) {
  logger->info("initializing...");

  const auto config_path = GlobalConfigExt::get_config_path("config_bbox_filter");
  logger->info("config_path={}", config_path);

  Config config(config_path);
  crop_bbox_frame = config.param<std::string>("bbox_filter", "crop_bbox_frame", "lidar");                             ///< Bounding box reference frame
  crop_bbox_min = config.param<Eigen::Vector3d>("bbox_filter", "crop_bbox_min", Eigen::Vector3d(-15.0, -3.0, -5.0));  ///< Bounding box min point
  crop_bbox_max = config.param<Eigen::Vector3d>("bbox_filter", "crop_bbox_max", Eigen::Vector3d(15.0, 3.0, 5.0));     ///< Bounding box max point

  if (crop_bbox_frame == "imu") {
    Config sensor_config(GlobalConfig::get_config_path("config_sensors"));
    const Eigen::Isometry3d T_lidar_imu = sensor_config.param<Eigen::Isometry3d>("sensors", "T_lidar_imu", Eigen::Isometry3d::Identity());
    T_imu_lidar = T_lidar_imu.inverse();
  } else {
    T_imu_lidar.setIdentity();
  }

  PreprocessCallbacks::on_preprocessing_begin.add([this](gtsam_points::PointCloudCPU::Ptr& points) { on_preprocessing_begin(points); });
  logger->info("ready");
}

BBoxFilterModule::~BBoxFilterModule() {}

void BBoxFilterModule::on_preprocessing_begin(gtsam_points::PointCloudCPU::Ptr& frame) {
  logger->trace("preprocessing begin: {} points", frame->size());

  // Cropbox filter
  if (crop_bbox_frame == "lidar") {
    auto is_inside_bbox = [&](const Eigen::Vector3d& p_lidar) { return (p_lidar.array() >= crop_bbox_min.array()).all() && (p_lidar.array() <= crop_bbox_max.array()).all(); };
    frame = gtsam_points::filter(frame, [&](const auto& pt) { return !is_inside_bbox(pt.template head<3>()); });
  } else if (crop_bbox_frame == "imu") {
    auto is_inside_bbox = [&](const Eigen::Vector3d& p_lidar) {
      const auto p_imu = T_imu_lidar * p_lidar;
      return (p_imu.array() >= crop_bbox_min.array()).all() && (p_imu.array() <= crop_bbox_max.array()).all();
    };

    frame = gtsam_points::filter(frame, [&](const auto& pt) { return !is_inside_bbox(pt.template head<3>()); });
  } else {
    logger->error("Unsupported crop bbox frame: {}", crop_bbox_frame);
  }

  logger->trace("filtered: {} points", frame->size());
}

}  // namespace glim