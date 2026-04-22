#include <glim_ext/bbox_filter_module.hpp>

#include <glim/util/logging.hpp>
#include <glim/preprocess/callbacks.hpp>

namespace glim {

BBoxFilterModule::BBoxFilterModule() : logger(create_module_logger("bbox")) {
  logger->info("initializing...");
  crop_bbox_frame = "lidar";          ///< Bounding box reference frame
  crop_bbox_min << -1.0, -1.0, -1.0;  ///< Bounding box min point
  crop_bbox_max << 1.0, 1.0, 1.0;     ///< Bounding box max point
  T_imu_lidar.setIdentity();          ///< LiDAR-IMU transformation when cropbox is defined in IMU frame

  PreprocessCallbacks::on_preprocessing_begin.add([this](gtsam_points::PointCloudCPU::Ptr& points) { on_preprocessing_begin(points); });
  logger->info("ready");
}

BBoxFilterModule::~BBoxFilterModule() {}

void BBoxFilterModule::on_preprocessing_begin(gtsam_points::PointCloudCPU::Ptr& frame) {
  logger->info("preprocessing begin: {} points", frame->size());

  // Cropbox filter
  // if (crop_bbox_frame == "lidar") {
  //   auto is_inside_bbox = [&](const Eigen::Vector3d& p_lidar) { return (p_lidar.array() >= crop_bbox_min.array()).all() && (p_lidar.array() <= crop_bbox_max.array()).all(); };
  //   frame = gtsam_points::filter(frame, [&](const auto& pt) { return !is_inside_bbox(pt.template head<3>()); });
  // } else if (crop_bbox_frame == "imu") {
  //   auto is_inside_bbox = [&](const Eigen::Vector3d& p_lidar) {
  //     const auto p_imu = T_imu_lidar * p_lidar;
  //     return (p_imu.array() >= crop_bbox_min.array()).all() && (p_imu.array() <= crop_bbox_max.array()).all();
  //   };

  //   frame = gtsam_points::filter(frame, [&](const auto& pt) { return !is_inside_bbox(pt.template head<3>()); });
  // } else {
  //   logger->error("Unsupported crop bbox frame: {}", crop_bbox_frame);
  // }

  logger->info("filtered: {} points", frame->size());
}

}  // namespace glim