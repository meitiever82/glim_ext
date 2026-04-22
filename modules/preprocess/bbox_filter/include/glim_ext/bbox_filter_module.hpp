#pragma once

#include <atomic>
#include <thread>
#include <unordered_map>
#include <spdlog/spdlog.h>

#include <glim/util/extension_module.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

namespace glim {

class BBoxFilterModule : public ExtensionModule {
public:
  BBoxFilterModule();
  ~BBoxFilterModule();

private:
  void on_preprocessing_begin(gtsam_points::PointCloudCPU::Ptr& points);

private:
  std::string crop_bbox_frame;    ///< Bounding box reference frame
  Eigen::Vector3d crop_bbox_min;  ///< Bounding box min point
  Eigen::Vector3d crop_bbox_max;  ///< Bounding box max point
  Eigen::Isometry3d T_imu_lidar;  ///< LiDAR-IMU transformation when cropbox is defined in IMU frame

  // logging
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim
