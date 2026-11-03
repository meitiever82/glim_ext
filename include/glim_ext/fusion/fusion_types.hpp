// Stage-2 cross-module fusion contract (spec: docs/features/spec/阶段二_Spec_RTK_IMU_LO融合定位_语义VoxelMap.md §5.1).
//
// This header is the only data contract shared by 2.1 (localization), 2.2 (semantic voxel
// map) and the later perception stage. It deliberately does NOT depend on gnss_core or ROS
// so that map/perception modules can consume it without pulling in the GNSS stack.

#pragma once

#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim {

// Normalized fix quality, mirroring gnss_core::Quality / gnss_msgs/RtkFix.QUALITY_*.
// Kept as a separate enum so this contract stays free of the GNSS dependency.
enum class RtkQuality : uint8_t {
  NONE = 0,
  SINGLE = 1,
  DGPS = 2,
  FLOAT = 3,
  FIXED = 4,
};

// Localization health snapshot (2.1 §8). Published at >= 5 Hz.
struct FusionStatus {
  // RTK observation quality (verdict of RtkNoisePolicy).
  RtkQuality rtk_quality = RtkQuality::NONE;
  double sigma_enu[3] = {0.0, 0.0, 0.0};  // effective noise (m, E/N/U)

  // Shape confidence contributed by RTK relative-translation constraints (0~1).
  float shape_confidence = 0.0f;

  // Absolute anchoring state (RTK absolute factor).
  bool absolute_anchored = false;
  double seconds_since_anchor = 0.0;  // since the last absolute constraint
  bool rtk_outage = false;            // RTK stream lost, running on relative + IMU

  // Degradation flags.
  bool degraded_rtk = false;
  bool degraded_lo = false;
  bool degraded_imu = false;

  // Reserved for Scan-to-Prior-Map; always false in stage 2.
  bool map_aligned = false;
};

struct FusionResult {
  double stamp = 0.0;
  Eigen::Isometry3d T_world_base = Eigen::Isometry3d::Identity();
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  FusionStatus status;
};

}  // namespace glim
