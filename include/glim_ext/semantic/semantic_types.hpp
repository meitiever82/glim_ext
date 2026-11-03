#pragma once
// Stage-2 (2.2) shared semantic map types: ground segmentation result, semantic voxel,
// submap metadata and the dynamic-mask contract owned by stage 4.
//
// Layout is deliberately stage-4/5 friendly: the base fields below are stable, and the
// extended fields listed in spec 2.2 §4 can be appended without breaking this layout.
// No ROS and no gnss_core dependency.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim {

// Label set from spec 2.2 §4. Only GROUND / NON_GROUND_STATIC are produced in stage 2;
// the rest are reserved for stage-4 perception (VEGETATION / ORE_PILE stay unused).
enum class SemanticLabel : uint8_t {
  UNKNOWN = 0,
  GROUND = 1,
  NON_GROUND_STATIC = 2,
  VEHICLE = 3,
  PEDESTRIAN = 4,
  VEGETATION = 5,
  ORE_PILE = 6,
  DYNAMIC_UNKNOWN = 7,
};

constexpr size_t kSemanticLabelCount = 8;

inline const char* semantic_label_name(SemanticLabel label) {
  switch (label) {
    case SemanticLabel::UNKNOWN: return "UNKNOWN";
    case SemanticLabel::GROUND: return "GROUND";
    case SemanticLabel::NON_GROUND_STATIC: return "NON_GROUND_STATIC";
    case SemanticLabel::VEHICLE: return "VEHICLE";
    case SemanticLabel::PEDESTRIAN: return "PEDESTRIAN";
    case SemanticLabel::VEGETATION: return "VEGETATION";
    case SemanticLabel::ORE_PILE: return "ORE_PILE";
    case SemanticLabel::DYNAMIC_UNKNOWN: return "DYNAMIC_UNKNOWN";
  }
  return "UNKNOWN";
}

// Ground plane in the world frame (spec 2.2 §3). Also consumed by 2.1's reserved
// insert_ground_plane slot (stage 4 visual ground soft constraint).
struct GroundPlaneParams {
  Eigen::Vector3f normal = Eigen::Vector3f(0.0f, 0.0f, 1.0f);  // unit normal, n·x + d = 0
  float d = 0.0f;
  float confidence = 0.0f;                                     // 0~1; below threshold means "do not inject"
};

struct GroundSegmentationResult {
  std::vector<uint32_t> ground_indices;       // indices into the input point array
  std::vector<uint32_t> non_ground_indices;
  GroundPlaneParams plane;                   // fitted on the ground points, world frame
  float confidence = 0.0f;                   // ground ratio / fit quality, 0~1
};

// One semantic voxel (spec 2.2 §4 base fields).
struct SemanticVoxel {
  using Ptr = std::shared_ptr<SemanticVoxel>;
  using ConstPtr = std::shared_ptr<const SemanticVoxel>;

  // Stage-2 addition: world position of the voxel center. The spec's base fields carry no
  // position, but SubmapManager::add_voxels / get_submap need one to reconstruct keys.
  Eigen::Vector3f position = Eigen::Vector3f::Zero();

  float occupancy_prob = 0.0f;
  uint8_t semantic_label = static_cast<uint8_t>(SemanticLabel::UNKNOWN);
  Eigen::Vector3f plane_normal = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
  float plane_d = 0.0f;
  uint32_t update_count = 0;
  double last_update_time = 0.0;
  float height_mean = 0.0f;
  float height_var = 0.0f;
};

// Submap metadata (spec 2.2 §6.2). anchor_pose is the map-frame pose of the submap center
// at creation time; version increments on every recenter.
struct SubmapMeta {
  Eigen::Isometry3d anchor_pose = Eigen::Isometry3d::Identity();
  double stamp = 0.0;
  float resolution = 0.0f;
  std::string vehicle_id;
  uint32_t version = 0;
  uint32_t extrinsic_version = 0;
};

// Stage-4 perception output (spec 2.2 §6.3). Stage 2 only declares and consumes it.
struct ObjectTrack {
  int id = 0;
  double stamp = 0.0;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();  // world, bbox center
  Eigen::Vector3f size = Eigen::Vector3f::Zero();          // bbox extent (m)
  uint8_t semantic_label = static_cast<uint8_t>(SemanticLabel::DYNAMIC_UNKNOWN);
  float confidence = 0.0f;
  Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
};

// Stage-4 dynamic mask (spec 2.2 §6.3). The stage-2 consumer accepts world-frame AABB
// regions; stage 4 may extend this struct with voxel/point-level flags without changing
// the base fields of SemanticVoxel.
struct DynamicMask {
  double stamp = 0.0;
  struct Region {
    Eigen::Vector3f min = Eigen::Vector3f::Zero();
    Eigen::Vector3f max = Eigen::Vector3f::Zero();
  };
  std::vector<Region> regions;  // world frame
};

}  // namespace glim
