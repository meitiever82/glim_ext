#pragma once
// Stage-2 (2.2) 100m submap manager (spec 2.2 §5/§6.2).
//
// Keeps a single active SemanticVoxelMap centred on the vehicle. When the centre drifts
// beyond half of submap_radius the submap is recentred: version increments, the anchor pose
// moves to the new centre, retained voxels are weight-decayed and everything outside the
// radius is pruned. Decay + prune is what keeps the boundary from jumping on a switch.

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/preprocess/preprocessed_frame.hpp>

#include "glim_ext/semantic/semantic_types.hpp"
#include "glim_ext/semantic/semantic_voxel_map.hpp"

namespace glim {

class SubmapManager {
public:
  using Ptr = std::shared_ptr<SubmapManager>;

  SubmapManager(const SemanticVoxelMap::Config& cfg,
                double submap_radius = 50.0,
                std::string vehicle_id = "",
                uint32_t extrinsic_version = 0);

  // Recentre when the requested centre is farther than 0.5 * submap_radius from the anchor.
  void set_center(const Eigen::Vector3d& center);

  void add_voxels(const std::vector<SemanticVoxel>& voxels);

  // Remove voxels farther than keep_radius from the current centre.
  void prune(double keep_radius);

  SemanticVoxelMap::Ptr get_active_submap(SubmapMeta* meta = nullptr) const;

  // Convenience: feed the active submap from a frame, updating the centre first.
  void update(const PreprocessedFrame& frame,
              const Eigen::Isometry3d& pose,
              const GroundSegmentationResult& seg);

  Eigen::Vector3d center() const;
  double submap_radius() const;
  uint32_t version() const;

private:
  mutable std::mutex mutex_;
  SemanticVoxelMap::Ptr map_;
  SubmapMeta meta_;
  Eigen::Vector3d center_ = Eigen::Vector3d::Zero();
  double submap_radius_ = 50.0;
  double recenter_ratio_ = 0.5;
  float switch_decay_ = 0.95f;
};

}  // namespace glim
