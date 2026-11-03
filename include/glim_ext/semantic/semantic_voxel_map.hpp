#pragma once
// Stage-2 (2.2) semantic voxel map (spec 2.2 §4/§6.1).
//
// Hash voxel map with two resolutions: near (0.15 m default) within far_switch_distance of
// the current sensor pose, far (0.4 m default) beyond it. Each voxel keeps the base semantic
// fields plus a world-frame position so submaps can be extracted/merged with metadata.
//
// Thread safety: all public methods lock an internal mutex; `get_active_submap` hands out a
// shared_ptr, and stage-4 readers are expected to go through the map's methods.

#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/preprocess/preprocessed_frame.hpp>

#include "glim_ext/semantic/semantic_types.hpp"

namespace glim {

class SemanticVoxelMap {
public:
  using Ptr = std::shared_ptr<SemanticVoxelMap>;

  struct Config {
    float near_resolution = 0.15f;       // m
    float far_resolution = 0.4f;         // m
    float far_switch_distance = 25.0f;   // m from the sensor pose
  };

  SemanticVoxelMap();
  explicit SemanticVoxelMap(const Config& cfg, const SubmapMeta& meta = SubmapMeta());

  // Spec 2.2 §6.1 signature.
  void update(const PreprocessedFrame& frame,
              const Eigen::Isometry3d& pose,
              const GroundSegmentationResult& seg);

  // Core overload (no glim frame dependency); indices in `seg` refer to `points`.
  void update(const std::vector<Eigen::Vector4d>& points,
              double stamp,
              const Eigen::Isometry3d& pose,
              const GroundSegmentationResult& seg);

  SemanticVoxel::Ptr query(const Eigen::Vector3d& point) const;

  std::vector<SemanticVoxel> get_submap(const Eigen::Vector3d& center, double radius) const;

  std::vector<SemanticVoxel> get_submap_with_meta(
    const Eigen::Vector3d& center,
    double radius,
    SubmapMeta& meta) const;

  // Stage-4 dynamic-mask write-back: marks voxels inside each track box as DYNAMIC_UNKNOWN and
  // excludes them from later static updates.
  void remove_dynamic_objects(const std::vector<ObjectTrack>& tracks);

  void apply_dynamic_mask(const DynamicMask& mask);

  // Merge restored/archived voxels (positions come from SemanticVoxel::position).
  void add_voxels(const std::vector<SemanticVoxel>& voxels);

  // Remove voxels farther than keep_radius from center (aging).
  void prune(const Eigen::Vector3d& center, double keep_radius);

  // Scale every voxel's occupancy by `factor` in [0, 1] (submap-switch weight decay).
  void decay_weights(float factor);

  size_t size() const;

  void set_meta(const SubmapMeta& meta);
  SubmapMeta meta() const;
  const Config& config() const { return cfg_; }

private:
  struct VoxelKey {
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;
    int32_t level = 0;   // 0 = near, 1 = far
    bool operator==(const VoxelKey& other) const {
      return x == other.x && y == other.y && z == other.z && level == other.level;
    }
  };

  struct VoxelKeyHash {
    size_t operator()(const VoxelKey& key) const;
  };

  VoxelKey key_of(const Eigen::Vector3d& world, int level) const;
  float resolution_of(int level) const;
  SemanticVoxel::Ptr get_or_create(const VoxelKey& key);
  void mark_region(const Eigen::Vector3f& min, const Eigen::Vector3f& max);

  Config cfg_;
  mutable std::mutex mutex_;
  std::unordered_map<VoxelKey, SemanticVoxel::Ptr, VoxelKeyHash> voxels_;
  SubmapMeta meta_;
};

}  // namespace glim
