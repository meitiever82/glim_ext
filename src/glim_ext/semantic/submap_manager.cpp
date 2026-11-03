#include "glim_ext/semantic/submap_manager.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace glim {

SubmapManager::SubmapManager(const SemanticVoxelMap::Config& cfg,
                             double submap_radius,
                             std::string vehicle_id,
                             uint32_t extrinsic_version)
: submap_radius_(submap_radius) {
  if (!(submap_radius_ > 0.0)) {
    throw std::invalid_argument("SubmapManager: submap_radius must be > 0");
  }

  meta_.anchor_pose = Eigen::Isometry3d::Identity();
  meta_.stamp = 0.0;
  meta_.vehicle_id = std::move(vehicle_id);
  meta_.version = 0;
  meta_.extrinsic_version = extrinsic_version;

  map_ = std::make_shared<SemanticVoxelMap>(cfg, meta_);
}

void SubmapManager::set_center(const Eigen::Vector3d& center) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!center.allFinite()) {
    return;
  }
  center_ = center;

  const double offset = (center - meta_.anchor_pose.translation()).norm();
  if (offset <= recenter_ratio_ * submap_radius_) {
    return;
  }

  // Recentre: bump the version so stage 4/5 can tell the submap changed, then decay the
  // retained weights and prune everything that falls outside the new 100m window.
  meta_.anchor_pose.translation() = center;
  meta_.version += 1;
  map_->set_meta(meta_);
  map_->decay_weights(switch_decay_);
  map_->prune(center, submap_radius_);
}

void SubmapManager::add_voxels(const std::vector<SemanticVoxel>& voxels) {
  std::lock_guard<std::mutex> lock(mutex_);
  map_->add_voxels(voxels);
}

void SubmapManager::prune(double keep_radius) {
  std::lock_guard<std::mutex> lock(mutex_);
  map_->prune(center_, keep_radius);
}

SemanticVoxelMap::Ptr SubmapManager::get_active_submap(SubmapMeta* meta) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (meta) {
    *meta = meta_;
    meta->resolution = map_->config().near_resolution;
  }
  return map_;
}

void SubmapManager::update(const PreprocessedFrame& frame,
                           const Eigen::Isometry3d& pose,
                           const GroundSegmentationResult& seg) {
  set_center(pose.translation());
  auto map = get_active_submap();
  map->update(frame, pose, seg);
}

Eigen::Vector3d SubmapManager::center() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return center_;
}

double SubmapManager::submap_radius() const {
  return submap_radius_;
}

uint32_t SubmapManager::version() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return meta_.version;
}

}  // namespace glim
