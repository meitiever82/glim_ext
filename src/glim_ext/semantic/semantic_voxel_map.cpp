#include "glim_ext/semantic/semantic_voxel_map.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace glim {

namespace {

int32_t coord_index(double value, float resolution) {
  return static_cast<int32_t>(std::floor(value / static_cast<double>(resolution)));
}

}  // namespace

size_t SemanticVoxelMap::VoxelKeyHash::operator()(const VoxelKey& key) const {
  // FNV-1a over the packed coordinates.
  uint64_t h = 1469598103934665603ull;
  const uint64_t parts[4] = {
    static_cast<uint64_t>(static_cast<uint32_t>(key.x)),
    static_cast<uint64_t>(static_cast<uint32_t>(key.y)),
    static_cast<uint64_t>(static_cast<uint32_t>(key.z)),
    static_cast<uint64_t>(static_cast<uint32_t>(key.level))};
  for (uint64_t p : parts) {
    h ^= p;
    h *= 1099511628211ull;
  }
  return static_cast<size_t>(h);
}

SemanticVoxelMap::SemanticVoxelMap() : SemanticVoxelMap(Config(), SubmapMeta()) {}

SemanticVoxelMap::SemanticVoxelMap(const Config& cfg, const SubmapMeta& meta) : cfg_(cfg), meta_(meta) {
  if (!(cfg_.near_resolution > 0.0f) || !(cfg_.far_resolution > 0.0f)) {
    throw std::invalid_argument("SemanticVoxelMap: near_resolution/far_resolution must be > 0");
  }
  if (cfg_.far_resolution < cfg_.near_resolution) {
    throw std::invalid_argument("SemanticVoxelMap: far_resolution must be >= near_resolution");
  }
  if (!(cfg_.far_switch_distance > 0.0f)) {
    throw std::invalid_argument("SemanticVoxelMap: far_switch_distance must be > 0");
  }
  meta_.resolution = cfg_.near_resolution;
}

float SemanticVoxelMap::resolution_of(int level) const {
  return level == 0 ? cfg_.near_resolution : cfg_.far_resolution;
}

SemanticVoxelMap::VoxelKey SemanticVoxelMap::key_of(const Eigen::Vector3d& world, int level) const {
  const float res = resolution_of(level);
  VoxelKey key;
  key.x = coord_index(world.x(), res);
  key.y = coord_index(world.y(), res);
  key.z = coord_index(world.z(), res);
  key.level = level;
  return key;
}

SemanticVoxel::Ptr SemanticVoxelMap::get_or_create(const VoxelKey& key) {
  auto found = voxels_.find(key);
  if (found != voxels_.end()) {
    return found->second;
  }
  const float res = resolution_of(key.level);
  auto voxel = std::make_shared<SemanticVoxel>();
  voxel->position = Eigen::Vector3f(
    (key.x + 0.5f) * res,
    (key.y + 0.5f) * res,
    (key.z + 0.5f) * res);
  auto inserted = voxels_.emplace(key, voxel);
  return inserted.first->second;
}

void SemanticVoxelMap::update(
  const PreprocessedFrame& frame,
  const Eigen::Isometry3d& pose,
  const GroundSegmentationResult& seg) {
  update(frame.points, frame.stamp, pose, seg);
}

void SemanticVoxelMap::update(
  const std::vector<Eigen::Vector4d>& points,
  double stamp,
  const Eigen::Isometry3d& pose,
  const GroundSegmentationResult& seg) {
  std::lock_guard<std::mutex> lock(mutex_);

  const Eigen::Vector3d sensor = pose.translation();
  const bool plane_valid = seg.plane.confidence > 0.0f;
  const Eigen::Vector3f plane_normal = seg.plane.normal;
  const float plane_d = seg.plane.d;

  auto accumulate = [&](uint32_t idx, bool ground) {
    if (idx >= points.size()) {
      return;
    }
    const Eigen::Vector4d& p = points[idx];
    if (!p.allFinite()) {
      return;
    }
    const Eigen::Vector3d world = pose * Eigen::Vector3d(p.x(), p.y(), p.z());
    if (!world.allFinite()) {
      return;
    }

    const double distance = (world - sensor).norm();
    const int level = distance > static_cast<double>(cfg_.far_switch_distance) ? 1 : 0;
    auto voxel = get_or_create(key_of(world, level));

    // Dynamic-masked voxels do not take part in static map updates (spec 2.2 §4).
    if (voxel->semantic_label == static_cast<uint8_t>(SemanticLabel::DYNAMIC_UNKNOWN)) {
      voxel->last_update_time = stamp;
      return;
    }

    voxel->semantic_label = static_cast<uint8_t>(
      ground ? SemanticLabel::GROUND : SemanticLabel::NON_GROUND_STATIC);

    const uint32_t n = voxel->update_count + 1;
    const float z = static_cast<float>(world.z());
    const float delta = z - voxel->height_mean;
    voxel->height_mean += delta / static_cast<float>(n);
    voxel->height_var += delta * (z - voxel->height_mean);
    voxel->height_var /= static_cast<float>(n);
    voxel->update_count = n;
    voxel->last_update_time = stamp;

    // Hit evidence accumulates geometrically; the exact semantics are not used downstream yet.
    voxel->occupancy_prob = 1.0f - (1.0f - voxel->occupancy_prob) * 0.6f;

    if (ground && plane_valid) {
      voxel->plane_normal = plane_normal;
      voxel->plane_d = plane_d;
    }
  };

  for (uint32_t idx : seg.ground_indices) {
    accumulate(idx, true);
  }
  for (uint32_t idx : seg.non_ground_indices) {
    accumulate(idx, false);
  }

  if (stamp > meta_.stamp) {
    meta_.stamp = stamp;
  }
}

SemanticVoxel::Ptr SemanticVoxelMap::query(const Eigen::Vector3d& point) const {
  std::lock_guard<std::mutex> lock(mutex_);
  for (int level = 0; level <= 1; ++level) {
    const auto found = voxels_.find(key_of(point, level));
    if (found != voxels_.end()) {
      return found->second;
    }
  }
  return nullptr;
}

std::vector<SemanticVoxel> SemanticVoxelMap::get_submap(const Eigen::Vector3d& center, double radius) const {
  SubmapMeta ignored;
  return get_submap_with_meta(center, radius, ignored);
}

std::vector<SemanticVoxel> SemanticVoxelMap::get_submap_with_meta(
  const Eigen::Vector3d& center,
  double radius,
  SubmapMeta& meta) const {
  std::lock_guard<std::mutex> lock(mutex_);
  meta = meta_;

  std::vector<SemanticVoxel> out;
  if (!(radius > 0.0)) {
    return out;
  }
  const double radius2 = radius * radius;
  out.reserve(voxels_.size() / 4);
  for (const auto& kv : voxels_) {
    const Eigen::Vector3f& p = kv.second->position;
    const Eigen::Vector3d d = p.cast<double>() - center;
    if (d.squaredNorm() <= radius2) {
      out.push_back(*kv.second);
    }
  }
  return out;
}

void SemanticVoxelMap::add_voxels(const std::vector<SemanticVoxel>& voxels) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& v : voxels) {
    if (!v.position.allFinite()) {
      continue;
    }
    // Restored voxels go to the near level; the stored cell center replaces their position.
    const Eigen::Vector3d world = v.position.cast<double>();
    auto voxel = get_or_create(key_of(world, 0));
    const Eigen::Vector3f cell_center = voxel->position;
    *voxel = v;
    voxel->position = cell_center;
  }
}

void SemanticVoxelMap::prune(const Eigen::Vector3d& center, double keep_radius) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!(keep_radius > 0.0)) {
    return;
  }
  const double radius2 = keep_radius * keep_radius;
  for (auto it = voxels_.begin(); it != voxels_.end();) {
    const Eigen::Vector3d d = it->second->position.cast<double>() - center;
    if (d.squaredNorm() > radius2) {
      it = voxels_.erase(it);
    } else {
      ++it;
    }
  }
}

void SemanticVoxelMap::decay_weights(float factor) {
  std::lock_guard<std::mutex> lock(mutex_);
  const float f = std::clamp(factor, 0.0f, 1.0f);
  for (auto& kv : voxels_) {
    kv.second->occupancy_prob *= f;
  }
}

size_t SemanticVoxelMap::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return voxels_.size();
}

void SemanticVoxelMap::set_meta(const SubmapMeta& meta) {
  std::lock_guard<std::mutex> lock(mutex_);
  meta_ = meta;
  meta_.resolution = cfg_.near_resolution;
}

SubmapMeta SemanticVoxelMap::meta() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return meta_;
}

void SemanticVoxelMap::mark_region(const Eigen::Vector3f& min, const Eigen::Vector3f& max) {
  for (auto& kv : voxels_) {
    const Eigen::Vector3f& p = kv.second->position;
    if ((p.array() >= min.array()).all() && (p.array() <= max.array()).all()) {
      kv.second->semantic_label = static_cast<uint8_t>(SemanticLabel::DYNAMIC_UNKNOWN);
    }
  }
}

void SemanticVoxelMap::remove_dynamic_objects(const std::vector<ObjectTrack>& tracks) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& track : tracks) {
    const Eigen::Vector3f center = track.pose.translation().cast<float>();
    const Eigen::Vector3f half = track.size.cwiseAbs() * 0.5f;
    mark_region(center - half, center + half);
  }
}

void SemanticVoxelMap::apply_dynamic_mask(const DynamicMask& mask) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& region : mask.regions) {
    mark_region(region.min, region.max);
  }
}

}  // namespace glim
