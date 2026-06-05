#include <glim_ext/prior_map_manager.hpp>

#include <cmath>
#include <limits>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/features/covariance_estimation.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

namespace glim {

PriorMapManager::PriorMapManager(const Config& config)
  : config_(config),
    map_min_(Eigen::Vector3d::Constant(std::numeric_limits<double>::max())),
    map_max_(Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest())) {}

long long PriorMapManager::grid_key(int ix, int iy) const {
  return (static_cast<long long>(ix) << 32) | static_cast<long long>(static_cast<unsigned int>(iy));
}

void PriorMapManager::build_from_points(const std::vector<Eigen::Vector4d>& points) {
  if (points.empty()) {
    voxelmap_.reset();
    height_grid_.clear();
    return;
  }
  auto covs = gtsam_points::estimate_covariances(points, config_.cov_k_neighbors, config_.num_threads);
  auto pc = std::make_shared<gtsam_points::PointCloudCPU>(points);
  pc->add_covs(covs);

  voxelmap_ = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(config_.voxel_resolution);
  voxelmap_->insert(*pc);

  height_grid_.clear();
  map_min_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
  map_max_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
  const double gr = config_.ground_grid_resolution;
  for (const auto& p : points) {
    map_min_ = map_min_.cwiseMin(p.head<3>());
    map_max_ = map_max_.cwiseMax(p.head<3>());
    int ix = static_cast<int>(std::floor(p.x() / gr));
    int iy = static_cast<int>(std::floor(p.y() / gr));
    long long k = grid_key(ix, iy);
    auto it = height_grid_.find(k);
    if (it == height_grid_.end() || p.z() < it->second) height_grid_[k] = p.z();
  }
}

bool PriorMapManager::load_map(const std::string& path) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, cloud) != 0) return false;
  std::vector<Eigen::Vector4d> points;
  points.reserve(cloud.size());
  for (const auto& p : cloud) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    points.emplace_back(p.x, p.y, p.z, 1.0);
  }
  if (points.empty()) return false;
  build_from_points(points);
  return true;
}

std::optional<double> PriorMapManager::query_ground_height(double x, double y) const {
  const double gr = config_.ground_grid_resolution;
  int ix = static_cast<int>(std::floor(x / gr));
  int iy = static_cast<int>(std::floor(y / gr));
  auto it = height_grid_.find(grid_key(ix, iy));
  if (it == height_grid_.end()) return std::nullopt;
  return it->second;
}

bool PriorMapManager::is_in_bounds(const Eigen::Vector3d& p) const {
  return (p.array() >= map_min_.array()).all() && (p.array() <= map_max_.array()).all();
}

double PriorMapManager::compute_overlap_ratio(const gtsam_points::PointCloud::ConstPtr& cloud,
                                              const Eigen::Isometry3d& T_map_sensor) const {
  if (!voxelmap_ || !cloud || cloud->size() == 0) return 0.0;
  const Eigen::Matrix4d T = T_map_sensor.matrix();
  int hits = 0;
  for (size_t i = 0; i < cloud->size(); ++i) {
    const Eigen::Vector4d& p = cloud->points[i];
    const Eigen::Vector4d pm = T * Eigen::Vector4d(p.x(), p.y(), p.z(), 1.0);
    const Eigen::Vector3i c = voxelmap_->voxel_coord(pm);
    if (voxelmap_->lookup_voxel_index(c) >= 0) ++hits;
  }
  return static_cast<double>(hits) / static_cast<double>(cloud->size());
}

}  // namespace glim
