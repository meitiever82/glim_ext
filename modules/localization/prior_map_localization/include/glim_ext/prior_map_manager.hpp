#pragma once

#include <string>
#include <optional>
#include <vector>
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtsam_points/types/point_cloud.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>

namespace glim {

class PriorMapManager {
public:
  struct Config {
    double voxel_resolution = 0.5;
    int cov_k_neighbors = 10;
    int num_threads = 4;
    double ground_grid_resolution = 1.0;
    double max_corr_dist = 1.0;  // reserved; overlap uses voxel occupancy
  };

  explicit PriorMapManager(const Config& config);

  // Build from in-memory points (homogeneous x,y,z,1). Releases the raw cloud after building.
  void build_from_points(const std::vector<Eigen::Vector4d>& points);

  // Load a .pcd via PCL, then build_from_points. Returns false on read failure / empty.
  bool load_map(const std::string& path);

  gtsam_points::GaussianVoxelMapCPU::ConstPtr voxelmap() const { return voxelmap_; }

  // Ground height at (x,y) from the 2D height grid. std::nullopt if no cell.
  std::optional<double> query_ground_height(double x, double y) const;

  // Fraction of `cloud` points (mapped to map frame via T_map_sensor) hitting an occupied voxel.
  double compute_overlap_ratio(const gtsam_points::PointCloud::ConstPtr& cloud,
                               const Eigen::Isometry3d& T_map_sensor) const;

  bool is_in_bounds(const Eigen::Vector3d& p) const;

private:
  Config config_;
  gtsam_points::GaussianVoxelMapCPU::Ptr voxelmap_;
  std::unordered_map<long long, double> height_grid_;  // key = grid_key(ix,iy)
  Eigen::Vector3d map_min_, map_max_;

  long long grid_key(int ix, int iy) const;
};

}  // namespace glim
