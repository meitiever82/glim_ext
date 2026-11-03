#pragma once
// Stage-2 (2.2) geometric ground segmentation — LineFit (spec 2.2 §3).
//
// LineFit splits the scan into azimuth sectors and radial bins, fits a line z = a*r + b
// per sector on the lowest point of each bin (trimmed least squares), and classifies
// points close to the line as ground. It handles slopes/uneven mine floors better than a
// single global plane and is cheap enough for a 10 Hz LiDAR stream.
//
// Pure compute, no ROS: the shell (semantic_map module) feeds sensor-frame points and
// receives indices plus a world-frame plane fit.

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "glim_ext/semantic/semantic_types.hpp"

namespace glim {

class LineFitGroundSegmentation {
public:
  struct Config {
    int n_sectors = 60;              // azimuth bins over [-pi, pi)
    double range_bin_width = 0.5;    // m; radial bin width
    double min_range = 1.0;          // m; ignore points closer than this
    double max_range = 60.0;         // m; ignore points farther than this
    double max_line_dist = 0.15;     // m; |z - line| within this counts as ground
    double max_slope = 0.35;         // |dz/dr|; steeper sectors are not treated as ground
    double plane_inlier_dist = 0.2;  // m; inlier distance for the world-frame plane fit
    size_t min_ground_points = 100;  // below this the result is reported as invalid (confidence 0)
    int line_fit_iterations = 2;     // trimmed least-squares iterations per sector
  };

  LineFitGroundSegmentation() = default;
  explicit LineFitGroundSegmentation(const Config& cfg) : cfg_(cfg) {}

  // points: sensor-frame homogeneous points (x, y, z, 1). T_world_sensor: pose of the
  // sensor frame in the map/world frame. Output indices refer to `points`.
  void segment(const std::vector<Eigen::Vector4d>& points,
               const Eigen::Isometry3d& T_world_sensor,
               GroundSegmentationResult& result) const;

  const Config& config() const { return cfg_; }

private:
  Config cfg_;
};

}  // namespace glim
