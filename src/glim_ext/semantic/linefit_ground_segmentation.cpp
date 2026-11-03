#include "glim_ext/semantic/linefit_ground_segmentation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Eigenvalues>

namespace glim {

namespace {

struct PointInfo {
  uint32_t sector = 0;
  uint32_t bin = 0;
  float range = 0.0f;
  float z = 0.0f;
  bool usable = false;   // within [min_range, max_range]
};

struct BinRep {
  float range = std::numeric_limits<float>::max();
  float z = 0.0f;
  bool has = false;
};

// z = a*r + b over the bin representatives; returns false when there is nothing to fit.
bool fit_line(const std::vector<BinRep>& reps, int iterations, double max_dist, double& a, double& b) {
  std::vector<const BinRep*> used;
  for (const auto& r : reps) {
    if (r.has) {
      used.push_back(&r);
    }
  }
  if (used.size() < 2) {
    return false;
  }

  for (int it = 0; it <= iterations; ++it) {
    double sr = 0.0, sz = 0.0, srr = 0.0, srz = 0.0;
    double n = 0.0;
    for (const auto* p : used) {
      n += 1.0;
      sr += p->range;
      sz += p->z;
      srr += static_cast<double>(p->range) * p->range;
      srz += static_cast<double>(p->range) * p->z;
    }
    const double denom = n * srr - sr * sr;
    if (std::abs(denom) < 1e-9) {
      return false;
    }
    a = (n * srz - sr * sz) / denom;
    b = (sz - a * sr) / n;

    if (it == iterations) {
      break;
    }

    // Trim: keep representatives within 3x the ground tolerance so a ramp inliers dominate
    // while isolated returns (walls, vegetation) are removed before the next fit.
    const double trim = std::max(max_dist, 1e-3) * 3.0;
    std::vector<const BinRep*> keep;
    for (const auto* p : used) {
      if (std::abs(p->z - (a * p->range + b)) <= trim) {
        keep.push_back(p);
      }
    }
    if (keep.size() < 2 || keep.size() == used.size()) {
      used.swap(keep);
      break;
    }
    used.swap(keep);
  }
  return true;
}

}  // namespace

void LineFitGroundSegmentation::segment(
  const std::vector<Eigen::Vector4d>& points,
  const Eigen::Isometry3d& T_world_sensor,
  GroundSegmentationResult& result) const {
  result.ground_indices.clear();
  result.non_ground_indices.clear();
  result.confidence = 0.0f;
  result.plane = GroundPlaneParams();

  if (cfg_.n_sectors <= 0 || points.empty()) {
    return;
  }

  const double max_range = std::max(cfg_.max_range, cfg_.min_range);
  const double n_bins = std::max(1.0, std::ceil((max_range - cfg_.min_range) / std::max(cfg_.range_bin_width, 1e-3)));
  const double sector_width = 2.0 * M_PI / cfg_.n_sectors;

  std::vector<PointInfo> info(points.size());
  std::vector<std::vector<BinRep>> reps(
    static_cast<size_t>(cfg_.n_sectors), std::vector<BinRep>(static_cast<size_t>(n_bins)));

  size_t sector_usable = 0;
  for (size_t i = 0; i < points.size(); ++i) {
    const Eigen::Vector4d& p = points[i];
    const double range = std::hypot(p.x(), p.y());
    PointInfo& pi = info[i];
    pi.range = static_cast<float>(range);
    pi.z = static_cast<float>(p.z());
    if (!std::isfinite(range) || !std::isfinite(p.z()) || range < cfg_.min_range || range >= max_range) {
      continue;
    }

    double azimuth = std::atan2(p.y(), p.x());
    int sector = static_cast<int>(std::floor((azimuth + M_PI) / sector_width));
    sector = std::min(std::max(sector, 0), cfg_.n_sectors - 1);
    int bin = static_cast<int>(std::floor((range - cfg_.min_range) / std::max(cfg_.range_bin_width, 1e-3)));
    bin = std::min(std::max(bin, 0), static_cast<int>(n_bins) - 1);

    pi.sector = static_cast<uint32_t>(sector);
    pi.bin = static_cast<uint32_t>(bin);
    pi.usable = true;

    BinRep& rep = reps[static_cast<size_t>(sector)][static_cast<size_t>(bin)];
    if (!rep.has || pi.z < rep.z) {
      rep.range = pi.range;
      rep.z = pi.z;
      rep.has = true;
    }
  }

  // Fit one line per sector and classify its points.
  std::vector<int> sector_ok(static_cast<size_t>(cfg_.n_sectors), 0);
  std::vector<double> slope(static_cast<size_t>(cfg_.n_sectors), 0.0);
  std::vector<double> intercept(static_cast<size_t>(cfg_.n_sectors), 0.0);
  for (int s = 0; s < cfg_.n_sectors; ++s) {
    double a = 0.0, b = 0.0;
    if (fit_line(reps[static_cast<size_t>(s)], cfg_.line_fit_iterations, cfg_.max_line_dist, a, b) &&
        std::abs(a) <= cfg_.max_slope) {
      sector_ok[static_cast<size_t>(s)] = 1;
      slope[static_cast<size_t>(s)] = a;
      intercept[static_cast<size_t>(s)] = b;
    }
  }

  const double max_dist = std::max(cfg_.max_line_dist, 1e-3);
  for (size_t i = 0; i < points.size(); ++i) {
    const PointInfo& pi = info[i];
    bool ground = false;
    if (pi.usable && sector_ok[pi.sector]) {
      const double predicted = slope[pi.sector] * pi.range + intercept[pi.sector];
      ground = std::abs(pi.z - predicted) <= max_dist;
    }
    if (ground) {
      result.ground_indices.push_back(static_cast<uint32_t>(i));
    } else {
      result.non_ground_indices.push_back(static_cast<uint32_t>(i));
    }
  }

  const size_t total = points.size();
  const size_t ground = result.ground_indices.size();
  const double ground_ratio = total ? static_cast<double>(ground) / static_cast<double>(total) : 0.0;
  size_t fitted_sectors = 0;
  for (int ok : sector_ok) {
    fitted_sectors += static_cast<size_t>(ok);
  }
  const double sector_ratio = cfg_.n_sectors ? static_cast<double>(fitted_sectors) / static_cast<double>(cfg_.n_sectors) : 0.0;

  if (ground < cfg_.min_ground_points) {
    // Too few ground returns (open pit, sensor pointing up, very sparse scan): report invalid
    // rather than a bogus plane. Points are still labelled so the voxel map can use them.
    return;
  }

  // World-frame plane by PCA on the ground points.
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (uint32_t idx : result.ground_indices) {
    const Eigen::Vector4d& p = points[idx];
    const Eigen::Vector3d w = T_world_sensor * Eigen::Vector3d(p.x(), p.y(), p.z());
    mean += w;
  }
  mean /= static_cast<double>(ground);
  for (uint32_t idx : result.ground_indices) {
    const Eigen::Vector4d& p = points[idx];
    const Eigen::Vector3d w = T_world_sensor * Eigen::Vector3d(p.x(), p.y(), p.z());
    const Eigen::Vector3d d = w - mean;
    cov += d * d.transpose();
  }
  cov /= static_cast<double>(ground);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
  if (eig.info() != Eigen::Success) {
    return;
  }
  Eigen::Vector3d normal = eig.eigenvectors().col(0);
  if (normal.z() < 0.0) {
    normal = -normal;
  }
  normal.normalize();
  const double plane_d = -normal.dot(mean);

  size_t inliers = 0;
  for (uint32_t idx : result.ground_indices) {
    const Eigen::Vector4d& p = points[idx];
    const Eigen::Vector3d w = T_world_sensor * Eigen::Vector3d(p.x(), p.y(), p.z());
    if (std::abs(normal.dot(w) + plane_d) <= cfg_.plane_inlier_dist) {
      ++inliers;
    }
  }
  const double inlier_ratio = static_cast<double>(inliers) / static_cast<double>(ground);

  result.plane.normal = normal.cast<float>();
  result.plane.d = static_cast<float>(plane_d);
  result.confidence = static_cast<float>(std::clamp(0.5 * ground_ratio + 0.5 * sector_ratio * inlier_ratio, 0.0, 1.0));
  result.plane.confidence = result.confidence;
}

}  // namespace glim
