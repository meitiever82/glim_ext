#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim_ext/semantic/linefit_ground_segmentation.hpp>
#include <glim_ext/semantic/semantic_voxel_map.hpp>
#include <glim_ext/semantic/submap_manager.hpp>

namespace {

using glim::GroundSegmentationResult;
using glim::LineFitGroundSegmentation;
using glim::SemanticLabel;
using glim::SemanticVoxel;
using glim::SemanticVoxelMap;
using glim::SubmapManager;

Eigen::Vector4d pt(double x, double y, double z) {
  return Eigen::Vector4d(x, y, z, 1.0);
}

// Flat ground disc (z = 0) plus a vertical wall at x = 5.
std::vector<Eigen::Vector4d> flat_scan_with_wall() {
  std::vector<Eigen::Vector4d> points;
  for (double r = 1.0; r <= 10.0; r += 0.5) {
    for (int a = 0; a < 120; ++a) {
      const double th = -M_PI + 2.0 * M_PI * a / 120.0;
      points.push_back(pt(r * std::cos(th), r * std::sin(th), 0.0));
    }
  }
  for (double y = -5.0; y <= 5.0; y += 0.5) {
    for (double z = 0.5; z <= 3.0; z += 0.5) {
      points.push_back(pt(5.0, y, z));
    }
  }
  return points;
}

}  // namespace

TEST(LineFitGroundSegmentation, FlatGroundIsLabeledGround) {
  LineFitGroundSegmentation seg;
  GroundSegmentationResult result;
  const auto points = flat_scan_with_wall();

  seg.segment(points, Eigen::Isometry3d::Identity(), result);

  // 2280 ground samples, 5*6=30 wall samples.
  EXPECT_GE(result.ground_indices.size(), 2000u);
  EXPECT_LE(result.ground_indices.size(), 2280u + 30u);
  EXPECT_GE(result.non_ground_indices.size(), 20u);
  EXPECT_GT(result.confidence, 0.5f);

  EXPECT_NEAR(result.plane.normal.x(), 0.0f, 1e-3f);
  EXPECT_NEAR(result.plane.normal.y(), 0.0f, 1e-3f);
  EXPECT_NEAR(result.plane.normal.z(), 1.0f, 1e-3f);
  EXPECT_NEAR(result.plane.d, 0.0f, 1e-2f);
}

TEST(LineFitGroundSegmentation, RampIsNotFlipped) {
  LineFitGroundSegmentation seg;
  GroundSegmentationResult result;

  std::vector<Eigen::Vector4d> points;
  for (double r = 1.0; r <= 20.0; r += 0.5) {
    for (int a = 0; a < 120; ++a) {
      const double th = -M_PI + 2.0 * M_PI * a / 120.0;
      // 10% ramp along +x: slope stays below max_slope.
      points.push_back(pt(r * std::cos(th), r * std::sin(th), 0.1 * r * std::cos(th)));
    }
  }

  seg.segment(points, Eigen::Isometry3d::Identity(), result);

  const double ratio = static_cast<double>(result.ground_indices.size()) / points.size();
  EXPECT_GT(ratio, 0.9);
  EXPECT_GT(result.confidence, 0.5f);
  EXPECT_LT(result.plane.normal.z(), 1.0f);
}

TEST(LineFitGroundSegmentation, TooFewGroundPointsReportsInvalid) {
  LineFitGroundSegmentation::Config cfg;
  cfg.min_ground_points = 5000;
  LineFitGroundSegmentation seg(cfg);

  GroundSegmentationResult result;
  seg.segment(flat_scan_with_wall(), Eigen::Isometry3d::Identity(), result);

  EXPECT_EQ(result.confidence, 0.0f);
  EXPECT_EQ(result.plane.confidence, 0.0f);
  // Points are still labelled so the voxel map can consume them.
  EXPECT_FALSE(result.ground_indices.empty());
}

TEST(SemanticVoxelMap, UpdateAndQuery) {
  SemanticVoxelMap::Config cfg;
  cfg.near_resolution = 0.15f;
  SemanticVoxelMap map(cfg);

  GroundSegmentationResult seg;
  const std::vector<Eigen::Vector4d> points = {pt(0.3, 0.4, 0.0), pt(0.3, 0.4, 1.0)};
  seg.ground_indices = {0};
  seg.non_ground_indices = {1};
  seg.plane.normal = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
  seg.plane.d = 0.0f;
  seg.plane.confidence = 0.9f;
  seg.confidence = 0.9f;

  map.update(points, 1.0, Eigen::Isometry3d::Identity(), seg);

  auto ground = map.query(Eigen::Vector3d(0.3, 0.4, 0.0));
  ASSERT_NE(ground, nullptr);
  EXPECT_EQ(ground->semantic_label, static_cast<uint8_t>(SemanticLabel::GROUND));
  EXPECT_EQ(ground->update_count, 1u);
  EXPECT_NEAR(ground->plane_normal.z(), 1.0f, 1e-6f);
  EXPECT_NEAR(ground->last_update_time, 1.0, 1e-9);

  auto above = map.query(Eigen::Vector3d(0.3, 0.4, 1.0));
  ASSERT_NE(above, nullptr);
  EXPECT_EQ(above->semantic_label, static_cast<uint8_t>(SemanticLabel::NON_GROUND_STATIC));

  EXPECT_EQ(map.meta().stamp, 1.0);
}

TEST(SemanticVoxelMap, GetSubmapAppliesRadius) {
  SemanticVoxelMap::Config cfg;
  cfg.near_resolution = 0.5f;
  cfg.far_resolution = 0.5f;
  SemanticVoxelMap map(cfg);

  GroundSegmentationResult seg;
  const std::vector<Eigen::Vector4d> points = {pt(0.0, 0.0, 0.0), pt(10.0, 0.0, 0.0)};
  seg.ground_indices = {0, 1};
  seg.confidence = 0.5f;
  map.update(points, 0.5, Eigen::Isometry3d::Identity(), seg);

  EXPECT_EQ(map.get_submap(Eigen::Vector3d::Zero(), 2.0).size(), 1u);

  glim::SubmapMeta meta;
  const auto all = map.get_submap_with_meta(Eigen::Vector3d::Zero(), 20.0, meta);
  EXPECT_EQ(all.size(), map.size());
  EXPECT_EQ(meta.resolution, cfg.near_resolution);
}

TEST(SemanticVoxelMap, DynamicVoxelsAreNotOverwrittenByStaticUpdates) {
  SemanticVoxelMap::Config cfg;
  cfg.near_resolution = 0.2f;
  cfg.far_resolution = 0.2f;
  SemanticVoxelMap map(cfg);

  GroundSegmentationResult seg;
  const std::vector<Eigen::Vector4d> points = {pt(1.0, 1.0, 0.0)};
  seg.ground_indices = {0};
  seg.confidence = 0.5f;
  map.update(points, 1.0, Eigen::Isometry3d::Identity(), seg);

  glim::ObjectTrack track;
  track.pose.translation() = Eigen::Vector3d(1.0, 1.0, 0.0);
  track.size = Eigen::Vector3f(1.0f, 1.0f, 1.0f);
  map.remove_dynamic_objects({track});

  auto voxel = map.query(Eigen::Vector3d(1.0, 1.0, 0.0));
  ASSERT_NE(voxel, nullptr);
  EXPECT_EQ(voxel->semantic_label, static_cast<uint8_t>(SemanticLabel::DYNAMIC_UNKNOWN));
  const uint32_t count = voxel->update_count;

  map.update(points, 2.0, Eigen::Isometry3d::Identity(), seg);

  voxel = map.query(Eigen::Vector3d(1.0, 1.0, 0.0));
  ASSERT_NE(voxel, nullptr);
  EXPECT_EQ(voxel->semantic_label, static_cast<uint8_t>(SemanticLabel::DYNAMIC_UNKNOWN));
  EXPECT_EQ(voxel->update_count, count);
  EXPECT_EQ(voxel->last_update_time, 2.0);
}

TEST(SubmapManager, RecenterBumpsVersionAndDecays) {
  SemanticVoxelMap::Config cfg;
  cfg.near_resolution = 0.5f;
  cfg.far_resolution = 0.5f;
  SubmapManager manager(cfg, 20.0, "test_vehicle", 3);

  EXPECT_EQ(manager.version(), 0u);
  manager.set_center(Eigen::Vector3d::Zero());
  EXPECT_EQ(manager.version(), 0u);

  SemanticVoxel voxel;
  voxel.position = Eigen::Vector3f(12.0f, 0.0f, 0.0f);
  voxel.occupancy_prob = 0.8f;
  voxel.semantic_label = static_cast<uint8_t>(SemanticLabel::GROUND);
  manager.add_voxels({voxel});

  // Move 21m: beyond 0.5 * 20m, so the submap recenters, decays and prunes (radius 20).
  manager.set_center(Eigen::Vector3d(21.0, 0.0, 0.0));
  EXPECT_EQ(manager.version(), 1u);

  const auto voxels = manager.get_active_submap()->get_submap(Eigen::Vector3d(21.0, 0.0, 0.0), 20.0);
  ASSERT_EQ(voxels.size(), 1u);
  EXPECT_NEAR(voxels.front().occupancy_prob, 0.8f * 0.95f, 1e-5f);

  glim::SubmapMeta meta;
  manager.get_active_submap(&meta);
  EXPECT_EQ(meta.vehicle_id, "test_vehicle");
  EXPECT_EQ(meta.extrinsic_version, 3u);
  EXPECT_EQ(meta.version, 1u);
  EXPECT_NEAR(meta.anchor_pose.translation().x(), 21.0, 1e-9);
  EXPECT_EQ(meta.resolution, cfg.near_resolution);
}

TEST(SubmapManager, PruneDropsVoxelsOutsideRadius) {
  SemanticVoxelMap::Config cfg;
  cfg.near_resolution = 0.5f;
  cfg.far_resolution = 0.5f;
  SubmapManager manager(cfg, 20.0);

  manager.set_center(Eigen::Vector3d(10.0, 0.0, 0.0));

  SemanticVoxel near;
  near.position = Eigen::Vector3f(8.0f, 0.0f, 0.0f);
  SemanticVoxel far;
  far.position = Eigen::Vector3f(25.0f, 0.0f, 0.0f);
  manager.add_voxels({near, far});
  EXPECT_EQ(manager.get_active_submap()->size(), 2u);

  manager.prune(5.0);
  EXPECT_EQ(manager.get_active_submap()->size(), 1u);

  auto kept = manager.get_active_submap()->query(Eigen::Vector3d(8.0, 0.0, 0.0));
  EXPECT_NE(kept, nullptr);
  EXPECT_EQ(manager.get_active_submap()->query(Eigen::Vector3d(25.0, 0.0, 0.0)), nullptr);
}
