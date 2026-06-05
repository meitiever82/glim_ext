#include <gtest/gtest.h>
#include <glim_ext/prior_map_manager.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

using glim::PriorMapManager;

static std::vector<Eigen::Vector4d> make_plane(int n, double z) {
  std::vector<Eigen::Vector4d> pts;
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j)
      pts.emplace_back(i * 0.1, j * 0.1, z, 1.0);
  return pts;
}

TEST(PriorMapManager, BuildsVoxelmap) {
  PriorMapManager::Config cfg;
  cfg.voxel_resolution = 0.5;
  PriorMapManager mgr(cfg);
  mgr.build_from_points(make_plane(50, 0.0));   // 2500 points on a 5m x 5m plane

  ASSERT_NE(mgr.voxelmap(), nullptr);
  EXPECT_NEAR(mgr.voxelmap()->voxel_resolution(), 0.5, 1e-9);
}

TEST(PriorMapManager, EmptyInputNoCrash) {
  PriorMapManager::Config cfg;
  PriorMapManager mgr(cfg);
  mgr.build_from_points({});                       // must not crash
  EXPECT_EQ(mgr.voxelmap(), nullptr);
  EXPECT_FALSE(mgr.query_ground_height(0.0, 0.0).has_value());
}

TEST(PriorMapManager, GroundHeightOnSlope) {
  PriorMapManager::Config cfg;
  cfg.voxel_resolution = 0.5;
  cfg.ground_grid_resolution = 1.0;
  PriorMapManager mgr(cfg);

  // Sloped floor: z = 0.1 * x, over x in [0,10), y in [0,5)
  std::vector<Eigen::Vector4d> pts;
  for (double x = 0; x < 10; x += 0.2)
    for (double y = 0; y < 5; y += 0.2)
      pts.emplace_back(x, y, 0.1 * x, 1.0);
  mgr.build_from_points(pts);

  auto z0 = mgr.query_ground_height(0.5, 2.5);
  auto z8 = mgr.query_ground_height(8.5, 2.5);
  ASSERT_TRUE(z0.has_value());
  ASSERT_TRUE(z8.has_value());
  EXPECT_NEAR(*z0, 0.0, 0.3);   // near x=0
  EXPECT_NEAR(*z8, 0.8, 0.3);   // near x=8
  EXPECT_FALSE(mgr.query_ground_height(100.0, 100.0).has_value());  // outside map
}

TEST(PriorMapManager, OverlapRatio) {
  PriorMapManager::Config cfg;
  cfg.voxel_resolution = 0.5;
  PriorMapManager mgr(cfg);

  std::vector<Eigen::Vector4d> map_pts;
  for (double x = 0; x < 5; x += 0.1)
    for (double y = 0; y < 5; y += 0.1)
      map_pts.emplace_back(x, y, 0.0, 1.0);
  mgr.build_from_points(map_pts);

  // A scan identical to a patch of the map, at identity -> overlap ~ 1
  std::vector<Eigen::Vector4d> scan_pts;
  for (double x = 1; x < 3; x += 0.1)
    for (double y = 1; y < 3; y += 0.1)
      scan_pts.emplace_back(x, y, 0.0, 1.0);
  auto scan = std::make_shared<gtsam_points::PointCloudCPU>(scan_pts);

  double ov_in = mgr.compute_overlap_ratio(scan, Eigen::Isometry3d::Identity());
  EXPECT_GT(ov_in, 0.9);

  Eigen::Isometry3d far = Eigen::Isometry3d::Identity();
  far.translation() = Eigen::Vector3d(100, 100, 0);
  double ov_out = mgr.compute_overlap_ratio(scan, far);
  EXPECT_LT(ov_out, 0.05);
}
