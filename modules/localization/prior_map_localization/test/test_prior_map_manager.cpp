#include <gtest/gtest.h>
#include <glim_ext/prior_map_manager.hpp>

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
