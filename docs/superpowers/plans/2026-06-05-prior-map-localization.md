# prior_map_localization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a tightly-coupled prior-map localization glim_ext module (`libprior_map_localization.so`) that injects Scan-to-Prior-Map VGICP factors into GLIM's sliding-window factor graph, degrading to pure LIO when the map is missing.

**Architecture:** A `glim::ExtensionModuleROS2` subclass registers GLIM odometry callbacks (`on_new_frame`/`on_smoother_update`). On each frame it estimates the scan↔map overlap, and (when overlapping, outside a relocalization cooldown) builds a `gtsam_points::IntegratedVGICPFactor` keyed on the new frame pose `X(id)` with the prior map as fixed target, stashing it for flush in `on_smoother_update` (idiom mirrors `velocity_suppressor`). Localization runs in GLIM's `odom` frame; a constant `T_map_odom` (computed from a 6DoF-completed `/initialpose`) maps it to the prior map and is published as `map→odom` TF (做法 A).

**Tech Stack:** C++17, ROS2 (ament_cmake), GTSAM 4.x, gtsam_points 1.2.0, PCL (PCD loading), Eigen. Tests via `ament_cmake_gtest`.

**Spec:** `docs/superpowers/specs/2026-06-05-prior-map-localization-design.md` (read §9 "已核实源码事实" for verified APIs).

---

## Verified APIs (from spec §9 — use these exactly)

```cpp
// PCD → points (PCL is available via pcl_ros)
pcl::io::loadPCDFile<pcl::PointXYZ>(path, pcl_cloud);   // returns 0 on success
// points as homogeneous Vector4d (x,y,z,1)

// Covariances (no precomputed neighbors needed)
std::vector<Eigen::Matrix4d> gtsam_points::estimate_covariances(
    const std::vector<Eigen::Vector4d>& points, int k_neighbors = 10, int num_threads = 1);

// Point cloud + voxelmap
auto pc = std::make_shared<gtsam_points::PointCloudCPU>(points /*vector<Vector4d>*/);
pc->add_covs(covs);
auto vm = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
vm->insert(*pc);
Eigen::Vector3i c = vm->voxel_coord(Eigen::Vector4d);   // point → voxel coord
int idx = vm->lookup_voxel_index(c);                    // >= 0 means occupied

// S2M factor (fixed target, unary on source pose). NO noise model param.
gtsam_points::IntegratedVGICPFactor f(
    gtsam::Pose3 T_odom_map, gtsam::Key X(id),
    gtsam_points::GaussianVoxelMap::ConstPtr vm,
    gtsam_points::PointCloud::ConstPtr src);
f.set_num_threads(n);

// Random subsample (random, not truncation)
gtsam_points::PointCloudCPU::Ptr s =
    gtsam_points::random_sampling(cloud /*PointCloud::ConstPtr*/, ratio, mt /*std::mt19937&*/);

// GLIM callbacks (register in constructor)
glim::OdometryEstimationCallbacks::on_new_frame.add(std::bind(&Cls::on_new_frame, this, _1));
glim::OdometryEstimationCallbacks::on_smoother_update.add(std::bind(&Cls::on_smoother_update, this, _1,_2,_3,_4));
// signature: (IncrementalFixedLagSmootherExtWithFallback&, NonlinearFactorGraph& new_factors,
//             Values& new_values, std::map<uint64_t,double>& new_stamps)

// EstimationFrame fields: frame->id (long), frame->stamp (double),
//   frame->T_world_imu (Eigen::Isometry3d), frame->frame (gtsam_points::PointCloud::ConstPtr)

// Pose key
using gtsam::symbol_shorthand::X;   // X(frame->id)

// Config
glim::Config config(glim::GlobalConfigExt::get_config_path("config_prior_map_localization"));
double v = config.param<double>("prior_map_localization", "voxel_resolution", 0.5);

// ROS (ExtensionModuleROS2): create_subscriptions(rclcpp::Node& node) — also build publishers + TF here
auto sub = std::make_shared<glim::TopicSubscription<geometry_msgs::msg::PoseWithCovarianceStamped>>(
    "/initialpose", [this](const auto& msg){ ... });
tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);
pose_pub = node.create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 10);
```

---

## File Structure

```
src/glim_ext/
├── CMakeLists.txt                         # MODIFY: add option + add_subdirectory
├── package.xml                            # MODIFY: add ROS/PCL deps
├── config/config_prior_map_localization.json   # CREATE
└── modules/localization/prior_map_localization/
    ├── CMakeLists.txt                     # CREATE
    ├── include/glim_ext/
    │   ├── prior_map_manager.hpp          # CREATE  (map load, voxelmap, height grid, overlap)
    │   ├── inject_decision.hpp            # CREATE  (hysteresis state)
    │   ├── localization_math.hpp          # CREATE  (6DoF init pose, T_map_odom)
    │   ├── initial_pose_source.hpp        # CREATE  (/initialpose + default, thread-safe)
    │   └── prior_map_localization.hpp     # CREATE  (ExtensionModuleROS2)
    ├── src/glim_ext/
    │   ├── prior_map_manager.cpp
    │   ├── initial_pose_source.cpp
    │   └── prior_map_localization.cpp     # CREATE  (impl + create_extension_module)
    └── test/
        ├── test_prior_map_manager.cpp
        ├── test_inject_decision.cpp
        └── test_localization_math.cpp
```

**Build:** `colcon build --packages-select glim_ext` (from `/home/steve/mapping_ws`)
**Test:** `colcon test --packages-select glim_ext && colcon test-result --verbose`

> ⚠️ Pre-req: `src/glim_ext` git has an unmerged `CMakeLists.txt` conflict from a prior operation. Resolve it (or `git checkout --theirs/--ours` as appropriate) before building, otherwise the build/commit will fail.

---

### Task 0: Module scaffolding (compiles + loads as a stub)

**Files:**
- Create: `modules/localization/prior_map_localization/CMakeLists.txt`
- Create: `modules/localization/prior_map_localization/src/glim_ext/prior_map_localization.cpp`
- Create: `modules/localization/prior_map_localization/include/glim_ext/prior_map_localization.hpp`
- Modify: `CMakeLists.txt` (glim_ext root)
- Modify: `package.xml`

- [ ] **Step 1: Add deps to `package.xml`**

Insert after `<depend>glim</depend>`:

```xml
  <depend>rclcpp</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_eigen</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>pcl_ros</depend>
  <depend>libpcl-all-dev</depend>
  <test_depend>ament_cmake_gtest</test_depend>
```

- [ ] **Step 2: Add option + subdirectory to glim_ext root `CMakeLists.txt`**

Add near the other `option(ENABLE_*)` lines:

```cmake
option(ENABLE_PRIOR_MAP_LOC "Enable prior-map localization" ON)
```

Add near the other `add_subdirectory(modules/...)` lines:

```cmake
if(ENABLE_PRIOR_MAP_LOC)
  add_subdirectory(modules/localization/prior_map_localization)
  list(APPEND glim_ext_LIBRARIES prior_map_localization)
endif()
```

- [ ] **Step 3: Write the module `CMakeLists.txt`**

```cmake
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(gtsam_points REQUIRED)

add_library(prior_map_localization SHARED
  src/glim_ext/prior_map_manager.cpp
  src/glim_ext/initial_pose_source.cpp
  src/glim_ext/prior_map_localization.cpp
)
target_include_directories(prior_map_localization PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(prior_map_localization
  glim::glim
  gtsam_points::gtsam_points
  ${PCL_LIBRARIES}
)
ament_target_dependencies(prior_map_localization
  rclcpp tf2_ros tf2_eigen geometry_msgs sensor_msgs std_msgs
)
target_compile_options(prior_map_localization PRIVATE -std=c++17)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_prior_map_localization
    test/test_prior_map_manager.cpp
    test/test_inject_decision.cpp
    test/test_localization_math.cpp
  )
  target_link_libraries(test_prior_map_localization prior_map_localization)
endif()
```

- [ ] **Step 4: Write the stub header `prior_map_localization.hpp`**

```cpp
#pragma once

#include <glim/util/extension_module_ros2.hpp>

namespace glim {

class PriorMapLocalization : public ExtensionModuleROS2 {
public:
  PriorMapLocalization();
  ~PriorMapLocalization() override;

  std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override;
};

}  // namespace glim
```

- [ ] **Step 5: Write the stub impl `prior_map_localization.cpp`**

```cpp
#include <glim_ext/prior_map_localization.hpp>

#include <glim/util/logging.hpp>

namespace glim {

PriorMapLocalization::PriorMapLocalization() {
  auto logger = create_module_logger("prior_loc");
  logger->info("prior_map_localization stub loaded");
}

PriorMapLocalization::~PriorMapLocalization() {}

std::vector<GenericTopicSubscription::Ptr> PriorMapLocalization::create_subscriptions(rclcpp::Node& node) {
  return {};
}

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::PriorMapLocalization();
}
```

- [ ] **Step 6: Build to verify it compiles**

Run: `colcon build --packages-select glim_ext`
Expected: build succeeds; `install/glim_ext/lib/libprior_map_localization.so` exists.

- [ ] **Step 7: Commit**

```bash
git add modules/localization/prior_map_localization CMakeLists.txt package.xml
git commit -m "feat(prior_loc): scaffold prior_map_localization module"
```

---

### Task 1: PriorMapManager — load PCD, build GaussianVoxelMap

**Files:**
- Create: `include/glim_ext/prior_map_manager.hpp`
- Create: `src/glim_ext/prior_map_manager.cpp`
- Test: `test/test_prior_map_manager.cpp`

- [ ] **Step 1: Write the header**

```cpp
#pragma once

#include <optional>
#include <vector>
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

  // Load a .pcd/.ply via PCL, then build_from_points. Returns false on read failure.
  bool load_map(const std::string& path);

  gtsam_points::GaussianVoxelMapCPU::ConstPtr voxelmap() const { return voxelmap_; }

  // Ground height at (x,y) from the 2D height grid. std::nullopt if no cell.
  std::optional<double> query_ground_height(double x, double y) const;

  // Fraction of `cloud` points (in map frame, via T_map_sensor) that hit an occupied voxel.
  double compute_overlap_ratio(const gtsam_points::PointCloud::ConstPtr& cloud,
                               const Eigen::Isometry3d& T_map_sensor) const;

  bool is_in_bounds(const Eigen::Vector3d& p) const;

private:
  Config config_;
  gtsam_points::GaussianVoxelMapCPU::Ptr voxelmap_;
  std::unordered_map<long long, double> height_grid_;  // key = ix * STRIDE + iy
  Eigen::Vector3d map_min_, map_max_;

  long long grid_key(int ix, int iy) const;
};

}  // namespace glim
```

- [ ] **Step 2: Write the failing test for voxelmap build**

```cpp
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
```

- [ ] **Step 3: Run test to verify it fails**

Run: `colcon build --packages-select glim_ext && colcon test --packages-select glim_ext --ctest-args -R test_prior_map_localization`
Expected: FAIL — link error / `build_from_points` not implemented.

- [ ] **Step 4: Implement `prior_map_manager.cpp` (build path)**

```cpp
#include <glim_ext/prior_map_manager.hpp>

#include <cmath>
#include <limits>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/features/covariance_estimation.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

namespace glim {

PriorMapManager::PriorMapManager(const Config& config) : config_(config) {}

long long PriorMapManager::grid_key(int ix, int iy) const {
  return static_cast<long long>(ix) * 1000000LL + static_cast<long long>(iy);
}

void PriorMapManager::build_from_points(const std::vector<Eigen::Vector4d>& points) {
  // covariances + voxelmap
  auto covs = gtsam_points::estimate_covariances(points, config_.cov_k_neighbors, config_.num_threads);
  auto pc = std::make_shared<gtsam_points::PointCloudCPU>(points);
  pc->add_covs(covs);

  voxelmap_ = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(config_.voxel_resolution);
  voxelmap_->insert(*pc);

  // 2D height grid (lowest-z per cell) + bounds
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
  // raw `points` goes out of scope here — not retained.
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
  int hits = 0;
  for (size_t i = 0; i < cloud->size(); ++i) {
    Eigen::Vector4d ps = cloud->points[i];
    Eigen::Vector4d pm = T_map_sensor.matrix() * Eigen::Vector4d(ps.x(), ps.y(), ps.z(), 1.0);
    Eigen::Vector3i c = voxelmap_->voxel_coord(pm);
    if (voxelmap_->lookup_voxel_index(c) >= 0) ++hits;
  }
  return static_cast<double>(hits) / static_cast<double>(cloud->size());
}

}  // namespace glim
```

- [ ] **Step 5: Run test to verify it passes**

Run: `colcon build --packages-select glim_ext && colcon test --packages-select glim_ext --ctest-args -R test_prior_map_localization && colcon test-result --verbose`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add modules/localization/prior_map_localization/include modules/localization/prior_map_localization/src modules/localization/prior_map_localization/test
git commit -m "feat(prior_loc): PriorMapManager PCD load + GaussianVoxelMap"
```

---

### Task 2: PriorMapManager — query_ground_height on a slope

**Files:**
- Test: `test/test_prior_map_manager.cpp` (add cases)

- [ ] **Step 1: Write the failing test**

```cpp
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
```

- [ ] **Step 2: Run test to verify it fails or passes**

Run: `colcon build --packages-select glim_ext && colcon test --packages-select glim_ext --ctest-args -R test_prior_map_localization && colcon test-result --verbose`
Expected: PASS (implementation from Task 1 already covers this). If it fails, fix `query_ground_height`/grid binning until green.

- [ ] **Step 3: Commit**

```bash
git add modules/localization/prior_map_localization/test/test_prior_map_manager.cpp
git commit -m "test(prior_loc): ground-height query on slope"
```

---

### Task 3: PriorMapManager — overlap ratio

**Files:**
- Test: `test/test_prior_map_manager.cpp` (add cases)

- [ ] **Step 1: Write the failing test**

```cpp
#include <gtsam_points/types/point_cloud_cpu.hpp>

TEST(PriorMapManager, OverlapRatio) {
  PriorMapManager::Config cfg;
  cfg.voxel_resolution = 0.5;
  PriorMapManager mgr(cfg);

  std::vector<Eigen::Vector4d> map_pts;
  for (double x = 0; x < 5; x += 0.1)
    for (double y = 0; y < 5; y += 0.1)
      map_pts.emplace_back(x, y, 0.0, 1.0);
  mgr.build_from_points(map_pts);

  // A scan identical to a patch of the map, at identity → overlap ≈ 1
  std::vector<Eigen::Vector4d> scan_pts;
  for (double x = 1; x < 3; x += 0.1)
    for (double y = 1; y < 3; y += 0.1)
      scan_pts.emplace_back(x, y, 0.0, 1.0);
  auto scan = std::make_shared<gtsam_points::PointCloudCPU>(scan_pts);

  double ov_in = mgr.compute_overlap_ratio(scan, Eigen::Isometry3d::Identity());
  EXPECT_GT(ov_in, 0.9);

  // Same scan shifted 100m away → overlap ≈ 0
  Eigen::Isometry3d far = Eigen::Isometry3d::Identity();
  far.translation() = Eigen::Vector3d(100, 100, 0);
  double ov_out = mgr.compute_overlap_ratio(scan, far);
  EXPECT_LT(ov_out, 0.05);
}
```

- [ ] **Step 2: Run test to verify it passes**

Run: `colcon build --packages-select glim_ext && colcon test --packages-select glim_ext --ctest-args -R test_prior_map_localization && colcon test-result --verbose`
Expected: PASS. If `ov_in` is low, verify `voxel_coord`/`lookup_voxel_index` usage and that `add_covs` ran before `insert`.

- [ ] **Step 3: Commit**

```bash
git add modules/localization/prior_map_localization/test/test_prior_map_manager.cpp
git commit -m "test(prior_loc): scan-to-map overlap ratio"
```

---

### Task 4: InjectDecision — hysteresis state machine

**Files:**
- Create: `include/glim_ext/inject_decision.hpp`
- Test: `test/test_inject_decision.cpp`

- [ ] **Step 1: Write the header (inline, header-only)**

```cpp
#pragma once

namespace glim {

// Stateful hysteresis: enter at >= min_overlap, exit at < 0.8 * min_overlap.
class InjectDecision {
public:
  explicit InjectDecision(double min_overlap) : min_overlap_(min_overlap) {}

  // Update with current overlap; returns whether S2M should be injected this frame.
  bool update(double overlap) {
    if (active_) {
      active_ = overlap >= min_overlap_ * 0.8;
    } else {
      active_ = overlap >= min_overlap_;
    }
    return active_;
  }

  bool active() const { return active_; }
  void reset() { active_ = false; }

private:
  double min_overlap_;
  bool active_ = false;
};

}  // namespace glim
```

- [ ] **Step 2: Write the failing test**

```cpp
#include <gtest/gtest.h>
#include <glim_ext/inject_decision.hpp>

TEST(InjectDecision, Hysteresis) {
  glim::InjectDecision d(0.10);          // enter 0.10, exit 0.08

  EXPECT_FALSE(d.update(0.09));          // below enter, not active
  EXPECT_TRUE(d.update(0.11));           // crosses enter → active
  EXPECT_TRUE(d.update(0.09));           // 0.09 >= 0.08 exit → stays active
  EXPECT_FALSE(d.update(0.07));          // below exit → deactivates
  EXPECT_FALSE(d.update(0.09));          // 0.09 < 0.10 enter → stays inactive
}
```

- [ ] **Step 3: Run test to verify it fails, then passes**

Run: `colcon build --packages-select glim_ext && colcon test --packages-select glim_ext --ctest-args -R test_prior_map_localization && colcon test-result --verbose`
Expected: PASS (header-only; fails only if not wired into the gtest target — confirm `test_inject_decision.cpp` is listed in CMake).

- [ ] **Step 4: Commit**

```bash
git add modules/localization/prior_map_localization/include/glim_ext/inject_decision.hpp modules/localization/prior_map_localization/test/test_inject_decision.cpp
git commit -m "feat(prior_loc): InjectDecision hysteresis"
```

---

### Task 5: localization_math — 6DoF init pose + T_map_odom

**Files:**
- Create: `include/glim_ext/localization_math.hpp`
- Test: `test/test_localization_math.cpp`

- [ ] **Step 1: Write the header (inline)**

```cpp
#pragma once

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim {

// Extract yaw (Z) from a rotation, returning the roll/pitch-only rotation (yaw removed).
inline Eigen::Matrix3d roll_pitch_of(const Eigen::Matrix3d& R) {
  // yaw from atan2(R(1,0), R(0,0)); remove it: R_rp = Rz(-yaw) * R
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  const Eigen::Matrix3d Rz_inv = Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return Rz_inv * R;
}

// Build 6DoF T_map_imu from a 2D guess (x,y,yaw) + map ground z + roll/pitch taken from X(t).
inline Eigen::Isometry3d compose_initial_pose(double x, double y, double yaw, double z,
                                              const Eigen::Isometry3d& X_t) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  const Eigen::Matrix3d R_rp = roll_pitch_of(X_t.rotation());
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * R_rp;
  T.translation() = Eigen::Vector3d(x, y, z);
  return T;
}

// T_map_odom = T_map_imu * X(t)^-1  (X(t) = T_odom_imu at apply time; NOT assumed identity).
inline Eigen::Isometry3d compute_T_map_odom(const Eigen::Isometry3d& T_map_imu,
                                            const Eigen::Isometry3d& X_t) {
  return T_map_imu * X_t.inverse();
}

}  // namespace glim
```

- [ ] **Step 2: Write the failing test**

```cpp
#include <gtest/gtest.h>
#include <glim_ext/localization_math.hpp>

TEST(LocalizationMath, TMapOdomIdentityXt) {
  Eigen::Isometry3d T_map_imu = Eigen::Isometry3d::Identity();
  T_map_imu.translation() = Eigen::Vector3d(10, 5, 1);
  Eigen::Isometry3d T = glim::compute_T_map_odom(T_map_imu, Eigen::Isometry3d::Identity());
  EXPECT_TRUE(T.isApprox(T_map_imu, 1e-9));
}

TEST(LocalizationMath, TMapOdomNonIdentityXt) {
  // If X(t) already = T_map_imu, T_map_odom must be identity.
  Eigen::Isometry3d X_t = Eigen::Isometry3d::Identity();
  X_t.translation() = Eigen::Vector3d(3, 4, 0);
  X_t.linear() = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Isometry3d T = glim::compute_T_map_odom(X_t, X_t);
  EXPECT_TRUE(T.isApprox(Eigen::Isometry3d::Identity(), 1e-9));
}

TEST(LocalizationMath, ComposeTakesRollPitchFromXt) {
  // X(t) tilted 0.2 rad about X axis (pitch); compose with yaw=0 → result keeps the tilt, z set.
  Eigen::Isometry3d X_t = Eigen::Isometry3d::Identity();
  X_t.linear() = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  Eigen::Isometry3d T = glim::compose_initial_pose(1.0, 2.0, 0.0, 0.5, X_t);
  EXPECT_NEAR(T.translation().z(), 0.5, 1e-9);
  // pitch preserved: rotating UnitZ should tilt by ~0.2 rad
  Eigen::Vector3d up = T.linear() * Eigen::Vector3d::UnitZ();
  EXPECT_NEAR(std::acos(up.z()), 0.2, 1e-6);
}
```

- [ ] **Step 3: Run test to verify it passes**

Run: `colcon build --packages-select glim_ext && colcon test --packages-select glim_ext --ctest-args -R test_prior_map_localization && colcon test-result --verbose`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add modules/localization/prior_map_localization/include/glim_ext/localization_math.hpp modules/localization/prior_map_localization/test/test_localization_math.cpp
git commit -m "feat(prior_loc): 6DoF init pose + T_map_odom math"
```

---

### Task 6: InitialPoseSource — /initialpose + config default (thread-safe)

**Files:**
- Create: `include/glim_ext/initial_pose_source.hpp`
- Create: `src/glim_ext/initial_pose_source.cpp`
- Test: `test/test_inject_decision.cpp` (reuse file; add cases) — pure logic, no ROS spin

- [ ] **Step 1: Write the header**

```cpp
#pragma once

#include <mutex>
#include <optional>
#include <array>

namespace glim {

// Holds the latest pending 2D pose guess (x, y, yaw in map). Thread-safe.
class InitialPoseSource {
public:
  struct Pose2D { double x, y, yaw; };

  // Push a 2D guess (from /initialpose callback or config default).
  void push(double x, double y, double yaw);

  // Returns and clears the pending pose, or nullopt if none.
  std::optional<Pose2D> pop_pending();

private:
  std::mutex mutex_;
  std::optional<Pose2D> pending_;
};

}  // namespace glim
```

- [ ] **Step 2: Write the impl**

```cpp
#include <glim_ext/initial_pose_source.hpp>

namespace glim {

void InitialPoseSource::push(double x, double y, double yaw) {
  std::lock_guard<std::mutex> lock(mutex_);
  pending_ = Pose2D{x, y, yaw};
}

std::optional<InitialPoseSource::Pose2D> InitialPoseSource::pop_pending() {
  std::lock_guard<std::mutex> lock(mutex_);
  auto p = pending_;
  pending_.reset();
  return p;
}

}  // namespace glim
```

- [ ] **Step 3: Write the failing test (append to test_inject_decision.cpp)**

```cpp
#include <glim_ext/initial_pose_source.hpp>

TEST(InitialPoseSource, PushPopOnce) {
  glim::InitialPoseSource src;
  EXPECT_FALSE(src.pop_pending().has_value());
  src.push(1.0, 2.0, 0.5);
  auto p = src.pop_pending();
  ASSERT_TRUE(p.has_value());
  EXPECT_DOUBLE_EQ(p->x, 1.0);
  EXPECT_DOUBLE_EQ(p->yaw, 0.5);
  EXPECT_FALSE(src.pop_pending().has_value());  // consumed
}
```

- [ ] **Step 4: Run test to verify it fails then passes**

Run: `colcon build --packages-select glim_ext && colcon test --packages-select glim_ext --ctest-args -R test_prior_map_localization && colcon test-result --verbose`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add modules/localization/prior_map_localization/include/glim_ext/initial_pose_source.hpp modules/localization/prior_map_localization/src/glim_ext/initial_pose_source.cpp modules/localization/prior_map_localization/test/test_inject_decision.cpp
git commit -m "feat(prior_loc): InitialPoseSource thread-safe pending pose"
```

---

### Task 7: PriorMapLocalization — full module (config, callbacks, injection, cooldown, publishing)

**Files:**
- Modify: `include/glim_ext/prior_map_localization.hpp`
- Modify: `src/glim_ext/prior_map_localization.cpp`

This task wires the verified pieces together. It is integration glue (validated by Task 9), so steps are write-code → build → commit.

- [ ] **Step 1: Replace the header with the full declaration**

```cpp
#pragma once

#include <mutex>
#include <random>
#include <memory>
#include <atomic>

#include <Eigen/Geometry>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <glim/util/extension_module_ros2.hpp>
#include <glim/odometry/estimation_frame.hpp>

#include <glim_ext/prior_map_manager.hpp>
#include <glim_ext/inject_decision.hpp>
#include <glim_ext/initial_pose_source.hpp>

namespace glim {

class PriorMapLocalization : public ExtensionModuleROS2 {
public:
  PriorMapLocalization();
  ~PriorMapLocalization() override;

  std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override;

private:
  enum class State { UNINITIALIZED, TRACKING };

  void on_new_frame(const EstimationFrame::ConstPtr& frame);
  void on_smoother_update(
    gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    gtsam::NonlinearFactorGraph& new_factors,
    gtsam::Values& new_values,
    std::map<std::uint64_t, double>& new_stamps);

  // Apply a pending 2D init pose using the latest frame X(t); sets T_map_odom_, state_, cooldown.
  void try_apply_initial_pose(const EstimationFrame::ConstPtr& frame);

  struct Config {
    std::string prior_map_path;
    bool assume_gravity_aligned = true;
    double voxel_resolution = 0.5;
    double min_overlap = 0.05;
    double s2m_points_ratio = 1.0;
    int s2m_num_threads = 4;
    double relocalization_cooldown_sec = 5.0;
    std::array<double, 4> default_init_pose = {0, 0, 0, 0};
    bool has_default_init_pose = false;
    double imu_height_above_ground = 0.0;
    double ground_grid_resolution = 1.0;
  };
  Config config_;
  double effective_cooldown_ = 5.0;

  std::unique_ptr<PriorMapManager> map_;
  InitialPoseSource init_source_;
  InjectDecision inject_{0.05};

  std::mutex state_mutex_;
  State state_ = State::UNINITIALIZED;
  Eigen::Isometry3d T_map_odom_ = Eigen::Isometry3d::Identity();
  double cooldown_until_stamp_ = -1.0;
  std::mt19937 mt_{12345};

  // factors built in on_new_frame, flushed in on_smoother_update (velocity_suppressor idiom)
  gtsam::NonlinearFactorGraph pending_factors_;
  std::mutex pending_mutex_;

  // ROS publishing (created in create_subscriptions)
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr overlap_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";

  std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace glim
```

- [ ] **Step 2: Implement the constructor (config load, map load, callback registration)**

Replace `prior_map_localization.cpp` constructor + add includes:

```cpp
#include <glim_ext/prior_map_localization.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/make_shared.h>   // gtsam::make_shared (Eigen-aligned)
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>

#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim_ext/util/config_ext.hpp>
#include <glim_ext/localization_math.hpp>

namespace glim {

PriorMapLocalization::PriorMapLocalization() : logger_(create_module_logger("prior_loc")) {
  logger_->info("Starting prior_map_localization ...");

  glim::Config config(glim::GlobalConfigExt::get_config_path("config_prior_map_localization"));
  const std::string sec = "prior_map_localization";
  config_.prior_map_path = config.param<std::string>(sec, "prior_map_path", "");
  config_.assume_gravity_aligned = config.param<bool>(sec, "assume_gravity_aligned", true);
  config_.voxel_resolution = config.param<double>(sec, "voxel_resolution", 0.5);
  config_.min_overlap = config.param<double>(sec, "min_overlap", 0.05);
  config_.s2m_points_ratio = config.param<double>(sec, "s2m_points_ratio", 1.0);
  config_.s2m_num_threads = config.param<int>(sec, "s2m_num_threads", 4);
  config_.relocalization_cooldown_sec = config.param<double>(sec, "relocalization_cooldown_sec", 5.0);
  config_.imu_height_above_ground = config.param<double>(sec, "imu_height_above_ground", 0.0);
  config_.ground_grid_resolution = config.param<double>(sec, "ground_grid_resolution", 1.0);

  auto def = config.param<std::vector<double>>(sec, "default_init_pose", std::vector<double>{});
  if (def.size() == 4 && !(def[0] == 0 && def[1] == 0 && def[2] == 0 && def[3] == 0)) {
    config_.default_init_pose = {def[0], def[1], def[2], def[3]};
    config_.has_default_init_pose = true;
  }

  inject_ = InjectDecision(config_.min_overlap);

  // Cooldown lower-bound = smoother_lag + margin (correctness constraint, spec §2.4).
  // smoother_lag lives in the active GLIM odometry config (section "odometry_estimation").
  double smoother_lag = 5.0;
  try {
    glim::Config odom_config(glim::GlobalConfig::get_config_path("config_odometry"));
    smoother_lag = odom_config.param<double>("odometry_estimation", "smoother_lag", 5.0);
  } catch (const std::exception& e) {
    logger_->warn("could not read smoother_lag ({}); assuming {:.1f}s", e.what(), smoother_lag);
  }
  effective_cooldown_ = std::max(config_.relocalization_cooldown_sec, smoother_lag + 0.5);
  if (effective_cooldown_ > config_.relocalization_cooldown_sec) {
    logger_->warn("cooldown raised to {:.1f}s to cover smoother_lag {:.1f}s", effective_cooldown_, smoother_lag);
  }

  // Load prior map.
  PriorMapManager::Config mcfg;
  mcfg.voxel_resolution = config_.voxel_resolution;
  mcfg.num_threads = config_.s2m_num_threads;
  mcfg.ground_grid_resolution = config_.ground_grid_resolution;
  map_ = std::make_unique<PriorMapManager>(mcfg);
  if (config_.prior_map_path.empty() || !map_->load_map(config_.prior_map_path)) {
    logger_->error("failed to load prior map: '{}'", config_.prior_map_path);
  } else {
    logger_->info("prior map loaded: {}", config_.prior_map_path);
  }

  // Seed default init pose if provided (applied on the first frame).
  if (config_.has_default_init_pose) {
    init_source_.push(config_.default_init_pose[0], config_.default_init_pose[1], config_.default_init_pose[3]);
  }

  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;
  OdometryEstimationCallbacks::on_new_frame.add(std::bind(&PriorMapLocalization::on_new_frame, this, _1));
  OdometryEstimationCallbacks::on_smoother_update.add(std::bind(&PriorMapLocalization::on_smoother_update, this, _1, _2, _3, _4));
}

PriorMapLocalization::~PriorMapLocalization() {}

}  // namespace glim
```

> Note: `default_init_pose` is `[x, y, z, yaw]`; only `x, y, yaw` (indices 0,1,3) feed the 2D guess — `z` is overridden by the map ground query.

- [ ] **Step 3: Implement `try_apply_initial_pose` + `on_new_frame`**

Add to the `glim` namespace block:

```cpp
void PriorMapLocalization::try_apply_initial_pose(const EstimationFrame::ConstPtr& frame) {
  auto guess = init_source_.pop_pending();
  if (!guess) return;

  const Eigen::Isometry3d X_t = frame->T_world_imu;  // T_odom_imu(t)

  double z = 0.0;
  auto gz = map_->query_ground_height(guess->x, guess->y);
  if (gz) {
    z = *gz + config_.imu_height_above_ground;
  } else {
    z = config_.has_default_init_pose ? config_.default_init_pose[2] : 0.0;
    logger_->warn("no ground height at ({:.1f},{:.1f}); using z={:.2f} (guess outside map?)", guess->x, guess->y, z);
  }

  const Eigen::Isometry3d T_map_imu = compose_initial_pose(guess->x, guess->y, guess->yaw, z, X_t);

  std::lock_guard<std::mutex> lock(state_mutex_);
  const bool was_tracking = (state_ == State::TRACKING);
  T_map_odom_ = compute_T_map_odom(T_map_imu, X_t);
  if (was_tracking) {
    cooldown_until_stamp_ = frame->stamp + effective_cooldown_;  // reloc: cool down
    inject_.reset();
    logger_->info("relocalized; cooling down {:.1f}s", effective_cooldown_);
  }
  state_ = State::TRACKING;
}

void PriorMapLocalization::on_new_frame(const EstimationFrame::ConstPtr& frame) {
  using gtsam::symbol_shorthand::X;

  try_apply_initial_pose(frame);

  Eigen::Isometry3d T_map_odom;
  double cooldown_until;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ != State::TRACKING) return;
    T_map_odom = T_map_odom_;
    cooldown_until = cooldown_until_stamp_;
  }
  if (frame->stamp < cooldown_until) return;  // cooldown: pure LIO

  const Eigen::Isometry3d T_map_sensor = T_map_odom * frame->T_world_imu;
  const double overlap = map_->compute_overlap_ratio(frame->frame, T_map_sensor);

  if (overlap_pub_) {
    std_msgs::msg::Float64 m; m.data = overlap; overlap_pub_->publish(m);
  }

  if (!inject_.update(overlap)) return;  // hysteresis decided no injection

  // Build the S2M factor: fixed target = T_odom_map = (T_map_odom)^-1.
  const gtsam::Pose3 T_odom_map(T_map_odom.inverse().matrix());
  auto src = gtsam_points::random_sampling(frame->frame, config_.s2m_points_ratio, mt_);

  auto f = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(
    T_odom_map, X(frame->id), map_->voxelmap(), src);
  f->set_num_threads(config_.s2m_num_threads);

  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_factors_.add(f);
  }
}
```

- [ ] **Step 4: Implement `on_smoother_update` (flush) + publishing**

```cpp
void PriorMapLocalization::on_smoother_update(
  gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
  gtsam::NonlinearFactorGraph& new_factors,
  gtsam::Values& new_values,
  std::map<std::uint64_t, double>& new_stamps) {
  //
  std::lock_guard<std::mutex> lock(pending_mutex_);
  new_factors.add(pending_factors_);
  pending_factors_.resize(0);
}
```

Add publishing at the end of `on_new_frame` (after the injection block), using the new frame's pose:

```cpp
  // --- publish map->odom TF + status (做法 A: T_map_odom constant) ---
  if (tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = rclcpp::Time(static_cast<int64_t>(frame->stamp * 1e9));
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    const Eigen::Quaterniond q(T_map_odom.rotation());
    tf.transform.translation.x = T_map_odom.translation().x();
    tf.transform.translation.y = T_map_odom.translation().y();
    tf.transform.translation.z = T_map_odom.translation().z();
    tf.transform.rotation.x = q.x(); tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z(); tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
  }
  if (pose_pub_) {
    const Eigen::Isometry3d T_map_imu = T_map_odom * frame->T_world_imu;
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = rclcpp::Time(static_cast<int64_t>(frame->stamp * 1e9));
    ps.header.frame_id = map_frame_;
    const Eigen::Quaterniond q(T_map_imu.rotation());
    ps.pose.position.x = T_map_imu.translation().x();
    ps.pose.position.y = T_map_imu.translation().y();
    ps.pose.position.z = T_map_imu.translation().z();
    ps.pose.orientation.x = q.x(); ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z(); ps.pose.orientation.w = q.w();
    pose_pub_->publish(ps);
  }
  if (status_pub_) {
    std_msgs::msg::String s; s.data = "TRACKING"; status_pub_->publish(s);
  }
```

> Place this block at the very end of `on_new_frame` (it reuses the local `T_map_odom` and runs every tracking frame, including cooldown frames where no factor was injected).

- [ ] **Step 5: Implement `create_subscriptions` (subscribe /initialpose, build publishers + TF)**

```cpp
std::vector<GenericTopicSubscription::Ptr> PriorMapLocalization::create_subscriptions(rclcpp::Node& node) {
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  pose_pub_ = node.create_publisher<geometry_msgs::msg::PoseStamped>("~/localization/pose", 10);
  overlap_pub_ = node.create_publisher<std_msgs::msg::Float64>("~/localization/overlap", 10);
  status_pub_ = node.create_publisher<std_msgs::msg::String>("~/localization/status", 10);

  auto sub = std::make_shared<TopicSubscription<geometry_msgs::msg::PoseWithCovarianceStamped>>(
    "/initialpose",
    [this](const std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped>& msg) {
      const auto& q = msg->pose.pose.orientation;
      const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      init_source_.push(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
      logger_->info("/initialpose received: ({:.2f},{:.2f}, yaw {:.2f})", msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
    });
  return {sub};
}
```

- [ ] **Step 6: Build**

Run: `colcon build --packages-select glim_ext`
Expected: build succeeds.

- [ ] **Step 7: Commit**

```bash
git add modules/localization/prior_map_localization/include modules/localization/prior_map_localization/src
git commit -m "feat(prior_loc): full module — S2M injection, cooldown, TF/topic publishing"
```

---

### Task 8: Config file + enable in GLIM

**Files:**
- Create: `config/config_prior_map_localization.json`

- [ ] **Step 1: Write the config (matches spec §6.1)**

```jsonc
{
  "prior_map_localization": {
    "prior_map_path": "/path/to/prior_map.pcd",
    "assume_gravity_aligned": true,
    "voxel_resolution": 0.5,
    "min_overlap": 0.05,
    "max_corr_dist": 1.0,
    "s2m_points_ratio": 1.0,
    "s2m_num_threads": 4,
    "relocalization_cooldown_sec": 5.0,
    "default_init_pose": [0, 0, 0, 0],
    "ground_grid_resolution": 1.0,
    "imu_height_above_ground": 0.0,
    "full_weight_overlap": 0.30,
    "use_gpu": false
  }
}
```

- [ ] **Step 2: Build + commit (so the config installs)**

Run: `colcon build --packages-select glim_ext`
Expected: success; `install/glim_ext/share/glim_ext/config/config_prior_map_localization.json` present (confirm glim_ext CMake installs the `config/` dir; if not, add an `install(DIRECTORY config ...)` rule).

```bash
git add config/config_prior_map_localization.json
git commit -m "feat(prior_loc): config_prior_map_localization.json"
```

- [ ] **Step 3: Document enabling (no commit — user edits their active config)**

To activate at runtime, add `"libprior_map_localization.so"` to `extension_modules` in the active `config_ros.json`, and set `prior_map_path` in `config_prior_map_localization.json`.

---

### Task 9: Integration test (bag + PCD, manual)

**Files:** none (runbook). Requires a recorded bag with IMU + LiDAR over an area covered by a `.pcd` prior map.

- [ ] **Step 1: Configure**

Set `prior_map_path` to the test `.pcd`; add `libprior_map_localization.so` to `extension_modules`.

- [ ] **Step 2: Launch GLIM + play bag, give /initialpose in RViz**

Run the project's GLIM launch, then in RViz publish a `2D Pose Estimate` near the true start.

- [ ] **Step 3: Verify acceptance (spec §7.2)**

- In-map: `map→odom` converges; `/prior_loc/localization/pose` tracks the map; APE small.
- Slope start: with 2D `/initialpose`, z is filled from the map; VGICP converges (no divergence).
- Map gap: drive out of the mapped area → `/prior_loc/localization/overlap` → 0, factors stop (pure LIO), no fly-away; on return, re-converges.
- Mid-run relocalization: publish `/initialpose` again while TRACKING → `T_map_odom` jumps once; **no oscillation** during the ~5s cooldown; smooth re-lock afterward.

- [ ] **Step 4: Record results**

Note APE and the gap/relocalization behavior in a short comment on the PR/commit.

---

## Self-Review Notes

- **Spec coverage:** map load (T1), voxelmap (T1), height grid/ground query (T1–T2), overlap (T3), hysteresis (T4), 6DoF init + T_map_odom incl. non-identity X(t) (T5), /initialpose source (T6), S2M injection via on_new_frame/on_smoother_update with random subsample + set_num_threads + cooldown (first-frame skip, ≥smoother_lag) (T7), TF/topics (T7), config (T8), integration incl. gap-degrade & reloc-no-oscillation (T9). Gravity check (§2.4) — warning-only: implement as a `query_ground_height`/normal heuristic warning inside `load_map` if desired; deferred as non-blocking (declaration bears responsibility).
- **Types:** `compute_T_map_odom`/`compose_initial_pose` (T5) used in T7; `InjectDecision::update` (T4) used in T7; `PriorMapManager` API (T1) used in T7; `InitialPoseSource::{push,pop_pending}` (T6) used in T7. Consistent.
- **GPU/continuous-weight/FPFH/LOST state machine:** out of scope (v2), per spec §1.3.
