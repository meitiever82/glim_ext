// 2026-10-19 shared-gauge(F1) + FIXED-only bootstrap(F2) 的单元测试
#include <gtest/gtest.h>

#include <cmath>

#include "gnss_core/anchor_pipeline.hpp"
#include "gnss_core/gauge_registry.hpp"
#include "gnss_core/geodetic.hpp"
#include "gnss_core/synth.hpp"
#include "synth_fixtures.hpp"

using namespace gnss_core;
using gnss_core::test_fixtures::straight_then_turn;

namespace {

Eigen::Isometry3d rotated_frame(double yaw, const Eigen::Vector3d& t) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() = t;
  return T;
}

// 与 test_anchor_pipeline.cpp 相同的场景（world 相对 ENU 有已知 yaw+平移）
struct Scene {
  SynthConfig cfg;
  std::vector<TrajPose> traj;
  SynthResult synth;
  std::vector<Anchor> anchors;

  explicit Scene(double anchor_dt = 0.5) {
    cfg.lever_imu = Eigen::Vector3d::Zero();
    cfg.sigma_fixed = {0.005, 0.005, 0.01};
    cfg.T_enu_world = rotated_frame(0.7, {50, -20, 3});
    traj = straight_then_turn();
    synth = synthesize(traj, cfg, {});
    const int step = static_cast<int>(std::lround(anchor_dt / 0.1));
    long id = 0;
    for (size_t k = 0; k < traj.size(); k += step) {
      anchors.push_back(Anchor{id++, traj[k].stamp, traj[k].T_world_imu.translation()});
    }
  }
};

PipelineConfig default_config() {
  PipelineConfig pc;
  pc.noise.sigma_floor = {0.02, 0.02, 0.05};
  pc.min_baseline = 30.0;
  return pc;
}

struct RegistryGuard {   // 测试间隔离：退出时清空 registry
  RegistryGuard() { GaugeRegistry::instance().reset(); }
  ~RegistryGuard() { GaugeRegistry::instance().reset(); }
};

}  // namespace

// F1 基础语义：先发布者胜，之后拒绝；reset 后可再发布
TEST(GaugeRegistry, FirstPublishWinsAndReset) {
  RegistryGuard g;
  auto& reg = GaugeRegistry::instance();
  EXPECT_FALSE(reg.initialized());

  const auto T1 = rotated_frame(0.3, {1, 2, 3});
  const auto T2 = rotated_frame(-0.5, {9, 9, 9});
  EXPECT_TRUE(reg.try_publish(T1, "mod_a"));
  EXPECT_TRUE(reg.initialized());
  EXPECT_EQ(reg.owner(), "mod_a");
  EXPECT_FALSE(reg.try_publish(T2, "mod_b"));   // 后来者被拒
  EXPECT_EQ(reg.owner(), "mod_a");
  EXPECT_TRUE(reg.T_world_enu().isApprox(T1, 1e-12));

  // 非有限 gauge 拒绝
  GaugeRegistry::instance().reset();
  Eigen::Isometry3d bad = Eigen::Isometry3d::Identity();
  bad.translation().x() = std::nan("");
  EXPECT_FALSE(reg.try_publish(bad, "mod_nan"));
  EXPECT_FALSE(reg.initialized());
}

// F2：FLOAT 样本不参与 gauge 求解——前半程 FLOAT（带 3m 偏置）只 deferred 不定基准；
// 后半程 FIXED 正常 bootstrap，且 gauge 不受 FLOAT 偏置污染（约束仍贴真值）。
TEST(AnchorPipeline, FloatSamplesNeverBootstrap) {
  RegistryGuard g;
  Scene sc;
  const double t_split = 25.0;   // 50s 轨迹：前半 x 向直行，后半 y 向直行(基线充足)

  AnchorPipeline pl(default_config());   // aligner_min_quality 默认 4(FIXED)
  for (auto s : sc.synth.samples) {
    if (s.stamp < t_split) {
      s.quality = Quality::FLOAT;
      s.sigma_enu = Eigen::Vector3d(0.2, 0.2, 0.5);   // 过因子门控(min_quality=3)
      s.lat += 3.0 / 111320.0;   // FLOAT 游走:东向 ~3m 系统偏置
    }
    pl.push_fix(s);
  }
  for (const auto& a : sc.anchors) pl.push_anchor(a);

  const auto out = pl.process();
  ASSERT_EQ(out.size(), sc.anchors.size());   // bootstrap 后 deferred 全补发
  EXPECT_TRUE(pl.initialized());
  EXPECT_EQ(pl.gauge_source(), "local");
  EXPECT_GT(pl.stats().deferred_released, 0u);
  // gauge 只由 FIXED 段求解：FIXED 段约束贴真值(<5cm)；
  // FLOAT 段的 deferred 因子带着它们自己的偏置测量(~3m)——因子照常用、基准没被污染。
  for (const auto& c : out) {
    const auto& a = sc.anchors.at(static_cast<size_t>(c.id));
    const double err = (c.p_world - a.t_world).norm();
    if (a.stamp < t_split) {
      EXPECT_GT(err, 2.5) << "FLOAT anchor " << c.id << " should carry its biased measurement";
    } else {
      EXPECT_LT(err, 0.05) << "FIXED anchor " << c.id;
    }
  }
}

// F1（可选开关，默认关——见 PipelineConfig 注释）：显式开启后两个 pipeline 共享
TEST(AnchorPipeline, SecondPipelineAdoptsSharedGauge) {
  RegistryGuard g;
  Scene sc;

  auto cfgA = default_config();
  cfgA.name = "rtk_odometry";
  cfgA.shared_gauge = true;
  AnchorPipeline plA(cfgA);
  for (const auto& s : sc.synth.samples) plA.push_fix(s);
  for (const auto& a : sc.anchors) plA.push_anchor(a);
  const auto outA = plA.process();
  ASSERT_EQ(outA.size(), sc.anchors.size());
  ASSERT_TRUE(plA.initialized());
  EXPECT_EQ(plA.gauge_source(), "local");
  ASSERT_TRUE(GaugeRegistry::instance().initialized());
  EXPECT_EQ(GaugeRegistry::instance().owner(), "rtk_odometry");

  // B:只给少量 anchor（远不够 min_baseline 自解）→ 应采用共享 gauge
  auto cfgB = default_config();
  cfgB.name = "rtk_global";
  cfgB.shared_gauge = true;
  AnchorPipeline plB(cfgB);
  for (const auto& s : sc.synth.samples) plB.push_fix(s);
  plB.push_anchor(sc.anchors.front());
  const auto outB = plB.process();
  ASSERT_EQ(outB.size(), 1u);
  EXPECT_TRUE(plB.initialized());
  EXPECT_EQ(plB.gauge_source(), "shared");
  EXPECT_TRUE(plB.T_world_enu().isApprox(plA.T_world_enu(), 1e-12));
  // B 的约束用共享 gauge:p_world 与 A 的 gauge × enu 一致 → 接近 anchor 真值位置
  EXPECT_LT((outB[0].p_world - sc.anchors.front().t_world).norm(), 0.05);
}

// 默认（shared_gauge=false）：B 不参与共享（即使 registry 里有已发布的 gauge）
TEST(AnchorPipeline, SharedGaugeDisabledKeepsLocalOnly) {
  RegistryGuard g;
  Scene sc;

  auto cfgA = default_config();
  cfgA.name = "rtk_odometry";
  cfgA.shared_gauge = true;   // A 显式开启并发布,B 默认关闭不得采用
  AnchorPipeline plA(cfgA);
  for (const auto& s : sc.synth.samples) plA.push_fix(s);
  for (const auto& a : sc.anchors) plA.push_anchor(a);
  ASSERT_EQ(plA.process().size(), sc.anchors.size());
  ASSERT_TRUE(GaugeRegistry::instance().initialized());

  auto cfgB = default_config();
  cfgB.name = "rtk_global";
  cfgB.shared_gauge = false;
  AnchorPipeline plB(cfgB);
  for (const auto& s : sc.synth.samples) plB.push_fix(s);
  plB.push_anchor(sc.anchors.front());
  EXPECT_TRUE(plB.process().empty());   // 不采用共享,本地又未达基线 → deferred
  EXPECT_FALSE(plB.initialized());
}
