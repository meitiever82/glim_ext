#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "gnss_core/relative_pipeline.hpp"
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

double yaw_of(const Eigen::Isometry3d& T) { return std::atan2(T.linear()(1, 0), T.linear()(0, 0)); }

// 与 test_anchor_pipeline 相同的场景:straight_then_turn 轨迹,world 相对 ENU 有已知 yaw。
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
    for (size_t k = 0; k < traj.size(); k += static_cast<size_t>(step)) {
      anchors.push_back(Anchor{id++, traj[k].stamp, traj[k].T_world_imu.translation()});
    }
  }
};

PipelineConfig pipeline_config() {
  PipelineConfig pc;
  pc.noise.sigma_floor = {0.02, 0.02, 0.05};
  return pc;
}

RelativeConfig relative_config() {
  RelativeConfig rc;
  rc.enabled = true;
  rc.sigma_floor = 0.05;
  rc.yaw_baseline = 8.0;
  rc.max_pair_gap = 2.5;
  return rc;
}

}  // namespace

TEST(RelativePipeline, EmitsPairsAfterYawBootstrap) {
  Scene sc;
  RelativePipeline pl(pipeline_config(), relative_config());
  for (const auto& s : sc.synth.samples) pl.push_fix(s);
  for (const auto& a : sc.anchors) pl.push_anchor(a);

  const auto out = pl.process();
  ASSERT_FALSE(out.empty());
  EXPECT_TRUE(pl.yaw_initialized());
  EXPECT_EQ(pl.pending_anchors(), 0u);
  EXPECT_EQ(pl.stats().fixes, sc.synth.samples.size());
  EXPECT_EQ(pl.stats().anchors, sc.anchors.size());
  EXPECT_EQ(pl.stats().pairs, out.size());
  EXPECT_EQ(pl.stats().too_old + pl.stats().gap + pl.stats().gated + pl.stats().pair_gap, 0u);

  // 独立 yaw bootstrap:rotation 与绝对 gauge 无关,只由 world→ENU 的 0.7 rad 决定
  EXPECT_NEAR(yaw_of(pl.T_world_enu()), -0.7, 0.3 * M_PI / 180.0);
  EXPECT_GT(pl.stats().yaw_bootstrapped_stamp, 0.0);

  // 直行段 5 m/s、anchor 间隔 0.5 s:世界系位移应为 (2.5, 0) ± 噪声
  size_t checked = 0;
  for (const auto& c : out) {
    ASSERT_TRUE(c.model);
    ASSERT_LT(c.stamp1, c.stamp2);
    EXPECT_LT(c.id1, c.id2);
    EXPECT_TRUE(c.dp_world_xy.allFinite());
    if (c.stamp2 <= 19.5 && c.stamp1 >= 2.0) {
      EXPECT_NEAR(c.dp_world_xy.x(), 2.5, 0.03) << "pair " << c.id1 << "->" << c.id2;
      EXPECT_NEAR(c.dp_world_xy.y(), 0.0, 0.03) << "pair " << c.id1 << "->" << c.id2;
      ++checked;
    }
  }
  EXPECT_GT(checked, 10u);
}

TEST(RelativePipeline, PairsStartOnlyAfterYawBootstrap) {
  Scene sc;
  RelativePipeline pl(pipeline_config(), relative_config());
  for (const auto& s : sc.synth.samples) pl.push_fix(s);

  size_t first_pair_index = std::numeric_limits<size_t>::max();
  for (size_t i = 0; i < sc.anchors.size(); ++i) {
    pl.push_anchor(sc.anchors[i]);
    const auto out = pl.process();
    if (!out.empty() && first_pair_index == std::numeric_limits<size_t>::max()) {
      first_pair_index = i;
    }
  }
  EXPECT_TRUE(pl.yaw_initialized());
  ASSERT_NE(first_pair_index, std::numeric_limits<size_t>::max());
  // 8 m 基线 / 5 m/s = 1.6 s,anchor 0.5 s → 第 4 个 anchor(t=1.5)后才可能出现
  EXPECT_GE(first_pair_index, 4u);
  EXPECT_LE(first_pair_index, 5u);
}

TEST(RelativePipeline, GatedSamplesProduceNoPairs) {
  Scene sc;
  for (auto& s : sc.synth.samples) {
    s.quality = Quality::FLOAT;   // 低于 min_quality=4,全部被门控
  }
  auto pc = pipeline_config();
  pc.noise.min_quality = 4;

  RelativePipeline pl(pc, relative_config());
  for (const auto& s : sc.synth.samples) pl.push_fix(s);
  for (const auto& a : sc.anchors) pl.push_anchor(a);

  const auto out = pl.process();
  EXPECT_TRUE(out.empty());
  EXPECT_GT(pl.stats().gated, 0u);
  EXPECT_EQ(pl.stats().pairs, 0u);
  EXPECT_FALSE(pl.yaw_initialized());
}

TEST(RelativePipeline, PairGapSkipsWidelySpacedEpochs) {
  Scene sc(3.0);   // anchor 间隔 3 s > max_pair_gap 2.5 s
  RelativePipeline pl(pipeline_config(), relative_config());
  for (const auto& s : sc.synth.samples) pl.push_fix(s);
  for (const auto& a : sc.anchors) pl.push_anchor(a);

  const auto out = pl.process();
  EXPECT_TRUE(out.empty());
  EXPECT_EQ(pl.stats().pairs, 0u);
  EXPECT_GT(pl.stats().pair_gap, 0u);
}

TEST(RelativePipeline, AnchorsWaitUntilFixesCoverThem) {
  Scene sc;
  RelativePipeline pl(pipeline_config(), relative_config());
  for (const auto& a : sc.anchors) pl.push_anchor(a);

  EXPECT_TRUE(pl.process().empty());
  EXPECT_EQ(pl.pending_anchors(), sc.anchors.size());

  for (const auto& s : sc.synth.samples) pl.push_fix(s);
  const auto out = pl.process();
  ASSERT_FALSE(out.empty());
  EXPECT_EQ(pl.pending_anchors(), 0u);
}

TEST(RelativePipeline, TooOldAnchorsAreDropped) {
  Scene sc;
  RelativePipeline pl(pipeline_config(), relative_config());
  // 只保留最新的 5 个样本:更早的 anchor 落在 buffer 之外
  const size_t keep = 5;
  std::vector<RtkFixSample> tail(sc.synth.samples.end() - keep, sc.synth.samples.end());
  for (const auto& s : tail) pl.push_fix(s);
  for (const auto& a : sc.anchors) pl.push_anchor(a);

  const auto out = pl.process();
  EXPECT_TRUE(out.empty());
  EXPECT_EQ(pl.pending_anchors(), 0u);
  // 只有落在最新缓冲区间内的最后一个 anchor 不被判 too_old(但也没有可配对的 prev)
  EXPECT_EQ(pl.stats().too_old, sc.anchors.size() - 1);
  EXPECT_EQ(pl.stats().pairs, 0u);
}

TEST(RelativePipeline, DisabledClearsPendingAnchors) {
  Scene sc;
  auto rc = relative_config();
  rc.enabled = false;
  RelativePipeline pl(pipeline_config(), rc);
  for (const auto& s : sc.synth.samples) pl.push_fix(s);
  for (const auto& a : sc.anchors) pl.push_anchor(a);

  EXPECT_TRUE(pl.process().empty());
  EXPECT_EQ(pl.pending_anchors(), 0u);
  EXPECT_FALSE(pl.yaw_initialized());
  EXPECT_EQ(pl.stats().pairs, 0u);
}

TEST(RelativePipeline, RejectsInvalidConfig) {
  auto rc = relative_config();
  rc.max_pair_gap = 0.0;
  EXPECT_THROW(RelativePipeline(pipeline_config(), rc), std::invalid_argument);

  auto rc2 = relative_config();
  rc2.yaw_baseline = -1.0;
  EXPECT_THROW(RelativePipeline(pipeline_config(), rc2), std::invalid_argument);

  auto rc3 = relative_config();
  rc3.sigma_floor = 0.0;
  EXPECT_THROW(RelativePipeline(pipeline_config(), rc3), std::invalid_argument);
}
