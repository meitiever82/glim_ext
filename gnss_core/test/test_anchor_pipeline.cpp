#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <stdexcept>

#include "gnss_core/anchor_pipeline.hpp"
#include "gnss_core/geodetic.hpp"
#include "gnss_core/synth.hpp"
#include "synth_fixtures.hpp"

using namespace gnss_core;
using gnss_core::test_fixtures::straight_then_turn;

namespace {

double yaw_of(const Eigen::Isometry3d& T) { return std::atan2(T.linear()(1, 0), T.linear()(0, 0)); }

Eigen::Isometry3d rotated_frame(double yaw, const Eigen::Vector3d& t) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() = t;
  return T;
}

// 共用场景:straight_then_turn 轨迹、world 相对 ENU 有已知 yaw+平移、杆臂 0。
struct Scene {
  SynthConfig cfg;
  std::vector<TrajPose> traj;
  SynthResult synth;
  std::vector<Anchor> anchors;   // 每 anchor_dt 秒一个,直接取轨迹真值位置

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
  // 管线的 ENU 原点(lat/lon/alt)一般不等于 synth 的 lat0/lon0/alt0:
  // enu_pipeline ≈ enu_synth − enu0,故 T_world_enu_true = T_enu_world⁻¹ · Translate(enu0)
  // (两原点相距 < 200 m,LocalCartesian 切平面的姿态差 ~1e-5 rad,可忽略)。
  Eigen::Isometry3d T_world_enu_true(const std::array<double, 3>& origin) const {
    const Eigen::Vector3d enu0 = LlaToEnu(cfg.lat0, cfg.lon0, cfg.alt0).forward(origin[0], origin[1], origin[2]);
    return cfg.T_enu_world.inverse() * Eigen::Translation3d(enu0);
  }
  const Anchor& anchor_by_id(long id) const { return anchors.at(static_cast<size_t>(id)); }
};

PipelineConfig default_config() {
  PipelineConfig pc;
  pc.noise.sigma_floor = {0.02, 0.02, 0.05};
  pc.min_baseline = 30.0;   // 30 m 基线让 yaw 估计足够准(1 cm 级噪声下远端 < 5 cm)
  return pc;
}

}  // namespace

// 1. 干净数据:所有 anchor 最终 Constrained(含 bootstrap 前的 deferred 补发),p_world 与真值 < 5 cm,
//    T_world_enu 恢复真值。
TEST(AnchorPipeline, CleanDataConstrainsEveryAnchor) {
  Scene sc;
  AnchorPipeline pl(default_config());
  for (const auto& s : sc.synth.samples) pl.push_fix(s);
  for (const auto& a : sc.anchors) pl.push_anchor(a);

  const auto out = pl.process();
  EXPECT_EQ(pl.pending_anchors(), 0u);
  EXPECT_EQ(pl.deferred(), 0u);
  ASSERT_EQ(out.size(), sc.anchors.size());
  EXPECT_TRUE(pl.initialized());
  EXPECT_EQ(pl.last_outcome(), AnchorOutcome::Constrained);

  const auto& st = pl.stats();
  EXPECT_EQ(st.fixes, sc.synth.samples.size());
  EXPECT_EQ(st.anchors, sc.anchors.size());
  EXPECT_EQ(st.constrained, sc.anchors.size());
  EXPECT_GT(st.deferred_released, 0u);              // 前 30 m 的 anchor 在 bootstrap 时补发
  EXPECT_EQ(st.too_old + st.gap + st.gated, 0u);

  // 输出顺序:先 deferred(按 id 递增),再当前;整体 id 单调递增且不重复
  std::map<long, Constraint> by_id;
  for (size_t i = 0; i < out.size(); ++i) {
    if (i > 0) { EXPECT_GT(out[i].id, out[i - 1].id); }
    by_id[out[i].id] = out[i];
  }
  EXPECT_EQ(by_id.size(), sc.anchors.size());
  for (const auto& [id, c] : by_id) {
    const Anchor& a = sc.anchor_by_id(id);
    EXPECT_DOUBLE_EQ(c.stamp, a.stamp);
    ASSERT_TRUE(c.model);
    EXPECT_LT((c.p_world - a.t_world).norm(), 0.05) << "anchor " << id;
  }

  // 原点未配置 → 取首个过门限样本(即 t=0 的插值样本)
  ASSERT_TRUE(pl.enu_origin().has_value());
  EXPECT_NEAR((*pl.enu_origin())[0], sc.synth.samples.front().lat, 1e-9);
  EXPECT_NEAR((*pl.enu_origin())[1], sc.synth.samples.front().lon, 1e-9);

  const Eigen::Isometry3d T_true = sc.T_world_enu_true(*pl.enu_origin());
  EXPECT_NEAR(yaw_of(pl.T_world_enu()), yaw_of(T_true), 0.1 * M_PI / 180.0);
  EXPECT_LT((pl.T_world_enu().translation() - T_true.translation()).norm(), 0.05);
}

// 2. fix 不够时 anchor 等待:process() 返回空、pending 不减;补上 fix 后处理。
TEST(AnchorPipeline, AnchorsWaitUntilFixesCoverThem) {
  Scene sc(1.0);
  AnchorPipeline pl(default_config());

  // 只有 1 个样本:缓冲不足 2 个 → 一律等待
  pl.push_fix(sc.synth.samples[0]);
  pl.push_anchor(sc.anchor_by_id(0));
  EXPECT_TRUE(pl.process().empty());
  EXPECT_EQ(pl.pending_anchors(), 1u);
  EXPECT_EQ(pl.last_outcome(), AnchorOutcome::WaitingForFixes);

  // 推到 t=0.5 s:t=0 的 anchor 可处理(Deferred),t=1,2 的 anchor 晚于最新样本 → 等待,后面的不动
  for (size_t i = 1; i <= 5; ++i) pl.push_fix(sc.synth.samples[i]);
  pl.push_anchor(sc.anchor_by_id(1));
  pl.push_anchor(sc.anchor_by_id(2));
  EXPECT_TRUE(pl.process().empty());
  EXPECT_EQ(pl.pending_anchors(), 2u);
  EXPECT_EQ(pl.deferred(), 1u);
  EXPECT_EQ(pl.last_outcome(), AnchorOutcome::WaitingForFixes);
  EXPECT_FALSE(pl.initialized());

  // 再 process 一次不应有副作用
  EXPECT_TRUE(pl.process().empty());
  EXPECT_EQ(pl.pending_anchors(), 2u);
  EXPECT_EQ(pl.stats().anchors, 3u);

  // 补齐全部 fix 与其余 anchor → 全部处理
  for (size_t i = 6; i < sc.synth.samples.size(); ++i) pl.push_fix(sc.synth.samples[i]);
  for (size_t k = 3; k < sc.anchors.size(); ++k) pl.push_anchor(sc.anchors[k]);
  const auto out = pl.process();
  EXPECT_EQ(pl.pending_anchors(), 0u);
  EXPECT_EQ(out.size(), sc.anchors.size());
  EXPECT_EQ(pl.stats().constrained, sc.anchors.size());
  EXPECT_TRUE(pl.initialized());
}

// 3. anchor 早于缓冲最老样本(已被 prune)→ TooOld 丢弃,不阻塞后续。
TEST(AnchorPipeline, AnchorOlderThanBufferIsDropped) {
  Scene sc(1.0);
  PipelineConfig pc = default_config();
  pc.fix_buffer_horizon = 20.0;   // 推完 50 s 的 fix 后缓冲只剩 [30, 50]
  AnchorPipeline pl(pc);
  for (const auto& s : sc.synth.samples) pl.push_fix(s);

  pl.push_anchor(sc.anchor_by_id(5));    // t=5 < oldest=30
  pl.push_anchor(sc.anchor_by_id(35));   // t=35 在缓冲内
  const auto out = pl.process();
  EXPECT_TRUE(out.empty());              // 单点不足以 bootstrap
  EXPECT_EQ(pl.pending_anchors(), 0u);
  EXPECT_EQ(pl.stats().too_old, 1u);
  EXPECT_EQ(pl.stats().anchors, 2u);
  EXPECT_EQ(pl.deferred(), 1u);
  EXPECT_EQ(pl.last_outcome(), AnchorOutcome::Deferred);

  // 再推一个 TooOld,看 last_outcome
  pl.push_anchor(sc.anchor_by_id(6));
  EXPECT_TRUE(pl.process().empty());
  EXPECT_EQ(pl.last_outcome(), AnchorOutcome::TooOld);
  EXPECT_EQ(pl.stats().too_old, 2u);
}

// 4. 中间 60 s 无 fix 的段内的 anchor → Gap;段前后的 anchor 不受影响。
TEST(AnchorPipeline, AnchorInsideFixGapIsRejected) {
  Scene sc(1.0);
  // 把 t > 30 的样本与 anchor 整体后移 60 s:形成 (30.0, 90.1) 的 60 s 空洞
  // (t=30 的 anchor 留在洞前沿,与样本 30.0 重合,可插值)
  for (auto& s : sc.synth.samples) {
    if (s.stamp > 30.05) s.stamp += 60.0;
  }
  for (auto& a : sc.anchors) {
    if (a.stamp > 30.05) a.stamp += 60.0;
  }

  AnchorPipeline pl(default_config());
  for (const auto& s : sc.synth.samples) pl.push_fix(s);
  for (const auto& a : sc.anchors) pl.push_anchor(a);
  Anchor in_gap{999, 60.0, sc.anchor_by_id(30).t_world};   // 转弯末尾,位置不变
  pl.push_anchor(in_gap);

  const auto out = pl.process();
  EXPECT_EQ(pl.pending_anchors(), 0u);
  EXPECT_EQ(pl.stats().gap, 1u);
  EXPECT_EQ(pl.last_outcome(), AnchorOutcome::Gap);
  EXPECT_EQ(out.size(), sc.anchors.size());
  for (const auto& c : out) EXPECT_NE(c.id, 999);
  EXPECT_EQ(pl.stats().constrained, sc.anchors.size());
}

// 5. 全 SINGLE:全部 Gated,不参与 aligner,不定原点,不初始化。
TEST(AnchorPipeline, GatedSamplesDoNotFeedAligner) {
  Scene sc(1.0);
  for (auto& s : sc.synth.samples) s.quality = Quality::SINGLE;
  AnchorPipeline pl(default_config());   // min_quality=3 (FLOAT)
  for (const auto& s : sc.synth.samples) pl.push_fix(s);
  for (const auto& a : sc.anchors) pl.push_anchor(a);

  EXPECT_TRUE(pl.process().empty());
  EXPECT_EQ(pl.pending_anchors(), 0u);
  EXPECT_EQ(pl.deferred(), 0u);
  EXPECT_EQ(pl.stats().gated, sc.anchors.size());
  EXPECT_GT(pl.stats().gated_by_reason[static_cast<size_t>(RejectReason::BelowMinQuality)], 0u);
  EXPECT_EQ(pl.stats().gated_by_reason[static_cast<size_t>(RejectReason::BelowMinQuality)],
            pl.stats().gated);
  EXPECT_EQ(pl.stats().constrained, 0u);
  EXPECT_EQ(pl.last_outcome(), AnchorOutcome::Gated);
  EXPECT_FALSE(pl.initialized());
  EXPECT_FALSE(pl.enu_origin().has_value());
  EXPECT_TRUE(pl.T_world_enu().isApprox(Eigen::Isometry3d::Identity()));
}

// 6. 配置了 enu_origin:enu_origin() 等于配置值而非首个样本;结果仍与真值一致(原点差被 T_world_enu 吸收)。
TEST(AnchorPipeline, ConfiguredEnuOriginIsUsed) {
  Scene sc(1.0);
  PipelineConfig pc = default_config();
  pc.enu_origin = {sc.cfg.lat0 + 0.001, sc.cfg.lon0 - 0.002, sc.cfg.alt0 + 12.0};
  AnchorPipeline pl(pc);

  ASSERT_TRUE(pl.enu_origin().has_value());   // 构造即已定
  EXPECT_DOUBLE_EQ((*pl.enu_origin())[0], pc.enu_origin[0]);
  EXPECT_DOUBLE_EQ((*pl.enu_origin())[1], pc.enu_origin[1]);
  EXPECT_DOUBLE_EQ((*pl.enu_origin())[2], pc.enu_origin[2]);

  for (const auto& s : sc.synth.samples) pl.push_fix(s);
  for (const auto& a : sc.anchors) pl.push_anchor(a);
  const auto out = pl.process();
  ASSERT_EQ(out.size(), sc.anchors.size());
  EXPECT_DOUBLE_EQ((*pl.enu_origin())[0], pc.enu_origin[0]);   // 处理后仍是配置值
  for (const auto& c : out) EXPECT_LT((c.p_world - sc.anchor_by_id(c.id).t_world).norm(), 0.05);
  const Eigen::Isometry3d T_true = sc.T_world_enu_true(*pl.enu_origin());
  EXPECT_NEAR(yaw_of(pl.T_world_enu()), yaw_of(T_true), 0.1 * M_PI / 180.0);
  EXPECT_LT((pl.T_world_enu().translation() - T_true.translation()).norm(), 0.05);
}

// 7. 非法配置构造抛异常。
TEST(AnchorPipeline, InvalidConfigThrows) {
  PipelineConfig pc;
  pc.noise.min_quality = 9;
  EXPECT_THROW(AnchorPipeline{pc}, std::invalid_argument);

  PipelineConfig pc2;
  pc2.enu_origin = {44.5, 90.28};   // 必须为空或 3 个
  EXPECT_THROW(AnchorPipeline{pc2}, std::invalid_argument);

  PipelineConfig pc3;
  pc3.fix_max_gap = 0.0;
  EXPECT_THROW(AnchorPipeline{pc3}, std::invalid_argument);
}
