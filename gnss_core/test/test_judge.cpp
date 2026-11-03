// Judge 单元测试：分支矩阵 + 状态机时序。
// golden parity（localization/golden/）是逐帧对旧代码的验收；本文件是
// Judge 自身的隔离单测（含 golden 覆盖不到的 fault 9 语义辅助与参数边界）。
#include <gtest/gtest.h>
#include "gnss_core/judge.hpp"

using namespace gnss_core::judge;

namespace {

Evidence ByEvidence() {
  Evidence e;
  e.now = 0.0;
  e.pos_type = PosType::RtkFixed;
  e.ins_good = true;
  e.diff_age = 1.0;
  e.sats_main = 18;
  e.sats_aux = 20;
  e.heading_narrow_int = true;
  e.heading_length = 1.601;
  e.heading_valid_frame = true;
  return e;
}

Evidence HcEvidence() {
  Evidence e;
  e.now = 0.0;
  e.huace_gnss_pos_type = 4;    // RTK fix
  e.ins_good = true;            // 组合导航
  e.hw_warning = false;
  e.huace_gnss_pos_valid = true;
  e.sats_main = 20;
  e.sats_aux = 18;
  return e;
}

Config DefaultCfg() { return Config{}; }

}  // namespace

// === 北云分支矩阵 ===

TEST(JudgeBeiyun, RtkFixedPassthrough) {
  auto s = GateState{};
  // 第一分支：RTKFIXED 直通（不查 ins_good——对齐 golden by_003 实证行为）
  auto e = ByEvidence();
  e.ins_good = false;  // INS_ALIGNING 也直通
  auto v = judge_beiyun(e, s, DefaultCfg());
  EXPECT_TRUE(v.is_valid);
  EXPECT_EQ(v.rtk_status, RtkStatus::RtkFix);
  EXPECT_EQ(v.fault_code, FaultCode::None);
}

TEST(JudgeBeiyun, RtkFloatInsGood) {
  auto s = GateState{};
  auto e = ByEvidence();
  e.pos_type = PosType::RtkFloat;
  auto v = judge_beiyun(e, s, DefaultCfg());
  EXPECT_TRUE(v.is_valid);
  EXPECT_EQ(v.rtk_status, RtkStatus::RtkFloat);
}

TEST(JudgeBeiyun, InsNotConverged) {
  auto s = GateState{};
  auto e = ByEvidence();
  e.pos_type = PosType::RtkFloat;
  e.ins_good = false;
  auto v = judge_beiyun(e, s, DefaultCfg());
  EXPECT_FALSE(v.is_valid);
  EXPECT_EQ(v.fault_code, FaultCode::InsNotConverged);
}

TEST(JudgeBeiyun, SinglePointAgeGrace) {
  auto s = GateState{};
  // 先建立有效状态
  auto e = ByEvidence();
  judge_beiyun(e, s, DefaultCfg());
  // INS 变差 + 单点 + 龄期小 → 仅降级 rtk_status，is_valid 保留
  e.now = 0.5;
  e.ins_good = false;
  e.pos_type = PosType::Single;
  e.diff_age = 1.0;  // < 10s 阈值
  auto v = judge_beiyun(e, s, DefaultCfg());
  EXPECT_TRUE(v.is_valid);
  EXPECT_EQ(v.rtk_status, RtkStatus::RtkFloat);
  EXPECT_EQ(v.fault_code, FaultCode::None);
}

TEST(JudgeBeiyun, StickyTimeout) {
  Config cfg = DefaultCfg();
  cfg.sticky_grace_time = 2.0;
  auto s = GateState{};
  auto e = ByEvidence();
  judge_beiyun(e, s, DefaultCfg());
  // 降级为差分（INS 良好）→ 粘滞
  e.pos_type = PosType::Psrdiff;
  e.now = 0.5;
  auto v1 = judge_beiyun(e, s, cfg);
  EXPECT_TRUE(v1.is_valid);   // 沿用
  // 宽限内
  e.now = 1.5;
  auto v2 = judge_beiyun(e, s, cfg);
  EXPECT_TRUE(v2.is_valid);
  // 超时（0.5 + 2.2 > 2.0）
  e.now = 2.7;
  auto v3 = judge_beiyun(e, s, cfg);
  EXPECT_FALSE(v3.is_valid);
  EXPECT_EQ(v3.fault_code, FaultCode::GraceTimeout);
  EXPECT_EQ(v3.rtk_status, RtkStatus::RtcmDiff);  // PSRDIFF → RtcmDiff
}

TEST(JudgeBeiyun, StickyRecoverResetsTimer) {
  Config cfg = DefaultCfg();
  cfg.sticky_grace_time = 2.0;
  auto s = GateState{};
  auto e = ByEvidence();
  judge_beiyun(e, s, cfg);
  e.pos_type = PosType::Psrdiff; e.now = 0.5;
  judge_beiyun(e, s, cfg);
  // 宽限内恢复固定
  e.pos_type = PosType::RtkFixed; e.now = 1.5;
  auto v = judge_beiyun(e, s, cfg);
  EXPECT_TRUE(v.is_valid);
  EXPECT_FALSE(s.in_sticky);
  // 再次降级，重新起算（新一轮降级事件）
  e.pos_type = PosType::Psrdiff; e.now = 2.0;
  judge_beiyun(e, s, cfg);
  e.now = 3.9;  // 距第二次降级 1.9s < 2s
  auto v2 = judge_beiyun(e, s, cfg);
  EXPECT_TRUE(v2.is_valid);
}

TEST(JudgeBeiyun, StickyZeroGraceFirstFrameStillSticky) {
  // 参数边界 0 的真实语义（golden by_010 实证）：首个降级帧 sticky_sec=0，
  // 0 > 0 不成立 → 仍沿用；次帧 sticky_sec>0 → 立即超时
  Config cfg = DefaultCfg();
  cfg.sticky_grace_time = 0.0;
  auto s = GateState{};
  auto e = ByEvidence();
  judge_beiyun(e, s, cfg);
  e.pos_type = PosType::Psrdiff; e.now = 0.5;
  auto v1 = judge_beiyun(e, s, cfg);
  EXPECT_TRUE(v1.is_valid);                       // 首帧仍沿用
  e.now = 0.7;
  auto v2 = judge_beiyun(e, s, cfg);
  EXPECT_FALSE(v2.is_valid);                      // 次帧超时
  EXPECT_EQ(v2.fault_code, FaultCode::GraceTimeout);
}

TEST(JudgeBeiyun, SatelliteInsufficient) {
  auto s = GateState{};
  auto e = ByEvidence();
  e.sats_main = 8;  // < 10
  auto v = judge_beiyun(e, s, DefaultCfg());
  EXPECT_FALSE(v.is_valid);
  EXPECT_EQ(v.fault_code, FaultCode::SatelliteInsufficient);
  EXPECT_EQ(v.rtk_status, RtkStatus::RtkFix);  // 硬门控不改 rtk_status
}

TEST(JudgeBeiyun, BaselineAbnormalOnlyNarrowInt) {
  auto s = GateState{};
  auto e = ByEvidence();
  e.heading_length = 2.1;  // 偏离 1.6 超 0.2
  auto v = judge_beiyun(e, s, DefaultCfg());
  EXPECT_FALSE(v.is_valid);
  EXPECT_EQ(v.fault_code, FaultCode::BaselineAbnormal);
  // 非 NARROW_INT 不检查
  auto s2 = GateState{};
  e.heading_narrow_int = false;
  auto v2 = judge_beiyun(e, s2, DefaultCfg());
  EXPECT_TRUE(v2.is_valid);
}

TEST(JudgeBeiyun, EnuOutOfBounds) {
  auto s = GateState{};
  auto e = ByEvidence();
  e.enu_x = 20000.0;
  auto v = judge_beiyun(e, s, DefaultCfg());
  EXPECT_FALSE(v.is_valid);
  EXPECT_EQ(v.fault_code, FaultCode::EnuOutOfBounds);
}

// === 华测分支矩阵 ===

TEST(JudgeHuace, RtkFixBasic) {
  auto s = GateState{};
  auto v = judge_huace(HcEvidence(), s, DefaultCfg());
  EXPECT_TRUE(v.is_valid);
  EXPECT_EQ(v.rtk_status, RtkStatus::RtkFix);  // 透传 sta 高4位
  EXPECT_EQ(v.fault_code, FaultCode::None);
}

TEST(JudgeHuace, HardwareWarning) {
  auto s = GateState{};
  auto e = HcEvidence();
  e.hw_warning = true;
  auto v = judge_huace(e, s, DefaultCfg());
  EXPECT_FALSE(v.is_valid);
  EXPECT_EQ(v.fault_code, FaultCode::HardwareWarning);
  EXPECT_EQ(v.rtk_status, RtkStatus::RtkFix);  // rtk_status 无条件透传
}

TEST(JudgeHuace, InsNotCombined) {
  auto s = GateState{};
  auto e = HcEvidence();
  e.ins_good = false;
  auto v = judge_huace(e, s, DefaultCfg());
  EXPECT_FALSE(v.is_valid);
  EXPECT_EQ(v.fault_code, FaultCode::InsNotConverged);
}

TEST(JudgeHuace, InsNotConvergedSuppressesGraceTimeout) {
  // INS 未收敛时不叠加 GraceTimeout（不遮蔽根因）
  Config cfg = DefaultCfg();
  cfg.unfixed_time = 1.0;
  auto s = GateState{};
  auto e = HcEvidence();
  judge_huace(e, s, cfg);
  e.huace_gnss_pos_type = 1;  // 单点
  e.ins_good = false;
  e.now = 3.0;  // 超宽限
  auto v = judge_huace(e, s, cfg);
  EXPECT_FALSE(v.is_valid);
  EXPECT_EQ(v.fault_code, FaultCode::InsNotConverged);  // 不含 GraceTimeout
}

TEST(JudgeHuace, GracePerMode) {
  Config cfg = DefaultCfg();
  cfg.unfixed_time = 1.0;
  cfg.rtk_float_grace_time = 2.0;
  cfg.combined_nav_mod_time = 3.0;
  // 浮点：2.0s 宽限
  {
    auto s = GateState{};
    auto e = HcEvidence();
    judge_huace(e, s, cfg);                       // t=0 fix
    e.huace_gnss_pos_type = 5; e.now = 1.5;       // 浮点 1.5s（超 unfixed=1 但 < float=2）
    auto v = judge_huace(e, s, cfg);
    EXPECT_TRUE(v.is_valid);                      // 浮点走 rtk_float_grace_time
    e.now = 2.6;
    v = judge_huace(e, s, cfg);
    EXPECT_FALSE(v.is_valid);
    EXPECT_EQ(v.fault_code, FaultCode::GraceTimeout);
  }
  // 组合推算：3.0s 宽限
  {
    auto s = GateState{};
    auto e = HcEvidence();
    judge_huace(e, s, cfg);
    e.huace_gnss_pos_type = 3; e.now = 2.5;       // 超 float 档但 < 组合推算档
    auto v = judge_huace(e, s, cfg);
    EXPECT_TRUE(v.is_valid);
    e.now = 3.5;
    v = judge_huace(e, s, cfg);
    EXPECT_FALSE(v.is_valid);
    EXPECT_EQ(v.fault_code, FaultCode::GraceTimeout);
  }
}

TEST(JudgeHuace, SatelliteInsufficient) {
  auto s = GateState{};
  auto e = HcEvidence();
  e.sats_main = 8;
  auto v = judge_huace(e, s, DefaultCfg());
  EXPECT_FALSE(v.is_valid);
  EXPECT_EQ(v.fault_code, FaultCode::SatelliteInsufficient);
}

// === RaiseFault 优先级 ===

TEST(JudgeFault, RaiseKeepsHighest) {
  FaultCode f = FaultCode::InsNotConverged;   // 1
  RaiseFault(f, FaultCode::HardwareWarning);  // 3 > 1
  EXPECT_EQ(f, FaultCode::HardwareWarning);
  RaiseFault(f, FaultCode::GraceTimeout);     // 2 < 3 不覆盖
  EXPECT_EQ(f, FaultCode::HardwareWarning);
}
