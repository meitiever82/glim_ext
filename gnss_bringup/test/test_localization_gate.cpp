#include <gtest/gtest.h>
#include <chrono>
#include <limits>
#include <string>
#include "gnss_bringup/localization_gate.hpp"

using namespace gnss_bringup::loc;

namespace {

using Clock = std::chrono::steady_clock;
constexpr Clock::time_point kBase = Clock::time_point{};

auto at(double seconds) -> Clock::time_point {
  return kBase + std::chrono::duration_cast<Clock::duration>(
                     std::chrono::duration<double>(seconds));
}

/// 一帧"设备状态正常"的输入，各用例只改关心的字段。
auto good_input(PosType type = PosType::InsRtkfixed,
                InsStatus ins = InsStatus::InsSolutionGood) -> GateInput {
  GateInput in;
  in.pos_type = type;
  in.ins_status = ins;
  in.age = 0.5;
  in.svs = 20;
  in.svs2 = 20;
  return in;
}

}  // namespace

// ---------- 三个确定性分支 ----------

TEST(LocGate, FixedSolutionIsValidRegardlessOfInsStatus) {
  LocalizationGate gate;
  GateInput in = good_input(PosType::InsRtkfixed, InsStatus::InsAligning);
  const GateOutput out = gate.update(in, at(0.0));
  EXPECT_TRUE(out.is_valid);
  EXPECT_EQ(out.rtk_status, RtkStatus::RtkFix);
  EXPECT_EQ(out.fault_code, FaultCode::None);
}

TEST(LocGate, FloatSolutionWithGoodInsIsValid) {
  LocalizationGate gate;
  const GateOutput out = gate.update(good_input(PosType::InsRtkfloat), at(0.0));
  EXPECT_TRUE(out.is_valid);
  EXPECT_EQ(out.rtk_status, RtkStatus::RtkFloat);
  EXPECT_EQ(out.fault_code, FaultCode::None);
}

TEST(LocGate, FloatSolutionWithoutGoodInsIsInvalid) {
  LocalizationGate gate;
  const GateOutput out = gate.update(
      good_input(PosType::InsRtkfloat, InsStatus::InsAligning), at(0.0));
  EXPECT_FALSE(out.is_valid);
  EXPECT_EQ(out.rtk_status, RtkStatus::RtkInit);
  EXPECT_EQ(out.fault_code, FaultCode::InsNotConverged);
}

TEST(LocGate, SinglePointGraceKeepsPreviousValidityAndDowngradesRtk) {
  LocalizationGate gate;
  // 先固定解可用，再降级到单点且 INS 未收敛、龄期小于阈值：
  // 语义是"宽限降级"——只改 rtk_status，is_valid / fault_code 沿用上一帧。
  ASSERT_TRUE(gate.update(good_input(), at(0.0)).is_valid);
  GateInput in = good_input(PosType::InsPsrsp, InsStatus::InsAligning);
  in.age = 5.0;
  const GateOutput out = gate.update(in, at(1.0));
  EXPECT_TRUE(out.is_valid);
  EXPECT_EQ(out.rtk_status, RtkStatus::RtkFloat);
  EXPECT_EQ(out.fault_code, FaultCode::None);
}

TEST(LocGate, SinglePointGraceOnColdStartStaysInvalidWithoutFault) {
  // 旧实现的已知行为：冷启动首帧单点宽限只改 rtk_status，is_valid 仍是初值 false、
  // fault_code 仍是 None。此处保持逐条一致，T2 的告警不能只看 is_valid 与
  // fault_code 的一致性假设。
  LocalizationGate gate;
  GateInput in = good_input(PosType::InsPsrsp, InsStatus::InsAligning);
  in.age = 5.0;
  const GateOutput out = gate.update(in, at(0.0));
  EXPECT_FALSE(out.is_valid);
  EXPECT_EQ(out.rtk_status, RtkStatus::RtkFloat);
  EXPECT_EQ(out.fault_code, FaultCode::None);
}

TEST(LocGate, SinglePointBeyondAgeThresholdIsInvalid) {
  LocalizationGate gate;
  GateInput in = good_input(PosType::InsPsrsp, InsStatus::InsAligning);
  in.age = 10.0;  // 阈值是严格小于
  const GateOutput out = gate.update(in, at(0.0));
  EXPECT_FALSE(out.is_valid);
  EXPECT_EQ(out.rtk_status, RtkStatus::RtkInit);
  EXPECT_EQ(out.fault_code, FaultCode::InsNotConverged);
}

// ---------- 粘滞与宽限 ----------

TEST(LocGate, StickyHoldsPreviousResultWithinGrace) {
  LocalizationGate gate;
  ASSERT_TRUE(gate.update(good_input(), at(0.0)).is_valid);
  const GateOutput out =
      gate.update(good_input(PosType::InsPsrdiff), at(1.0));
  EXPECT_TRUE(out.is_valid) << "宽限内保持上一次的固定解可用状态";
  EXPECT_EQ(out.rtk_status, RtkStatus::RtkFix);
  EXPECT_EQ(out.fault_code, FaultCode::None);
  EXPECT_TRUE(gate.in_sticky());
}

TEST(LocGate, StickyTimesOutAtGraceBoundary) {
  LocalizationGate gate;
  ASSERT_TRUE(gate.update(good_input(), at(0.0)).is_valid);
  ASSERT_TRUE(gate.update(good_input(PosType::InsPsrdiff), at(1.0)).is_valid);

  // 粘滞起点是 1.0s；默认宽限 30s：恰好 30s 时不超时（严格大于才判无效）
  GateOutput out = gate.update(good_input(PosType::InsPsrdiff), at(31.0));
  EXPECT_TRUE(out.is_valid);

  out = gate.update(good_input(PosType::InsPsrdiff), at(31.001));
  EXPECT_FALSE(out.is_valid);
  EXPECT_EQ(out.rtk_status, RtkStatus::RtcmDiff);
  EXPECT_EQ(out.fault_code, FaultCode::GraceTimeout);
}

TEST(LocGate, StickyTimeoutMapsSinglePointToAposit) {
  LocalizationGate gate;
  ASSERT_TRUE(gate.update(good_input(), at(0.0)).is_valid);
  ASSERT_TRUE(gate.update(good_input(PosType::InsPsrsp), at(1.0)).is_valid);
  const GateOutput out = gate.update(good_input(PosType::InsPsrsp), at(40.0));
  EXPECT_FALSE(out.is_valid);
  EXPECT_EQ(out.rtk_status, RtkStatus::Aposit);
  EXPECT_EQ(out.fault_code, FaultCode::GraceTimeout);
}

TEST(LocGate, StickyTimeoutKeepsUnknownSolutionTypeStatus) {
  // 无对应质量等级的解类型（None）：rtk_status 保持上一帧，仍判无效
  LocalizationGate gate;
  ASSERT_TRUE(gate.update(good_input(), at(0.0)).is_valid);
  ASSERT_TRUE(gate.update(good_input(PosType::None), at(1.0)).is_valid);
  const GateOutput out = gate.update(good_input(PosType::None), at(40.0));
  EXPECT_FALSE(out.is_valid);
  EXPECT_EQ(out.rtk_status, RtkStatus::RtkFix);
  EXPECT_EQ(out.fault_code, FaultCode::GraceTimeout);
}

TEST(LocGate, StickyGraceRestartsAfterDeterministicBranch) {
  LocalizationGate gate;
  ASSERT_TRUE(gate.update(good_input(), at(0.0)).is_valid);
  // 进入粘滞
  ASSERT_TRUE(gate.update(good_input(PosType::InsPsrdiff), at(1.0)).is_valid);
  ASSERT_TRUE(gate.in_sticky());
  // 恢复固定解 → 命中确定性分支，退出粘滞
  ASSERT_TRUE(gate.update(good_input(), at(2.0)).is_valid);
  EXPECT_FALSE(gate.in_sticky());
  // 再次降级：宽限重新起算
  const GateOutput out = gate.update(good_input(PosType::InsPsrdiff), at(20.0));
  EXPECT_TRUE(out.is_valid);
  EXPECT_EQ(out.rtk_status, RtkStatus::RtkFix);
}

TEST(LocGate, StickyTimeoutDoesNotLowerHigherPriorityFault) {
  // 上一帧因卫星不足被判无效（SatelliteInsufficient=4，高于 GraceTimeout=2），
  // 粘滞超时叠加原因时不得把根因降级。
  LocalizationGate gate;
  GateInput bad_sv = good_input();
  bad_sv.svs = 5;
  const GateOutput first = gate.update(bad_sv, at(0.0));
  ASSERT_FALSE(first.is_valid);
  ASSERT_EQ(first.fault_code, FaultCode::SatelliteInsufficient);

  // 粘滞路径沿用上一帧的无效状态与故障原因（不会因为解类型变化自己变回有效）
  const GateOutput within =
      gate.update(good_input(PosType::InsPsrdiff), at(1.0));
  EXPECT_FALSE(within.is_valid);
  EXPECT_EQ(within.fault_code, FaultCode::SatelliteInsufficient);

  const GateOutput out = gate.update(good_input(PosType::InsPsrdiff), at(40.0));
  EXPECT_FALSE(out.is_valid);
  EXPECT_EQ(out.fault_code, FaultCode::SatelliteInsufficient);
}

// ---------- 硬门控 ----------

TEST(LocGate, EnuOutOfBoundsInvalidatesButKeepsRtkStatus) {
  LocalizationGate gate;
  GateInput in = good_input();
  in.x = 20000.0;
  const GateOutput out = gate.update(in, at(0.0));
  EXPECT_FALSE(out.is_valid);
  EXPECT_EQ(out.rtk_status, RtkStatus::RtkFix) << "rtk_status 不被门控压低";
  EXPECT_EQ(out.fault_code, FaultCode::EnuOutOfBounds);
}

TEST(LocGate, NonFiniteEnuIsOutOfBounds) {
  LocalizationGate gate;
  GateInput in = good_input();
  in.y = std::numeric_limits<double>::quiet_NaN();
  const GateOutput out = gate.update(in, at(0.0));
  EXPECT_FALSE(out.is_valid);
  EXPECT_EQ(out.fault_code, FaultCode::EnuOutOfBounds);
}

TEST(LocGate, SatelliteInsufficientOnEitherAntenna) {
  LocalizationGate gate;
  GateInput main_low = good_input();
  main_low.svs = 9;
  GateOutput out = gate.update(main_low, at(0.0));
  EXPECT_FALSE(out.is_valid);
  EXPECT_EQ(out.fault_code, FaultCode::SatelliteInsufficient);

  LocalizationGate gate2;
  GateInput second_low = good_input();
  second_low.svs2 = 0;  // 第二天线无数据也判不足（旧实现无 0 豁免）
  out = gate2.update(second_low, at(0.0));
  EXPECT_FALSE(out.is_valid);
  EXPECT_EQ(out.fault_code, FaultCode::SatelliteInsufficient);

  LocalizationGate gate3;
  GateInput at_threshold = good_input();
  at_threshold.svs = 10;
  at_threshold.svs2 = 10;
  EXPECT_TRUE(gate3.update(at_threshold, at(0.0)).is_valid);
}

TEST(LocGate, HeadingBaselineAbnormalOnlyForNarrowInt) {
  LocalizationGate gate;
  GateInput in = good_input();
  in.heading_narrow_int = true;
  in.heading_length = 2.0;  // 默认标称 1.276、阈值 0.2
  GateOutput out = gate.update(in, at(0.0));
  EXPECT_FALSE(out.is_valid);
  EXPECT_EQ(out.fault_code, FaultCode::BaselineAbnormal);

  // 非 NARROW_INT 不参与基线门控
  LocalizationGate gate2;
  in.heading_narrow_int = false;
  EXPECT_TRUE(gate2.update(in, at(0.0)).is_valid);

  // 阈值内正常
  LocalizationGate gate3;
  in.heading_narrow_int = true;
  in.heading_length = 1.3;
  EXPECT_TRUE(gate3.update(in, at(0.0)).is_valid);
}

TEST(LocGate, FaultPriorityPicksHighestCode) {
  LocalizationGate gate;
  GateInput in = good_input();
  in.x = 20000.0;           // EnuOutOfBounds = 6
  in.svs = 5;               // SatelliteInsufficient = 4
  in.heading_narrow_int = true;
  in.heading_length = 3.0;  // BaselineAbnormal = 5
  const GateOutput out = gate.update(in, at(0.0));
  EXPECT_FALSE(out.is_valid);
  EXPECT_EQ(out.fault_code, FaultCode::EnuOutOfBounds);
}

TEST(LocGate, GoodFrameClearsPreviousFault) {
  LocalizationGate gate;
  GateInput bad = good_input();
  bad.svs = 5;
  ASSERT_EQ(gate.update(bad, at(0.0)).fault_code,
            FaultCode::SatelliteInsufficient);
  const GateOutput out = gate.update(good_input(), at(0.1));
  EXPECT_TRUE(out.is_valid);
  EXPECT_EQ(out.fault_code, FaultCode::None);
}

TEST(LocGate, CustomParamsAreHonoured) {
  GateParams p;
  p.stars_threshold = 15;
  p.xyz_threshold = 100.0;
  p.sticky_grace_time = -1.0;  // 负值：第一帧粘滞后即判无效
  LocalizationGate gate(p);

  GateInput in = good_input();
  in.svs = 12;  // 旧默认 10 够用，新阈值 15 不足
  EXPECT_EQ(gate.update(in, at(0.0)).fault_code,
            FaultCode::SatelliteInsufficient);

  LocalizationGate gate2(p);
  GateInput far = good_input();
  far.x = 150.0;
  EXPECT_EQ(gate2.update(far, at(0.0)).fault_code, FaultCode::EnuOutOfBounds);

  LocalizationGate gate3(p);
  ASSERT_TRUE(gate3.update(good_input(), at(0.0)).is_valid);
  const GateOutput out =
      gate3.update(good_input(PosType::InsPsrdiff), at(1.0));
  EXPECT_FALSE(out.is_valid) << "负宽限应在首帧粘滞后立即判无效";
  EXPECT_EQ(out.fault_code, FaultCode::GraceTimeout);
}
