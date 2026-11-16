#include <gtest/gtest.h>
#include <chrono>
#include <limits>
#include <string>
#include "gnss_bringup/loc_alarm_state.hpp"

using namespace gnss_bringup::loc;

namespace {

using Clock = std::chrono::steady_clock;
constexpr Clock::time_point kBase = Clock::time_point{};

auto at(double seconds) -> Clock::time_point {
  return kBase + std::chrono::duration_cast<Clock::duration>(
                     std::chrono::duration<double>(seconds));
}

}  // namespace

TEST(LocAlarm, StartupGraceSuppressesEverything) {
  LocAlarmState state(LocAlarmState::Params{}, kBase);
  EXPECT_FALSE(state.update(true, "定位无效：X", at(5.0)).has_value());
  EXPECT_FALSE(state.active());
}

TEST(LocAlarm, InvalidAddsImmediatelyAfterGrace) {
  LocAlarmState state(LocAlarmState::Params{}, kBase);
  const auto act = state.update(true, "定位无效：X", at(10.0));
  ASSERT_TRUE(act.has_value());
  EXPECT_TRUE(act->add);
  EXPECT_EQ(act->msg, "定位无效：X");
  EXPECT_TRUE(state.active());
}

TEST(LocAlarm, ValidWithoutActiveAlarmDoesNothing) {
  LocAlarmState state(LocAlarmState::Params{}, kBase);
  EXPECT_FALSE(state.update(false, "", at(50.0)).has_value());
  EXPECT_FALSE(state.active());
}

TEST(LocAlarm, DescriptionChangeIsDebouncedByMinInterval) {
  LocAlarmState state(LocAlarmState::Params{}, kBase);
  ASSERT_TRUE(state.update(true, "A", at(10.0)).has_value());

  // 距上次发布 5s < min_interval 10s：不重发
  EXPECT_FALSE(state.update(true, "B", at(15.0)).has_value());
  // 满 10s：重发 ADD 并更新描述
  const auto act = state.update(true, "B", at(20.0));
  ASSERT_TRUE(act.has_value());
  EXPECT_TRUE(act->add);
  EXPECT_EQ(act->msg, "B");
}

TEST(LocAlarm, SameDescriptionRepublishesOnCooldown) {
  LocAlarmState state(LocAlarmState::Params{}, kBase);
  ASSERT_TRUE(state.update(true, "A", at(10.0)).has_value());
  EXPECT_FALSE(state.update(true, "A", at(100.0)).has_value());
  const auto act = state.update(true, "A", at(310.0));
  ASSERT_TRUE(act.has_value());
  EXPECT_TRUE(act->add);
  EXPECT_EQ(act->msg, "A");
}

TEST(LocAlarm, RecoveryNeedsTwoStableSeconds) {
  LocAlarmState state(LocAlarmState::Params{}, kBase);
  ASSERT_TRUE(state.update(true, "A", at(10.0)).has_value());

  EXPECT_FALSE(state.update(false, "", at(20.0)).has_value());
  EXPECT_FALSE(state.update(false, "", at(21.9)).has_value());
  const auto act = state.update(false, "", at(22.0));
  ASSERT_TRUE(act.has_value());
  EXPECT_FALSE(act->add);
  EXPECT_EQ(act->msg, LocAlarmState::kRecoverMsg);
  EXPECT_FALSE(state.active());
}

TEST(LocAlarm, RecoveryInterruptedByFaultRestartsStableWindow) {
  LocAlarmState state(LocAlarmState::Params{}, kBase);
  ASSERT_TRUE(state.update(true, "A", at(10.0)).has_value());

  EXPECT_FALSE(state.update(false, "", at(20.0)).has_value());
  // 恢复期闪断一次 → 稳定计时作废
  EXPECT_FALSE(state.update(true, "A", at(21.0)).has_value());
  EXPECT_FALSE(state.update(false, "", at(22.0)).has_value());
  EXPECT_FALSE(state.update(false, "", at(23.9)).has_value());
  const auto act = state.update(false, "", at(24.0));
  ASSERT_TRUE(act.has_value());
  EXPECT_FALSE(act->add);
  EXPECT_FALSE(state.active());
}

TEST(LocAlarm, InvalidParamsFallBackToDefaults) {
  LocAlarmState::Params p;
  p.startup_grace_s = std::numeric_limits<double>::quiet_NaN();
  p.recover_stable_s = 0.0;
  p.min_interval_s = -1.0;
  p.cooldown_s = std::numeric_limits<double>::quiet_NaN();
  LocAlarmState state(p, kBase);

  // startup_grace 回退 10s
  EXPECT_FALSE(state.update(true, "A", at(9.9)).has_value());
  ASSERT_TRUE(state.update(true, "A", at(10.0)).has_value());
  // recover_stable 回退 2s
  EXPECT_FALSE(state.update(false, "", at(11.9)).has_value());
  const auto act = state.update(false, "", at(13.9));
  ASSERT_TRUE(act.has_value());
  EXPECT_FALSE(act->add);
}

TEST(LocAlarmBuildMsg, IncludesReasonAndRtkContext) {
  LocAlarmInput in;
  in.is_valid = false;
  in.fault_code = FaultCode::InsNotConverged;
  in.rtk_status = RtkStatus::RtkFix;
  in.svs = 20;
  in.svs2 = 18;
  in.age = 1.5;
  EXPECT_EQ(LocAlarmState::build_fault_msg(in),
            "定位无效：INS未收敛（rtk=RtkFix(4) svs=20/18 age=1.5s）");

  in.fault_code = FaultCode::GraceTimeout;
  EXPECT_EQ(LocAlarmState::build_fault_msg(in),
            "定位无效：非固定解超过宽限期（rtk=RtkFix(4) svs=20/18 age=1.5s）");
}

TEST(LocAlarmBuildMsg, NoDataUsesConnectionStateInsteadOfRtkContext) {
  LocAlarmInput in;
  in.is_valid = false;
  in.fault_code = FaultCode::NoData;
  in.client_connected = true;
  EXPECT_EQ(LocAlarmState::build_fault_msg(in),
            "定位无效：未收到定位数据（设备已连接）");

  in.fault_code = FaultCode::DataTimeout;
  in.client_connected = false;
  EXPECT_EQ(LocAlarmState::build_fault_msg(in),
            "定位无效：定位数据超时（设备未连接）");
}

TEST(LocAlarmBuildMsg, UnknownFaultCodeStillProducesMessage) {
  LocAlarmInput in;
  in.fault_code = static_cast<FaultCode>(200);
  EXPECT_NE(LocAlarmState::build_fault_msg(in).find("未知原因"),
            std::string::npos);
}
