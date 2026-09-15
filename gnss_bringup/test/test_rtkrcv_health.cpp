#include <gtest/gtest.h>

#include <string>

#include "gnss_bringup/rtkrcv_health.hpp"
using namespace gnss_bringup;
using S = diagnostic_msgs::msg::DiagnosticStatus;

namespace {
RtkrcvHealthInput healthy() {
  RtkrcvHealthInput in;
  in.child_running = true;
  in.since_start_s = 600.0;
  in.since_solution_s = 0.4;
  in.since_uplink_s = 0.1;
  return in;
}
bool contains(const std::string& s, const char* needle) { return s.find(needle) != std::string::npos; }
}  // namespace

TEST(RtkrcvHealth, RecentSolutionIsOk) {
  const auto st = evaluate_rtkrcv_health(healthy(), 30.0);
  EXPECT_EQ(st.name, "rtkrcv_node");
  EXPECT_EQ(st.level, S::OK);
}

TEST(RtkrcvHealth, ChildNotRunningIsErrorEvenWithAFreshSolution) {
  auto in = healthy();
  in.child_running = false;
  const auto st = evaluate_rtkrcv_health(in, 30.0);
  EXPECT_EQ(st.level, S::ERROR);
  EXPECT_TRUE(contains(st.message, "未在运行")) << st.message;
}

TEST(RtkrcvHealth, SilentSolverWithoutUplinkPointsAtTheLink) {
  auto in = healthy();
  in.since_solution_s = 31.0;
  in.since_uplink_s.reset();
  auto st = evaluate_rtkrcv_health(in, 30.0);
  EXPECT_EQ(st.level, S::WARN);
  EXPECT_TRUE(contains(st.message, "上行无数据")) << st.message;
  in.since_uplink_s = 45.0;   // 有过上行,但也停了
  st = evaluate_rtkrcv_health(in, 30.0);
  EXPECT_TRUE(contains(st.message, "上行无数据")) << st.message;
}

TEST(RtkrcvHealth, SilentSolverWithUplinkPointsAtTheConf) {
  auto in = healthy();
  in.since_solution_s = 31.0;
  const auto st = evaluate_rtkrcv_health(in, 30.0);
  EXPECT_EQ(st.level, S::WARN);
  EXPECT_TRUE(contains(st.message, "base_pos_type")) << st.message;
}

TEST(RtkrcvHealth, NeverSolvedCountsFromStartAndTheBoundaryIsStrict) {
  auto in = healthy();
  in.since_solution_s.reset();
  in.since_start_s = 30.0;
  auto st = evaluate_rtkrcv_health(in, 30.0);
  EXPECT_EQ(st.level, S::OK) << "恰好等于门限不报";
  EXPECT_TRUE(contains(st.message, "等待")) << st.message;
  in.since_start_s = 30.5;
  st = evaluate_rtkrcv_health(in, 30.0);
  EXPECT_EQ(st.level, S::WARN);
}
