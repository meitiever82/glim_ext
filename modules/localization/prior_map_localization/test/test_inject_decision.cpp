#include <gtest/gtest.h>
#include <glim_ext/inject_decision.hpp>
#include <glim_ext/initial_pose_source.hpp>

TEST(InjectDecision, Hysteresis) {
  glim::InjectDecision d(0.10);          // enter 0.10, exit 0.08
  EXPECT_FALSE(d.update(0.09));          // below enter, not active
  EXPECT_TRUE(d.update(0.11));           // crosses enter -> active
  EXPECT_TRUE(d.update(0.09));           // 0.09 >= 0.08 exit -> stays active
  EXPECT_FALSE(d.update(0.07));          // below exit -> deactivates
  EXPECT_FALSE(d.update(0.09));          // 0.09 < 0.10 enter -> stays inactive
}

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
