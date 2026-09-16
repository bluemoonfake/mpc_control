#include "mpc_controller/px4/pid_reference.hpp"
#include <gtest/gtest.h>

namespace pv = mpc_controller::pid_validation;
pv::Reference line() {
  pv::Point a, b;
  a.position = {1, 2, 3}; a.velocity = {2, 4, 6};
  b = a; b.time_from_start = 1; b.position = {3, 6, 9};
  return {100, false, {a, b}};
}
TEST(PidReference, SamplesCurrentTimeAndConvertsAxes) {
  const auto p = pv::sampleNed(line(), 100.25, .01, 1.5);
  ASSERT_TRUE(p);
  EXPECT_FLOAT_EQ(p->position.x(), 3); EXPECT_FLOAT_EQ(p->position.y(), 1.5);
  EXPECT_FLOAT_EQ(p->position.z(), -4.5);
  EXPECT_FLOAT_EQ(p->velocity.x(), 4); EXPECT_FLOAT_EQ(p->velocity.z(), -6);
  EXPECT_NEAR(p->yaw, M_PI/2, 1e-6);
}
TEST(PidReference, AccelerationAndYawWrap) {
  auto r = line(); r.points[0].yaw = 179*M_PI/180; r.points[1].yaw = -179*M_PI/180;
  for (auto &p : r.points) {p.acceleration = {1,2,3}; p.yaw_rate = .3;}
  const auto p = pv::sampleNed(r,100.5,.01,1.5); ASSERT_TRUE(p);
  EXPECT_NEAR(p->yaw,-M_PI/2,1e-6); EXPECT_FLOAT_EQ(p->yaw_rate,-.3f);
  EXPECT_FLOAT_EQ(p->acceleration.x(),2); EXPECT_FLOAT_EQ(p->acceleration.y(),1);
  EXPECT_FLOAT_EQ(p->acceleration.z(),-3);
}
TEST(PidReference, RejectsInvalidStaleAndOutOfRange) {
  auto r = line();
  EXPECT_FALSE(pv::sampleNed(r,100.1,2,1.5));
  EXPECT_FALSE(pv::sampleNed(r,102,0,1.5));
  EXPECT_FALSE(pv::sampleNed(r,99,0,1.5));
  EXPECT_FALSE(pv::sampleNed(r,101.1,0,1.5));
  EXPECT_FALSE(pv::sampleNed(r,NAN,0,1.5));
  EXPECT_FALSE(pv::sampleNed(r,100,0,-1));
  r.points[1].time_from_start=0; EXPECT_FALSE(pv::sampleNed(r,100,0,1.5));
  r=line(); r.points[0].velocity[0]=NAN; EXPECT_FALSE(pv::sampleNed(r,100,0,1.5));
}
TEST(PidReference, FinalHoldZerosFeedforwardOnlyWhileFresh) {
  auto r=line(); r.hold_after_end=true;
  auto p=pv::sampleNed(r,101.1,.01,1.5); ASSERT_TRUE(p);
  EXPECT_FLOAT_EQ(p->position.z(),-9); EXPECT_FLOAT_EQ(p->velocity.norm(),0);
  EXPECT_FLOAT_EQ(p->acceleration.norm(),0); EXPECT_FLOAT_EQ(p->yaw_rate,0);
  EXPECT_FALSE(pv::sampleNed(r,102,0,1.5));
}
