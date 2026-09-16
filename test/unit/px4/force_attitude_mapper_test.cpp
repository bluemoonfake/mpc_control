#include "mpc_controller/px4/force_attitude_mapper.hpp"
#include "mpc_controller/px4/geometric_mapping.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

constexpr double kTolerance = 1.0e-9;

TEST(ForceAttitudeMapper, HoverProducesLevelAttitudeAndGravitySpecificForce) {
  mpc_controller::force_attitude::Parameters parameters;
  parameters.max_tilt_rad = 0.75;

  mpc_controller::force_attitude::Input input;
  input.desired_acceleration_m_s2.setZero();
  input.desired_yaw_rad = 0.0;
  input.valid = true;

  const auto output =
      mpc_controller::force_attitude::compute(parameters, input);

  ASSERT_TRUE(output.valid);
  EXPECT_NEAR(output.desired_specific_force_world_m_s2.x(), 0.0, kTolerance);
  EXPECT_NEAR(output.desired_specific_force_world_m_s2.y(), 0.0, kTolerance);
  EXPECT_NEAR(output.desired_specific_force_world_m_s2.z(),
              parameters.gravity_m_s2, kTolerance);
  EXPECT_NEAR(output.desired_collective_specific_force_m_s2,
              parameters.gravity_m_s2, kTolerance);
  EXPECT_NEAR(output.tilt_angle_rad, 0.0, kTolerance);
  EXPECT_NEAR(output.desired_body_to_world.angularDistance(
                  Eigen::Quaterniond::Identity()),
              0.0, kTolerance);
}

TEST(ForceAttitudeMapper, RejectsAccelerationOutsideTiltLimit) {
  mpc_controller::force_attitude::Parameters parameters;
  parameters.max_tilt_rad = 0.1;

  mpc_controller::force_attitude::Input input;
  input.desired_acceleration_m_s2 = Eigen::Vector3d(5.0, 0.0, 0.0);
  input.desired_yaw_rad = 0.0;
  input.valid = true;

  const auto output =
      mpc_controller::force_attitude::compute(parameters, input);

  EXPECT_FALSE(output.valid);
  EXPECT_EQ(output.failure_reason,
            mpc_controller::force_attitude::FailureReason::tilt_limit);
}

TEST(Px4FrameConversion, ConvertsEnuYawRateToNedWithOppositeSign) {
  const auto positive =
      mpc_controller::px4_control::enuYawRateToNed(0.75);
  const auto negative =
      mpc_controller::px4_control::enuYawRateToNed(-0.75);
  const auto invalid = mpc_controller::px4_control::enuYawRateToNed(NAN);

  ASSERT_TRUE(positive.has_value());
  ASSERT_TRUE(negative.has_value());
  EXPECT_DOUBLE_EQ(*positive, -0.75);
  EXPECT_DOUBLE_EQ(*negative, 0.75);
  EXPECT_FALSE(invalid.has_value());
}

} // namespace
