#include "mpc_control_px4/px4_trajectory_adapter.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace
{

using mpc_control_msgs::msg::TrajectoryCommand;
using mpc_control_px4::Px4AdapterFailureReason;
using mpc_control_px4::Px4TrajectoryAdapter;

constexpr double kPi = 3.141592653589793238462643383279502884;

TrajectoryCommand makeCommand()
{
  TrajectoryCommand command;
  command.header.frame_id = "map";
  command.trajectory_id = 11U;
  command.sequence = 3U;
  command.position = {1.0, 2.0, 3.0};
  command.velocity = {4.0, 5.0, 6.0};
  command.acceleration = {7.0, 8.0, 9.0};
  command.yaw = 0.0;
  command.yaw_rate = 0.25;
  command.yaw_valid = true;
  return command;
}

TEST(Px4TrajectoryAdapterTest, MapsPositionVelocityAndAccelerationEnuToNed)
{
  Px4TrajectoryAdapter adapter;
  const auto result = adapter.convert(makeCommand(), 1000000U);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.setpoint.timestamp, 1000000U);
  EXPECT_FLOAT_EQ(result.setpoint.position[0], 2.0F);
  EXPECT_FLOAT_EQ(result.setpoint.position[1], 1.0F);
  EXPECT_FLOAT_EQ(result.setpoint.position[2], -3.0F);
  EXPECT_FLOAT_EQ(result.setpoint.velocity[0], 5.0F);
  EXPECT_FLOAT_EQ(result.setpoint.velocity[1], 4.0F);
  EXPECT_FLOAT_EQ(result.setpoint.velocity[2], -6.0F);
  EXPECT_FLOAT_EQ(result.setpoint.acceleration[0], 8.0F);
  EXPECT_FLOAT_EQ(result.setpoint.acceleration[1], 7.0F);
  EXPECT_FLOAT_EQ(result.setpoint.acceleration[2], -9.0F);
}

TEST(Px4TrajectoryAdapterTest, MapsYawAndYawRateToNed)
{
  Px4TrajectoryAdapter adapter;
  auto command = makeCommand();
  command.yaw = 0.0;
  command.yaw_rate = 0.25;
  const auto result = adapter.convert(command, 1U);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.setpoint.yaw, static_cast<float>(kPi / 2.0), 1.0e-6F);
  EXPECT_FLOAT_EQ(result.setpoint.yawspeed, -0.25F);
}

TEST(Px4TrajectoryAdapterTest, WrapsYawDeterministically)
{
  Px4TrajectoryAdapter adapter;
  auto command = makeCommand();
  command.yaw = -kPi / 2.0;
  auto result = adapter.convert(command, 1U);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.setpoint.yaw, static_cast<float>(-kPi), 1.0e-6F);

  adapter.reset();
  command.yaw = 3.0 * kPi / 2.0;
  result = adapter.convert(command, 2U);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.setpoint.yaw, static_cast<float>(-kPi), 1.0e-6F);
}

TEST(Px4TrajectoryAdapterTest, InvalidYawDisablesYawControlOnly)
{
  Px4TrajectoryAdapter adapter;
  auto command = makeCommand();
  command.yaw = std::numeric_limits<double>::quiet_NaN();
  command.yaw_rate = std::numeric_limits<double>::quiet_NaN();
  command.yaw_valid = false;
  const auto result = adapter.convert(command, 1U);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(std::isnan(result.setpoint.yaw));
  EXPECT_TRUE(std::isnan(result.setpoint.yawspeed));
  EXPECT_TRUE(std::isfinite(result.setpoint.position[0]));
  EXPECT_TRUE(std::isfinite(result.setpoint.velocity[0]));
  EXPECT_TRUE(std::isfinite(result.setpoint.acceleration[0]));
  EXPECT_TRUE(std::isnan(result.setpoint.jerk[0]));
}

TEST(Px4TrajectoryAdapterTest, RejectsWrongFrameAndDoesNotDoubleConvertNed)
{
  Px4TrajectoryAdapter adapter;
  auto command = makeCommand();
  command.header.frame_id = "NED";
  const auto result = adapter.convert(command, 1U);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure_reason, Px4AdapterFailureReason::InvalidFrame);
}

TEST(Px4TrajectoryAdapterTest, RejectsNonFiniteAndFloatOverflow)
{
  Px4TrajectoryAdapter adapter;
  auto command = makeCommand();
  command.position[0] = std::numeric_limits<double>::quiet_NaN();
  auto result = adapter.convert(command, 1U);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(
    result.failure_reason,
    Px4AdapterFailureReason::NonFiniteTranslationalCommand);

  adapter.reset();
  command = makeCommand();
  command.position[0] = std::numeric_limits<double>::max();
  result = adapter.convert(command, 1U);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(
    result.failure_reason,
    Px4AdapterFailureReason::NonFiniteTranslationalCommand);
}

TEST(Px4TrajectoryAdapterTest, RequiresStrictlyIncreasingPx4Microseconds)
{
  Px4TrajectoryAdapter adapter;
  const auto command = makeCommand();

  auto result = adapter.convert(command, 100U);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.setpoint.timestamp, 100U);

  result = adapter.convert(command, 100U);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(
    result.failure_reason,
    Px4AdapterFailureReason::TimestampNotMonotonic);

  result = adapter.convert(command, 99U);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(
    result.failure_reason,
    Px4AdapterFailureReason::TimestampNotMonotonic);

  result = adapter.convert(command, 101U);
  EXPECT_TRUE(result.valid);

  adapter.reset();
  result = adapter.convert(command, 1U);
  EXPECT_TRUE(result.valid);
}

TEST(Px4TrajectoryAdapterTest, RejectsZeroTimestamp)
{
  Px4TrajectoryAdapter adapter;
  const auto result = adapter.convert(makeCommand(), 0U);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure_reason, Px4AdapterFailureReason::InvalidTimestamp);
}

}  // namespace
