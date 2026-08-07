#include "mpc_control_px4/px4_estimator_validity_monitor.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <limits>

namespace
{

using namespace std::chrono_literals;
using mpc_control_px4::Px4EstimatorValidityMonitor;
using px4_msgs::msg::VehicleLocalPosition;

VehicleLocalPosition validMessage(const std::uint64_t timestamp)
{
  VehicleLocalPosition message;
  message.timestamp = timestamp;
  message.xy_valid = true;
  message.z_valid = true;
  message.v_xy_valid = true;
  message.v_z_valid = true;
  message.x = 1.0F;
  message.y = 2.0F;
  message.z = -3.0F;
  message.vx = 0.1F;
  message.vy = 0.2F;
  message.vz = 0.3F;
  message.heading = 0.4F;
  message.heading_good_for_control = true;
  return message;
}

TEST(Px4EstimatorValidityMonitorTest, RequiresAllControlValidityFlags)
{
  Px4EstimatorValidityMonitor monitor;
  const auto now = Px4EstimatorValidityMonitor::Clock::now();
  auto message = validMessage(100U);

  auto result = monitor.update(message, now);
  EXPECT_TRUE(result.accepted);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(monitor.usable(now + 10ms, 0.25));

  message.timestamp = 200U;
  message.v_xy_valid = false;
  result = monitor.update(message, now + 20ms);
  EXPECT_TRUE(result.accepted);
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(monitor.usable(now + 20ms, 0.25));

  message.timestamp = 300U;
  message.v_xy_valid = true;
  message.heading = std::numeric_limits<float>::quiet_NaN();
  result = monitor.update(message, now + 40ms);
  EXPECT_TRUE(result.accepted);
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(monitor.usable(now + 40ms, 0.25));
}

TEST(Px4EstimatorValidityMonitorTest, RejectsStaleAndOutOfOrderUpdates)
{
  Px4EstimatorValidityMonitor monitor;
  const auto now = Px4EstimatorValidityMonitor::Clock::now();
  EXPECT_FALSE(monitor.update(validMessage(0U), now).accepted);
  EXPECT_TRUE(monitor.update(validMessage(100U), now).accepted);

  const auto result = monitor.update(validMessage(99U), now + 10ms);
  EXPECT_FALSE(result.accepted);
  EXPECT_TRUE(result.out_of_order);
  EXPECT_TRUE(monitor.usable(now + 100ms, 0.25));
  EXPECT_FALSE(monitor.usable(now + 251ms, 0.25));
}

TEST(Px4EstimatorValidityMonitorTest, LatchesAnyResetCounterChangeUntilAcknowledged)
{
  Px4EstimatorValidityMonitor monitor;
  const auto now = Px4EstimatorValidityMonitor::Clock::now();
  auto message = validMessage(100U);
  EXPECT_TRUE(monitor.update(message, now).valid);
  EXPECT_FALSE(monitor.resetPending());

  message.timestamp = 200U;
  message.xy_reset_counter = 1U;
  auto result = monitor.update(message, now + 20ms);
  EXPECT_TRUE(result.accepted);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.reset_detected);
  EXPECT_TRUE(monitor.resetPending());
  EXPECT_EQ(monitor.resetGeneration(), 1U);
  EXPECT_FALSE(monitor.usable(now + 20ms, 0.25));

  EXPECT_TRUE(monitor.acknowledgeReset());
  EXPECT_FALSE(monitor.resetPending());
  EXPECT_TRUE(monitor.usable(now + 20ms, 0.25));
  EXPECT_FALSE(monitor.acknowledgeReset());
}

}  // namespace
