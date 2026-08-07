#include "mpc_control_px4/px4_timestamp_source.hpp"

#include <gtest/gtest.h>

#include <chrono>

namespace
{

using namespace std::chrono_literals;
using mpc_control_px4::Px4TimestampSource;
using px4_msgs::msg::TimesyncStatus;

TimesyncStatus makeStatus(const std::uint64_t timestamp)
{
  TimesyncStatus status;
  status.timestamp = timestamp;
  status.source_protocol = TimesyncStatus::SOURCE_PROTOCOL_DDS;
  return status;
}

TEST(Px4TimestampSourceTest, AcceptsFreshAnchorAfterTimesyncGap)
{
  Px4TimestampSource source;
  const auto t0 = Px4TimestampSource::Clock::now();

  ASSERT_TRUE(source.update(makeStatus(1000U), t0));
  ASSERT_TRUE(source.nextTimestamp(t0 + 100ms, 0.5).has_value());
  EXPECT_FALSE(source.nextTimestamp(t0 + 700ms, 0.5).has_value());

  ASSERT_TRUE(source.update(makeStatus(2000U), t0 + 800ms));
  const auto timestamp = source.nextTimestamp(t0 + 900ms, 0.5);
  ASSERT_TRUE(timestamp.has_value());
  EXPECT_EQ(*timestamp, 102000U);
}

TEST(Px4TimestampSourceTest, RejectsTimestampRollbackAfterTransportRestart)
{
  Px4TimestampSource source;
  const auto t0 = Px4TimestampSource::Clock::now();

  ASSERT_TRUE(source.update(makeStatus(5000U), t0));
  ASSERT_TRUE(source.nextTimestamp(t0 + 100ms, 0.5).has_value());

  EXPECT_FALSE(source.update(makeStatus(1000U), t0 + 200ms));
  ASSERT_TRUE(source.nextTimestamp(t0 + 300ms, 0.5).has_value());
  EXPECT_FALSE(source.nextTimestamp(t0 + 700ms, 0.5).has_value());
}

TEST(Px4TimestampSourceTest, RejectsNonDdsAndZeroAnchors)
{
  Px4TimestampSource source;
  const auto t0 = Px4TimestampSource::Clock::now();

  auto zero = makeStatus(0U);
  EXPECT_FALSE(source.update(zero, t0));

  auto mavlink = makeStatus(1000U);
  mavlink.source_protocol = TimesyncStatus::SOURCE_PROTOCOL_MAVLINK;
  EXPECT_FALSE(source.update(mavlink, t0));
  EXPECT_FALSE(source.nextTimestamp(t0 + 1ms, 0.5).has_value());
}

}  // namespace
