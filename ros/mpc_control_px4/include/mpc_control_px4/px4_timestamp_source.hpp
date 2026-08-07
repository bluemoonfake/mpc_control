#pragma once

#include <px4_msgs/msg/timesync_status.hpp>

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>

namespace mpc_control_px4
{

class Px4TimestampSource final
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  bool update(
      const px4_msgs::msg::TimesyncStatus& status,
      TimePoint received_at = Clock::now()) noexcept;

  std::optional<std::uint64_t> nextTimestamp(
      TimePoint now = Clock::now(),
      double max_age_seconds = 0.5) noexcept;

  void reset() noexcept;

private:
  struct Anchor
  {
    std::uint64_t px4_timestamp_us = 0;
    TimePoint received_at{};
  };

  std::mutex mutex_;
  std::optional<Anchor> anchor_;
  std::optional<std::uint64_t> last_issued_timestamp_us_;
};

}  // namespace mpc_control_px4
