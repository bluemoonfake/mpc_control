#include "mpc_control_px4/px4_timestamp_source.hpp"

#include <cmath>
#include <limits>

namespace mpc_control_px4
{

bool Px4TimestampSource::update(
    const px4_msgs::msg::TimesyncStatus& status,
    const TimePoint received_at) noexcept
{
  if (status.source_protocol != px4_msgs::msg::TimesyncStatus::SOURCE_PROTOCOL_DDS
      || status.timestamp == 0U) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (anchor_ && status.timestamp <= anchor_->px4_timestamp_us) {
    return false;
  }

  anchor_ = Anchor{status.timestamp, received_at};
  return true;
}

std::optional<std::uint64_t> Px4TimestampSource::nextTimestamp(
    const TimePoint now,
    const double max_age_seconds) noexcept
{
  if (!std::isfinite(max_age_seconds) || max_age_seconds <= 0.0) {
    return std::nullopt;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!anchor_) {
    return std::nullopt;
  }

  const double age_seconds = std::chrono::duration<double>(
      now - anchor_->received_at).count();
  if (!std::isfinite(age_seconds) || age_seconds < 0.0
      || age_seconds > max_age_seconds) {
    return std::nullopt;
  }

  const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
      now - anchor_->received_at);
  if (elapsed.count() < 0) {
    return std::nullopt;
  }
  const auto elapsed_us = static_cast<std::uint64_t>(elapsed.count());
  if (anchor_->px4_timestamp_us >
      std::numeric_limits<std::uint64_t>::max() - elapsed_us) {
    return std::nullopt;
  }

  auto timestamp = anchor_->px4_timestamp_us + elapsed_us;
  if (last_issued_timestamp_us_ && timestamp <= *last_issued_timestamp_us_) {
    if (*last_issued_timestamp_us_ == std::numeric_limits<std::uint64_t>::max()) {
      return std::nullopt;
    }
    timestamp = *last_issued_timestamp_us_ + 1U;
  }

  last_issued_timestamp_us_ = timestamp;
  return timestamp;
}

void Px4TimestampSource::reset() noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  anchor_.reset();
  last_issued_timestamp_us_.reset();
}

}  // namespace mpc_control_px4
