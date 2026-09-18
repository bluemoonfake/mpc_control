#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace mpc_controller::state_admission
{

struct TimestampHistory
{
  std::uint64_t boot_time = 0;
  std::uint64_t synchronized_time = 0;
  bool last_was_synchronized = false;
  bool received = false;
};

struct TimestampDecision
{
  bool accepted = false;
  bool domain_switched = false;
};

inline bool synchronizedTimestamp(std::uint64_t timestamp) noexcept
{
  constexpr std::uint64_t synchronized_epoch_threshold_us = 1000000000000ULL;
  return timestamp >= synchronized_epoch_threshold_us;
}

inline TimestampDecision acceptTimestamp(
  TimestampHistory &history, std::uint64_t timestamp) noexcept
{
  if (timestamp == 0) {
    return {};
  }

  const bool synchronized = synchronizedTimestamp(timestamp);
  std::uint64_t &previous = synchronized ? history.synchronized_time : history.boot_time;
  if (previous != 0 && timestamp < previous) {
    return {};
  }

  const bool domain_switched = history.received && synchronized != history.last_was_synchronized;
  previous = timestamp;
  history.last_was_synchronized = synchronized;
  history.received = true;
  return {true, domain_switched};
}

struct Timing
{
  std::uint64_t sample_time = 0;
  double age = std::numeric_limits<double>::infinity();
  bool received = false;
};

enum class Reject : std::uint8_t
{
  none,
  position_stale,
  attitude_stale,
  rate_stale,
  sample_skew,
  timestamp,
  invalid_configuration
};

struct Decision
{
  Reject reason = Reject::timestamp;
  double skew = std::numeric_limits<double>::infinity();
  bool valid = false;
};

inline bool fresh(const Timing &source, double timeout) noexcept
{
  return source.received && std::isfinite(source.age) && source.age >= 0.0
    && source.age <= timeout;
}

inline Decision evaluate(
  const Timing &position, const Timing &attitude, const Timing &rate,
  double timeout, double max_skew) noexcept
{
  Decision result;
  if (!std::isfinite(timeout) || !std::isfinite(max_skew) || timeout < 0.0 || max_skew < 0.0) {
    result.reason = Reject::invalid_configuration;
    return result;
  }
  if (!fresh(position, timeout)) {
    result.reason = Reject::position_stale;
    return result;
  }
  if (!fresh(attitude, timeout)) {
    result.reason = Reject::attitude_stale;
    return result;
  }
  if (!fresh(rate, timeout)) {
    result.reason = Reject::rate_stale;
    return result;
  }
  if (position.sample_time == 0 || attitude.sample_time == 0 || rate.sample_time == 0) {
    result.reason = Reject::timestamp;
    return result;
  }
  const auto oldest = std::min({position.sample_time, attitude.sample_time, rate.sample_time});
  const auto newest = std::max({position.sample_time, attitude.sample_time, rate.sample_time});
  result.skew = static_cast<double>(newest - oldest) * 1.0e-6;
  if (!std::isfinite(result.skew) || result.skew > max_skew) {
    result.reason = Reject::sample_skew;
    return result;
  }
  result.reason = Reject::none;
  result.valid = true;
  return result;
}

inline const char *reasonName(Reject reason) noexcept
{
  switch (reason) {
    case Reject::none: return "NONE";
    case Reject::position_stale: return "POSITION_STALE";
    case Reject::attitude_stale: return "ATTITUDE_STALE";
    case Reject::rate_stale: return "ANGULAR_VELOCITY_STALE";
    case Reject::sample_skew: return "SAMPLE_SKEW";
    case Reject::timestamp: return "TIMESTAMP";
    case Reject::invalid_configuration: return "INVALID_CONFIGURATION";
  }
  return "UNKNOWN";
}

}  // namespace mpc_controller::state_admission
