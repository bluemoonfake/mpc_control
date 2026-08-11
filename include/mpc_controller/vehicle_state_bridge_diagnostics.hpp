#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace mpc_controller::vehicle_state_diagnostics
{

enum class RejectReason : std::uint8_t
{
  none = 0,
  position_stale = 1,
  attitude_stale = 2,
  angular_velocity_stale = 3,
  sample_skew = 4,
  non_finite = 5,
  timestamp = 6
};

struct SourceTiming
{
  std::uint64_t sample_timestamp = 0;
  std::uint64_t receipt_steady_timestamp_ns = 0;
  double age_seconds = std::numeric_limits<double>::infinity();
  double interarrival_seconds = std::numeric_limits<double>::infinity();
  bool received = false;
};

struct Decision
{
  RejectReason reason = RejectReason::timestamp;
  double sample_skew_seconds = std::numeric_limits<double>::infinity();
  bool valid = false;
};

inline double ageSeconds(
  const std::uint64_t evaluation_steady_timestamp_ns,
  const std::uint64_t receipt_steady_timestamp_ns) noexcept
{
  if (evaluation_steady_timestamp_ns < receipt_steady_timestamp_ns) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return static_cast<double>(evaluation_steady_timestamp_ns - receipt_steady_timestamp_ns) * 1.0e-9;
}

inline bool fresh(const SourceTiming &source, const double timeout_seconds) noexcept
{
  return source.received && std::isfinite(source.age_seconds) && source.age_seconds >= 0.0 &&
         source.age_seconds <= timeout_seconds;
}

inline Decision evaluate(
  const SourceTiming &position,
  const SourceTiming &attitude,
  const SourceTiming &angular_velocity,
  const std::uint64_t evaluation_steady_timestamp_ns,
  const double timeout_seconds,
  const double max_sample_skew_seconds) noexcept
{
  Decision result;
  result.sample_skew_seconds = std::numeric_limits<double>::infinity();
  if (!fresh(position, timeout_seconds)) {
    result.reason = RejectReason::position_stale;
    return result;
  }
  if (!fresh(attitude, timeout_seconds)) {
    result.reason = RejectReason::attitude_stale;
    return result;
  }
  if (!fresh(angular_velocity, timeout_seconds)) {
    result.reason = RejectReason::angular_velocity_stale;
    return result;
  }
  if (position.sample_timestamp == 0 || attitude.sample_timestamp == 0 ||
    angular_velocity.sample_timestamp == 0)
  {
    result.reason = RejectReason::timestamp;
    return result;
  }
  const auto minimum = std::min({
      position.sample_timestamp, attitude.sample_timestamp, angular_velocity.sample_timestamp});
  const auto maximum = std::max({
      position.sample_timestamp, attitude.sample_timestamp, angular_velocity.sample_timestamp});
  result.sample_skew_seconds = static_cast<double>(maximum - minimum) * 1.0e-6;
  if (!std::isfinite(result.sample_skew_seconds) ||
    result.sample_skew_seconds > max_sample_skew_seconds)
  {
    result.reason = RejectReason::sample_skew;
    return result;
  }
  (void)evaluation_steady_timestamp_ns;
  result.reason = RejectReason::none;
  result.valid = true;
  return result;
}

inline const char *rejectReasonName(const RejectReason reason) noexcept
{
  switch (reason) {
    case RejectReason::none: return "NONE";
    case RejectReason::position_stale: return "POSITION_STALE";
    case RejectReason::attitude_stale: return "ATTITUDE_STALE";
    case RejectReason::angular_velocity_stale: return "ANGULAR_VELOCITY_STALE";
    case RejectReason::sample_skew: return "SAMPLE_SKEW";
    case RejectReason::non_finite: return "NON_FINITE";
    case RejectReason::timestamp: return "TIMESTAMP";
  }
  return "UNKNOWN";
}

}  // namespace mpc_controller::vehicle_state_diagnostics
