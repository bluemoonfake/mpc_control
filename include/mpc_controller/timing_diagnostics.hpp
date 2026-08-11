#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>

namespace mpc_controller::timing
{

enum class FreshnessSource : std::uint8_t
{
  none = 0,
  status_not_ready = 1,
  missing_stamp = 2,
  m3 = 3,
  m2 = 4,
  status = 5,
  invalid_age = 6
};

struct FreshnessDecision
{
  bool valid = false;
  FreshnessSource source = FreshnessSource::invalid_age;
};

inline FreshnessDecision classifyFreshness(
  const double m3_age_seconds,
  const double m2_age_seconds,
  const double status_age_seconds,
  const bool status_ready,
  const bool stamps_valid,
  const double timeout_seconds) noexcept
{
  FreshnessDecision result;
  if (!std::isfinite(timeout_seconds) || timeout_seconds <= 0.0 ||
    !std::isfinite(m3_age_seconds) || !std::isfinite(m2_age_seconds) ||
    !std::isfinite(status_age_seconds)) {
    result.source = FreshnessSource::invalid_age;
    return result;
  }
  if (!status_ready) {
    result.source = FreshnessSource::status_not_ready;
    return result;
  }
  if (!stamps_valid) {
    result.source = FreshnessSource::missing_stamp;
    return result;
  }
  const auto fresh = [timeout_seconds](const double age) {
      return age >= 0.0 && age <= timeout_seconds;
    };
  if (!fresh(m3_age_seconds)) {
    result.source = FreshnessSource::m3;
    return result;
  }
  if (!fresh(m2_age_seconds)) {
    result.source = FreshnessSource::m2;
    return result;
  }
  if (!fresh(status_age_seconds)) {
    result.source = FreshnessSource::status;
    return result;
  }
  result.valid = true;
  result.source = FreshnessSource::none;
  return result;
}

inline const char *freshnessSourceName(const FreshnessSource source) noexcept
{
  switch (source) {
    case FreshnessSource::none: return "NONE";
    case FreshnessSource::status_not_ready: return "STATUS_NOT_READY";
    case FreshnessSource::missing_stamp: return "MISSING_STAMP";
    case FreshnessSource::m3: return "M3_RECEIPT_AGE";
    case FreshnessSource::m2: return "M2_RECEIPT_AGE";
    case FreshnessSource::status: return "STATUS_RECEIPT_AGE";
    case FreshnessSource::invalid_age: return "INVALID_AGE";
  }
  return "UNKNOWN";
}

inline std::uint64_t steadyNowNs() noexcept
{
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}

struct TimerSample
{
  std::uint64_t sequence = 0;
  std::uint64_t configured_period_ns = 0;
  std::uint64_t expected_fire_steady_timestamp_ns = 0;
  std::uint64_t actual_start_steady_timestamp_ns = 0;
  std::uint64_t callback_end_steady_timestamp_ns = 0;
  double scheduling_lateness_ms = 0.0;
  double callback_duration_ms = 0.0;
};

class TimerTracker final
{
public:
  explicit TimerTracker(const double period_seconds = 0.0) noexcept
  {
    setPeriodSeconds(period_seconds);
  }

  void setPeriodSeconds(const double period_seconds) noexcept
  {
    period_ns_ = std::isfinite(period_seconds) && period_seconds > 0.0
      ? static_cast<std::uint64_t>(period_seconds * 1.0e9) : 0U;
    expected_fire_ns_ = 0U;
    sample_ = TimerSample{};
  }

  TimerSample begin(const std::uint64_t actual_start_ns) noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++sequence_;
    if (expected_fire_ns_ == 0U) {
      expected_fire_ns_ = actual_start_ns;
    } else if (period_ns_ > 0U) {
      expected_fire_ns_ += period_ns_;
    }
    sample_ = TimerSample{};
    sample_.sequence = sequence_;
    sample_.configured_period_ns = period_ns_;
    sample_.expected_fire_steady_timestamp_ns = expected_fire_ns_;
    sample_.actual_start_steady_timestamp_ns = actual_start_ns;
    const auto lateness_ns = static_cast<std::int64_t>(actual_start_ns) -
      static_cast<std::int64_t>(expected_fire_ns_);
    sample_.scheduling_lateness_ms = static_cast<double>(lateness_ns) * 1.0e-6;
    active_start_ns_ = actual_start_ns;
    return sample_;
  }

  void finish(const std::uint64_t sequence, const std::uint64_t callback_end_ns) noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (sequence != sample_.sequence || callback_end_ns < active_start_ns_) {
      return;
    }
    sample_.callback_end_steady_timestamp_ns = callback_end_ns;
    sample_.callback_duration_ms = static_cast<double>(callback_end_ns - active_start_ns_) * 1.0e-6;
  }

  TimerSample last() const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return sample_;
  }

private:
  std::uint64_t period_ns_ = 0;
  std::uint64_t expected_fire_ns_ = 0;
  std::uint64_t sequence_ = 0;
  std::uint64_t active_start_ns_ = 0;
  TimerSample sample_{};
  mutable std::mutex mutex_;
};

class TimerGuard final
{
public:
  TimerGuard(TimerTracker &tracker, const std::uint64_t start_ns) noexcept
  : tracker_(tracker), sample_(tracker_.begin(start_ns)) {}

  ~TimerGuard() {tracker_.finish(sample_.sequence, steadyNowNs());}

  TimerGuard(const TimerGuard &) = delete;
  TimerGuard &operator=(const TimerGuard &) = delete;

  const TimerSample &sample() const noexcept {return sample_;}

private:
  TimerTracker &tracker_;
  TimerSample sample_{};
};

}  // namespace mpc_controller::timing
