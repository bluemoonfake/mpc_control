#include "mpc_control_px4/px4_estimator_validity_monitor.hpp"

#include <cmath>

namespace mpc_control_px4
{

bool Px4EstimatorValidityMonitor::finiteState(
    const px4_msgs::msg::VehicleLocalPosition& message) noexcept
{
  return std::isfinite(message.x) && std::isfinite(message.y)
    && std::isfinite(message.z) && std::isfinite(message.vx)
    && std::isfinite(message.vy) && std::isfinite(message.vz)
    && std::isfinite(message.heading);
}

Px4EstimatorUpdateResult Px4EstimatorValidityMonitor::update(
    const px4_msgs::msg::VehicleLocalPosition& message,
    const TimePoint received_at) noexcept
{
  Px4EstimatorUpdateResult result;
  if (message.timestamp == 0U) {
    return result;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (have_message_ && message.timestamp <= last_message_timestamp_) {
    result.valid = latest_valid_;
    result.out_of_order = true;
    return result;
  }

  const ResetCounters next_counters{
    message.xy_reset_counter,
    message.z_reset_counter,
    message.vxy_reset_counter,
    message.vz_reset_counter,
    message.heading_reset_counter};

  if (have_message_ &&
      (next_counters.xy != counters_.xy || next_counters.z != counters_.z ||
       next_counters.vxy != counters_.vxy || next_counters.vz != counters_.vz ||
       next_counters.heading != counters_.heading)) {
    reset_pending_ = true;
    ++reset_generation_;
    result.reset_detected = true;
  }

  have_message_ = true;
  latest_valid_ = message.xy_valid && message.z_valid && message.v_xy_valid
    && message.v_z_valid && message.heading_good_for_control
    && finiteState(message);
  last_message_timestamp_ = message.timestamp;
  last_received_at_ = received_at;
  counters_ = next_counters;

  result.accepted = true;
  result.valid = latest_valid_;
  return result;
}

bool Px4EstimatorValidityMonitor::usable(
    const TimePoint now,
    const double max_age_seconds) const noexcept
{
  if (!std::isfinite(max_age_seconds) || max_age_seconds <= 0.0) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!have_message_ || !latest_valid_ || reset_pending_) {
    return false;
  }

  const double age_seconds = std::chrono::duration<double>(
      now - last_received_at_).count();
  return std::isfinite(age_seconds) && age_seconds >= 0.0
    && age_seconds <= max_age_seconds;
}

bool Px4EstimatorValidityMonitor::resetPending() const noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  return reset_pending_;
}

bool Px4EstimatorValidityMonitor::acknowledgeReset() noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!have_message_ || !latest_valid_ || !reset_pending_) {
    return false;
  }
  reset_pending_ = false;
  return true;
}

std::uint64_t Px4EstimatorValidityMonitor::resetGeneration() const noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  return reset_generation_;
}

void Px4EstimatorValidityMonitor::reset() noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  have_message_ = false;
  latest_valid_ = false;
  reset_pending_ = false;
  reset_generation_ = 0;
  last_message_timestamp_ = 0;
  last_received_at_ = TimePoint{};
  counters_ = ResetCounters{};
}

}  // namespace mpc_control_px4
