#pragma once

#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>

namespace mpc_control_px4
{

struct Px4EstimatorUpdateResult
{
  bool accepted = false;
  bool valid = false;
  bool reset_detected = false;
  bool out_of_order = false;
};

class Px4EstimatorValidityMonitor final
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  Px4EstimatorUpdateResult update(
      const px4_msgs::msg::VehicleLocalPosition& message,
      TimePoint received_at = Clock::now()) noexcept;

  bool usable(
      TimePoint now = Clock::now(),
      double max_age_seconds = 0.25) const noexcept;

  bool resetPending() const noexcept;
  bool acknowledgeReset() noexcept;
  std::uint64_t resetGeneration() const noexcept;
  void reset() noexcept;

private:
  struct ResetCounters
  {
    std::uint8_t xy = 0;
    std::uint8_t z = 0;
    std::uint8_t vxy = 0;
    std::uint8_t vz = 0;
    std::uint8_t heading = 0;
  };

  static bool finiteState(
      const px4_msgs::msg::VehicleLocalPosition& message) noexcept;

  mutable std::mutex mutex_;
  bool have_message_ = false;
  bool latest_valid_ = false;
  bool reset_pending_ = false;
  std::uint64_t reset_generation_ = 0;
  std::uint64_t last_message_timestamp_ = 0;
  TimePoint last_received_at_{};
  ResetCounters counters_{};
};

}  // namespace mpc_control_px4
