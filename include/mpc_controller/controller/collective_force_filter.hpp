#pragma once

#include <cmath>
#include <optional>

namespace mpc_controller::state_estimation {

class CollectiveForceFilter {
public:
  bool configure(double time_constant_seconds, double sample_time_seconds) noexcept
  {
    if (!std::isfinite(time_constant_seconds) || time_constant_seconds < 0.0 ||
        !std::isfinite(sample_time_seconds) || sample_time_seconds <= 0.0) {
      return false;
    }
    smoothing_factor_ = time_constant_seconds == 0.0
                            ? 1.0
                            : 1.0 - std::exp(-sample_time_seconds /
                                             time_constant_seconds);
    reset();
    return true;
  }

  void reset() noexcept { filtered_value_.reset(); }

  std::optional<double> update(double measurement) noexcept
  {
    if (!std::isfinite(measurement) || !std::isfinite(smoothing_factor_) ||
        smoothing_factor_ <= 0.0 || smoothing_factor_ > 1.0) {
      return std::nullopt;
    }
    if (!filtered_value_) {
      filtered_value_ = measurement;
    } else {
      *filtered_value_ += smoothing_factor_ * (measurement - *filtered_value_);
    }
    return filtered_value_;
  }

private:
  double smoothing_factor_ = 0.0;
  std::optional<double> filtered_value_;
};

} // namespace mpc_controller::state_estimation
