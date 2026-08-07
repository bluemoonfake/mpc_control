#pragma once

#include "reference_trajectory.hpp"

#include <array>
#include <cstddef>

namespace mpc_control
{

inline constexpr std::size_t kReferenceHorizonLength = 26;

struct ReferenceHorizon
{
  double current_time_seconds = 0.0;
  std::array<double, kReferenceHorizonLength> relative_times{};
  std::array<ReferencePoint, kReferenceHorizonLength> samples{};
};

struct ReferenceSamplerConfig
{
  double dt_first = 0.01;
  double dt_later = 0.20;
  bool hold_after_end = true;
  ReferenceValidationOptions validation{};
};

struct ReferenceSampleResult
{
  bool valid = false;
  ReferenceError error = ReferenceError::InvalidGrid;
  std::size_t error_index = 0;
  ReferenceHorizon horizon{};
};

class ReferenceSampler
{
public:
  ReferenceSampleResult sample(
      const ReferenceTrajectory& trajectory,
      double current_time_seconds,
      const ReferenceSamplerConfig& configuration = {}) const noexcept;

private:
  static bool validGrid(const ReferenceSamplerConfig& configuration) noexcept;
  static ReferencePoint interpolate(
      const ReferencePoint& lower,
      const ReferencePoint& upper,
      double time_seconds) noexcept;
  static double wrapYaw(double yaw) noexcept;
};

}  // namespace mpc_control
