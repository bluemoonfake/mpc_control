#pragma once

#include "mpc_controller/domain/model/types.hpp"

#include <algorithm>
#include <cmath>

namespace mpc_controller::translational
{

struct SolveLimits
{
  double max_speed_xy = 0.0;
  double max_speed_z = 0.0;
  double max_acceleration_xy = 0.0;
  double max_acceleration_z = 0.0;
  double max_control_rate_xy = 0.0;
  double max_control_rate_z = 0.0;
};

inline bool finite(const SolveLimits &limits) noexcept
{
  const double values[] = {
    limits.max_speed_xy, limits.max_speed_z,
    limits.max_acceleration_xy, limits.max_acceleration_z,
    limits.max_control_rate_xy, limits.max_control_rate_z};
  for (const double value : values) {
    if (!std::isfinite(value) || value <= 0.0) {
      return false;
    }
  }
  return true;
}

inline bool convertTrackingLimits(
  const reference::TrackingLimits &requested,
  const SolveLimits &configured,
  SolveLimits &effective) noexcept
{
  if (!finite(configured)) {
    return false;
  }
  const double requested_values[] = {
    requested.max_speed_xy, requested.max_speed_z,
    requested.max_acceleration_xy, requested.max_acceleration_z,
    requested.max_control_rate_xy, requested.max_control_rate_z};
  const double configured_values[] = {
    configured.max_speed_xy, configured.max_speed_z,
    configured.max_acceleration_xy, configured.max_acceleration_z,
    configured.max_control_rate_xy, configured.max_control_rate_z};
  double *effective_values[] = {
    &effective.max_speed_xy, &effective.max_speed_z,
    &effective.max_acceleration_xy, &effective.max_acceleration_z,
    &effective.max_control_rate_xy, &effective.max_control_rate_z};

  for (std::size_t index = 0; index < 6U; ++index) {
    if (!std::isfinite(requested_values[index]) || requested_values[index] < 0.0) {
      return false;
    }
    *effective_values[index] = requested_values[index] > 0.0
      ? std::min(requested_values[index], configured_values[index])
      : configured_values[index];
  }
  return true;
}

}  // namespace mpc_controller::translational
