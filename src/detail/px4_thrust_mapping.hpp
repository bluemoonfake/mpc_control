#pragma once

#include <cmath>
#include <optional>

namespace mpc_controller::px4_thrust
{

inline constexpr double kNormalizedLimit = 1.0;
inline constexpr double kDefaultVehicleMassKg = 2.0;
inline constexpr double kDefaultGravityMps2 = 9.80665;
inline constexpr double kDefaultHoverThrustNormalized = 0.765;

struct Mapping
{
  double vehicle_mass_kg = kDefaultVehicleMassKg;
  double gravity_mps2 = kDefaultGravityMps2;
  double hover_thrust_normalized = kDefaultHoverThrustNormalized;
};

inline bool valid(const Mapping &mapping) noexcept
{
  return std::isfinite(mapping.vehicle_mass_kg) && mapping.vehicle_mass_kg > 0.0
    && std::isfinite(mapping.gravity_mps2) && mapping.gravity_mps2 > 0.0
    && std::isfinite(mapping.hover_thrust_normalized)
    && mapping.hover_thrust_normalized > 0.0
    && mapping.hover_thrust_normalized <= kNormalizedLimit;
}

inline std::optional<double> forceToBodyFrdZ(
  double thrust_force_n, const Mapping &mapping) noexcept
{
  if (!valid(mapping) || !std::isfinite(thrust_force_n) || thrust_force_n < 0.0) {
    return std::nullopt;
  }

  const double hover_force_n = mapping.vehicle_mass_kg * mapping.gravity_mps2;
  const double maximum_supported_force_n =
    hover_force_n / mapping.hover_thrust_normalized;
  if (!std::isfinite(hover_force_n)
    || !std::isfinite(maximum_supported_force_n)
    || thrust_force_n > maximum_supported_force_n) {
    return std::nullopt;
  }

  // Positive FLU collective thrust maps to PX4's negative body-FRD Z.
  return -mapping.hover_thrust_normalized * thrust_force_n / hover_force_n;
}

}  // namespace mpc_controller::px4_thrust
