#pragma once

#include <Eigen/Core>

#include <cmath>
#include <optional>

namespace mpc_controller::thrust_feasibility
{

using Vector3 = Eigen::Vector3d;

// This is the external-controller contract, not the Gazebo plant limit.
struct Parameters
{
  double mass_kg = 2.0;
  double gravity_m_s2 = 9.80665;
  double hover_thrust_normalized = 0.765;
  double max_normalized_collective_thrust = 1.0;
};

struct Result
{
  Vector3 raw_acceleration_m_s2{};
  Vector3 feasible_acceleration_m_s2{};
  Vector3 raw_force_world_n{};
  Vector3 feasible_force_world_n{};
  double raw_force_norm_n = 0.0;
  double feasible_force_norm_n = 0.0;
  double force_limit_n = 0.0;
  double acceleration_correction_norm_m_s2 = 0.0;
  bool constraint_active = false;
};

inline bool finite(const Vector3 &value) noexcept
{
  return value.allFinite();
}

inline bool validParameters(const Parameters &parameters) noexcept
{
  return std::isfinite(parameters.mass_kg) && parameters.mass_kg > 0.0
    && std::isfinite(parameters.gravity_m_s2) && parameters.gravity_m_s2 > 0.0
    && std::isfinite(parameters.hover_thrust_normalized)
    && parameters.hover_thrust_normalized > 0.0
    && parameters.hover_thrust_normalized <= 1.0
    && std::isfinite(parameters.max_normalized_collective_thrust)
    && parameters.max_normalized_collective_thrust > 0.0
    && parameters.max_normalized_collective_thrust <= 1.0;
}

inline double controlMaxCollectiveThrustN(const Parameters &parameters) noexcept
{
  if (!validParameters(parameters)) {
    return NAN;
  }
  return parameters.max_normalized_collective_thrust * parameters.mass_kg
    * parameters.gravity_m_s2 / parameters.hover_thrust_normalized;
}

// Project the unconstrained three-axis acceleration command radially in force
// space. This is deliberately explicit command shaping between independent
// X/Y/Z MPCs and M3; it is not a hidden M4 clamp and is not claimed to be
// identical to PX4 PositionControl's vertical-priority saturation.
inline std::optional<Result> project(const Parameters &parameters, const Vector3 &raw)
noexcept
{
  if (!validParameters(parameters) || !finite(raw)) {
    return std::nullopt;
  }

  const Vector3 gravity_world(0.0, 0.0, -parameters.gravity_m_s2);
  const Vector3 raw_force = parameters.mass_kg * (raw - gravity_world);
  const double raw_norm = raw_force.norm();
  const double force_limit = controlMaxCollectiveThrustN(parameters);
  // A zero or downward force cannot define the requested upward body-Z
  // direction. The lower collective boundary is therefore F_z > 0, or
  // equivalently a_z > -g for this attitude/thrust path.
  if (!finite(raw_force) || !std::isfinite(raw_norm) || raw_norm < 1.0e-9
    || raw_force.z() <= 1.0e-9 || !std::isfinite(force_limit) || force_limit <= 0.0) {
    return std::nullopt;
  }

  Result result;
  result.raw_acceleration_m_s2 = raw;
  result.raw_force_world_n = raw_force;
  result.raw_force_norm_n = raw_norm;
  result.force_limit_n = force_limit;
  result.constraint_active = raw_norm > force_limit;
  result.feasible_force_world_n = result.constraint_active
    ? raw_force * (force_limit / raw_norm) : raw_force;
  result.feasible_force_norm_n = result.feasible_force_world_n.norm();
  result.feasible_acceleration_m_s2 = result.feasible_force_world_n / parameters.mass_kg
    + gravity_world;
  result.acceleration_correction_norm_m_s2 =
    (result.feasible_acceleration_m_s2 - raw).norm();

  if (!finite(result.feasible_force_world_n) || !finite(result.feasible_acceleration_m_s2)
    || !std::isfinite(result.feasible_force_norm_n)
    || result.feasible_force_world_n.z() <= 1.0e-9
    || result.feasible_force_norm_n > force_limit + 1.0e-9) {
    return std::nullopt;
  }
  return result;
}

}  // namespace mpc_controller::thrust_feasibility
