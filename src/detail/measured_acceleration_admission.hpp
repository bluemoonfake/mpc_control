#pragma once

#include <array>
#include <cmath>

namespace mpc_controller::measured_acceleration
{

using Vector3 = std::array<double, 3>;

// This gate is deliberately an admission policy, not a filter. M2 models the
// latest measured x0=[p,v,a]; replacing an out-of-envelope acceleration with
// a clamped, zero, or old value would silently break that feedback contract.
enum class Reason
{
  none,
  invalid_limit,
  non_finite_measurement,
  z_out_of_model_envelope
};

struct Limits
{
  // Must be compatible with the configured Z acceleration state constraint.
  double max_abs_z_m_s2 = 2.0;
};

struct Result
{
  bool valid = false;
  // Identity conditioning: use only after valid=true.
  Vector3 conditioned_acceleration_m_s2{};
  Reason reason = Reason::invalid_limit;
};

inline bool finite(const Vector3 &acceleration) noexcept
{
  return std::isfinite(acceleration[0]) && std::isfinite(acceleration[1])
    && std::isfinite(acceleration[2]);
}

inline bool validLimits(const Limits &limits) noexcept
{
  return std::isfinite(limits.max_abs_z_m_s2) && limits.max_abs_z_m_s2 > 0.0;
}

inline Result admit(const Vector3 &raw_acceleration_m_s2, const Limits &limits) noexcept
{
  Result result;
  result.conditioned_acceleration_m_s2 = raw_acceleration_m_s2;
  if (!validLimits(limits)) {
    result.reason = Reason::invalid_limit;
    return result;
  }
  if (!finite(raw_acceleration_m_s2)) {
    result.reason = Reason::non_finite_measurement;
    return result;
  }
  if (std::abs(raw_acceleration_m_s2[2]) > limits.max_abs_z_m_s2) {
    result.reason = Reason::z_out_of_model_envelope;
    return result;
  }
  result.valid = true;
  result.reason = Reason::none;
  return result;
}

inline const char *reasonName(const Reason reason) noexcept
{
  switch (reason) {
    case Reason::none: return "NONE";
    case Reason::invalid_limit: return "INVALID_LIMIT";
    case Reason::non_finite_measurement: return "NON_FINITE_MEASUREMENT";
    case Reason::z_out_of_model_envelope: return "Z_OUT_OF_MODEL_ENVELOPE";
  }
  return "UNKNOWN";
}

}  // namespace mpc_controller::measured_acceleration
