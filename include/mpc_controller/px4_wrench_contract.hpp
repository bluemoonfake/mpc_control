#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/QR>

#include <array>
#include <cmath>
#include <limits>
#include <optional>

namespace mpc_controller::px4_wrench
{

using Vector3 = Eigen::Vector3d;

// Pinned PX4/gz_x500 model and runtime airframe parameters. PX4's
// VehicleThrustSetpoint is not T/Tmax at this boundary: the allocator maps
// the normalized control vector to actuator signals, then MixingOutput maps
// [-1, 1] to the Gazebo motor-speed interval.
inline constexpr int kRotorCount = 4;
inline constexpr double kRotorThrustCoefficient = 8.54858e-06;
inline constexpr double kMotorCommandMin = 150.0;
inline constexpr double kMotorCommandMax = 1000.0;
inline constexpr double kMaxCollectiveThrustN = 34.19432;
inline constexpr double kMinCollectiveThrustN =
  kRotorCount * kRotorThrustCoefficient * kMotorCommandMin * kMotorCommandMin;
inline constexpr double kNormalizedLimit = 1.0;
inline constexpr double kDefaultVehicleMassKg = 2.0;
inline constexpr double kDefaultGravityMps2 = 9.80665;
// Measured from the stock PX4 hover interval in the pinned gz_x500 ULog. This
// is a boundary calibration, not a motor-speed model parameter.
inline constexpr double kObservedHoverThrustNormalized = 0.765;

using Matrix6x4 = Eigen::Matrix<double, 6, 4>;
using Matrix4x6 = Eigen::Matrix<double, 4, 6>;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Vector4 = Eigen::Matrix<double, 4, 1>;

struct X500Effectiveness
{
  Matrix6x4 raw{};
  Matrix4x6 normalized_mix{};
  Vector6 normalization_scale{};
};

enum class Status
{
  within_range,
  saturated,
  invalid
};

enum class FailureReason
{
  none,
  non_finite,
  invalid_thrust_mapping,
  negative_thrust,
  thrust_out_of_range,
  normalized_torque_out_of_range
};

struct ConversionResult
{
  Status status = Status::invalid;
  FailureReason failure_reason = FailureReason::non_finite;
  Vector3 thrust_frd{};
  Vector3 torque_frd{};
};

struct ThrustMapping
{
  double vehicle_mass_kg = kDefaultVehicleMassKg;
  double gravity_mps2 = kDefaultGravityMps2;
  double hover_thrust_normalized = kObservedHoverThrustNormalized;
};

inline bool validThrustMapping(const ThrustMapping &mapping) noexcept
{
  return std::isfinite(mapping.vehicle_mass_kg) && mapping.vehicle_mass_kg > 0.0
    && std::isfinite(mapping.gravity_mps2) && mapping.gravity_mps2 > 0.0
    && std::isfinite(mapping.hover_thrust_normalized)
    && mapping.hover_thrust_normalized > 0.0
    && mapping.hover_thrust_normalized <= kNormalizedLimit;
}

struct OffboardFlags
{
  bool position = false;
  bool velocity = false;
  bool acceleration = false;
  bool attitude = false;
  bool body_rate = false;
  bool thrust_and_torque = true;
  bool direct_actuator = false;
};

inline bool commandReady(
  bool valid, bool math_valid, bool active_control_ready, bool timestamp_valid,
  double age_seconds, double timeout_seconds) noexcept
{
  return valid && math_valid && active_control_ready && timestamp_valid
    && std::isfinite(age_seconds) && age_seconds >= 0.0
    && std::isfinite(timeout_seconds) && timeout_seconds > 0.0
    && age_seconds <= timeout_seconds;
}

inline constexpr OffboardFlags wrenchOffboardFlags() noexcept
{
  return {};
}

inline Vector3 fluToFrd(const Vector3 &value) noexcept
{
  return Vector3(value.x(), -value.y(), -value.z());
}

inline X500Effectiveness x500Effectiveness()
{
  X500Effectiveness result;
  result.raw.setZero();
  result.normalization_scale.setOnes();

  const std::array<Vector3, kRotorCount> positions{
    Vector3(0.13, 0.22, 0.0), Vector3(-0.13, -0.20, 0.0),
    Vector3(0.13, -0.22, 0.0), Vector3(-0.13, 0.20, 0.0)};
  const std::array<double, kRotorCount> moment_ratios{0.05, 0.05, -0.05, -0.05};
  const Vector3 axis(0.0, 0.0, -1.0);

  for (int i = 0; i < kRotorCount; ++i) {
    const Vector3 thrust = 6.5 * axis;
    const Vector3 moment = 6.5 * positions[i].cross(axis)
      - 6.5 * moment_ratios[i] * axis;
    result.raw.block<3, 1>(0, i) = moment;
    result.raw.block<3, 1>(3, i) = thrust;
  }

  // This reproduces ControlAllocationPseudoInverse.cpp for the x500 matrix:
  // pseudo-inverse first, then per-axis normalization of the inverse matrix.
  const Matrix4x6 pseudo_inverse =
    result.raw.completeOrthogonalDecomposition().pseudoInverse();
  for (int axis_index = 0; axis_index < 6; ++axis_index) {
    const auto column = pseudo_inverse.col(axis_index);
    if (axis_index == 0 || axis_index == 1) {
      int non_zero = 0;
      for (int i = 0; i < kRotorCount; ++i) {
        non_zero += std::abs(column(i)) > 1.0e-3 ? 1 : 0;
      }
      result.normalization_scale(axis_index) = non_zero > 0
        ? std::sqrt(column.squaredNorm() / (static_cast<double>(non_zero) / 2.0))
        : 1.0;
    } else if (axis_index == 2) {
      result.normalization_scale(axis_index) = column.maxCoeff();
    } else {
      int non_zero = 0;
      double sum = 0.0;
      for (int i = 0; i < kRotorCount; ++i) {
        const double magnitude = std::abs(column(i));
        sum += magnitude;
        non_zero += magnitude > std::numeric_limits<double>::epsilon() ? 1 : 0;
      }
      result.normalization_scale(axis_index) = non_zero > 0
        ? sum / static_cast<double>(non_zero) : 1.0;
    }
  }
  result.normalization_scale(1) = result.normalization_scale(0);
  result.normalized_mix = pseudo_inverse;
  for (int axis_index = 0; axis_index < 6; ++axis_index) {
    result.normalized_mix.col(axis_index) /= result.normalization_scale(axis_index);
  }
  return result;
}

inline Vector4 allocateX500Normalized(const Vector6 &control_setpoint)
{
  return x500Effectiveness().normalized_mix * control_setpoint;
}

inline std::optional<double> thrustForceToPx4Z(
  double thrust_force_n, const ThrustMapping &mapping) noexcept
{
  if (!validThrustMapping(mapping) || !std::isfinite(thrust_force_n) || thrust_force_n < 0.0) {
    return std::nullopt;
  }
  const double hover_force_n = mapping.vehicle_mass_kg * mapping.gravity_mps2;
  const double maximum_supported_force_n = hover_force_n / mapping.hover_thrust_normalized;
  if (!std::isfinite(hover_force_n) || !std::isfinite(maximum_supported_force_n)
    || thrust_force_n > maximum_supported_force_n) {
    return std::nullopt;
  }

  // PX4's PositionControl uses -hover_thrust_normalized for a level hover.
  // The direct wrench bridge preserves that calibrated force ratio at the
  // allocator boundary. Positive FLU thrust is negative FRD Z.
  return -mapping.hover_thrust_normalized * thrust_force_n / hover_force_n;
}

inline ConversionResult convert(
  double thrust_force_flu_n, const Vector3 &normalized_torque_flu,
  const ThrustMapping &mapping = {}) noexcept
{
  ConversionResult result;
  if (!std::isfinite(thrust_force_flu_n) || !normalized_torque_flu.allFinite()) {
    result.failure_reason = FailureReason::non_finite;
    return result;
  }
  if (!validThrustMapping(mapping)) {
    result.failure_reason = FailureReason::invalid_thrust_mapping;
    return result;
  }
  if (thrust_force_flu_n < 0.0) {
    result.failure_reason = FailureReason::negative_thrust;
    return result;
  }
  if ((normalized_torque_flu.array().abs() > kNormalizedLimit).any()) {
    result.failure_reason = FailureReason::normalized_torque_out_of_range;
    return result;
  }

  const auto thrust_z = thrustForceToPx4Z(thrust_force_flu_n, mapping);
  if (!thrust_z) {
    result.failure_reason = FailureReason::thrust_out_of_range;
    return result;
  }
  // M3 supplies a scalar body-FLU thrust magnitude. Upward thrust is +Z_FLU;
  // PX4's body-axis setpoint uses FRD, hence the normalized vector is -Z_FRD.
  result.thrust_frd = Vector3(0.0, 0.0, *thrust_z);
  result.torque_frd = fluToFrd(normalized_torque_flu);
  if (!result.thrust_frd.allFinite() || !result.torque_frd.allFinite()
    || (result.thrust_frd.array().abs() > kNormalizedLimit).any()
    || (result.torque_frd.array().abs() > kNormalizedLimit).any()) {
    result.failure_reason = FailureReason::non_finite;
    return result;
  }

  result.status = Status::within_range;
  result.failure_reason = FailureReason::none;
  return result;
}

}  // namespace mpc_controller::px4_wrench
