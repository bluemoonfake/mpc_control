#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <optional>

namespace mpc_controller::force_attitude
{

using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;

struct Parameters
{
  double gravity_m_s2 = 9.80665;
  double max_tilt_rad = 1.5707963267948966;
};

struct Input
{
  // Translational MPC command and requested ENU heading are sufficient to
  // construct the desired rotation; measured attitude belongs to SO(3).
  Vector3 desired_acceleration_m_s2{};
  double desired_yaw_rad = 0.0;
  bool valid = false;
};

enum class FailureReason
{
  none,
  invalid_parameters,
  non_finite_input,
  invalid_input,
  degenerate_force,
  tilt_limit,
  degenerate_heading_basis,
  invalid_rotation,
  non_finite_output
};

inline const char *failureReasonName(FailureReason reason) noexcept
{
  switch (reason) {
    case FailureReason::none: return "NONE";
    case FailureReason::invalid_parameters: return "INVALID_PARAMETERS";
    case FailureReason::non_finite_input: return "NON_FINITE_INPUT";
    case FailureReason::invalid_input: return "INVALID_INPUT";
    case FailureReason::degenerate_force: return "DEGENERATE_FORCE";
    case FailureReason::tilt_limit: return "TILT_LIMIT";
    case FailureReason::degenerate_heading_basis: return "DEGENERATE_HEADING_BASIS";
    case FailureReason::invalid_rotation: return "INVALID_ROTATION";
    case FailureReason::non_finite_output: return "NON_FINITE_OUTPUT";
  }
  return "UNKNOWN";
}

struct Output
{
  bool valid = false;
  FailureReason failure_reason = FailureReason::none;
  Vector3 desired_acceleration_m_s2{};
  Vector3 desired_specific_force_world_m_s2{};
  Eigen::Quaterniond desired_body_to_world{Eigen::Quaterniond::Identity()};
  double desired_collective_specific_force_m_s2 = 0.0;
  double tilt_angle_rad = 0.0;
};

inline bool finite(const Vector3 &value) noexcept
{
  return value.allFinite();
}

inline bool finite(const Matrix3 &value) noexcept
{
  return value.allFinite();
}

inline bool validParameters(const Parameters &parameters) noexcept
{
  return std::isfinite(parameters.gravity_m_s2) && parameters.gravity_m_s2 > 0.0
    && std::isfinite(parameters.max_tilt_rad) && parameters.max_tilt_rad > 0.0
    && parameters.max_tilt_rad <= 0.5 * M_PI;
}

inline std::optional<Vector3> desiredSpecificForce(
  const Parameters &parameters, const Vector3 &desired_acceleration) noexcept
{
  if (!validParameters(parameters) || !finite(desired_acceleration)) {
    return std::nullopt;
  }

  // ENU gravity is [0, 0, -g]. The rotor force required to produce a_des is:
  // f_des / m = a_des - gravity = a_des + [0, 0, g].
  const Vector3 gravity_world(0.0, 0.0, -parameters.gravity_m_s2);
  const Vector3 force = desired_acceleration - gravity_world;
  if (!finite(force) || force.norm() < 1.0e-9 || force.z() <= 1.0e-9) {
    return std::nullopt;
  }
  return force;
}

inline std::optional<Vector3> sanitizeDesiredSpecificForce(
  const Vector3 &force_world, double max_tilt_rad, double &tilt_angle_rad) noexcept
{
  if (!finite(force_world) || !std::isfinite(max_tilt_rad) || max_tilt_rad <= 0.0
    || max_tilt_rad > 0.5 * M_PI) {
    return std::nullopt;
  }
  const double norm = force_world.norm();
  if (!std::isfinite(norm) || norm < 1.0e-9 || force_world.z() <= 1.0e-9) {
    return std::nullopt;
  }
  // Desired body-Z is b3=F_d/||F_d||. Its angle to ENU +Z is the vehicle tilt.
  const Vector3 normalized = force_world / norm;
  tilt_angle_rad = std::acos(std::clamp(normalized.z(), -1.0, 1.0));
  if (!std::isfinite(tilt_angle_rad) || tilt_angle_rad > max_tilt_rad + 1.0e-9) {
    return std::nullopt;
  }
  return normalized;
}

inline std::optional<Matrix3> so3Transform(const Vector3 &body_z_world, double desired_yaw_rad) noexcept
{
  if (!finite(body_z_world) || !std::isfinite(desired_yaw_rad)
    || body_z_world.norm() < 1.0e-9) {
    return std::nullopt;
  }

  // Complete R_d=[b1 b2 b3] from desired thrust direction and yaw heading:
  // b2=normalize(b3 x heading), b1=b2 x b3.
  const Vector3 b3 = body_z_world.normalized();
  const Vector3 heading(std::cos(desired_yaw_rad), std::sin(desired_yaw_rad), 0.0);
  Vector3 b2 = b3.cross(heading);
  if (!finite(b2) || b2.norm() < 1.0e-9) {
    return std::nullopt;
  }
  b2.normalize();
  const Vector3 b1 = b2.cross(b3);
  if (!finite(b1) || b1.norm() < 1.0e-9) {
    return std::nullopt;
  }

  Matrix3 rotation;
  rotation.col(0) = b1.normalized();
  rotation.col(1) = b2;
  rotation.col(2) = b3;
  return rotation;
}

inline Output compute(const Parameters &parameters, const Input &input) noexcept
{
  Output output;
  output.desired_acceleration_m_s2 = input.desired_acceleration_m_s2;
  if (!validParameters(parameters)) {
    output.failure_reason = FailureReason::invalid_parameters;
    return output;
  }
  if (!finite(input.desired_acceleration_m_s2) || !std::isfinite(input.desired_yaw_rad)) {
    output.failure_reason = FailureReason::non_finite_input;
    return output;
  }
  if (!input.valid) {
    output.failure_reason = FailureReason::invalid_input;
    return output;
  }

  const auto force = desiredSpecificForce(parameters, output.desired_acceleration_m_s2);
  if (!force) {
    output.failure_reason = FailureReason::degenerate_force;
    return output;
  }
  output.desired_specific_force_world_m_s2 = *force;
  const auto body_z = sanitizeDesiredSpecificForce(*force, parameters.max_tilt_rad, output.tilt_angle_rad);
  if (!body_z) {
    output.failure_reason = FailureReason::tilt_limit;
    return output;
  }
  const auto desired_rotation = so3Transform(*body_z, input.desired_yaw_rad);
  if (!desired_rotation) {
    output.failure_reason = FailureReason::degenerate_heading_basis;
    return output;
  }
  if (!finite(*desired_rotation)) {
    output.failure_reason = FailureReason::invalid_rotation;
    return output;
  }
  output.desired_body_to_world = Eigen::Quaterniond(*desired_rotation).normalized();

  // q_d aligns body Z with the desired force, so collective magnitude is the
  // invariant norm. The adapter maps it with the active PX4 HTE calibration.
  output.desired_collective_specific_force_m_s2 = force->norm();
  if (!std::isfinite(output.desired_collective_specific_force_m_s2)) {
    output.failure_reason = FailureReason::non_finite_output;
    return output;
  }

  if (!finite(output.desired_specific_force_world_m_s2)) {
    output.failure_reason = FailureReason::non_finite_output;
    return output;
  }

  output.valid = true;
  output.failure_reason = FailureReason::none;
  return output;
}

}  // namespace mpc_controller::force_attitude
