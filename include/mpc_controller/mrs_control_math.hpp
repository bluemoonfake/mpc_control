#pragma once

#include "mpc_controller/coupled_thrust_feasibility.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>

namespace mpc_controller::mrs_control
{

using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;

struct Parameters
{
  double mass_kg = 2.0;
  double gravity_m_s2 = 9.80665;
  double max_tilt_rad = 1.5707963267948966;

  // External PX4 normalized collective contract.
  double hover_thrust_normalized = 0.765;
  double max_normalized_collective_thrust = 1.0;
  bool enable_thrust_feasibility = true;
  bool use_force_norm_for_collective_thrust = false;
};

struct Input
{
  Vector3 desired_acceleration_m_s2{};  // M2 first predicted acceleration a[k+1]
  Eigen::Quaterniond measured_body_to_world{Eigen::Quaterniond::Identity()};
  double desired_yaw_rad = 0.0;
  bool attitude_valid = false;
  bool heading_valid = false;
  bool control_ready = false;
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
  rate_command_invalid,
  non_finite_output,
  feasibility_invalid,
  invalid_collective_projection
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
    case FailureReason::rate_command_invalid: return "RATE_COMMAND_INVALID";
    case FailureReason::non_finite_output: return "NON_FINITE_OUTPUT";
    case FailureReason::feasibility_invalid: return "FEASIBILITY_INVALID";
    case FailureReason::invalid_collective_projection: return "INVALID_COLLECTIVE_PROJECTION";
  }
  return "UNKNOWN";
}

struct Output
{
  bool valid = false;
  bool active_control_ready = false;
  FailureReason failure_reason = FailureReason::none;
  Vector3 desired_acceleration_m_s2{};
  Vector3 desired_force_world_n{};
  Vector3 raw_desired_acceleration_m_s2{};
  Vector3 feasible_desired_acceleration_m_s2{};
  Vector3 raw_desired_force_world_n{};
  Vector3 feasible_desired_force_world_n{};
  double raw_force_norm_n = 0.0;
  double feasible_force_norm_n = 0.0;
  double force_limit_n = 0.0;
  double feasibility_correction_norm_m_s2 = 0.0;
  bool feasibility_constraint_active = false;
  Matrix3 desired_rotation_world_from_body = Matrix3::Identity();
  Eigen::Quaterniond desired_body_to_world{Eigen::Quaterniond::Identity()};
  double desired_thrust_force_n = 0.0;
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
  return std::isfinite(parameters.mass_kg) && parameters.mass_kg > 0.0
    && std::isfinite(parameters.gravity_m_s2) && parameters.gravity_m_s2 > 0.0
    && std::isfinite(parameters.max_tilt_rad) && parameters.max_tilt_rad > 0.0
    && parameters.max_tilt_rad <= 0.5 * M_PI
    && thrust_feasibility::validParameters(thrust_feasibility::Parameters{
      parameters.mass_kg, parameters.gravity_m_s2, parameters.hover_thrust_normalized,
      parameters.max_normalized_collective_thrust});
}

inline std::optional<Vector3> desiredForce(
  const Parameters &parameters, const Vector3 &desired_acceleration) noexcept
{
  if (!validParameters(parameters) || !finite(desired_acceleration)) {
    return std::nullopt;
  }

  // ENU gravity is [0, 0, -g]. The rotor force required to produce a_des is:
  // f_des = m * (a_des - gravity) = m * (a_des + [0, 0, g]).
  const Vector3 gravity_world(0.0, 0.0, -parameters.gravity_m_s2);
  const Vector3 force = parameters.mass_kg * (desired_acceleration - gravity_world);
  if (!finite(force) || force.norm() < 1.0e-9 || force.z() <= 1.0e-9) {
    return std::nullopt;
  }
  return force;
}

inline std::optional<Vector3> sanitizeDesiredForce(
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
  const Vector3 normalized = force_world / norm;
  tilt_angle_rad = std::acos(std::clamp(normalized.z(), -1.0, 1.0));
  if (!std::isfinite(tilt_angle_rad) || tilt_angle_rad > max_tilt_rad + 1.0e-9) {
    return std::nullopt;
  }
  return normalized;
}

inline std::optional<Matrix3> so3Transform(
  const Vector3 &body_z_world, double desired_yaw_rad) noexcept
{
  if (!finite(body_z_world) || !std::isfinite(desired_yaw_rad)
    || body_z_world.norm() < 1.0e-9) {
    return std::nullopt;
  }

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
  output.raw_desired_acceleration_m_s2 = input.desired_acceleration_m_s2;
  output.desired_acceleration_m_s2 = input.desired_acceleration_m_s2;
  if (!validParameters(parameters)) {
    output.failure_reason = FailureReason::invalid_parameters;
    return output;
  }
  if (!finite(input.desired_acceleration_m_s2) || !std::isfinite(input.desired_yaw_rad)) {
    output.failure_reason = FailureReason::non_finite_input;
    return output;
  }
  if (!input.attitude_valid) {
    output.failure_reason = FailureReason::invalid_input;
    return output;
  }

  const double quaternion_norm = input.measured_body_to_world.norm();
  if (!std::isfinite(quaternion_norm) || quaternion_norm < 1.0e-9) {
    output.failure_reason = FailureReason::invalid_input;
    return output;
  }
  const Eigen::Quaterniond current_quaternion = input.measured_body_to_world.normalized();
  const Matrix3 current_rotation = current_quaternion.toRotationMatrix();
  if (!finite(current_rotation) || !std::isfinite(current_rotation.determinant()) ||
    current_rotation.determinant() <= 0.0) {
    output.failure_reason = FailureReason::invalid_rotation;
    return output;
  }

  if (parameters.enable_thrust_feasibility) {
    const auto feasibility = thrust_feasibility::project(
      thrust_feasibility::Parameters{
        parameters.mass_kg, parameters.gravity_m_s2, parameters.hover_thrust_normalized,
        parameters.max_normalized_collective_thrust},
      input.desired_acceleration_m_s2);
    if (!feasibility) {
      output.failure_reason = FailureReason::feasibility_invalid;
      return output;
    }
    output.raw_desired_force_world_n = feasibility->raw_force_world_n;
    output.feasible_desired_acceleration_m_s2 = feasibility->feasible_acceleration_m_s2;
    output.feasible_desired_force_world_n = feasibility->feasible_force_world_n;
    output.raw_force_norm_n = feasibility->raw_force_norm_n;
    output.feasible_force_norm_n = feasibility->feasible_force_norm_n;
    output.force_limit_n = feasibility->force_limit_n;
    output.feasibility_correction_norm_m_s2 = feasibility->acceleration_correction_norm_m_s2;
    output.feasibility_constraint_active = feasibility->constraint_active;
    output.desired_acceleration_m_s2 = feasibility->feasible_acceleration_m_s2;
  } else {
    const auto raw_force = desiredForce(parameters, input.desired_acceleration_m_s2);
    if (!raw_force) {
      output.failure_reason = FailureReason::degenerate_force;
      return output;
    }
    output.raw_desired_force_world_n = *raw_force;
    output.feasible_desired_acceleration_m_s2 = input.desired_acceleration_m_s2;
    output.feasible_desired_force_world_n = *raw_force;
    output.raw_force_norm_n = raw_force->norm();
    output.feasible_force_norm_n = output.raw_force_norm_n;
    output.force_limit_n = std::numeric_limits<double>::infinity();
    output.feasibility_correction_norm_m_s2 = 0.0;
    output.feasibility_constraint_active = false;
  }

  const auto force = desiredForce(parameters, output.desired_acceleration_m_s2);
  if (!force) {
    output.failure_reason = FailureReason::degenerate_force;
    return output;
  }
  output.desired_force_world_n = *force;
  const auto body_z = sanitizeDesiredForce(
    *force, parameters.max_tilt_rad, output.tilt_angle_rad);
  if (!body_z) {
    output.failure_reason = FailureReason::tilt_limit;
    return output;
  }
  const auto desired_rotation = so3Transform(*body_z, input.desired_yaw_rad);
  if (!desired_rotation) {
    output.failure_reason = FailureReason::degenerate_heading_basis;
    return output;
  }
  output.desired_rotation_world_from_body = *desired_rotation;
  output.desired_body_to_world = Eigen::Quaterniond(*desired_rotation).normalized();

  // The attitude-setpoint profile commands ||F_d|| while q_d aligns the
  // desired body-Z axis with F_d. Projection mode remains available as a
  // configuration choice.
  output.desired_thrust_force_n = parameters.use_force_norm_for_collective_thrust
    ? force->norm() : force->dot(current_rotation.col(2));
  if (!std::isfinite(output.desired_thrust_force_n) || output.desired_thrust_force_n < 0.0
    || output.desired_thrust_force_n > force->norm() + 1.0e-9) {
    output.failure_reason = FailureReason::invalid_collective_projection;
    return output;
  }

  if (!finite(output.desired_force_world_n)
    || !finite(output.desired_rotation_world_from_body)) {
    output.failure_reason = FailureReason::non_finite_output;
    return output;
  }

  output.valid = true;
  output.active_control_ready = input.control_ready && input.heading_valid;
  output.failure_reason = FailureReason::none;
  return output;
}

}  // namespace mpc_controller::mrs_control
