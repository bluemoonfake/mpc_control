#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace mpc_controller::mrs_control
{

using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;

struct Parameters
{
  // Verified from the PX4 gz_x500 SDF.
  double mass_kg = 2.0;
  Vector3 inertia_kg_m2{0.02166666666666667, 0.02166666666666667, 0.04000000000000001};
  double gravity_m_s2 = 9.80665;

  // Values match the pinned MRS public MPC controller configuration.
  Vector3 attitude_gain_rad_s{5.0, 5.0, 1.0};
  Vector3 rate_gain_s_inv{4.0, 4.0, 4.0};

  // PX4's VehicleTorqueSetpoint is a normalized allocator coordinate. These
  // gains are deliberately separate from the inertia-weighted MRS parity
  // gains above. Defaults follow PX4 v1.17 gz_x500 rate P gains with
  // MC_*RATE_K=1: roll/pitch 0.15, yaw 0.20.
  Vector3 normalized_rate_gain_s_inv{0.15, 0.15, 0.20};
  Vector3 normalized_torque_limit{1.0, 1.0, 1.0};

  // The pinned MRS MPC config uses a 90 degree tilt failsafe. These rate
  // bounds are explicit shadow-mode bounds; active PX4 limits are an M4 task.
  Vector3 max_body_rate_rad_s{4.0, 4.0, 4.0};
  double max_tilt_rad = 1.5707963267948966;
};

struct Input
{
  Vector3 desired_acceleration_m_s2{};  // M2 control_input, not predicted a[k+1]
  Vector3 measured_body_rate_rad_s{};   // FLU body frame
  Eigen::Quaterniond measured_body_to_world{Eigen::Quaterniond::Identity()};
  double desired_yaw_rad = 0.0;
  double desired_yaw_rate_rad_s = 0.0;
  bool desired_yaw_rate_valid = false;
  bool attitude_valid = false;
  bool body_rate_valid = false;
  bool heading_valid = false;
  bool control_ready = false;
};

enum class FailureReason
{
  none,
  invalid_parameters,
  invalid_input,
  degenerate_force,
  tilt_limit,
  heading_singularity,
  invalid_output,
  non_finite_output
};

struct Output
{
  bool valid = false;
  bool active_control_ready = false;
  FailureReason failure_reason = FailureReason::none;
  Vector3 desired_acceleration_m_s2{};
  Vector3 desired_force_world_n{};
  Matrix3 desired_rotation_world_from_body = Matrix3::Identity();
  Eigen::Quaterniond desired_body_to_world{Eigen::Quaterniond::Identity()};
  double desired_thrust_force_n = 0.0;
  double tilt_angle_rad = 0.0;
  Vector3 orientation_error{};
  Vector3 desired_body_rate_rad_s{};
  Vector3 body_rate_error_rad_s{};
  // MRS attitude-rate control-group action. This is dimensionless at the
  // MRS hardware boundary and remains diagnostic only.
  Vector3 control_group_action{};
  // Explicit PX4 normalized torque command in body FLU coordinates.
  Vector3 normalized_torque_command_flu{};
  bool normalized_torque_saturated = false;
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
    && finite(parameters.inertia_kg_m2)
    && (parameters.inertia_kg_m2.array() > 0.0).all()
    && std::isfinite(parameters.gravity_m_s2) && parameters.gravity_m_s2 > 0.0
    && finite(parameters.attitude_gain_rad_s)
    && (parameters.attitude_gain_rad_s.array() >= 0.0).all()
    && finite(parameters.rate_gain_s_inv)
    && (parameters.rate_gain_s_inv.array() >= 0.0).all()
    && finite(parameters.normalized_rate_gain_s_inv)
    && (parameters.normalized_rate_gain_s_inv.array() >= 0.0).all()
    && finite(parameters.normalized_torque_limit)
    && (parameters.normalized_torque_limit.array() > 0.0).all()
    && (parameters.normalized_torque_limit.array() <= 1.0).all()
    && finite(parameters.max_body_rate_rad_s)
    && (parameters.max_body_rate_rad_s.array() > 0.0).all()
    && std::isfinite(parameters.max_tilt_rad) && parameters.max_tilt_rad > 0.0
    && parameters.max_tilt_rad <= 0.5 * M_PI;
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

// Exact MRS common::orientationError() formulation.
inline Vector3 orientationError(const Matrix3 &current, const Matrix3 &desired) noexcept
{
  const Matrix3 error = 0.5 * (desired.transpose() * current - current.transpose() * desired);
  return Vector3(
    (error(1, 2) - error(2, 1)) / 2.0,
    (error(2, 0) - error(0, 2)) / 2.0,
    (error(0, 1) - error(1, 0)) / 2.0);
}

inline Vector3 saturate(const Vector3 &value, const Vector3 &limits) noexcept
{
  Vector3 output = value;
  for (int i = 0; i < 3; ++i) {
    output[i] = std::clamp(output[i], -limits[i], limits[i]);
  }
  return output;
}

inline Vector3 normalizedRateTorque(
  const Vector3 &desired_body_rate, const Vector3 &measured_body_rate,
  const Vector3 &gain_s_inv, const Vector3 &limits,
  bool &saturated) noexcept
{
  const Vector3 unsaturated = (desired_body_rate - measured_body_rate)
    .cwiseProduct(gain_s_inv);
  const Vector3 limited = saturate(unsaturated, limits);
  saturated = (limited - unsaturated).norm() > 1.0e-12;
  return limited;
}

inline Output compute(const Parameters &parameters, const Input &input) noexcept
{
  Output output;
  output.desired_acceleration_m_s2 = input.desired_acceleration_m_s2;
  if (!validParameters(parameters) || !finite(input.desired_acceleration_m_s2)
    || !finite(input.measured_body_rate_rad_s) || !std::isfinite(input.desired_yaw_rad)
    || (input.desired_yaw_rate_valid && !std::isfinite(input.desired_yaw_rate_rad_s))
    || !input.attitude_valid || !input.body_rate_valid) {
    output.failure_reason = !validParameters(parameters)
      ? FailureReason::invalid_parameters : FailureReason::invalid_input;
    return output;
  }

  const double quaternion_norm = input.measured_body_to_world.norm();
  if (!std::isfinite(quaternion_norm) || quaternion_norm < 1.0e-9) {
    output.failure_reason = FailureReason::invalid_input;
    return output;
  }
  const Eigen::Quaterniond current_quaternion = input.measured_body_to_world.normalized();
  const Matrix3 current_rotation = current_quaternion.toRotationMatrix();

  const auto force = desiredForce(parameters, input.desired_acceleration_m_s2);
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
    output.failure_reason = FailureReason::heading_singularity;
    return output;
  }
  output.desired_rotation_world_from_body = *desired_rotation;
  output.desired_body_to_world = Eigen::Quaterniond(*desired_rotation).normalized();

  // MRS uses f dot current R.col(2) before converting force to throttle.
  output.desired_thrust_force_n = force->dot(current_rotation.col(2));
  if (!std::isfinite(output.desired_thrust_force_n) || output.desired_thrust_force_n < 0.0) {
    output.failure_reason = FailureReason::invalid_output;
    return output;
  }

  output.orientation_error = orientationError(current_rotation, *desired_rotation);
  Vector3 feedforward_body = Vector3::Zero();
  if (input.desired_yaw_rate_valid) {
    const Vector3 yaw_rate_world(0.0, 0.0, input.desired_yaw_rate_rad_s);
    feedforward_body = desired_rotation->transpose() * yaw_rate_world;
  }
  output.desired_body_rate_rad_s = saturate(
    output.orientation_error.cwiseProduct(parameters.attitude_gain_rad_s) + feedforward_body,
    parameters.max_body_rate_rad_s);
  output.body_rate_error_rad_s = output.desired_body_rate_rad_s - input.measured_body_rate_rad_s;

  // MRS parity diagnostic: retain the inertia-weighted rate-error quantity
  // used by the MRS control-group path. It is not the active PX4 command and
  // is intentionally not assigned a physical N*m contract here.
  const Vector3 inertia_weighted_rate_gain =
    parameters.inertia_kg_m2.cwiseProduct(parameters.rate_gain_s_inv);
  output.control_group_action = output.body_rate_error_rad_s.cwiseProduct(inertia_weighted_rate_gain);
  output.normalized_torque_command_flu = normalizedRateTorque(
    output.desired_body_rate_rad_s, input.measured_body_rate_rad_s,
    parameters.normalized_rate_gain_s_inv, parameters.normalized_torque_limit,
    output.normalized_torque_saturated);
  if (!finite(output.desired_force_world_n) || !finite(output.desired_rotation_world_from_body)
    || !finite(output.desired_body_rate_rad_s) || !finite(output.body_rate_error_rad_s)
    || !finite(output.control_group_action) || !finite(output.normalized_torque_command_flu)) {
    output.failure_reason = FailureReason::non_finite_output;
    return output;
  }

  output.valid = true;
  output.active_control_ready = input.control_ready && input.heading_valid;
  output.failure_reason = FailureReason::none;
  return output;
}

}  // namespace mpc_controller::mrs_control
