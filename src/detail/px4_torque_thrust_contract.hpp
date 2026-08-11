#pragma once

#include "px4_thrust_mapping.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace mpc_controller::px4_control
{

using Quaternion = Eigen::Quaterniond;
using Vector3 = Eigen::Vector3d;

enum class FailureReason
{
  none,
  invalid_input,
  invalid_attitude,
  invalid_body_rate,
  invalid_parameters,
  invalid_thrust,
  invalid_torque
};

struct TorqueParameters
{
  // Gains produce PX4 normalized torque directly. Physical-inertia
  // feedforward is intentionally deferred until the baseline is stable.
  Vector3 attitude_gain{0.8, 0.8, 0.4};
  Vector3 rate_gain{0.15, 0.15, 0.10};
  Vector3 normalized_limit{0.30, 0.30, 0.20};
};

struct Input
{
  // Desired rotation is body FLU -> world ENU, Hamilton quaternion order.
  Quaternion desired_body_flu_to_world_enu{Quaternion::Identity()};
  Quaternion measured_body_flu_to_world_enu{Quaternion::Identity()};
  Vector3 measured_body_rate_flu_rad_s{};
  // M3's verified collective-force projection, in Newtons.
  double desired_collective_thrust_n = 0.0;
  bool valid = false;
};

struct Output
{
  // Desired attitude is retained for diagnostics only.
  std::array<float, 4> q_d_wxyz{1.0F, 0.0F, 0.0F, 0.0F};
  // PX4 normalized torque in body FRD.
  std::array<float, 3> torque_body_frd{0.0F, 0.0F, 0.0F};
  // PX4 body FRD normalized thrust. Multicopter upward thrust is negative Z.
  std::array<float, 3> thrust_body_frd{0.0F, 0.0F, 0.0F};
  std::array<double, 3> attitude_error{};
  std::array<double, 3> body_rate_error_rad_s{};
  bool torque_saturated = false;
  bool valid = false;
  FailureReason failure_reason = FailureReason::invalid_input;
};

inline bool finiteQuaternion(const Quaternion &q) noexcept
{
  return std::isfinite(q.w()) && std::isfinite(q.x())
    && std::isfinite(q.y()) && std::isfinite(q.z());
}

inline std::optional<Quaternion> fluEnuToFrdNed(const Quaternion &q_flu_enu) noexcept
{
  if (!finiteQuaternion(q_flu_enu)) {
    return std::nullopt;
  }
  const double norm = q_flu_enu.norm();
  if (!std::isfinite(norm) || norm < 1.0e-9) {
    return std::nullopt;
  }

  // Coordinate transforms: v_NED = C_NED_ENU v_ENU and v_FLU = C_FLU_FRD v_FRD.
  const Eigen::Matrix3d c_ned_enu = (Eigen::Matrix3d() <<
    0.0, 1.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 0.0, -1.0).finished();
  const Eigen::Matrix3d c_flu_frd = (Eigen::Matrix3d() <<
    1.0, 0.0, 0.0,
    0.0, -1.0, 0.0,
    0.0, 0.0, -1.0).finished();
  const Eigen::Matrix3d r_ned_frd = c_ned_enu
    * q_flu_enu.normalized().toRotationMatrix() * c_flu_frd;
  if (!r_ned_frd.allFinite() || std::abs(r_ned_frd.determinant() - 1.0) > 1.0e-9) {
    return std::nullopt;
  }

  const Quaternion q_ned_frd(r_ned_frd);
  if (!finiteQuaternion(q_ned_frd) || q_ned_frd.norm() < 1.0e-9) {
    return std::nullopt;
  }
  return q_ned_frd.normalized();
}

// Both convention-change matrices are self-inverse, so the reverse mapping
// has the same matrix operation as fluEnuToFrdNed().
inline std::optional<Quaternion> frdNedToFluEnu(const Quaternion &q_frd_ned) noexcept
{
  return fluEnuToFrdNed(q_frd_ned);
}

// Preserve the desired body-Z/tilt direction and replace only its ENU heading.
// This lets the adapter latch yaw at the PX4 Offboard ownership boundary while
// leaving translational-force construction in M3.
inline std::optional<Quaternion> withEnuYaw(
  const Quaternion &desired_body_flu_to_world_enu, double yaw_enu_rad) noexcept
{
  if (!finiteQuaternion(desired_body_flu_to_world_enu) || !std::isfinite(yaw_enu_rad)) {
    return std::nullopt;
  }
  const double norm = desired_body_flu_to_world_enu.norm();
  if (!std::isfinite(norm) || norm < 1.0e-9) {
    return std::nullopt;
  }

  const Vector3 body_z = desired_body_flu_to_world_enu.normalized().toRotationMatrix().col(2);
  const Vector3 heading(std::cos(yaw_enu_rad), std::sin(yaw_enu_rad), 0.0);
  Vector3 body_y = body_z.cross(heading);
  if (!body_z.allFinite() || !body_y.allFinite() || body_y.norm() < 1.0e-9) {
    return std::nullopt;
  }
  body_y.normalize();
  Vector3 body_x = body_y.cross(body_z);
  if (!body_x.allFinite() || body_x.norm() < 1.0e-9) {
    return std::nullopt;
  }
  body_x.normalize();

  Eigen::Matrix3d rotation;
  rotation.col(0) = body_x;
  rotation.col(1) = body_y;
  rotation.col(2) = body_z.normalized();
  if (!rotation.allFinite() || std::abs(rotation.determinant() - 1.0) > 1.0e-9) {
    return std::nullopt;
  }
  return Quaternion(rotation).normalized();
}

inline bool validTorqueParameters(const TorqueParameters &parameters) noexcept
{
  return parameters.attitude_gain.allFinite() && parameters.rate_gain.allFinite()
    && parameters.normalized_limit.allFinite()
    && (parameters.attitude_gain.array() >= 0.0).all()
    && (parameters.rate_gain.array() >= 0.0).all()
    && (parameters.normalized_limit.array() > 0.0).all()
    && (parameters.normalized_limit.array() <= 1.0).all();
}

inline Vector3 vee(const Eigen::Matrix3d &skew) noexcept
{
  return {skew(2, 1), skew(0, 2), skew(1, 0)};
}

inline Output convert(
  const Input &input,
  const px4_thrust::Mapping &thrust_mapping,
  const TorqueParameters &torque_parameters) noexcept
{
  Output output;
  if (!input.valid || !std::isfinite(input.desired_collective_thrust_n)) {
    output.failure_reason = FailureReason::invalid_input;
    return output;
  }
  if (!validTorqueParameters(torque_parameters)) {
    output.failure_reason = FailureReason::invalid_parameters;
    return output;
  }

  if (!finiteQuaternion(input.desired_body_flu_to_world_enu)
    || !finiteQuaternion(input.measured_body_flu_to_world_enu)) {
    output.failure_reason = FailureReason::invalid_attitude;
    return output;
  }
  if (!input.measured_body_rate_flu_rad_s.allFinite()) {
    output.failure_reason = FailureReason::invalid_body_rate;
    return output;
  }

  const double desired_norm = input.desired_body_flu_to_world_enu.norm();
  const double measured_norm = input.measured_body_flu_to_world_enu.norm();
  if (!std::isfinite(desired_norm) || !std::isfinite(measured_norm)
    || desired_norm < 1.0e-9 || measured_norm < 1.0e-9) {
    output.failure_reason = FailureReason::invalid_attitude;
    return output;
  }
  const Quaternion desired = input.desired_body_flu_to_world_enu.normalized();
  const Quaternion measured = input.measured_body_flu_to_world_enu.normalized();
  const Eigen::Matrix3d desired_rotation = desired.toRotationMatrix();
  const Eigen::Matrix3d measured_rotation = measured.toRotationMatrix();
  const Vector3 attitude_error = 0.5 * vee(
    desired_rotation.transpose() * measured_rotation
    - measured_rotation.transpose() * desired_rotation);
  const Vector3 rate_error = input.measured_body_rate_flu_rad_s;
  if (!attitude_error.allFinite() || !rate_error.allFinite()) {
    output.failure_reason = FailureReason::invalid_torque;
    return output;
  }

  const Vector3 torque_flu = -torque_parameters.attitude_gain.cwiseProduct(attitude_error)
    - torque_parameters.rate_gain.cwiseProduct(rate_error);
  Vector3 limited_torque_flu;
  for (int axis = 0; axis < 3; ++axis) {
    limited_torque_flu[axis] = std::clamp(
      torque_flu[axis], -torque_parameters.normalized_limit[axis],
      torque_parameters.normalized_limit[axis]);
    output.torque_saturated = output.torque_saturated
      || std::abs(limited_torque_flu[axis] - torque_flu[axis]) > 1.0e-12;
    output.attitude_error[axis] = attitude_error[axis];
    output.body_rate_error_rad_s[axis] = rate_error[axis];
  }
  if (!limited_torque_flu.allFinite()) {
    output.failure_reason = FailureReason::invalid_torque;
    return output;
  }

  const auto thrust_z = px4_thrust::forceToBodyFrdZ(
    input.desired_collective_thrust_n, thrust_mapping);
  if (!thrust_z || !std::isfinite(*thrust_z)) {
    output.failure_reason = FailureReason::invalid_thrust;
    return output;
  }

  const auto q_ned_frd = fluEnuToFrdNed(desired);
  if (!q_ned_frd) {
    output.failure_reason = FailureReason::invalid_attitude;
    return output;
  }
  output.q_d_wxyz = {
    static_cast<float>(q_ned_frd->w()), static_cast<float>(q_ned_frd->x()),
    static_cast<float>(q_ned_frd->y()), static_cast<float>(q_ned_frd->z())};
  // FLU -> FRD is diag(1, -1, -1) for body vectors.
  output.torque_body_frd = {
    static_cast<float>(limited_torque_flu.x()),
    static_cast<float>(-limited_torque_flu.y()),
    static_cast<float>(-limited_torque_flu.z())};
  output.thrust_body_frd = {0.0F, 0.0F, static_cast<float>(*thrust_z)};
  output.valid = true;
  output.failure_reason = FailureReason::none;
  return output;
}

}  // namespace mpc_controller::px4_control
