#pragma once

#include "px4_thrust_mapping.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <optional>

namespace mpc_controller::px4_attitude
{

using Quaternion = Eigen::Quaterniond;
using Vector3 = Eigen::Vector3d;

enum class FailureReason
{
  none,
  invalid_input,
  invalid_attitude,
  invalid_thrust
};

struct Input
{
  // Desired rotation is body FLU -> world ENU, Hamilton quaternion order.
  Quaternion desired_body_flu_to_world_enu{Quaternion::Identity()};
  // M3's verified collective-force projection, in Newtons.
  double desired_collective_thrust_n = 0.0;
  // Positive about +Z_ENU, in rad/s. This is not a body-rate command.
  double desired_yaw_rate_enu_rad_s = 0.0;
  bool valid = false;
};

struct Output
{
  // PX4 VehicleAttitudeSetpoint q_d: body FRD -> world NED.
  std::array<float, 4> q_d_wxyz{1.0F, 0.0F, 0.0F, 0.0F};
  // PX4 body FRD normalized thrust. Multicopter upward thrust is negative Z.
  std::array<float, 3> thrust_body_frd{0.0F, 0.0F, 0.0F};
  double yaw_sp_move_rate_ned_rad_s = 0.0;
  bool valid = false;
  FailureReason failure_reason = FailureReason::invalid_input;
};

struct MotorModelThrustMapping
{
  int motor_count = 4;
  double thrust_coefficient_n_per_rad_s2 = 4.6e-6;
  double maximum_motor_speed_rad_s = 1100.0;
};

inline bool validMotorModelThrustMapping(const MotorModelThrustMapping &mapping) noexcept
{
  return mapping.motor_count > 0
    && std::isfinite(mapping.thrust_coefficient_n_per_rad_s2)
    && mapping.thrust_coefficient_n_per_rad_s2 > 0.0
    && std::isfinite(mapping.maximum_motor_speed_rad_s)
    && mapping.maximum_motor_speed_rad_s > 0.0;
}

inline std::optional<double> maximumCollectiveThrustN(
  const MotorModelThrustMapping &mapping) noexcept
{
  if (!validMotorModelThrustMapping(mapping)) {
    return std::nullopt;
  }
  const double maximum = static_cast<double>(mapping.motor_count)
    * mapping.thrust_coefficient_n_per_rad_s2
    * mapping.maximum_motor_speed_rad_s * mapping.maximum_motor_speed_rad_s;
  return std::isfinite(maximum) && maximum > 0.0
    ? std::optional<double>(maximum) : std::nullopt;
}

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

inline Output convert(
  const Input &input,
  const px4_thrust::Mapping &thrust_mapping = {}) noexcept
{
  Output output;
  if (!input.valid || !std::isfinite(input.desired_collective_thrust_n)
    || !std::isfinite(input.desired_yaw_rate_enu_rad_s)) {
    output.failure_reason = FailureReason::invalid_input;
    return output;
  }

  const auto q_ned_frd = fluEnuToFrdNed(input.desired_body_flu_to_world_enu);
  if (!q_ned_frd) {
    output.failure_reason = FailureReason::invalid_attitude;
    return output;
  }

  const auto thrust_z = px4_thrust::forceToBodyFrdZ(
    input.desired_collective_thrust_n, thrust_mapping);
  if (!thrust_z || !std::isfinite(*thrust_z)) {
    output.failure_reason = FailureReason::invalid_thrust;
    return output;
  }

  output.q_d_wxyz = {
    static_cast<float>(q_ned_frd->w()), static_cast<float>(q_ned_frd->x()),
    static_cast<float>(q_ned_frd->y()), static_cast<float>(q_ned_frd->z())};
  output.thrust_body_frd = {0.0F, 0.0F, static_cast<float>(*thrust_z)};
  // yaw_NED = pi/2 - yaw_ENU, so its derivative reverses sign.
  output.yaw_sp_move_rate_ned_rad_s = -input.desired_yaw_rate_enu_rad_s;
  output.valid = std::isfinite(output.yaw_sp_move_rate_ned_rad_s);
  output.failure_reason = output.valid ? FailureReason::none : FailureReason::invalid_input;
  return output;
}

inline Output convertMotorModel(
  const Input &input, const MotorModelThrustMapping &mapping) noexcept
{
  Output output;
  if (!input.valid || !std::isfinite(input.desired_collective_thrust_n)
    || !std::isfinite(input.desired_yaw_rate_enu_rad_s)) {
    output.failure_reason = FailureReason::invalid_input;
    return output;
  }
  const auto maximum_thrust_n = maximumCollectiveThrustN(mapping);
  const auto q_ned_frd = fluEnuToFrdNed(input.desired_body_flu_to_world_enu);
  if (!q_ned_frd) {
    output.failure_reason = FailureReason::invalid_attitude;
    return output;
  }
  if (!maximum_thrust_n || input.desired_collective_thrust_n < 0.0) {
    output.failure_reason = FailureReason::invalid_thrust;
    return output;
  }

  // Deliberately not clamped during SITL-first tuning. Values outside PX4's
  // nominal normalized range remain visible instead of hiding controller/model
  // errors behind an adapter saturation.
  const double thrust_z = -input.desired_collective_thrust_n / *maximum_thrust_n;
  if (!std::isfinite(thrust_z)) {
    output.failure_reason = FailureReason::invalid_thrust;
    return output;
  }
  output.q_d_wxyz = {
    static_cast<float>(q_ned_frd->w()), static_cast<float>(q_ned_frd->x()),
    static_cast<float>(q_ned_frd->y()), static_cast<float>(q_ned_frd->z())};
  output.thrust_body_frd = {0.0F, 0.0F, static_cast<float>(thrust_z)};
  output.yaw_sp_move_rate_ned_rad_s = -input.desired_yaw_rate_enu_rad_s;
  output.valid = true;
  output.failure_reason = FailureReason::none;
  return output;
}

}  // namespace mpc_controller::px4_attitude
