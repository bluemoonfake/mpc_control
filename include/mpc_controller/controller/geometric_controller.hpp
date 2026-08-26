#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace mpc_controller::px4_thrust
{

inline constexpr double kNormalizedLimit = 1.0;

// Linear force-to-PX4 mapping around hover. A fresh PX4 HTE updates hover_thrust.
struct Mapping
{
  double gravity_mps2 = 9.80665;
  double hover_thrust_normalized = 0.60;
};

inline bool valid(const Mapping &mapping) noexcept
{
  return std::isfinite(mapping.gravity_mps2) && mapping.gravity_mps2 > 0.0
    && std::isfinite(mapping.hover_thrust_normalized)
    && mapping.hover_thrust_normalized > 0.0
    && mapping.hover_thrust_normalized <= kNormalizedLimit;
}

inline std::optional<double> specificForceToBodyFrdZ(
  double specific_force_m_s2, const Mapping &mapping) noexcept
{
  if (!valid(mapping) || !std::isfinite(specific_force_m_s2)
    || specific_force_m_s2 < 0.0) {
    return std::nullopt;
  }
  // Linearized HTE mapping: thrust_FRD,z=-h*||F_d/m||/g, where h is PX4's
  // current normalized hover-thrust estimate.
  const double maximum_specific_force =mapping.gravity_mps2 / mapping.hover_thrust_normalized;
  if (!std::isfinite(maximum_specific_force) || specific_force_m_s2 > maximum_specific_force) {
    return std::nullopt;
  }
  // Positive FLU collective force becomes negative body-FRD Z thrust.
  return -mapping.hover_thrust_normalized * specific_force_m_s2 / mapping.gravity_mps2;
}

}  // namespace mpc_controller::px4_thrust

namespace mpc_controller::px4_control
{

using Quaternion = Eigen::Quaterniond;
using Vector3 = Eigen::Vector3d;

inline bool finiteQuaternion(const Quaternion &q) noexcept
{
  return std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z());
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

inline std::optional<Quaternion> frdNedToFluEnu(const Quaternion &q_frd_ned) noexcept
{
  return fluEnuToFrdNed(q_frd_ned);
}

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

}  // namespace mpc_controller::px4_control
