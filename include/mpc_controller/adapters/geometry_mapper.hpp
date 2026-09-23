#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
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

inline std::optional<Matrix3> so3Transform(
  const Vector3 &body_z_world, double desired_yaw_rad) noexcept
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
  const auto body_z = sanitizeDesiredSpecificForce(
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
  const double maximum_specific_force = mapping.gravity_mps2 / mapping.hover_thrust_normalized;
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
  return std::isfinite(q.w()) && std::isfinite(q.x()) &&
         std::isfinite(q.y()) && std::isfinite(q.z());
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
  const Eigen::Matrix3d r_ned_frd = c_ned_enu *
    q_flu_enu.normalized().toRotationMatrix() * c_flu_frd;
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

inline std::optional<double> enuYawRateToNed(double yaw_rate_enu_rad_s) noexcept
{
  if (!std::isfinite(yaw_rate_enu_rad_s)) {
    return std::nullopt;
  }
  // yaw_NED = pi/2 - yaw_ENU, hence yaw_rate_NED = -yaw_rate_ENU.
  return -yaw_rate_enu_rad_s;
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

namespace mpc_controller::frame
{

using Vector3 = std::array<double, 3>;
using Quaternion = std::array<double, 4>;  // Hamilton order: w, x, y, z

struct Px4LocalPositionSample
{
  uint64_t timestamp_sample = 0;
  bool xy_valid = false;
  bool z_valid = false;
  bool v_xy_valid = false;
  bool v_z_valid = false;
  bool heading_good_for_control = false;
  Vector3 position_ned{};
  Vector3 velocity_ned{};
  Vector3 acceleration_ned{};
};

struct Px4AttitudeSample
{
  uint64_t timestamp_sample = 0;
  Quaternion body_frd_to_world_ned{1.0, 0.0, 0.0, 0.0};
};

struct Px4AngularVelocitySample
{
  uint64_t timestamp_sample = 0;
  Vector3 body_rate_frd{};
};

struct VehicleStateData
{
  Vector3 position_enu{};
  Vector3 velocity_enu{};
  Vector3 acceleration_enu{};
  Quaternion body_flu_to_world_enu{1.0, 0.0, 0.0, 0.0};
  double yaw_enu = 0.0;
  bool position_valid = false;
  bool velocity_valid = false;
  bool acceleration_valid = false;
  bool attitude_valid = false;
  bool body_rate_valid = false;
  bool heading_valid = false;
  bool control_ready = false;
};

inline bool timestampMonotonic(uint64_t previous, uint64_t current) noexcept
{
  return current != 0 && (previous == 0 || current >= previous);
}

inline bool synchronizedTimestamp(uint64_t timestamp) noexcept
{
  // Microseconds since Unix epoch are currently O(10^15); PX4 boot time is
  // O(10^8..10^11). Keep independent monotonic histories for both domains.
  constexpr uint64_t synchronized_epoch_threshold_us = 1000000000000ULL;
  return timestamp >= synchronized_epoch_threshold_us;
}

inline bool finite(const Vector3 &value) noexcept
{
  return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

inline bool finite(const Quaternion &value) noexcept
{
  return std::isfinite(value[0]) && std::isfinite(value[1])
    && std::isfinite(value[2]) && std::isfinite(value[3]);
}

inline Vector3 nedToEnu(const Vector3 &value_ned) noexcept
{
  return {value_ned[1], value_ned[0], -value_ned[2]};
}

inline std::array<std::array<double, 3>, 3> quaternionToMatrix(const Quaternion &q) noexcept
{
  const double w = q[0];
  const double x = q[1];
  const double y = q[2];
  const double z = q[3];
  return {{
    {1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)},
    {2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)},
    {2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)}
  }};
}

inline Quaternion matrixToQuaternion(const std::array<std::array<double, 3>, 3> &r) noexcept
{
  Quaternion q{};
  const double trace = r[0][0] + r[1][1] + r[2][2];
  if (trace > 0.0) {
    const double s = 0.5 / std::sqrt(trace + 1.0);
    q[0] = 0.25 / s;
    q[1] = (r[2][1] - r[1][2]) * s;
    q[2] = (r[0][2] - r[2][0]) * s;
    q[3] = (r[1][0] - r[0][1]) * s;
  } else if (r[0][0] > r[1][1] && r[0][0] > r[2][2]) {
    const double s = 2.0 * std::sqrt(std::max(1.0e-16, 1.0 + r[0][0] - r[1][1] - r[2][2]));
    q[0] = (r[2][1] - r[1][2]) / s;
    q[1] = 0.25 * s;
    q[2] = (r[0][1] + r[1][0]) / s;
    q[3] = (r[0][2] + r[2][0]) / s;
  } else if (r[1][1] > r[2][2]) {
    const double s = 2.0 * std::sqrt(std::max(1.0e-16, 1.0 + r[1][1] - r[0][0] - r[2][2]));
    q[0] = (r[0][2] - r[2][0]) / s;
    q[1] = (r[0][1] + r[1][0]) / s;
    q[2] = 0.25 * s;
    q[3] = (r[1][2] + r[2][1]) / s;
  } else {
    const double s = 2.0 * std::sqrt(std::max(1.0e-16, 1.0 + r[2][2] - r[0][0] - r[1][1]));
    q[0] = (r[1][0] - r[0][1]) / s;
    q[1] = (r[0][2] + r[2][0]) / s;
    q[2] = (r[1][2] + r[2][1]) / s;
    q[3] = 0.25 * s;
  }
  return q;
}

inline bool convert(
  const Px4LocalPositionSample &local_position,
  const Px4AttitudeSample &attitude,
  const Px4AngularVelocitySample &angular_velocity,
  VehicleStateData &output) noexcept
{
  output.position_valid = local_position.xy_valid && local_position.z_valid
    && finite(local_position.position_ned);
  output.velocity_valid = local_position.v_xy_valid && local_position.v_z_valid
    && finite(local_position.velocity_ned);
  output.acceleration_valid = finite(local_position.acceleration_ned);
  output.attitude_valid = finite(attitude.body_frd_to_world_ned);
  output.body_rate_valid = finite(angular_velocity.body_rate_frd);
  output.heading_valid = local_position.heading_good_for_control;
  output.control_ready = false;

  if (local_position.timestamp_sample == 0 || attitude.timestamp_sample == 0
    || angular_velocity.timestamp_sample == 0 || !output.position_valid
    || !output.velocity_valid || !output.acceleration_valid || !output.attitude_valid
    || !output.body_rate_valid) {
    return false;
  }

  const auto &q_px4 = attitude.body_frd_to_world_ned;
  const double q_norm = std::sqrt(
    q_px4[0] * q_px4[0] + q_px4[1] * q_px4[1]
    + q_px4[2] * q_px4[2] + q_px4[3] * q_px4[3]);
  if (!std::isfinite(q_norm) || q_norm < 1.0e-9) {
    output.attitude_valid = false;
    return false;
  }

  Quaternion q_normalized{
    q_px4[0] / q_norm, q_px4[1] / q_norm, q_px4[2] / q_norm, q_px4[3] / q_norm};
  const auto r_ned_frd = quaternionToMatrix(q_normalized);

  // C_ENU_NED * R_NED_FRD * C_FRD_FLU.
  const std::array<int, 3> world_axis{1, 0, 2};
  const std::array<double, 3> world_sign{1.0, 1.0, -1.0};
  const std::array<double, 3> body_sign{1.0, -1.0, -1.0};
  std::array<std::array<double, 3>, 3> r_enu_flu{};
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      r_enu_flu[row][col] = world_sign[row] * r_ned_frd[world_axis[row]][col] * body_sign[col];
    }
  }

  output.position_enu = nedToEnu(local_position.position_ned);
  output.velocity_enu = nedToEnu(local_position.velocity_ned);
  output.acceleration_enu = nedToEnu(local_position.acceleration_ned);
  output.body_flu_to_world_enu = matrixToQuaternion(r_enu_flu);
  output.yaw_enu = std::atan2(r_enu_flu[1][0], r_enu_flu[0][0]);
  const bool finite_output = std::isfinite(output.yaw_enu) && finite(output.position_enu)
    && finite(output.velocity_enu) && finite(output.acceleration_enu)
    && finite(output.body_flu_to_world_enu);
  if (!finite_output) {
    output.position_valid = false;
    output.velocity_valid = false;
    output.acceleration_valid = false;
    output.attitude_valid = false;
    output.body_rate_valid = false;
    return false;
  }

  output.control_ready = output.position_valid && output.velocity_valid
    && output.acceleration_valid && output.attitude_valid
    && output.body_rate_valid && output.heading_valid;
  return true;
}

}  // namespace mpc_controller::frame

namespace mpc_controller::state_check
{

// Timing data used to admit three asynchronous PX4 state streams.
struct Timing
{
  std::uint64_t sample_time = 0;
  double age = std::numeric_limits<double>::infinity();
  bool received = false;
};

enum class Reject : std::uint8_t
{
  none,
  position_stale,
  attitude_stale,
  rate_stale,
  sample_skew,
  timestamp
};

struct Decision
{
  Reject reason = Reject::timestamp;
  double skew = std::numeric_limits<double>::infinity();
  bool valid = false;
};

inline bool fresh(const Timing &source, double timeout) noexcept
{
  return source.received && std::isfinite(source.age) && source.age >= 0.0
    && source.age <= timeout;
}

inline Decision evaluate(
  const Timing &position, const Timing &attitude, const Timing &rate,
  double timeout, double max_skew) noexcept
{
  Decision result;
  if (!fresh(position, timeout)) {
    result.reason = Reject::position_stale;
    return result;
  }
  if (!fresh(attitude, timeout)) {
    result.reason = Reject::attitude_stale;
    return result;
  }
  if (!fresh(rate, timeout)) {
    result.reason = Reject::rate_stale;
    return result;
  }
  if (position.sample_time == 0 || attitude.sample_time == 0 || rate.sample_time == 0) {
    result.reason = Reject::timestamp;
    return result;
  }
  const auto oldest = std::min({position.sample_time, attitude.sample_time, rate.sample_time});
  const auto newest = std::max({position.sample_time, attitude.sample_time, rate.sample_time});
  result.skew = static_cast<double>(newest - oldest) * 1.0e-6;
  if (!std::isfinite(result.skew) || result.skew > max_skew) {
    result.reason = Reject::sample_skew;
    return result;
  }
  result.reason = Reject::none;
  result.valid = true;
  return result;
}

inline const char *reasonName(Reject reason) noexcept
{
  switch (reason) {
    case Reject::none: return "NONE";
    case Reject::position_stale: return "POSITION_STALE";
    case Reject::attitude_stale: return "ATTITUDE_STALE";
    case Reject::rate_stale: return "ANGULAR_VELOCITY_STALE";
    case Reject::sample_skew: return "SAMPLE_SKEW";
    case Reject::timestamp: return "TIMESTAMP";
  }
  return "UNKNOWN";
}

}  // namespace mpc_controller::state_check
