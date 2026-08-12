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
  double vehicle_mass_kg = 2.0;
  double gravity_mps2 = 9.80665;
  double hover_thrust_normalized = 0.765;
};

inline bool valid(const Mapping &mapping) noexcept
{
  return std::isfinite(mapping.vehicle_mass_kg) && mapping.vehicle_mass_kg > 0.0
    && std::isfinite(mapping.gravity_mps2) && mapping.gravity_mps2 > 0.0
    && std::isfinite(mapping.hover_thrust_normalized)
    && mapping.hover_thrust_normalized > 0.0
    && mapping.hover_thrust_normalized <= kNormalizedLimit;
}

inline std::optional<double> forceToBodyFrdZ(
  double force_n, const Mapping &mapping) noexcept
{
  if (!valid(mapping) || !std::isfinite(force_n) || force_n < 0.0) {
    return std::nullopt;
  }
  const double hover_force = mapping.vehicle_mass_kg * mapping.gravity_mps2;
  const double max_force = hover_force / mapping.hover_thrust_normalized;
  if (!std::isfinite(max_force) || force_n > max_force) {
    return std::nullopt;
  }
  // Positive FLU collective force becomes negative body-FRD Z thrust.
  return -mapping.hover_thrust_normalized * force_n / hover_force;
}

}  // namespace mpc_controller::px4_thrust

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
  // All terms produce PX4 normalized torque directly. normalized_inertia is
  // J divided by the physical-to-normalized torque scale of each body axis.
  Vector3 attitude_gain{0.8, 0.8, 0.4};
  Vector3 rate_gain{0.15, 0.15, 0.10};
  Vector3 normalized_inertia{Vector3::Zero()};
  Vector3 normalized_limit{0.30, 0.30, 0.20};
  bool enable_dynamics_compensation = false;
};

struct DesiredKinematicsParameters
{
  double filter_time_constant_seconds = 0.05;
  double max_sample_interval_seconds = 0.10;
  Vector3 max_body_rate_rad_s{2.0, 2.0, 1.0};
  Vector3 max_body_acceleration_rad_s2{10.0, 10.0, 5.0};
};

struct DesiredKinematics
{
  Vector3 body_rate_rad_s{Vector3::Zero()};
  Vector3 body_acceleration_rad_s2{Vector3::Zero()};
  bool valid = false;
};

struct Input
{
  // Desired rotation is body FLU -> world ENU, Hamilton quaternion order.
  Quaternion desired_body_flu_to_world_enu{Quaternion::Identity()};
  Quaternion measured_body_flu_to_world_enu{Quaternion::Identity()};
  Vector3 desired_body_rate_flu_rad_s{Vector3::Zero()};
  Vector3 desired_body_acceleration_flu_rad_s2{Vector3::Zero()};
  Vector3 measured_body_rate_flu_rad_s{Vector3::Zero()};
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
  std::array<double, 3> desired_body_rate_flu_rad_s{};
  std::array<double, 3> desired_body_acceleration_flu_rad_s2{};
  std::array<double, 3> feedback_torque_body_flu_normalized{};
  std::array<double, 3> dynamics_torque_body_flu_normalized{};
  bool dynamics_compensation_enabled = false;
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
    && parameters.normalized_inertia.allFinite() && parameters.normalized_limit.allFinite()
    && (parameters.attitude_gain.array() >= 0.0).all()
    && (parameters.rate_gain.array() >= 0.0).all()
    && (parameters.normalized_inertia.array() >= 0.0).all()
    && (!parameters.enable_dynamics_compensation
      || (parameters.normalized_inertia.array() > 0.0).all())
    && (parameters.normalized_limit.array() > 0.0).all()
    && (parameters.normalized_limit.array() <= 1.0).all();
}

inline bool validDesiredKinematicsParameters(
  const DesiredKinematicsParameters &parameters) noexcept
{
  return std::isfinite(parameters.filter_time_constant_seconds)
    && parameters.filter_time_constant_seconds >= 0.0
    && std::isfinite(parameters.max_sample_interval_seconds)
    && parameters.max_sample_interval_seconds > 0.0
    && parameters.max_body_rate_rad_s.allFinite()
    && parameters.max_body_acceleration_rad_s2.allFinite()
    && (parameters.max_body_rate_rad_s.array() > 0.0).all()
    && (parameters.max_body_acceleration_rad_s2.array() > 0.0).all();
}

inline Vector3 vee(const Eigen::Matrix3d &skew) noexcept
{
  return {skew(2, 1), skew(0, 2), skew(1, 0)};
}

inline Eigen::Matrix3d hat(const Vector3 &vector) noexcept
{
  Eigen::Matrix3d output;
  output << 0.0, -vector.z(), vector.y(),
    vector.z(), 0.0, -vector.x(),
    -vector.y(), vector.x(), 0.0;
  return output;
}

class DesiredKinematicsEstimator
{
public:
  explicit DesiredKinematicsEstimator(const DesiredKinematicsParameters &parameters)
  : parameters_(parameters) {}

  void reset() noexcept
  {
    initialized_ = false;
    rate_initialized_ = false;
    previous_time_seconds_ = 0.0;
    previous_rotation_.setIdentity();
    filtered_rate_.setZero();
    filtered_acceleration_.setZero();
  }

  DesiredKinematics update(
    const Quaternion &desired_body_to_world, double sample_time_seconds) noexcept
  {
    DesiredKinematics output;
    if (!validDesiredKinematicsParameters(parameters_)
      || !finiteQuaternion(desired_body_to_world) || !std::isfinite(sample_time_seconds)
      || desired_body_to_world.norm() < 1.0e-9) {
      reset();
      return output;
    }
    const Eigen::Matrix3d rotation = desired_body_to_world.normalized().toRotationMatrix();
    if (!rotation.allFinite()) {
      reset();
      return output;
    }
    if (!initialized_) {
      previous_rotation_ = rotation;
      previous_time_seconds_ = sample_time_seconds;
      initialized_ = true;
      return output;
    }

    const double dt = sample_time_seconds - previous_time_seconds_;
    if (!std::isfinite(dt) || dt <= 1.0e-5
      || dt > parameters_.max_sample_interval_seconds) {
      previous_rotation_ = rotation;
      previous_time_seconds_ = sample_time_seconds;
      filtered_rate_.setZero();
      filtered_acceleration_.setZero();
      rate_initialized_ = false;
      return output;
    }

    const Eigen::AngleAxisd delta(previous_rotation_.transpose() * rotation);
    Vector3 raw_rate = delta.axis() * delta.angle() / dt;
    if (!raw_rate.allFinite()) {
      reset();
      return output;
    }
    for (int axis = 0; axis < 3; ++axis) {
      raw_rate[axis] = std::clamp(
        raw_rate[axis], -parameters_.max_body_rate_rad_s[axis],
        parameters_.max_body_rate_rad_s[axis]);
    }

    const double alpha = parameters_.filter_time_constant_seconds > 0.0
      ? dt / (parameters_.filter_time_constant_seconds + dt) : 1.0;
    if (!rate_initialized_) {
      // Two rotations establish omega_d, but not omega_dot_d. Avoid a
      // synthetic acceleration impulse after reset or Offboard handover.
      filtered_rate_ = raw_rate;
      filtered_acceleration_.setZero();
      rate_initialized_ = true;
    } else {
      const Vector3 previous_filtered_rate = filtered_rate_;
      filtered_rate_ += alpha * (raw_rate - filtered_rate_);
      Vector3 raw_acceleration = (filtered_rate_ - previous_filtered_rate) / dt;
      for (int axis = 0; axis < 3; ++axis) {
        raw_acceleration[axis] = std::clamp(
          raw_acceleration[axis], -parameters_.max_body_acceleration_rad_s2[axis],
          parameters_.max_body_acceleration_rad_s2[axis]);
      }
      filtered_acceleration_ += alpha * (raw_acceleration - filtered_acceleration_);
    }

    previous_rotation_ = rotation;
    previous_time_seconds_ = sample_time_seconds;
    output.body_rate_rad_s = filtered_rate_;
    output.body_acceleration_rad_s2 = filtered_acceleration_;
    output.valid = filtered_rate_.allFinite() && filtered_acceleration_.allFinite();
    return output;
  }

private:
  DesiredKinematicsParameters parameters_{};
  bool initialized_ = false;
  bool rate_initialized_ = false;
  double previous_time_seconds_ = 0.0;
  Eigen::Matrix3d previous_rotation_{Eigen::Matrix3d::Identity()};
  Vector3 filtered_rate_{Vector3::Zero()};
  Vector3 filtered_acceleration_{Vector3::Zero()};
};

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
  if (!input.measured_body_rate_flu_rad_s.allFinite()
    || !input.desired_body_rate_flu_rad_s.allFinite()
    || !input.desired_body_acceleration_flu_rad_s2.allFinite()) {
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
  const Eigen::Matrix3d measured_to_desired = measured_rotation.transpose() * desired_rotation;
  const Vector3 transported_desired_rate =
    measured_to_desired * input.desired_body_rate_flu_rad_s;
  const Vector3 rate_error = input.measured_body_rate_flu_rad_s - transported_desired_rate;
  if (!attitude_error.allFinite() || !rate_error.allFinite()) {
    output.failure_reason = FailureReason::invalid_torque;
    return output;
  }

  const Vector3 feedback_torque_flu =
    -torque_parameters.attitude_gain.cwiseProduct(attitude_error)
    - torque_parameters.rate_gain.cwiseProduct(rate_error);
  Vector3 dynamics_torque_flu{Vector3::Zero()};
  if (torque_parameters.enable_dynamics_compensation) {
    const Eigen::DiagonalMatrix<double, 3> normalized_inertia(
      torque_parameters.normalized_inertia);
    const Vector3 measured_rate = input.measured_body_rate_flu_rad_s;
    dynamics_torque_flu = measured_rate.cross(normalized_inertia * measured_rate)
      - normalized_inertia * (
      hat(measured_rate) * transported_desired_rate
      - measured_to_desired * input.desired_body_acceleration_flu_rad_s2);
  }
  const Vector3 torque_flu = feedback_torque_flu + dynamics_torque_flu;
  Vector3 limited_torque_flu;
  for (int axis = 0; axis < 3; ++axis) {
    limited_torque_flu[axis] = std::clamp(
      torque_flu[axis], -torque_parameters.normalized_limit[axis],
      torque_parameters.normalized_limit[axis]);
    output.torque_saturated = output.torque_saturated
      || std::abs(limited_torque_flu[axis] - torque_flu[axis]) > 1.0e-12;
    output.attitude_error[axis] = attitude_error[axis];
    output.body_rate_error_rad_s[axis] = rate_error[axis];
    output.desired_body_rate_flu_rad_s[axis] = input.desired_body_rate_flu_rad_s[axis];
    output.desired_body_acceleration_flu_rad_s2[axis] =
      input.desired_body_acceleration_flu_rad_s2[axis];
    output.feedback_torque_body_flu_normalized[axis] = feedback_torque_flu[axis];
    output.dynamics_torque_body_flu_normalized[axis] = dynamics_torque_flu[axis];
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
  output.dynamics_compensation_enabled = torque_parameters.enable_dynamics_compensation;
  output.valid = true;
  output.failure_reason = FailureReason::none;
  return output;
}

}  // namespace mpc_controller::px4_control
