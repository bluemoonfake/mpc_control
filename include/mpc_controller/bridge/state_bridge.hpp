#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

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
  double yaw_rate_enu = 0.0;
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

  //Q_norm validation, oke if between 0.999 - 1.001
  const auto &q_px4 = attitude.body_frd_to_world_ned;
  const double q_norm = std::sqrt(
    q_px4[0] * q_px4[0] + q_px4[1] * q_px4[1]
    + q_px4[2] * q_px4[2] + q_px4[3] * q_px4[3]);
  if (q_norm < 1.0e-9) {
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
  const double roll_enu = std::atan2(r_enu_flu[2][1], r_enu_flu[2][2]);
  const double pitch_enu = std::asin(std::clamp(-r_enu_flu[2][0], -1.0, 1.0));
  const double cos_pitch = std::cos(pitch_enu);
  if (!std::isfinite(cos_pitch) || std::abs(cos_pitch) < 1.0e-6) {
    output.body_rate_valid = false;
    return false;
  }
  const double pitch_rate_flu = -angular_velocity.body_rate_frd[1];
  const double yaw_rate_flu = -angular_velocity.body_rate_frd[2];
  output.yaw_rate_enu = (std::sin(roll_enu) * pitch_rate_flu + std::cos(roll_enu) * yaw_rate_flu) / cos_pitch;
  const bool finite_output = std::isfinite(output.yaw_enu) && std::isfinite(output.yaw_rate_enu)
    && finite(output.position_enu) && finite(output.velocity_enu) && finite(output.acceleration_enu)
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
