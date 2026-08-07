#include "mpc_control_px4/px4_trajectory_adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace mpc_control_px4
{

namespace
{

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTwoPi = 2.0 * kPi;

Px4TrajectoryAdapterResult failure(
    const Px4AdapterFailureReason reason,
    const char* detail)
{
  Px4TrajectoryAdapterResult result;
  result.valid = false;
  result.failure_reason = reason;
  result.detail = detail;
  return result;
}

}  // namespace

Px4TrajectoryAdapter::Px4TrajectoryAdapter(
    Px4TrajectoryAdapterConfig configuration)
: configuration_(std::move(configuration))
{
}

Px4TrajectoryAdapterResult Px4TrajectoryAdapter::convert(
    const mpc_control_msgs::msg::TrajectoryCommand& command,
    const std::uint64_t px4_timestamp_us) noexcept
{
  if (command.header.frame_id != configuration_.input_frame_id) {
    return failure(
        Px4AdapterFailureReason::InvalidFrame,
        "command frame_id does not match the configured ENU frame");
  }
  if (px4_timestamp_us == 0U) {
    return failure(
        Px4AdapterFailureReason::InvalidTimestamp,
        "PX4 timestamp must be non-zero microseconds");
  }
  if (last_timestamp_us_ && px4_timestamp_us <= *last_timestamp_us_) {
    return failure(
        Px4AdapterFailureReason::TimestampNotMonotonic,
        "PX4 timestamp is not strictly increasing");
  }

  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (!finiteFloat(command.position[axis])
        || !finiteFloat(command.velocity[axis])
        || !finiteFloat(command.acceleration[axis])) {
      return failure(
          Px4AdapterFailureReason::NonFiniteTranslationalCommand,
          "position, velocity and acceleration must convert to finite float values");
    }
  }

  if (command.yaw_valid
      && (!finiteFloat(command.yaw) || !finiteFloat(command.yaw_rate))) {
    return failure(
        Px4AdapterFailureReason::NonFiniteYawCommand,
        "yaw and yaw_rate must be finite when yaw_valid is true");
  }

  px4_msgs::msg::TrajectorySetpoint setpoint;
  setpoint.timestamp = px4_timestamp_us;

  // ROS ENU [east, north, up] becomes PX4 local NED [north, east, down].
  setpoint.position = {
    toFloat(command.position[1]),
    toFloat(command.position[0]),
    toFloat(-command.position[2])};
  setpoint.velocity = {
    toFloat(command.velocity[1]),
    toFloat(command.velocity[0]),
    toFloat(-command.velocity[2])};
  setpoint.acceleration = {
    toFloat(command.acceleration[1]),
    toFloat(command.acceleration[0]),
    toFloat(-command.acceleration[2])};

  const float nan = std::numeric_limits<float>::quiet_NaN();
  setpoint.jerk = {nan, nan, nan};
  if (command.yaw_valid) {
    setpoint.yaw = toFloat(wrapPi(kPi / 2.0 - command.yaw));
    setpoint.yawspeed = toFloat(-command.yaw_rate);
  } else {
    setpoint.yaw = nan;
    setpoint.yawspeed = nan;
  }

  last_timestamp_us_ = px4_timestamp_us;
  Px4TrajectoryAdapterResult result;
  result.setpoint = setpoint;
  result.valid = true;
  result.failure_reason = Px4AdapterFailureReason::None;
  return result;
}

void Px4TrajectoryAdapter::reset() noexcept
{
  last_timestamp_us_.reset();
}

bool Px4TrajectoryAdapter::finiteFloat(const double value) noexcept
{
  return std::isfinite(value) && std::isfinite(static_cast<float>(value));
}

double Px4TrajectoryAdapter::wrapPi(const double angle) noexcept
{
  double wrapped = std::fmod(angle + kPi, kTwoPi);
  if (wrapped < 0.0) {
    wrapped += kTwoPi;
  }
  return wrapped - kPi;
}

float Px4TrajectoryAdapter::toFloat(const double value) noexcept
{
  return static_cast<float>(value);
}

}  // namespace mpc_control_px4
