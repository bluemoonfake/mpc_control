#pragma once

#include <array>
#include <cmath>
#include <string>

namespace mpc_controller::reference
{

struct Sample
{
  std::array<double, 3> position{};
  std::array<double, 3> velocity{};
  std::array<double, 3> acceleration{};
  double yaw = 0.0;
  double yaw_rate = 0.0;
};

struct Parameters
{
  std::string type = "hold";
  std::array<double, 3> hold_position{0.0, 0.0, 1.0};
  std::array<double, 3> line_start{0.0, 0.0, 1.0};
  std::array<double, 3> line_end{1.0, 0.0, 1.0};
  double line_duration_seconds = 10.0;
  std::array<double, 3> circle_center{0.0, 0.0, 1.0};
  double circle_radius = 2.0;
  double circle_period_seconds = 60.0;
  double circle_ramp_seconds = 3.0;
  double circle_phase_rad = 0.0;
  int circle_direction = 1;
  double hold_yaw_rad = 0.0;
};

inline bool finite(const std::array<double, 3> &value) noexcept
{
  return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

inline bool valid(const Parameters &parameters) noexcept
{
  return (parameters.type == "hold" || parameters.type == "line" || parameters.type == "circle")
    && finite(parameters.hold_position) && finite(parameters.line_start)
    && finite(parameters.line_end) && finite(parameters.circle_center)
    && std::isfinite(parameters.line_duration_seconds) && parameters.line_duration_seconds > 0.0
    && std::isfinite(parameters.circle_radius) && parameters.circle_radius > 0.0
    && std::isfinite(parameters.circle_period_seconds) && parameters.circle_period_seconds > 0.0
    && std::isfinite(parameters.circle_ramp_seconds) && parameters.circle_ramp_seconds > 0.0
    && 2.0 * parameters.circle_ramp_seconds < parameters.circle_period_seconds
    && std::isfinite(parameters.circle_phase_rad) && std::isfinite(parameters.hold_yaw_rad)
    && (parameters.circle_direction == 1 || parameters.circle_direction == -1);
}

inline bool sample(const Parameters &parameters, double time_seconds, Sample &output) noexcept
{
  if (!valid(parameters) || !std::isfinite(time_seconds) || time_seconds < 0.0) {
    return false;
  }

  constexpr double pi = 3.14159265358979323846;
  if (parameters.type == "hold") {
    output.position = parameters.hold_position;
    output.yaw = parameters.hold_yaw_rad;
    return true;
  }

  if (parameters.type == "line") {
    const std::array<double, 3> delta{
      parameters.line_end[0] - parameters.line_start[0],
      parameters.line_end[1] - parameters.line_start[1],
      parameters.line_end[2] - parameters.line_start[2]};
    const double duration = parameters.line_duration_seconds;
    const double t = std::min(time_seconds, duration);
    const double alpha = t / duration;
    output.position = {
      parameters.line_start[0] + alpha * delta[0],
      parameters.line_start[1] + alpha * delta[1],
      parameters.line_start[2] + alpha * delta[2]};
    if (time_seconds < duration) {
      output.velocity = {delta[0] / duration, delta[1] / duration, delta[2] / duration};
      output.yaw = std::atan2(output.velocity[1], output.velocity[0]);
    } else {
      output.velocity = {0.0, 0.0, 0.0};
      output.yaw = std::atan2(delta[1], delta[0]);
    }
    output.acceleration = {0.0, 0.0, 0.0};
    return true;
  }

  // Use a quintic velocity blend at both ends of the lap. This keeps position,
  // velocity and acceleration continuous when transitioning to/from hold.
  // The integral of smoothstep5 over [0, 1] is 1/2, so the two ramps consume
  // the same angular distance as one ramp-duration at cruise speed.
  const double duration = parameters.circle_period_seconds;
  const double ramp = parameters.circle_ramp_seconds;
  const double time = std::min(time_seconds, duration);
  const double direction = static_cast<double>(parameters.circle_direction);
  const double cruise_omega = direction * 2.0 * pi / (duration - ramp);
  const auto smoothstep5 = [](double tau) noexcept {
      return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau));
    };
  const auto smoothstep5_derivative = [](double tau) noexcept {
      const double one_minus_tau = 1.0 - tau;
      return 30.0 * tau * tau * one_minus_tau * one_minus_tau;
    };
  const auto smoothstep5_integral = [](double tau) noexcept {
      const double tau2 = tau * tau;
      const double tau4 = tau2 * tau2;
      return tau4 * (2.5 + tau * (-3.0 + tau));
    };

  double phase_offset = 0.0;
  double omega = 0.0;
  double angular_acceleration = 0.0;
  if (time < ramp) {
    const double tau = time / ramp;
    phase_offset = cruise_omega * ramp * smoothstep5_integral(tau);
    omega = cruise_omega * smoothstep5(tau);
    angular_acceleration = cruise_omega / ramp * smoothstep5_derivative(tau);
  } else if (time <= duration - ramp) {
    phase_offset = cruise_omega * (time - 0.5 * ramp);
    omega = cruise_omega;
  } else {
    const double tau = (duration - time) / ramp;
    phase_offset = direction * 2.0 * pi -
      cruise_omega * ramp * smoothstep5_integral(tau);
    omega = cruise_omega * smoothstep5(tau);
    angular_acceleration = -cruise_omega / ramp * smoothstep5_derivative(tau);
  }

  const double phase = parameters.circle_phase_rad + phase_offset;
  const double c = std::cos(phase);
  const double s = std::sin(phase);
  output.position = {
    parameters.circle_center[0] + parameters.circle_radius * c,
    parameters.circle_center[1] + parameters.circle_radius * s,
    parameters.circle_center[2]};
  output.velocity = {
    -parameters.circle_radius * omega * s,
    parameters.circle_radius * omega * c,
    0.0};
  output.acceleration = {
    -parameters.circle_radius * (omega * omega * c + angular_acceleration * s),
    parameters.circle_radius * (angular_acceleration * c - omega * omega * s),
    0.0};
  output.yaw = phase + (direction >= 0.0 ? pi / 2.0 : -pi / 2.0);
  output.yaw_rate = omega;
  return true;
}

}  // namespace mpc_controller::reference
