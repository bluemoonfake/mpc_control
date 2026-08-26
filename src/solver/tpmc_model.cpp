#include "mpc_controller/solver/tpmc_model.hpp"

#include <algorithm>
#include <cmath>

namespace mpc_controller::tpmc {
namespace {

Vector3 bodyZDirectionInternal(const State &state) noexcept {
  const double roll_value = state[StateIndex::roll];
  const double pitch_value = state[StateIndex::pitch];
  const double yaw_value = state[StateIndex::yaw];
  const double sr = std::sin(roll_value);
  const double cr = std::cos(roll_value);
  const double sp = std::sin(pitch_value);
  const double cp = std::cos(pitch_value);
  const double sy = std::sin(yaw_value);
  const double cy = std::cos(yaw_value);

  return {cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr};
}

State addScaled(const State &left, const State &right, double scale) noexcept {
  State output{};
  for (std::size_t index = 0; index < kStateDimension; ++index) {
    output[index] = left[index] + scale * right[index];
  }
  return output;
}

State weightedSum(const State &k1, const State &k2, const State &k3,
                  const State &k4, double step_seconds) noexcept {
  State output{};
  for (std::size_t index = 0; index < kStateDimension; ++index) {
    output[index] = step_seconds / 6.0 *
                    (k1[index] + 2.0 * k2[index] + 2.0 * k3[index] + k4[index]);
  }
  return output;
}

double shortestAngularDifference(double from, double to) noexcept {
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

} // namespace

Vector3 bodyZDirection(const State &state) noexcept {
  return bodyZDirectionInternal(state);
}

double tiltAngle(const State &state) noexcept {
  const Vector3 body_z = bodyZDirection(state);
  return std::acos(std::clamp(body_z[2], -1.0, 1.0));
}

State continuousDynamics(const State &state, const Input &input,
                         const ModelParameters &parameters) noexcept {
  State derivative{};
  const Vector3 body_z = bodyZDirection(state);
  const double collective_force = state[StateIndex::collective_specific_force];

  derivative[StateIndex::position_x] = state[StateIndex::velocity_x];
  derivative[StateIndex::position_y] = state[StateIndex::velocity_y];
  derivative[StateIndex::position_z] = state[StateIndex::velocity_z];
  derivative[StateIndex::velocity_x] = body_z[0] * collective_force;
  derivative[StateIndex::velocity_y] = body_z[1] * collective_force;
  derivative[StateIndex::velocity_z] = body_z[2] * collective_force - parameters.gravity_m_s2;

  derivative[StateIndex::roll] =
      (input[InputIndex::roll_command] - state[StateIndex::roll]) /
      parameters.roll_time_constant_seconds;
  derivative[StateIndex::pitch] =
      (input[InputIndex::pitch_command] - state[StateIndex::pitch]) /
      parameters.pitch_time_constant_seconds;
  derivative[StateIndex::yaw] = state[StateIndex::yaw_rate];
  const double yaw_error = shortestAngularDifference(
      state[StateIndex::yaw], input[InputIndex::yaw_command]);
  const double yaw_natural_frequency =
      parameters.yaw_natural_frequency_rad_s;
  derivative[StateIndex::yaw_rate] =
      yaw_natural_frequency * yaw_natural_frequency * yaw_error -
      2.0 * parameters.yaw_damping_ratio * yaw_natural_frequency *
          state[StateIndex::yaw_rate];
  derivative[StateIndex::collective_specific_force] =
      (input[InputIndex::collective_specific_force_command] -
       state[StateIndex::collective_specific_force]) /
      parameters.collective_time_constant_seconds;
  return derivative;
}

State integrateErk4(const State &state, const Input &input, double step_seconds,
                    const ModelParameters &parameters) noexcept {
  const State k1 = continuousDynamics(state, input, parameters);
  const State k2 = continuousDynamics(addScaled(state, k1, 0.5 * step_seconds),
                                      input, parameters);
  const State k3 = continuousDynamics(addScaled(state, k2, 0.5 * step_seconds),
                                      input, parameters);
  const State k4 =
      continuousDynamics(addScaled(state, k3, step_seconds), input, parameters);
  const State increment = weightedSum(k1, k2, k3, k4, step_seconds);

  State output{};
  for (std::size_t index = 0; index < kStateDimension; ++index) {
    output[index] = state[index] + increment[index];
  }
  return output;
}

} // namespace mpc_controller::tpmc
