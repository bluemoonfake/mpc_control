#include "mpc_controller/solver/tpmc_constraints.hpp"

#include "mpc_controller/solver/tpmc_model.hpp"
#include "mpc_controller/solver/tpmc_reference.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

namespace mpc_controller::tpmc {
namespace {

double boundViolation(double value, double lower, double upper) noexcept {
  if (!std::isfinite(value)) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max({lower - value, 0.0, value - upper});
}

template <std::size_t Size>
std::string describeBoundViolation(const std::array<double, Size> &value,
                                   const std::array<double, Size> &lower,
                                   const std::array<double, Size> &upper,
                                   const std::array<const char *, Size> &names,
                                   const char *kind) {
  for (std::size_t index = 0; index < Size; ++index) {
    if (!std::isfinite(value[index])) {
      return std::string(kind) + "[" + names[index] + "]=non_finite";
    }
    if (value[index] < lower[index] || value[index] > upper[index]) {
      return std::string(kind) + "[" + names[index] +
             "]=" + std::to_string(value[index]) + " outside [" +
             std::to_string(lower[index]) + ", " +
             std::to_string(upper[index]) + "]";
    }
  }
  return {};
}

constexpr std::array<const char *, kStateDimension> kStateNames{
    "position_x", "position_y",
    "position_z", "velocity_x",
    "velocity_y", "velocity_z",
    "roll",       "pitch",
    "yaw",        "yaw_rate",
    "collective_specific_force"};

constexpr std::array<const char *, kInputDimension> kInputNames{
    "roll_command", "pitch_command", "yaw_command",
    "collective_specific_force_command"};

} // namespace

double stateConstraintViolation(const State &state,
                                const Configuration &configuration) noexcept {
  if (!finite(state)) {
    return std::numeric_limits<double>::infinity();
  }
  double violation = 0.0;
  for (std::size_t index = 0; index < kStateDimension; ++index) {
    violation =
        std::max(violation,
                 boundViolation(state[index], configuration.state_lower[index],
                                configuration.state_upper[index]));
  }
  const double tilt = tiltAngle(state);
  if (!std::isfinite(tilt)) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(violation, tilt - configuration.max_tilt_rad);
}

double inputConstraintViolation(const Input &input,
                                const Configuration &configuration) noexcept {
  if (!finite(input)) {
    return std::numeric_limits<double>::infinity();
  }
  double violation = 0.0;
  for (std::size_t index = 0; index < kInputDimension; ++index) {
    violation =
        std::max(violation,
                 boundViolation(input[index], configuration.input_lower[index],
                                configuration.input_upper[index]));
  }
  return violation;
}

double inputTransitionConstraintViolation(
    const Input &previous_input, const Input &input,
    const Configuration &configuration) noexcept {
  if (!finite(previous_input) || !finite(input)) {
    return std::numeric_limits<double>::infinity();
  }
  const double maximum_tilt_change = configuration.max_tilt_rate_rad_s *
                                    configuration.sample_time_seconds;
  const double roll_change = std::abs(previous_input[roll_command] - input[roll_command]);
  const double pitch_change = std::abs(previous_input[pitch_command] - input[pitch_command]);
  const double maximum_yaw_change = configuration.max_yaw_command_rate_rad_s *
                                    configuration.sample_time_seconds;
  const double yaw_change =
      std::abs(shortestAngle(previous_input[yaw_command], input[yaw_command]));
  const double maximum_collective_change = configuration.max_collective_rate_m_s3 *
                                           configuration.sample_time_seconds;
  const double collective_change = std::abs(previous_input[collective_specific_force_command] -
                                            input[collective_specific_force_command]);
  return std::max({inputConstraintViolation(input, configuration),
                   roll_change - maximum_tilt_change,
                   pitch_change - maximum_tilt_change,
                   yaw_change - maximum_yaw_change,
                   collective_change - maximum_collective_change});
}

double constraintViolation(const State &state, const Input &input,
                           const Configuration &configuration) noexcept {
  return std::max(stateConstraintViolation(state, configuration),
                  inputConstraintViolation(input, configuration));
}

bool hasValidCollectiveSpecificForce(
    const State &state, const Configuration &configuration) noexcept {
  const double collective = state[collective_specific_force];
  return std::isfinite(collective) &&
         collective >= configuration.state_lower[collective_specific_force] &&
         collective <= configuration.state_upper[collective_specific_force];
}

std::string describeStateViolation(const State &state,
                                   const Configuration &configuration) {
  const std::string bound_violation =
      describeBoundViolation(state, configuration.state_lower,
                             configuration.state_upper, kStateNames, "state");
  if (!bound_violation.empty()) {
    return bound_violation;
  }

  const double tilt = tiltAngle(state);
  if (!std::isfinite(tilt)) {
    return "state[tilt_angle]=non_finite";
  }
  if (tilt > configuration.max_tilt_rad) {
    return "state[tilt_angle]=" + std::to_string(tilt) + " exceeds " +
           std::to_string(configuration.max_tilt_rad);
  }
  return {};
}

std::string describeMeasuredStateViolation(
    const State &state, const Configuration &configuration) {
  for (std::size_t index = 0; index < kStateDimension; ++index) {
    if (!std::isfinite(state[index])) {
      return "state[" + std::string(kStateNames[index]) + "]=non_finite";
    }
  }
  for (std::size_t index = 0; index < kStateDimension; ++index) {
    if (index == roll || index == pitch || index == yaw ||
        index == yaw_rate) {
      continue;
    }
    if (state[index] < configuration.state_lower[index] ||
        state[index] > configuration.state_upper[index]) {
      return "state[" + std::string(kStateNames[index]) + "]=" +
             std::to_string(state[index]) + " outside [" +
             std::to_string(configuration.state_lower[index]) + ", " +
             std::to_string(configuration.state_upper[index]) + "]";
    }
  }
  return {};
}

std::string describeInputViolation(const Input &input,
                                   const Configuration &configuration) {
  return describeBoundViolation(input, configuration.input_lower,
                                configuration.input_upper, kInputNames,
                                "input");
}

std::string
describeInputTransitionViolation(const Input &previous_input,
                                 const Input &input,
                                 const Configuration &configuration) {
  const std::string input_violation =
      describeInputViolation(input, configuration);
  if (!input_violation.empty()) {
    return input_violation;
  }
  if (!finite(previous_input)) {
    return "previous_input=non_finite";
  }
  const double maximum_tilt_change = configuration.max_tilt_rate_rad_s *
                                    configuration.sample_time_seconds;
  const double roll_change = std::abs(previous_input[roll_command] - input[roll_command]);
  if (roll_change > maximum_tilt_change) {
    return "input_transition[roll_command]=" + std::to_string(roll_change) +
           " exceeds " + std::to_string(maximum_tilt_change);
  }
  const double pitch_change = std::abs(previous_input[pitch_command] - input[pitch_command]);
  if (pitch_change > maximum_tilt_change) {
    return "input_transition[pitch_command]=" + std::to_string(pitch_change) +
           " exceeds " + std::to_string(maximum_tilt_change);
  }
  const double maximum_yaw_change = configuration.max_yaw_command_rate_rad_s *
                                    configuration.sample_time_seconds;
  const double yaw_change =
      std::abs(shortestAngle(previous_input[yaw_command], input[yaw_command]));
  if (yaw_change > maximum_yaw_change) {
    return "input_transition[yaw_command]=" + std::to_string(yaw_change) +
           " exceeds " + std::to_string(maximum_yaw_change);
  }
  const double maximum_collective_change = configuration.max_collective_rate_m_s3 *
                                           configuration.sample_time_seconds;
  const double collective_change = std::abs(previous_input[collective_specific_force_command] -
                                            input[collective_specific_force_command]);
  if (collective_change > maximum_collective_change) {
    return "input_transition[collective_command]=" + std::to_string(collective_change) +
           " exceeds " + std::to_string(maximum_collective_change);
  }
  return {};
}

} // namespace mpc_controller::tpmc
