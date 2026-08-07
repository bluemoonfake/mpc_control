#include <algorithm>
#include <cmath>

namespace
{

constexpr int kHorizon = 8;
constexpr double kDt = 0.02;
constexpr double kMaxAcceleration = 4.0;
constexpr double kMaxAccelerationStep = 0.25;
constexpr double kPositionWeight = 12.0;
constexpr double kVelocityWeight = 2.0;
constexpr double kAccelerationWeight = 0.25;
constexpr double kInputWeight = 0.04;
constexpr double kTerminalWeight = 2.0;

double clamp(const double value, const double lower, const double upper) noexcept
{
  return std::max(lower, std::min(value, upper));
}

double candidate_cost(
    const double position,
    const double velocity,
    const double reference_position[kHorizon],
    const double reference_velocity[kHorizon],
    const double reference_acceleration[kHorizon],
    const double input,
    const double previous_input) noexcept
{
  double predicted_position = position;
  double predicted_velocity = velocity;
  double cost = 0.0;

  for (int step = 0; step < kHorizon; ++step) {
    predicted_position += kDt * predicted_velocity
        + 0.5 * kDt * kDt * input;
    predicted_velocity += kDt * input;

    const double position_error = predicted_position - reference_position[step];
    const double velocity_error = predicted_velocity - reference_velocity[step];
    const double acceleration_error = input - reference_acceleration[step];
    cost += kPositionWeight * position_error * position_error
        + kVelocityWeight * velocity_error * velocity_error
        + kAccelerationWeight * acceleration_error * acceleration_error;
  }

  const double input_step = input - previous_input;
  cost += kInputWeight * input * input
      + kTerminalWeight * input_step * input_step;
  return cost;
}

}  // namespace

extern "C" void mpc_reset() noexcept
{
}

extern "C" int mpc_update(
    const double measured_position[3],
    const double measured_velocity[3],
    const double reference_position[3][kHorizon],
    const double reference_velocity[3][kHorizon],
    const double reference_acceleration[3][kHorizon],
    double acceleration_command[3],
    double* last_yaw_command) noexcept
{
  if (measured_position == nullptr || measured_velocity == nullptr
      || reference_position == nullptr || reference_velocity == nullptr
      || reference_acceleration == nullptr || acceleration_command == nullptr
      || last_yaw_command == nullptr) {
    return 0;
  }

  static double previous_input[3] = {0.0, 0.0, 0.0};
  double next_input[3] = {0.0, 0.0, 0.0};

  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(measured_position[axis])
        || !std::isfinite(measured_velocity[axis])) {
      return 0;
    }

    double best_input = previous_input[axis];
    double best_cost = INFINITY;
    const double lower = clamp(
        previous_input[axis] - kMaxAccelerationStep,
        -kMaxAcceleration, kMaxAcceleration);
    const double upper = clamp(
        previous_input[axis] + kMaxAccelerationStep,
        -kMaxAcceleration, kMaxAcceleration);

    for (int candidate_index = 0; candidate_index <= 8; ++candidate_index) {
      const double ratio = static_cast<double>(candidate_index) / 8.0;
      const double candidate = lower + ratio * (upper - lower);
      const double cost = candidate_cost(
          measured_position[axis], measured_velocity[axis],
          reference_position[axis], reference_velocity[axis],
          reference_acceleration[axis], candidate, previous_input[axis]);
      if (cost < best_cost) {
        best_cost = cost;
        best_input = candidate;
      }
    }
    if (!std::isfinite(best_input) || !std::isfinite(best_cost)) {
      return 0;
    }
    next_input[axis] = best_input;
    acceleration_command[axis] = best_input;
  }

  previous_input[0] = next_input[0];
  previous_input[1] = next_input[1];
  previous_input[2] = next_input[2];
  if (!std::isfinite(*last_yaw_command)) {
    *last_yaw_command = 0.0;
  }
  return 1;
}

