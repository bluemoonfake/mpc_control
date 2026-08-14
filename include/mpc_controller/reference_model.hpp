#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <vector>

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

struct CircleTiming
{
  double cruise_speed_m_s = 0.0;
  double ramp_seconds = 0.0;
  double period_seconds = 0.0;
  bool valid = false;
};

inline bool finite(const std::array<double, 3> &value) noexcept
{
  return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

// Pure-math rolling planner used by the joystick path. Obstacle data is not
// required here: callers provide a collision predicate, which currently may
// accept every dynamically feasible candidate.
struct PlannerConfig
{
  double horizon_seconds = 5.0;
  double sample_seconds = 0.1;
  double response_seconds = 0.8;
  double max_speed_xy = 5.0;
  double max_speed_z = 3.0;
  double max_acceleration_xy = 2.5;
  double max_acceleration_z = 2.0;
  double max_jerk_xy = 4.0;
  double max_jerk_z = 3.0;
  std::vector<double> heading_offsets_rad{-0.785398, -0.523599, -0.261799,
    0.0, 0.261799, 0.523599, 0.785398};
  std::vector<double> speed_scales{0.5, 0.75, 1.0};
  double intent_weight = 4.0;
  double progress_weight = 1.0;
  double acceleration_weight = 0.15;
  double jerk_weight = 0.05;
  double switch_weight = 0.4;
  double hysteresis = 0.1;
};

struct PlannerState
{
  std::array<double, 3> position{};
  std::array<double, 3> velocity{};
  std::array<double, 3> acceleration{};
};

struct Candidate
{
  std::vector<Sample> samples;
  std::array<double, 3> target_velocity{};
  double cost = std::numeric_limits<double>::infinity();
  int id = -1;
  bool feasible = false;
};

struct Plan
{
  Candidate selected;
  int feasible_count = 0;
  bool valid = false;
};

inline double horizontalNorm(const std::array<double, 3> &value) noexcept
{
  return std::hypot(value[0], value[1]);
}

inline bool valid(const PlannerConfig &config) noexcept
{
  const auto finite_positive = [](double value) {
      return std::isfinite(value) && value > 0.0;
    };
  const bool offsets_valid = !config.heading_offsets_rad.empty()
    && std::all_of(
    config.heading_offsets_rad.begin(), config.heading_offsets_rad.end(),
    [](double value) {return std::isfinite(value);});
  const bool scales_valid = !config.speed_scales.empty()
    && std::all_of(
    config.speed_scales.begin(), config.speed_scales.end(),
    [](double value) {return std::isfinite(value) && value > 0.0 && value <= 1.0;});
  return finite_positive(config.horizon_seconds)
    && finite_positive(config.sample_seconds)
    && config.sample_seconds <= config.horizon_seconds
    && finite_positive(config.response_seconds)
    && finite_positive(config.max_speed_xy) && finite_positive(config.max_speed_z)
    && finite_positive(config.max_acceleration_xy)
    && finite_positive(config.max_acceleration_z)
    && finite_positive(config.max_jerk_xy) && finite_positive(config.max_jerk_z)
    && offsets_valid && scales_valid
    && std::isfinite(config.intent_weight) && config.intent_weight >= 0.0
    && std::isfinite(config.progress_weight) && config.progress_weight >= 0.0
    && std::isfinite(config.acceleration_weight) && config.acceleration_weight >= 0.0
    && std::isfinite(config.jerk_weight) && config.jerk_weight >= 0.0
    && std::isfinite(config.switch_weight) && config.switch_weight >= 0.0
    && std::isfinite(config.hysteresis) && config.hysteresis >= 0.0;
}

inline std::array<double, 3> limitedAcceleration(
  const PlannerConfig &config, const PlannerState &state,
  const std::array<double, 3> &target_velocity) noexcept
{
  std::array<double, 3> desired{
    (target_velocity[0] - state.velocity[0]) / config.response_seconds,
    (target_velocity[1] - state.velocity[1]) / config.response_seconds,
    (target_velocity[2] - state.velocity[2]) / config.response_seconds};
  const double horizontal = horizontalNorm(desired);
  if (horizontal > config.max_acceleration_xy) {
    const double scale = config.max_acceleration_xy / horizontal;
    desired[0] *= scale;
    desired[1] *= scale;
  }
  desired[2] = std::clamp(
    desired[2], -config.max_acceleration_z, config.max_acceleration_z);
  return desired;
}

inline PlannerState advanceUnchecked(
  const PlannerConfig &config, const PlannerState &state,
  const std::array<double, 3> &target_velocity, double dt) noexcept
{
  PlannerState next = state;
  // Reference dynamics:
  //   a* = sat((v_target-v)/tau),  |a_next-a| <= j_max*dt
  //   v_next = v + 0.5*(a+a_next)*dt
  //   p_next = p + 0.5*(v+v_next)*dt
  const auto desired = limitedAcceleration(config, state, target_velocity);
  double jerk_x = desired[0] - state.acceleration[0];
  double jerk_y = desired[1] - state.acceleration[1];
  const double jerk_xy = std::hypot(jerk_x, jerk_y);
  const double max_delta_xy = config.max_jerk_xy * dt;
  if (jerk_xy > max_delta_xy) {
    const double scale = max_delta_xy / jerk_xy;
    jerk_x *= scale;
    jerk_y *= scale;
  }
  next.acceleration[0] += jerk_x;
  next.acceleration[1] += jerk_y;
  const double max_delta_z = config.max_jerk_z * dt;
  next.acceleration[2] += std::clamp(
    desired[2] - state.acceleration[2], -max_delta_z, max_delta_z);

  for (std::size_t axis = 0; axis < 3; ++axis) {
    next.velocity[axis] += 0.5 * (state.acceleration[axis] + next.acceleration[axis]) * dt;
  }
  const double speed_xy = horizontalNorm(next.velocity);
  if (speed_xy > config.max_speed_xy) {
    const double scale = config.max_speed_xy / speed_xy;
    next.velocity[0] *= scale;
    next.velocity[1] *= scale;
  }
  next.velocity[2] = std::clamp(next.velocity[2], -config.max_speed_z, config.max_speed_z);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    next.position[axis] += 0.5 * (state.velocity[axis] + next.velocity[axis]) * dt;
  }
  return next;
}

inline PlannerState advance(
  const PlannerConfig &config, const PlannerState &state,
  const std::array<double, 3> &target_velocity, double dt) noexcept
{
  if (!valid(config) || !finite(state.position) || !finite(state.velocity)
    || !finite(state.acceleration) || !finite(target_velocity)
    || !std::isfinite(dt) || dt <= 0.0) return state;
  return advanceUnchecked(config, state, target_velocity, dt);
}

inline bool dynamicallyFeasible(
  const PlannerConfig &config, const std::vector<Sample> &samples) noexcept
{
  if (samples.empty()) return false;
  for (std::size_t index = 0; index < samples.size(); ++index) {
    const auto &state_sample = samples[index];
    if (!finite(state_sample.position) || !finite(state_sample.velocity)
      || !finite(state_sample.acceleration)
      || horizontalNorm(state_sample.velocity) > config.max_speed_xy + 1.0e-6
      || std::abs(state_sample.velocity[2]) > config.max_speed_z + 1.0e-6
      || horizontalNorm(state_sample.acceleration) > config.max_acceleration_xy + 1.0e-6
      || std::abs(state_sample.acceleration[2]) > config.max_acceleration_z + 1.0e-6) {
      return false;
    }
    if (index > 0) {
      std::array<double, 3> jerk{};
      for (std::size_t axis = 0; axis < 3; ++axis) {
        jerk[axis] =
          (state_sample.acceleration[axis] - samples[index - 1].acceleration[axis])
          / config.sample_seconds;
      }
      if (horizontalNorm(jerk) > config.max_jerk_xy + 1.0e-6
        || std::abs(jerk[2]) > config.max_jerk_z + 1.0e-6) return false;
    }
  }
  return true;
}

inline Candidate makeCandidate(
  const PlannerConfig &config, const PlannerState &initial,
  const std::array<double, 3> &intent_velocity,
  const std::array<double, 3> &target_velocity, int id) noexcept
{
  // Discrete planner cost:
  // J=sum dt*[w_i||v-v_user||^2-w_p(v.dir_user)
  //          +w_a||a||^2+w_j||jerk||^2].
  // Axis terms are normalized by their configured physical limits.
  Candidate candidate;
  candidate.id = id;
  candidate.target_velocity = target_velocity;
  const auto count = static_cast<std::size_t>(
    std::floor(config.horizon_seconds / config.sample_seconds)) + 1U;
  candidate.samples.reserve(count);
  PlannerState state = initial;
  double cost = 0.0;
  const double speed_xy_sq = config.max_speed_xy * config.max_speed_xy;
  const double speed_z_sq = config.max_speed_z * config.max_speed_z;
  const double acceleration_xy_sq = config.max_acceleration_xy * config.max_acceleration_xy;
  const double acceleration_z_sq = config.max_acceleration_z * config.max_acceleration_z;
  const double jerk_xy_sq = config.max_jerk_xy * config.max_jerk_xy;
  const double jerk_z_sq = config.max_jerk_z * config.max_jerk_z;
  const double intent_norm = std::sqrt(
    intent_velocity[0] * intent_velocity[0] + intent_velocity[1] * intent_velocity[1]
    + intent_velocity[2] * intent_velocity[2]);
  const std::array<double, 3> intent_direction = intent_norm > 1.0e-9 ?
    std::array<double, 3>{intent_velocity[0] / intent_norm,
      intent_velocity[1] / intent_norm, intent_velocity[2] / intent_norm} :
    std::array<double, 3>{0.0, 0.0, 0.0};

  for (std::size_t index = 0; index < count; ++index) {
    Sample state_sample;
    state_sample.position = state.position;
    state_sample.velocity = state.velocity;
    state_sample.acceleration = state.acceleration;
    candidate.samples.push_back(state_sample);
    const std::array<double, 3> velocity_error{
      state.velocity[0] - intent_velocity[0],
      state.velocity[1] - intent_velocity[1],
      state.velocity[2] - intent_velocity[2]};
    const double velocity_error_cost =
      (velocity_error[0] * velocity_error[0] + velocity_error[1] * velocity_error[1])
      / speed_xy_sq + velocity_error[2] * velocity_error[2] / speed_z_sq;
    const double acceleration_cost =
      (state.acceleration[0] * state.acceleration[0]
      + state.acceleration[1] * state.acceleration[1]) / acceleration_xy_sq
      + state.acceleration[2] * state.acceleration[2] / acceleration_z_sq;
    const double progress =
      (state.velocity[0] * intent_direction[0]
      + state.velocity[1] * intent_direction[1]) / config.max_speed_xy
      + state.velocity[2] * intent_direction[2] / config.max_speed_z;
    cost += config.sample_seconds * (
      config.intent_weight * velocity_error_cost - config.progress_weight * progress
      + config.acceleration_weight * acceleration_cost);
    if (index + 1U < count) {
      const auto next = advance(config, state, target_velocity, config.sample_seconds);
      const std::array<double, 3> jerk{
        (next.acceleration[0] - state.acceleration[0]) / config.sample_seconds,
        (next.acceleration[1] - state.acceleration[1]) / config.sample_seconds,
        (next.acceleration[2] - state.acceleration[2]) / config.sample_seconds};
      const double jerk_cost = (jerk[0] * jerk[0] + jerk[1] * jerk[1]) / jerk_xy_sq
        + jerk[2] * jerk[2] / jerk_z_sq;
      cost += config.sample_seconds * config.jerk_weight * jerk_cost;
      state = next;
    }
  }
  candidate.cost = cost;
  candidate.feasible = dynamicallyFeasible(config, candidate.samples);
  return candidate;
}

inline Plan selectPlan(
  const PlannerConfig &config, const PlannerState &initial,
  const std::array<double, 3> &intent_velocity, int previous_id,
  const std::function<bool(const std::vector<Sample> &)> &collision_free) noexcept
{
  Plan output;
  if (!valid(config) || !finite(initial.position) || !finite(initial.velocity)
    || !finite(initial.acceleration) || !finite(intent_velocity)) return output;

  // Motion primitives rotate the requested horizontal heading and scale its
  // speed. Later obstacle integration only needs to replace collision_free.
  std::vector<Candidate> candidates;
  const double intent_xy = horizontalNorm(intent_velocity);
  if (intent_xy > 1.0e-6 || std::abs(intent_velocity[2]) > 1.0e-6) {
    const double heading = std::atan2(intent_velocity[1], intent_velocity[0]);
    for (std::size_t scale_index = 0; scale_index < config.speed_scales.size(); ++scale_index) {
      const double scale = config.speed_scales[scale_index];
      for (std::size_t offset_index = 0;
        offset_index < config.heading_offsets_rad.size(); ++offset_index) {
        const double offset = config.heading_offsets_rad[offset_index];
        const double horizontal = scale * intent_xy;
        const std::array<double, 3> target{
          horizontal * std::cos(heading + offset),
          horizontal * std::sin(heading + offset),
          scale * intent_velocity[2]};
        const int id = static_cast<int>(
          scale_index * config.heading_offsets_rad.size() + offset_index);
        candidates.push_back(makeCandidate(config, initial, intent_velocity, target, id));
      }
    }
  }
  // Braking is always available and is the only candidate for centered or
  // stale sticks. Later collision checking can also make it the safe fallback.
  const int braking_id = static_cast<int>(
    config.speed_scales.size() * config.heading_offsets_rad.size());
  candidates.push_back(makeCandidate(
    config, initial, intent_velocity, {0.0, 0.0, 0.0}, braking_id));

  Candidate *best = nullptr;
  Candidate *previous = nullptr;
  for (auto &candidate : candidates) {
    candidate.feasible = candidate.feasible && collision_free(candidate.samples);
    if (!candidate.feasible) continue;
    // Penalize only changes between moving branches. Starting from hold and
    // returning to the braking candidate must remain immediately responsive.
    if (previous_id >= 0 && previous_id != braking_id
      && candidate.id != previous_id && candidate.id != braking_id) {
      candidate.cost += config.switch_weight;
    }
    ++output.feasible_count;
    if (!best || candidate.cost < best->cost) best = &candidate;
    if (candidate.id == previous_id) previous = &candidate;
  }
  if (!best) return output;
  if (previous && previous->id != braking_id && best->id != braking_id
    && previous->cost <= best->cost + config.hysteresis) best = previous;
  output.selected = std::move(*best);
  output.valid = true;
  return output;
}

inline CircleTiming deriveCircleTiming(
  double radius_m, double reference_speed_limit_m_s,
  double acceleration_limit_m_s2) noexcept
{
  CircleTiming output;
  if (!std::isfinite(radius_m) || radius_m <= 0.0
    || !std::isfinite(reference_speed_limit_m_s) || reference_speed_limit_m_s <= 0.0
    || !std::isfinite(acceleration_limit_m_s2) || acceleration_limit_m_s2 <= 0.0) {
    return output;
  }

  constexpr double pi = 3.14159265358979323846;
  // smoothstep5'(tau) peaks at 1.875. Limit cruise speed by both the
  // configured reference cap and the centripetal-acceleration cap, then pick
  // the ramp duration so its peak tangential acceleration uses the same cap.
  output.cruise_speed_m_s = std::min(
    reference_speed_limit_m_s, std::sqrt(acceleration_limit_m_s2 * radius_m));
  output.ramp_seconds = 1.875 * output.cruise_speed_m_s / acceleration_limit_m_s2;
  output.period_seconds = output.ramp_seconds
    + 2.0 * pi * radius_m / output.cruise_speed_m_s;
  output.valid = std::isfinite(output.cruise_speed_m_s)
    && std::isfinite(output.ramp_seconds) && std::isfinite(output.period_seconds)
    && output.cruise_speed_m_s > 0.0 && output.ramp_seconds > 0.0
    && 2.0 * output.ramp_seconds < output.period_seconds;
  return output;
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
