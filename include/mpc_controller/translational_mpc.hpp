#pragma once

#include "mpc_controller/mpc_solver.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace mpc_controller::translational
{

inline constexpr std::size_t kHorizonLength = coupled_mpc::kHorizonLength;
inline constexpr std::size_t kStateSize = 3;
inline constexpr std::size_t kAxisCount = 3;
inline constexpr double kDtFirst = 0.01;
inline constexpr double kDtLater = 0.20;

using Vector3 = std::array<double, 3>;

enum class Backend
{
  coupled
};

struct ReferencePoint
{
  double time_from_start = 0.0;
  Vector3 position{};
  Vector3 velocity{};
  Vector3 acceleration{};
  double yaw = 0.0;
  double yaw_rate = 0.0;
};

struct ReferenceTrajectoryData
{
  double header_time_seconds = 0.0;
  bool hold_after_end = false;
  std::vector<ReferencePoint> points;
};

struct ReferenceHorizon
{
  std::array<ReferencePoint, kHorizonLength> points{};
};

struct MeasuredState
{
  Vector3 position{};
  Vector3 velocity{};
  Vector3 acceleration{};
};

struct Config
{
  double dt_first = kDtFirst;
  double dt_later = kDtLater;
  double solver_deadline_seconds = 0.018;
  int max_iterations = 400;
  std::array<double, 3> model_time_constant_xyz{0.0, 0.0, 0.0};
  double admm_rho = 0.02;
  double coupled_admm_rho = 0.5;
  double solver_absolute_tolerance = 1.0e-5;
  double solver_relative_tolerance = 1.0e-5;
  std::array<double, 3> q_xy{500.0, 100.0, 100.0};
  std::array<double, 3> s_xy{1000.0, 300.0, 300.0};
  std::array<double, 3> q_z{100.0, 10.0, 10.0};
  std::array<double, 3> s_z{100.0, 10.0, 10.0};
  double control_weight_xy = 0.0;
  double control_rate_weight_xy = 0.0;
  double control_weight_z = 0.0;
  double control_rate_weight_z = 0.0;
  double max_speed_xy = 2.0;
  double max_acceleration_xy = 2.0;
  double max_control_xy = 2.0;
  double max_control_rate_xy = 5.0;
  double max_speed_z = 2.0;
  double max_acceleration_z = 2.0;
  double max_control_z = 50.0;
  double max_control_rate_z = 5.0;
  double gravity_m_s2 = 9.80665;
  double max_tilt_rad = 0.7853981633974483;
  double min_collective_specific_force_m_s2 = 1.0;
  double max_collective_specific_force_m_s2 = 16.0;
  double constraint_slack_weight = 1.0e4;
  double max_constraint_slack = 20.0;
  Backend backend = Backend::coupled;
};

enum class FailureReason
{
  none,
  invalid_configuration,
  invalid_measured_state,
  invalid_reference,
  solver_not_converged,
  non_finite_solver_output
};

struct UpdateResult
{
  bool valid = false;
  FailureReason failure_reason = FailureReason::none;
  Vector3 control{};
  Vector3 first_predicted_acceleration{};
  coupled_mpc::Result coupled{};
  bool coupled_solver_ran = true;
  bool coupled_control_active = true;
  double shadow_control_difference_norm = 0.0;
  double coupled_solve_time_seconds = 0.0;
  bool deadline_missed = false;
  double solve_time_seconds = 0.0;
};

inline bool finite(const Vector3 &value) noexcept
{
  return std::all_of(value.begin(), value.end(), [](double item) {
    return std::isfinite(item);
  });
}

inline bool finite(const ReferencePoint &point) noexcept
{
  return std::isfinite(point.time_from_start) && point.time_from_start >= 0.0
    && finite(point.position) && finite(point.velocity) && finite(point.acceleration)
    && std::isfinite(point.yaw) && std::isfinite(point.yaw_rate);
}

inline bool validConfig(const Config &config) noexcept
{
  const auto nonnegative = [](const std::array<double, 3> &values) {
      return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value) && value >= 0.0;
      });
    };
  const std::array positive_values{
    config.dt_first, config.dt_later, config.solver_deadline_seconds,
    config.admm_rho, config.coupled_admm_rho, config.solver_absolute_tolerance,
    config.solver_relative_tolerance, config.max_speed_xy,
    config.max_acceleration_xy, config.max_control_xy,
    config.max_control_rate_xy, config.max_speed_z,
    config.max_acceleration_z, config.max_control_z,
    config.max_control_rate_z, config.gravity_m_s2, config.max_tilt_rad,
    config.min_collective_specific_force_m_s2,
    config.max_collective_specific_force_m_s2, config.constraint_slack_weight,
    config.max_constraint_slack};
  return std::all_of(positive_values.begin(), positive_values.end(), [](double value) {
      return std::isfinite(value) && value > 0.0;
    }) && config.max_iterations > 0
    && nonnegative(config.q_xy) && nonnegative(config.s_xy)
    && nonnegative(config.q_z) && nonnegative(config.s_z)
    && nonnegative(config.model_time_constant_xyz)
    && std::isfinite(config.control_weight_xy) && config.control_weight_xy >= 0.0
    && std::isfinite(config.control_rate_weight_xy)
    && config.control_rate_weight_xy >= 0.0
    && std::isfinite(config.control_weight_z) && config.control_weight_z >= 0.0
    && std::isfinite(config.control_rate_weight_z)
    && config.control_rate_weight_z >= 0.0
    && config.max_tilt_rad < 0.5 * M_PI
    && config.min_collective_specific_force_m_s2
    < config.max_collective_specific_force_m_s2;
}

class ReferenceSampler final
{
public:
  static double shortestAngle(double from, double to) noexcept
  {
    return std::atan2(std::sin(to - from), std::cos(to - from));
  }

  static bool validTrajectory(const ReferenceTrajectoryData &trajectory) noexcept
  {
    if (!std::isfinite(trajectory.header_time_seconds) || trajectory.points.empty()) {
      return false;
    }
    for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
      if (!finite(trajectory.points[index])) {
        return false;
      }
      if (index > 0
        && trajectory.points[index].time_from_start
        <= trajectory.points[index - 1].time_from_start) {
        return false;
      }
    }
    return true;
  }

  static bool sampleAt(
    const ReferenceTrajectoryData &trajectory, double time_from_start,
    ReferencePoint &output) noexcept
  {
    if (!validTrajectory(trajectory)) {
      return false;
    }
    if (time_from_start <= trajectory.points.front().time_from_start) {
      output = trajectory.points.front();
      return true;
    }
    if (time_from_start >= trajectory.points.back().time_from_start) {
      output = trajectory.points.back();
      if (trajectory.hold_after_end) {
        output.velocity = {0.0, 0.0, 0.0};
        output.acceleration = {0.0, 0.0, 0.0};
        output.yaw_rate = 0.0;
      }
      return true;
    }

    const auto it = std::lower_bound(
      trajectory.points.begin(), trajectory.points.end(), time_from_start,
      [](const ReferencePoint &point, double target) {
        return point.time_from_start < target;
      });
    if (it == trajectory.points.begin()) {
      output = *it;
      return true;
    }
    const auto &previous = *(it - 1);
    const auto &next = *it;
    const double delta_t = next.time_from_start - previous.time_from_start;
    if (delta_t <= 1.0e-6) {
      output = next;
      return true;
    }

    const double fraction = std::clamp(
      (time_from_start - previous.time_from_start) / delta_t, 0.0, 1.0);
    output.time_from_start = time_from_start;
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      output.position[axis] = previous.position[axis]
        + fraction * (next.position[axis] - previous.position[axis]);
      output.velocity[axis] = previous.velocity[axis]
        + fraction * (next.velocity[axis] - previous.velocity[axis]);
      output.acceleration[axis] = previous.acceleration[axis]
        + fraction * (next.acceleration[axis] - previous.acceleration[axis]);
    }
    output.yaw = previous.yaw + fraction * shortestAngle(previous.yaw, next.yaw);
    output.yaw_rate = previous.yaw_rate
      + fraction * (next.yaw_rate - previous.yaw_rate);
    return true;
  }

  static bool sampleHorizon(
    const ReferenceTrajectoryData &trajectory, double trajectory_age_seconds,
    double dt_first, double dt_later, ReferenceHorizon &horizon) noexcept
  {
    if (trajectory_age_seconds < 0.0 || dt_first <= 0.0 || dt_later <= 0.0
      || !validTrajectory(trajectory)) {
      return false;
    }

    double stage_time = trajectory_age_seconds;
    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      stage_time += (step == 0) ? dt_first : dt_later;
      if (!sampleAt(trajectory, stage_time, horizon.points[step])) {
        return false;
      }
    }
    return true;
  }

  static bool buildHorizon(
    const ReferenceTrajectoryData &trajectory, double trajectory_age_seconds,
    double dt_first, double dt_later, ReferenceHorizon &horizon) noexcept
  {
    return sampleHorizon(trajectory, trajectory_age_seconds, dt_first, dt_later, horizon);
  }
};

class TranslationalMpc final
{
public:
  explicit TranslationalMpc(const Config &config)
  : config_(config), coupled_solver_(coupledConfiguration(config))
  {
  }

  void reset() noexcept {reset({0.0, 0.0, 0.0});}

  void reset(const Vector3 &input_memory) noexcept
  {
    last_input_ = input_memory;
    coupled_solver_.reset();
  }

  UpdateResult update(const MeasuredState &measured, const ReferenceHorizon &reference)
  {
    UpdateResult output;
    if (!validConfig(config_) || !coupled_solver_.configured()) {
      output.failure_reason = FailureReason::invalid_configuration;
      return output;
    }
    if (!finite(measured.position) || !finite(measured.velocity)
      || !finite(measured.acceleration)) {
      output.failure_reason = FailureReason::invalid_measured_state;
      return output;
    }
    if (!std::all_of(reference.points.begin(), reference.points.end(),
      [](const ReferencePoint &point) {return finite(point);})) {
      output.failure_reason = FailureReason::invalid_reference;
      return output;
    }

    setInputs(measured, reference);
    const auto start = coupled_mpc::Clock::now();
    const auto solve_budget = std::chrono::duration_cast<coupled_mpc::Clock::duration>(
      std::chrono::duration<double>(config_.solver_deadline_seconds));
    const auto coupled_deadline = start + solve_budget;

    output.coupled_solver_ran = true;
    output.coupled = coupled_solver_.solve(
      coupled_initial_, coupled_reference_,
      coupled_mpc::Input(last_input_[0], last_input_[1], last_input_[2]),
      coupled_deadline);
    output.coupled_solve_time_seconds = std::chrono::duration<double>(
      coupled_mpc::Clock::now() - start).count();
    output.solve_time_seconds = output.coupled_solve_time_seconds;
    output.deadline_missed = (output.coupled.status == coupled_mpc::Status::deadline_exceeded);

    if (!output.coupled.valid) {
      output.failure_reason = FailureReason::solver_not_converged;
      return output;
    }

    output.coupled_control_active = true;
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      output.control[axis] = output.coupled.first_control(axis);
      output.first_predicted_acceleration[axis] = output.coupled.prediction[0](6 + axis);
    }

    if (!std::isfinite(output.solve_time_seconds) || !finite(output.control)
      || !finite(output.first_predicted_acceleration)) {
      output.failure_reason = FailureReason::non_finite_solver_output;
      return output;
    }

    last_input_ = output.control;
    output.valid = true;
    output.failure_reason = FailureReason::none;
    return output;
  }

private:
  static coupled_mpc::Configuration coupledConfiguration(const Config &config)
  {
    coupled_mpc::Configuration output;
    output.dt_first = config.dt_first;
    output.dt_later = config.dt_later;
    output.model_time_constant_xyz = config.model_time_constant_xyz;
    output.stage_weights_xy = config.q_xy;
    output.terminal_weights_xy = config.s_xy;
    output.stage_weights_z = config.q_z;
    output.terminal_weights_z = config.s_z;
    output.control_weights = {
      config.control_weight_xy, config.control_weight_xy, config.control_weight_z};
    output.control_rate_weights = {
      config.control_rate_weight_xy, config.control_rate_weight_xy,
      config.control_rate_weight_z};
    output.max_speed_xy = config.max_speed_xy;
    output.max_speed_z = config.max_speed_z;
    output.max_acceleration_xy = config.max_acceleration_xy;
    output.max_acceleration_z = config.max_acceleration_z;
    output.max_control_xy = config.max_control_xy;
    output.max_control_z = config.max_control_z;
    output.max_control_rate_xy = config.max_control_rate_xy;
    output.max_control_rate_z = config.max_control_rate_z;
    output.gravity_m_s2 = config.gravity_m_s2;
    output.max_tilt_rad = config.max_tilt_rad;
    output.min_collective_specific_force_m_s2 =
      config.min_collective_specific_force_m_s2;
    output.max_collective_specific_force_m_s2 =
      config.max_collective_specific_force_m_s2;
    output.constraint_slack_weight = config.constraint_slack_weight;
    output.max_constraint_slack = config.max_constraint_slack;
    output.max_iterations = config.max_iterations;
    output.admm_rho = config.coupled_admm_rho;
    output.absolute_tolerance = config.solver_absolute_tolerance;
    output.relative_tolerance = config.solver_relative_tolerance;
    return output;
  }

  void setInputs(
    const MeasuredState &measured,
    const ReferenceHorizon &reference) noexcept
  {
    coupled_initial_ <<
      measured.position[0], measured.position[1], measured.position[2],
      measured.velocity[0], measured.velocity[1], measured.velocity[2],
      measured.acceleration[0], measured.acceleration[1], measured.acceleration[2];
    for (std::size_t step = 0; step < kHorizonLength; ++step) {
      coupled_reference_[step] <<
        reference.points[step].position[0], reference.points[step].position[1],
        reference.points[step].position[2], reference.points[step].velocity[0],
        reference.points[step].velocity[1], reference.points[step].velocity[2],
        reference.points[step].acceleration[0], reference.points[step].acceleration[1],
        reference.points[step].acceleration[2];
    }
  }

  Config config_;
  coupled_mpc::Solver coupled_solver_;
  coupled_mpc::State coupled_initial_ = coupled_mpc::State::Zero();
  coupled_mpc::Reference coupled_reference_{};
  Vector3 last_input_{};
};

}  // namespace mpc_controller::translational
