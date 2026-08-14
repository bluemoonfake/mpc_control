#pragma once

#include "mpc_controller/mpc_solver.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

namespace mpc_controller::translational
{

inline constexpr std::size_t kHorizonLength = axis_mpc::kHorizonLength;
inline constexpr std::size_t kStateSize = axis_mpc::kStateDimension;
inline constexpr std::size_t kAxisCount = 3;
inline constexpr double kDtFirst = 0.01;
inline constexpr double kDtLater = 0.20;

using Vector3 = std::array<double, 3>;

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
  double solver_absolute_tolerance = 1.0e-5;
  double solver_relative_tolerance = 1.0e-5;
  std::array<double, 3> q_xy{500.0, 100.0, 100.0};
  std::array<double, 3> s_xy{1000.0, 300.0, 300.0};
  std::array<double, 3> q_z{100.0, 10.0, 10.0};
  std::array<double, 3> s_z{100.0, 10.0, 10.0};
  double max_speed_xy = 2.0;
  double max_acceleration_xy = 2.0;
  double max_control_xy = 2.0;
  double max_control_rate_xy = 5.0;
  double max_speed_z = 2.0;
  double max_acceleration_z = 2.0;
  double max_control_z = 50.0;
  double max_control_rate_z = 5.0;
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
  std::array<axis_mpc::Result, kAxisCount> axes{};
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
    config.admm_rho, config.solver_absolute_tolerance,
    config.solver_relative_tolerance, config.max_speed_xy,
    config.max_acceleration_xy, config.max_control_xy,
    config.max_control_rate_xy, config.max_speed_z,
    config.max_acceleration_z, config.max_control_z,
    config.max_control_rate_z};
  return std::all_of(positive_values.begin(), positive_values.end(), [](double value) {
      return std::isfinite(value) && value > 0.0;
    }) && config.max_iterations > 0
    && nonnegative(config.q_xy) && nonnegative(config.s_xy)
    && nonnegative(config.q_z) && nonnegative(config.s_z)
    && nonnegative(config.model_time_constant_xyz);
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
    if (!validTrajectory(trajectory) || !std::isfinite(time_from_start)
      || time_from_start < 0.0) {
      return false;
    }
    if (time_from_start <= trajectory.points.front().time_from_start) {
      output = trajectory.points.front();
      return true;
    }
    for (std::size_t index = 1; index < trajectory.points.size(); ++index) {
      const auto &left = trajectory.points[index - 1];
      const auto &right = trajectory.points[index];
      if (time_from_start <= right.time_from_start) {
        const double alpha = (time_from_start - left.time_from_start)
          / (right.time_from_start - left.time_from_start);
        output = interpolate(left, right, alpha);
        return true;
      }
    }
    if (!trajectory.hold_after_end) {
      return false;
    }
    output = trajectory.points.back();
    return true;
  }

  static bool buildHorizon(
    const ReferenceTrajectoryData &trajectory, double elapsed_seconds,
    double dt_first, double dt_later, ReferenceHorizon &output) noexcept
  {
    if (!validTrajectory(trajectory) || !std::isfinite(elapsed_seconds)
      || elapsed_seconds < 0.0 || !std::isfinite(dt_first) || dt_first <= 0.0
      || !std::isfinite(dt_later) || dt_later <= 0.0) {
      return false;
    }
    for (std::size_t index = 0; index < kHorizonLength; ++index) {
      const double offset = dt_first + static_cast<double>(index) * dt_later;
      if (!sampleAt(trajectory, elapsed_seconds + offset, output.points[index])) {
        return false;
      }
    }
    return true;
  }

private:
  static ReferencePoint interpolate(
    const ReferencePoint &left, const ReferencePoint &right,
    double alpha) noexcept
  {
    ReferencePoint output;
    output.time_from_start = left.time_from_start
      + alpha * (right.time_from_start - left.time_from_start);
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      output.position[axis] = left.position[axis]
        + alpha * (right.position[axis] - left.position[axis]);
      output.velocity[axis] = left.velocity[axis]
        + alpha * (right.velocity[axis] - left.velocity[axis]);
      output.acceleration[axis] = left.acceleration[axis]
        + alpha * (right.acceleration[axis] - left.acceleration[axis]);
    }
    output.yaw = left.yaw + alpha * shortestAngle(left.yaw, right.yaw);
    output.yaw_rate = left.yaw_rate + alpha * (right.yaw_rate - left.yaw_rate);
    return output;
  }
};

class TranslationalMpc final
{
public:
  explicit TranslationalMpc(const Config &config)
  : config_(config)
  {
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      solvers_[axis] = std::make_unique<axis_mpc::Solver>(
        axisConfiguration(config, axis, axis == 2));
    }
  }

  void reset() noexcept {reset({0.0, 0.0, 0.0});}

  void reset(const Vector3 &input_memory) noexcept
  {
    last_input_ = input_memory;
    for (const auto &solver : solvers_) {
      solver->reset();
    }
  }

  UpdateResult update(const MeasuredState &measured, const ReferenceHorizon &reference)
  {
    UpdateResult output;
    if (!validConfig(config_) || !std::all_of(
        solvers_.begin(), solvers_.end(),
        [](const auto &solver) {return solver && solver->configured();})) {
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
    const auto start = axis_mpc::Clock::now();
    const auto deadline = start + std::chrono::duration_cast<axis_mpc::Clock::duration>(
      std::chrono::duration<double>(config_.solver_deadline_seconds));
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      output.axes[axis] = solvers_[axis]->solve(
        initial_[axis], references_[axis], last_input_[axis], deadline);
      output.deadline_missed = output.deadline_missed
        || output.axes[axis].status == axis_mpc::Status::deadline_exceeded;
    }
    output.solve_time_seconds = std::chrono::duration<double>(
      axis_mpc::Clock::now() - start).count();
    if (!std::all_of(output.axes.begin(), output.axes.end(),
      [](const axis_mpc::Result &result) {return result.valid;})) {
      output.failure_reason = FailureReason::solver_not_converged;
      return output;
    }

    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      output.control[axis] = output.axes[axis].first_control;
      output.first_predicted_acceleration[axis] = output.axes[axis].prediction[0](2);
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
  static axis_mpc::Configuration axisConfiguration(
    const Config &config, std::size_t axis, bool vertical)
  {
    axis_mpc::Configuration output;
    output.dt_first = config.dt_first;
    output.dt_later = config.dt_later;
    output.model_time_constant = config.model_time_constant_xyz[axis];
    output.stage_weights = vertical ? config.q_z : config.q_xy;
    output.terminal_weights = vertical ? config.s_z : config.s_xy;
    output.max_speed = vertical ? config.max_speed_z : config.max_speed_xy;
    output.max_acceleration = vertical ?
      config.max_acceleration_z : config.max_acceleration_xy;
    output.max_control = vertical ? config.max_control_z : config.max_control_xy;
    output.max_control_rate = vertical ?
      config.max_control_rate_z : config.max_control_rate_xy;
    output.max_iterations = config.max_iterations;
    output.admm_rho = config.admm_rho;
    output.absolute_tolerance = config.solver_absolute_tolerance;
    output.relative_tolerance = config.solver_relative_tolerance;
    return output;
  }

  void setInputs(
    const MeasuredState &measured,
    const ReferenceHorizon &reference) noexcept
  {
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      initial_[axis] = axis_mpc::State(
        measured.position[axis], measured.velocity[axis], measured.acceleration[axis]);
      for (std::size_t step = 0; step < kHorizonLength; ++step) {
        references_[axis][step] = axis_mpc::State(
          reference.points[step].position[axis],
          reference.points[step].velocity[axis],
          reference.points[step].acceleration[axis]);
      }
    }
  }

  Config config_;
  std::array<std::unique_ptr<axis_mpc::Solver>, kAxisCount> solvers_{};
  std::array<axis_mpc::State, 3> initial_{};
  std::array<axis_mpc::Reference, 3> references_{};
  Vector3 last_input_{};
};

}  // namespace mpc_controller::translational
